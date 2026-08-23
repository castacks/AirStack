#!/usr/bin/env python3
"""batch_damage.py — damage many assets in one Isaac Sim, in parallel.

    cd AirStack
    UV_ENV_FILE=$PWD/.env.host uv run python \
        scene_gen/tools/batch_damage.py \
        --assets-file assets.txt --severity 0.55 --material brick \
        --out /tmp/ruins --shot /tmp/ruins/sheet

WHERE THE TIME GOES, AND WHAT THIS DOES ABOUT IT
------------------------------------------------
Damaging one asset with `earthquake_damage.py` costs about 110 s, and only ~30 s
of that is the work. The rest is Isaac Sim booting, paid again for every asset.
Two things follow, and this tool does both:

**The geometry never needed Isaac.** `disaster.source`, `.solids`, `.fracture`
and `.earthquake` import numpy, trimesh, scipy and pxr — nothing from Kit. So
every asset's cut is computed in a worker PROCESS, all of them at once. That
matters more than threads would: the work is pure Python and holds the GIL, so
threads would serialise it.

**One boot, and one settle.** The assets are laid out on a grid in a single
stage, far enough apart not to touch, and PhysX runs ONCE over all of them
together. N settles become one, on top of N boots becoming one.

So the cost goes from `N x (boot + geometry + settle)` to
`boot + geometry/jobs + settle`, and the boot is amortised over the whole batch.

WHAT COMES OUT
--------------
One USD per asset in `--out`, holding the settled ruin and its materials, ready
to reference into a scene. Settled poses are written back explicitly rather than
trusting PhysX to have flushed them to the USD layer.

`omniverse://` assets are the one thing a worker cannot resolve — only Kit has
that resolver — so they are detected and run in the main process instead.
"""

from __future__ import annotations

import argparse
import math
import multiprocessing as mp
import os
import sys
import time

import numpy as np

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import asset_sets, earthquake                     # noqa: E402

MATERIAL_NAMES = frozenset(earthquake.MATERIALS)


# ---------------------------------------------------------------------------
# Asset packs
# ---------------------------------------------------------------------------


def _read_asset_set(spec):
    """Resolve a set into per-asset jobs, with materials applied on top.

    Thin wrapper over `disaster.asset_sets.read_asset_set` — that reader knows
    the pack format and nothing about earthquakes; the material-tag rule
    belongs here. MUST run in a child process: see that module's docstring.
    """
    name, categories, default_materials, target_size = spec
    from disaster.asset_sets import read_asset_set

    jobs = []
    for e in read_asset_set(name, categories, target_size):
        # A tag naming a structural material wins over --material. None of the
        # packs carry one yet (their tags are placement hints such as "front"
        # or "low"), so this is inert until they do — and then it needs no
        # code change here.
        mats = [t for t in e["tags"] if t in MATERIAL_NAMES] or default_materials
        jobs.append({**e, "materials": list(mats)})
    return jobs


# ---------------------------------------------------------------------------
# Worker — geometry only, no Isaac
# ---------------------------------------------------------------------------


def _geometry(job):
    """Cut one asset. Runs in a worker process; returns picklable arrays.

    Module-level and plain-dict in/out because `multiprocessing` has to pickle
    both ends. `Sdf.Path` in particular does not survive the trip, so material
    paths come back as strings — which is all `UsdShade.Material.Get` needs.

    Resolution happens HERE, not in the parent, and that is not a detail:
    `resolve_asset` reaches `scene_generator`, which imports `pxr` at module
    scope. Pulling usd-core into the parent before `SimulationApp` starts stops
    Kit prepending its own USD build, and Kit then dies on startup with
    "extension class wrapper for base class UsdTyped has not been created yet".
    A worker has no Kit to upset. The lock serialises the download an
    `objaverse://` miss triggers, so two workers cannot race on one cache slot.
    """
    from pxr import Usd, UsdGeom

    idx, asset, opts, lock = job
    t0 = time.time()
    try:
        from disaster.source import resolve_asset
        with lock:
            asset = resolve_asset(asset, opts["target_size"])
        if opts.get("scale"):
            # An explicit scale means the pack already states real size, so
            # resolution must not renormalise it.
            opts = dict(opts, target_size=0.0)
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        ruin = earthquake.shake(
            stage, asset, severity=opts["severity"],
            materials=opts["materials"], soft_story=opts["soft_story"],
            seed=opts["seed"] + idx, target_size=opts["target_size"],
            up_axis=opts["up_axis"], thickness=opts["thickness"],
            chunks=opts["chunks"], quiet=True, scale=opts.get("scale", 0.0))
    except Exception as exc:                       # one bad asset must not
        return {"idx": idx, "asset": asset,        # take the batch down
                "error": f"{type(exc).__name__}: {exc}"}
    return {
        "idx": idx, "asset": asset, "error": None,
        "used_materials": opts["materials"],
        "chunks": ruin.chunks, "damage": ruin.damage,
        "materials": [m.name for m in ruin.materials],
        "falls": ruin.falls, "field": ruin.field,
        "mat_paths": [str(p) if p is not None else None
                      for p in ruin.source.mat_paths],
        "lo": ruin.lo, "hi": ruin.hi, "seconds": time.time() - t0,
    }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--assets", nargs="*", default=[],
                   help="asset strings, as the asset sets spell them")
    p.add_argument("--asset-set",
                   help="asset pack: a name from config/asset_sets/ (e.g. "
                        "'urban_intact_local') or a path to one of those YAMLs")
    p.add_argument("--category", nargs="+", default=["buildings"],
                   help="dotted keys under the set's `usds:` to damage "
                        "(default: buildings)")
    p.add_argument("--limit", type=int, default=0,
                   help="damage at most N assets from the set (0 = all)")
    p.add_argument("--assets-file",
                   help="file with one asset per line; '#' starts a comment, "
                        "whole-line or trailing")
    p.add_argument("--out", required=True, help="directory for the ruin USDs")
    p.add_argument("--material", nargs="+", default=["concrete"],
                   choices=sorted(earthquake.MATERIALS))
    p.add_argument("--severity", type=float, default=0.5)
    p.add_argument("--soft-story", action="store_true")
    p.add_argument("--rubble", action="store_true")
    p.add_argument("--chunks", type=int, default=90)
    p.add_argument("--dirt", type=float, default=0.07)
    p.add_argument("--settle", type=float, default=4.0)
    p.add_argument("--spacing", type=float, default=0.0,
                   help="metres between assets on the grid (0 = 3x their size)")
    p.add_argument("--jobs", type=int, default=0,
                   help="worker processes (0 = one per core, capped at the "
                        "number of assets)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--target-size", type=float, default=8.0)
    p.add_argument("--up-axis", choices=("z", "y"), default="z")
    p.add_argument("--thickness", type=float, default=0.05)
    p.add_argument("--shot", default="", help="write a contact sheet here")
    p.add_argument("--view", action="store_true",
                   help="open the Isaac Sim window on the finished grid and "
                        "leave it up (still writes the USDs first)")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    assets = list(args.assets)
    if args.assets_file:
        assets += asset_sets.read_assets_file(args.assets_file)

    entries = [{"asset": a, "scale": 0.0, "target_size": args.target_size,
                "up_axis": args.up_axis, "materials": list(args.material),
                "category": "-"} for a in assets]
    if args.asset_set:
        entries += asset_sets.run_isolated(_read_asset_set, (
            (args.asset_set, args.category, list(args.material),
             args.target_size),))
    if args.limit:
        entries = entries[:args.limit]
    if not entries:
        print("[batch] no assets given", flush=True)
        return 1
    os.makedirs(args.out, exist_ok=True)
    t_start = time.time()

    base = dict(severity=args.severity, soft_story=args.soft_story,
                seed=args.seed, thickness=args.thickness, chunks=args.chunks)
    per_asset = [dict(base, materials=e["materials"], scale=e["scale"],
                      target_size=e["target_size"], up_axis=e["up_axis"])
                 for e in entries]
    assets = [e["asset"] for e in entries]

    parallel_idx, kit_idx = asset_sets.split_kit_only(assets)
    if args.asset_set:
        by_cat = {}
        for e in entries:
            by_cat[e["category"]] = by_cat.get(e["category"], 0) + 1
        print("[batch] asset set "
              + ", ".join(f"{k}={v}" for k, v in sorted(by_cat.items())),
              flush=True)
    if kit_idx:
        print(f"[batch] {len(kit_idx)} omniverse:// asset(s) will be cut in "
              f"the main process — only Kit resolves that scheme", flush=True)
    parallel = [(i, assets[i]) for i in sorted(parallel_idx)]

    ctx = mp.get_context("spawn")     # fork + Kit in the child is fragile
    jobs = args.jobs or min(mp.cpu_count(), max(len(parallel), 1))
    print(f"[batch] {len(assets)} assets | {jobs} workers | "
          f"severity {args.severity} | {'+'.join(args.material)}", flush=True)

    # GEOMETRY FIRST, THEN KIT — not overlapped, and that is deliberate.
    # Running the pool alongside `SimulationApp` is tempting, since the two are
    # independent and it would hide ~75 s of boot behind the cut. It hangs: with
    # a live Manager and worker pool in the process, Kit's startup never
    # completes and the run sits there forever with idle children. The pool is
    # fine on its own (verified separately), so the two simply do not coexist.
    # Geometry dominates on any real batch anyway, so the boot is amortised
    # over the whole list regardless.
    t0 = time.time()
    results = {}
    if parallel:
        manager = ctx.Manager()
        lock = manager.Lock()
        with ctx.Pool(processes=jobs) as pool:
            for res in pool.imap_unordered(
                    _geometry,
                    [(i, a, per_asset[i], lock) for i, a in parallel]):
                results[res["idx"]] = res
                tag = res["error"] or (
                    f"{len(res['chunks'])} bodies in {res['seconds']:.0f}s")
                print(f"[batch]   [{res['idx']:3d}] {tag}", flush=True)
        manager.shutdown()
    cpu = sum(r.get("seconds", 0) for r in results.values())
    t_geom = time.time() - t0
    print(f"[batch] geometry {t_geom:.0f}s wall for {cpu:.0f}s of CPU "
          f"across {jobs} workers ({cpu / max(t_geom, 1e-9):.1f}x)", flush=True)

    from isaacsim import SimulationApp
    simulation_app = SimulationApp(launch_config={"headless": not args.view})

    import omni.kit.app
    from isaacsim.core.api import World
    from isaacsim.core.prims import RigidPrim
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
    from disaster import authoring

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    authoring.add_lighting(world.stage)
    stage = world.stage
    rng = np.random.default_rng(args.seed)

    for i, a in enumerate(assets):                  # the Kit-only stragglers
        if i in kit_idx and i not in results:
            try:
                o = per_asset[i]
                ruin = earthquake.shake(
                    stage, a, severity=o["severity"],
                    materials=o["materials"], soft_story=o["soft_story"],
                    seed=o["seed"] + i,
                    target_size=0.0 if o["scale"] else o["target_size"],
                    up_axis=o["up_axis"], thickness=o["thickness"],
                    chunks=o["chunks"], quiet=True, scale=o["scale"])
                results[i] = {
                    "idx": i, "asset": a, "error": None, "chunks": ruin.chunks,
                    "used_materials": o["materials"],
                    "damage": ruin.damage, "falls": ruin.falls,
                    "materials": [m.name for m in ruin.materials],
                    "field": ruin.field,
                    "mat_paths": [str(p) if p is not None else None
                                  for p in ruin.source.mat_paths],
                    "lo": ruin.lo, "hi": ruin.hi, "seconds": 0.0}
            except Exception as exc:
                results[i] = {"idx": i, "asset": a,
                              "error": f"{type(exc).__name__}: {exc}"}

    good = [results[k] for k in sorted(results) if not results[k]["error"]]
    if not good:
        print("[batch] nothing survived", flush=True)
        simulation_app.close()
        return 1

    span = max(float((r["hi"] - r["lo"]).max()) for r in good)
    pitch = args.spacing or span * 3.0
    cols = max(1, int(math.ceil(math.sqrt(len(good)))))

    views, roots, offsets, rubble_views = [], [], [], []
    for n, r in enumerate(good):
        # The grid offset is baked into the chunk centroids rather than put on
        # a parent Xform, so each body's world pose is exactly what it says and
        # nothing depends on how PhysX composes ancestor transforms.
        off = np.array([(n % cols) * pitch, (n // cols) * pitch, 0.0])
        for c in r["chunks"]:
            c["centroid"] = c["centroid"] + off
        root = f"/World/item_{r['idx']:03d}"
        UsdGeom.Xform.Define(stage, root)
        _reference_materials(stage, f"{root}/_source", r["asset"],
                             args.up_axis)
        mats = [earthquake.MATERIALS[m] for m in r["materials"]]
        authoring.author_ruin(stage, f"{root}/ruin", r["chunks"],
                              r["mat_paths"], r["damage"], mats, args.dirt, rng)
        if args.rubble:
            authoring.author_rubble(
                stage, f"{root}/rubble", r["field"],
                [earthquake.MATERIALS[m]
                 for m in r.get("used_materials", args.material)],
                args.severity, float(r["falls"].mean()),
                r["lo"] + off, r["hi"] + off, rng)
        views.append(RigidPrim(f"{root}/ruin/chunk_.*", name=f"ruin{n}"))
        world.scene.add(views[-1])
        if args.rubble:
            rubble_views.append(RigidPrim(f"{root}/rubble/rubble_.*",
                                          name=f"rub{n}"))
            world.scene.add(rubble_views[-1])
        else:
            rubble_views.append(None)
        roots.append(root)
        offsets.append(off)

    world.reset()
    for root, r in zip(roots, good):
        for i in range(len(r["chunks"])):
            prim = stage.GetPrimAtPath(f"{root}/ruin/chunk_{i:04d}")
            if prim:
                UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(False)

    t0 = time.time()
    for _ in range(int(args.settle * 60)):
        world.step(render=False)
    print(f"[batch] settled {len(good)} ruins together in {time.time()-t0:.0f}s",
          flush=True)

    # --- contact sheet, while everything is still laid out on the grid ----
    if args.shot:
        _frame(stage, set_camera_view, authoring)
        import omni.kit.viewport.utility as vu
        for _ in range(90):
            omni.kit.app.get_app().update()
        vu.capture_viewport_to_file(vu.get_active_viewport(),
                                    file_path=f"{args.shot}.png")
        for _ in range(40):
            omni.kit.app.get_app().update()
        print(f"[batch] wrote {args.shot}.png", flush=True)

    # --- freeze, bake the settled poses, export ---------------------------
    # Poses are baked as they stand, so the live stage keeps its grid and is
    # worth looking at (--view). The grid offset is removed on the EXPORTED
    # COPY instead — a translate on the copied root — so each file still holds
    # a ruin sitting on its own origin, referenceable without knowing where it
    # happened to sit in the batch.
    for view, rview, root, r, off in zip(views, rubble_views, roots, good,
                                         offsets):
        n_c = len(r["chunks"])
        pos, quat = view.get_world_poses()
        authoring.bake_poses(
            stage, [f"{root}/ruin/chunk_{i:04d}" for i in range(n_c)],
            np.asarray(pos), np.asarray(quat))
        for i in range(n_c):
            prim = stage.GetPrimAtPath(f"{root}/ruin/chunk_{i:04d}")
            if prim:
                UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(True)
        if rview is not None:
            rpos, rquat = rview.get_world_poses()
            authoring.bake_poses(
                stage,
                [f"{root}/rubble/rubble_{k:04d}" for k in range(len(rpos))],
                np.asarray(rpos), np.asarray(rquat))

    written = 0
    for root, r, off in zip(roots, good, offsets):
        name = _stem(r["asset"])
        out = os.path.join(args.out, f"{name}.usd")
        if _export_subtree(stage, root, out, off):
            written += 1
    print(f"[batch] wrote {written} ruin USDs to {args.out}", flush=True)

    print(f"[batch] TOTAL {time.time() - t_start:.0f}s wall for "
          f"{len(good)} ruins ({(time.time() - t_start) / max(len(good), 1):.0f}s "
          f"each, against ~110s each run one at a time)", flush=True)
    bad = [r for r in results.values() if r["error"]]
    for r in bad:
        print(f"[batch] FAILED {r['asset']}: {r['error']}", flush=True)

    if args.view:
        _frame(stage, set_camera_view, authoring)
        print("[batch] window open — close it to exit", flush=True)
        while simulation_app.is_running():
            world.step(render=True)
    simulation_app.close()
    return 0 if not bad else 2


def _frame(stage, set_camera_view, authoring):
    """Point the camera at whatever is actually on the stage."""
    box = authoring.content_bounds(stage, "/World")
    if box is None:
        return
    lo, hi = box
    centre = (lo + hi) * 0.5
    reach = float(np.linalg.norm(hi - lo)) * 0.7 + 5.0
    set_camera_view([centre[0] + reach, centre[1] - reach, centre[2] + reach * 0.6],
                    [float(centre[0]), float(centre[1]), float(lo[2])])


def _reference_materials(stage, path, asset, up_axis):
    """Bring the asset's Material prims onto the stage, invisible.

    The worker sent geometry and the PATHS of the materials it referred to, not
    the materials themselves. Referencing the asset here recreates those prims
    at the same paths, so the bindings resolve — and since only the shading
    network is wanted, the geometry that comes with it is hidden.
    """
    from pxr import UsdGeom

    src = UsdGeom.Xform.Define(stage, path)
    src.GetPrim().GetReferences().AddReference(asset)
    if up_axis.lower() == "y":
        src.AddRotateXOp().Set(90.0)
    UsdGeom.Imageable(src).MakeInvisible()
    return src


def _export_subtree(stage, root, out_path, offset=None):
    """Save one item's subtree as a standalone USD with its own default prim.

    `offset` is the grid position this item was settled at; it is cancelled by
    a translate on the copied root so the file sits on its own origin, without
    disturbing the live stage.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom

    try:
        dst = Usd.Stage.CreateInMemory()
        name = root.rsplit("/", 1)[-1]
        dst.DefinePrim(f"/{name}", "Xform")
        Sdf.CopySpec(stage.GetRootLayer(), Sdf.Path(root),
                     dst.GetRootLayer(), Sdf.Path(f"/{name}"))
        prim = dst.GetPrimAtPath(f"/{name}")
        if offset is not None:
            UsdGeom.Xformable(prim).AddTranslateOp().Set(
                Gf.Vec3d(*[-float(v) for v in offset]))
        dst.SetDefaultPrim(prim)
        dst.GetRootLayer().Export(out_path)
        return True
    except Exception as exc:
        print(f"[batch] export failed for {root}: "
              f"{type(exc).__name__}: {exc}", flush=True)
        return False


def _stem(asset):
    s = asset.rstrip("/").rsplit("/", 1)[-1]
    return os.path.splitext(s)[0] or "ruin"


if __name__ == "__main__":
    sys.exit(main())
