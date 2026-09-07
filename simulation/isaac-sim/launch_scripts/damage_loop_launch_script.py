#!/usr/bin/env python
"""Fast loop: ONE asset, ONE rung, damage re-run from scratch, photographed.

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/loop \\
    DAMAGE_ASSET=midrise_14_0204a DAMAGE_LEVEL=pancaked \\
    ISAAC_SIM_SCRIPT_NAME=damage_loop_launch_script.py airstack up isaac-sim

Or, against a container that is already up, `scene_gen/tools/damage_loop.sh`.

NO ARCHETYPES, IN EITHER DIRECTION
----------------------------------
This never reads and never writes `assets/archetypes*`. It references the
source asset, runs the damage, settles it and takes a picture — so what you are
looking at is the CURRENT state of `disaster/`, not something a bake decided
hours ago. That is the whole point: editing `quake.py` or `mesh_damage.py` and
re-running this shows the edit, with nothing cached in between.

It is deliberately NOT `archetypes.bake.Baker`. The baker is about producing a
library — grid layout, manifests, convergence gates, export, batching — and all
of that is cost and indirection a look at one wreck does not need.

WHAT IT SKIPS, ON PURPOSE
-------------------------
The debris ring (`disaster.debris.shed`) a real bake authors around the wreck.
This is a mesh-damage loop: the question it answers is what the FRACTURE did,
and a few hundred rubble props around the base make that harder to see, not
easier.

PICKING AN ASSET
----------------
Cost scales with the building's SURFACE AREA, not its triangle count —
`mesh_damage.cells_for` is envelope area over fragment size squared. So the
lowest-poly asset in the pack is not the fastest: `tower_19_0728` is 17
triangles and 200,000 m3, which is ~5,600 cells and minutes per run.
`midrise_14_0204a` (22 triangles, 8,816 m3, ~420 cells) is the cheap one.
"""
import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

_HERE = os.path.dirname(os.path.abspath(__file__))
_ISAAC_SIM_DIR = os.path.dirname(_HERE)
_AIRSTACK = os.path.dirname(os.path.dirname(_ISAAC_SIM_DIR))
for _p in (os.path.join(_AIRSTACK, "scene_gen"), _ISAAC_SIM_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import random                                                    # noqa: E402
import time                                                      # noqa: E402
import numpy as np                                               # noqa: E402
import omni.usd                                                  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux                     # noqa: E402
import scene_generator as _sg                                     # noqa: E402
from compile_disaster import compile_earthquake                   # noqa: E402
from compile_disaster import load_scene_config                    # noqa: E402
from disaster import kinds, settle as S                           # noqa: E402

ASSET = os.environ.get("DAMAGE_ASSET", "midrise_14_0204a")
LEVEL = os.environ.get("DAMAGE_LEVEL", "pancaked")
CONFIG = os.environ.get("DAMAGE_CONFIG", "urban_v2")
SEED = int(os.environ.get("DAMAGE_SEED", "7"))
STEPS = int(os.environ.get("DAMAGE_STEPS", "1800"))
#: Seconds of simulated time per solver step. Cost is per STEP, so a coarser
#: dt buys the same simulated collapse for proportionally fewer of them.
DT = float(os.environ.get("DAMAGE_DT", "")) if os.environ.get("DAMAGE_DT") \
    else 1.0 / 60.0
#: Fraction of its triangles the source is reduced to before damage, or 0 to
#: leave it alone. The cutter costs cells x faces, so this is the only lever
#: that touches the second term.
DECIMATE = float(os.environ.get("DAMAGE_DECIMATE", "0") or 0.0)
#: Lateral displacement past which `settle` calls a piece thrown out and
#: deactivates it. 0 disables the cap entirely and keeps every piece.
MAX_TRAVEL = float(os.environ.get("DAMAGE_MAX_TRAVEL", "100") or 0.0)
#: Load this USD instead of the one the plan names for `DAMAGE_ASSET`, while
#: keeping that asset's scale, up-axis and material. For trying a rebuilt copy
#: of an asset — a baked low-poly, say — against the pipeline that will
#: eventually consume it, without touching an asset set.
SOURCE_OVERRIDE = os.environ.get("DAMAGE_SOURCE", "").strip()
#: Scatter the dedicated debris ASSETS around the building as well as its own
#: fragments — planks, rebar, sheeting, the mounds of fines. Off with 0.
DEBRIS = os.environ.get("DAMAGE_DEBRIS", "1") != "0"
#: How big a pile's COLLIDER is against the pile you can see. Below 1 the
#: collider sits inside the visible mound, so anything landing on it comes to
#: rest below the surface and reads as partly buried in the rubble rather than
#: perched on top of it.
PILE_COLLIDER = float(os.environ.get("DAMAGE_PILE_COLLIDER", "0.72"))
SNAP_DIR = os.environ.get("SNAP_DIR",
                          "/isaac-sim/.nvidia-omniverse/logs/damage_loop")


def _snaps():
    """`utils/snapshots.py` BY PATH — Kit already owns the name `utils`."""
    import importlib.util as ilu

    spec = ilu.spec_from_file_location(
        "airstack_snapshots", os.path.join(_ISAAC_SIM_DIR, "utils",
                                           "snapshots.py"))
    mod = ilu.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _find(config, name):
    """The asset pack's own entry for *name*: url, scale and axis-up.

    Reads the PACK, not the archetype library — the scale matters (these packs
    are centimetre-authored, and referencing at 1.0 builds an 8 km building).
    """
    from archetypes import plan as P

    want = str(name).lower().strip()
    items = [i for i in P.build_plan(config, "earthquake")
             if i.build == "library"]
    for it in items:
        if it.type.lower() == want:
            return it
    for it in items:
        if want in str(it.source).lower():
            return it
    near = sorted(i.type for i in items if want[:5] in i.type.lower())[:8]
    raise SystemExit(f"[loop] no asset {name!r} in {CONFIG}"
                     + (f"; close: {near}" if near else ""))


def _light(stage) -> None:
    """A dome and a key, matching `archetypes.bake.prepare_stage`.

    Without this the capture is a BLACK FRAME and everything upstream looks
    like it worked — the damage and settle both report real numbers, because
    neither of them cares about light. A bare `Usd.Stage` has no lighting at
    all; the benches get theirs from the scene they build, and this script
    builds no scene.
    """
    UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome")) \
        .CreateIntensityAttr(900.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2200.0)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 30.0))


def _ground(stage, half: float):
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    mesh.CreatePointsAttr([Gf.Vec3f(-half, -half, 0), Gf.Vec3f(half, -half, 0),
                           Gf.Vec3f(half, half, 0), Gf.Vec3f(-half, half, 0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.CreateExtentAttr([Gf.Vec3f(-half, -half, 0), Gf.Vec3f(half, half, 0)])
    return mesh.GetPrim()


def _spread(stage, config, names) -> int:
    """One damaged building per row, for comparing several at once.

    NOT THE THREE-STAGE ROW. That row exists to show one asset's pipeline —
    pristine, thickened, destroyed — and repeating it per asset triples the
    cost (a thicken and a cutaway each) to show the same three steps over and
    over. A spread is asking a different question: what does the SAME rung
    look like across assets. So each row is just the finished wreck, and the
    saving is what makes several of them cheap.

    ROWS RUN ACROSS THE FRAME, along +X, not away from it. Stacked in +Y they
    recede from the camera, and five of them at a pitch that clears their
    debris is ~490 m of DEPTH — which 16:9 cannot hold without either cutting
    rows off or shrinking every wreck to a speck. Along +X the subject is wide
    and shallow, which is the shape the frame actually is.
    """
    from disaster import debris as D
    from disaster.levels import ladder

    # A ROW MAY NAME ITS OWN RUNG, as `asset:rung`, so one picture can carry
    # the same asset at two severities — which is the comparison that actually
    # wants them side by side. Bare names take the run's `DAMAGE_LEVEL`.
    rungs = {r.name: r for r in ladder("earthquake", "structure")}
    spec = []
    for n in names:
        asset, _, lvl = n.partition(":")
        lvl = lvl or LEVEL
        if lvl not in rungs:
            raise SystemExit(f"[loop] no rung {lvl!r} on the earthquake ladder")
        spec.append((asset, rungs[lvl]))
    names = [a for a, _ in spec]
    model = kinds.get("earthquake")

    # Measure every asset first: the row pitch has to clear the widest one,
    # and debris reaches well past the building itself.
    items, sizes = [], []
    for i, name in enumerate(names):
        it = _find(config, name)
        probe = stage.DefinePrim(Sdf.Path(f"/World/probe_{i}"))
        probe.GetReferences().AddReference(
            str(it.source) if "://" in str(it.source)
            else _abs(config, it.source))
        pxf = UsdGeom.Xformable(probe)
        pxf.ClearXformOpOrder()
        if str(it.axis_up).upper() == "Y":
            pxf.AddRotateXOp().Set(90.0)
        pxf.AddScaleOp().Set(Gf.Vec3d(it.scale, it.scale, it.scale))
        rng = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_,
                                    UsdGeom.Tokens.render]
                                ).ComputeWorldBound(probe).ComputeAlignedRange()
        sizes.append(rng.GetSize())
        items.append(it)
        stage.RemovePrim(probe.GetPath())
    # Wide enough that neighbouring debris fields do not merge, and no wider:
    # every extra metre here is empty ground the camera has to frame.
    pitch = max(max(float(s[0]), float(s[1])) for s in sizes) * 1.3 + 55.0

    _ground(stage, pitch * len(names) + 120.0)
    _light(stage)

    loose, statics, wreck, t0_all = [], [], [], time.time()
    for i, (it, size, rung) in enumerate(
            zip(items, sizes, [r for _a, r in spec])):
        x = i * pitch
        path = f"/World/row_{i}_{it.type}"
        prim = stage.DefinePrim(Sdf.Path(path))
        prim.GetReferences().AddReference(
            str(it.source) if "://" in str(it.source)
            else _abs(config, it.source))
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 0.0))
        if str(it.axis_up).upper() == "Y":
            xf.AddRotateXOp().Set(90.0)
        xf.AddScaleOp().Set(Gf.Vec3d(it.scale, it.scale, it.scale))

        t = time.time()
        rep = model.damage_archetype(
            stage, prim, rung.name, seed=SEED + i, config=config,
            intensity=min(1.0, rung.at + 0.06), source=str(it.source))
        mine = list(rep.get("loose") or [])
        print(f"[loop] {it.type + '/' + rung.name:38} "
              f"{time.time() - t:5.1f}s  "
              f"cells={rep.get('cells')} loose={len(mine)} "
              f"{size[0]:.0f}x{size[1]:.0f}x{size[2]:.0f} m", flush=True)

        # SETTLE THE WRECK FIRST, so the debris can be scattered along where
        # the rubble ACTUALLY came to rest rather than along the plan the
        # building used to occupy. Baked, so it is static scenery for the
        # debris settle that follows.
        if mine:
            S.run(stage, mine, ["/World/ground"], steps=STEPS, kick=0.15,
                  bake_result=True, dt=DT, max_travel_m=MAX_TRAVEL)
            fp, ctr = D.settled_footprint(
                stage, rep.get("paths") or [],
                ((float(size[0]), float(size[1])), (x, 0.0)))
            print(f"[loop]   settled footprint {fp[0]:.0f}x{fp[1]:.0f} m "
                  f"(plan was {size[0]:.0f}x{size[1]:.0f})", flush=True)
        else:
            fp, ctr = (float(size[0]), float(size[1])), (x, 0.0)
        wreck.extend(rep.get("paths") or [])

        if DEBRIS:
            _dis = _sg._stage(config, "disaster") or {}
            if not float((_dis.get("debris") or {}).get("shed_m3_per_m") or 0.0):
                _dis["debris"] = compile_earthquake(
                    float(rung.at), {}, (1000.0, 1000.0))["debris"]
            shed = D.shed(
                stage, config, kind=str(rep.get("material") or "masonry"),
                rung=rung.name,
                centre=ctr,
                footprint_m=fp,
                height_m=float(size[2]),
                parent_path=f"{path}_debris",
                rng=random.Random(SEED + i * 97 + 4703))
            # The pieces JOIN THE SETTLE — they are dropped from `lift` and
            # only physics can seat them on whatever is under them. The mounds
            # are static, and get the shrunken collider so a piece landing on
            # one beds into it instead of perching on its silhouette.
            piles = list(shed["statics"])
            loose.extend(shed["paths"])
            statics.extend(D.pile_colliders(stage, piles, PILE_COLLIDER))
            print(f"[loop]   debris: {len(shed['paths'])} piece(s) dropped + "
                  f"{len(piles)} mound(s), r={shed['radius_m']:.1f} m",
                  flush=True)

    print(f"[loop] all damage {time.time() - t0_all:5.1f}s for {len(names)} "
          f"asset(s)", flush=True)
    t = time.time()
    if loose:
        # ONLY THE DEBRIS MOVES NOW. The wrecks are baked, and go in as static
        # colliders so the scatter lands on them instead of through them.
        info = S.run(stage, loose, statics + wreck + ["/World/ground"],
                     steps=STEPS, kick=0.15, bake_result=True, dt=DT,
                     max_travel_m=MAX_TRAVEL)
        print(f"[loop] debris settle {time.time() - t:5.1f}s  "
              f"steps={info.get('steps_used')}/{STEPS} "
              f"still_moving={info.get('still_moving')} "
              f"bodies={len(loose)}", flush=True)

    os.makedirs(SNAP_DIR, exist_ok=True)
    snaps = _snaps()
    # FRAME WHAT IS THERE, not the row pitch. Pitch is mostly the empty ground
    # between wrecks, and framing on it put five piles in the middle 5% of the
    # picture. The union of the rows and their debris is the actual subject.
    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_,
                                  UsdGeom.Tokens.render])
    lo = np.array([1e30, 1e30, 1e30])
    hi = -lo.copy()
    # BY NAME, not by walking /World. A DomeLight's world bound is effectively
    # infinite, so including the lights put `wide` in the kilometres and shrank
    # five wrecks to five specks.
    for prim in stage.GetPrimAtPath("/World").GetChildren():
        if not prim.IsActive() or not prim.GetName().startswith("row_"):
            continue
        rng_ = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng_.IsEmpty():
            continue
        lo = np.minimum(lo, np.array(rng_.GetMin()))
        hi = np.maximum(hi, np.array(rng_.GetMax()))
    # ANYTHING UNDER THE FLOOR IS NOT THE SUBJECT. At a coarse `dt` a fast
    # fragment can pass through the ground between solves; measured on this
    # spread at dt=1/10 one reached 440 m DOWN, which made `ext_z` 455 m, blew
    # `wide` up to 931 m and shrank five wrecks to five specks. Clamped rather
    # than chased: the escapee is a settle problem (see the note below), and
    # the camera should not be hostage to it.
    lo[2] = max(float(lo[2]), -2.0)
    ext = hi - lo
    mid = (hi + lo) / 2.0
    h = float(ext[2])
    print(f"[loop] frame bbox lo={np.round(lo,1)} hi={np.round(hi,1)} "
          f"ext={np.round(ext,1)}", flush=True)
    # The same rule the single-asset row uses: the vertical field is the narrow
    # one on 16:9, so a metre of height needs more pull-back than a metre of
    # width.
    wide = max(float(ext[0]), float(ext[1]), h * (1280.0 / 720.0)) * 1.15
    cx = float(mid[0])
    print(f"[loop] frame wide={wide:.0f} eye_y={float(mid[1]) - wide * 1.05:.0f}"
          f" eye_z={max(wide * 0.42, h * 0.55):.0f}", flush=True)
    snaps.place_camera(stage,
                       (cx, float(mid[1]) - wide * 1.05,
                        max(wide * 0.42, h * 0.55)),
                       (cx, float(mid[1]), h * 0.45))
    out = os.path.join(SNAP_DIR, f"spread_{len(names)}_{LEVEL}.png")
    snaps.snapshot(out, 50)
    print(f"[loop] DONE -> {out}", flush=True)
    while simulation_app.is_running():
        simulation_app.update()
    return 0


def main() -> int:
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))

    config = load_scene_config(CONFIG)
    names = [a.strip() for a in ASSET.split(",") if a.strip()]
    if len(names) > 1:
        print(f"[loop] spread of {len(names)}: {', '.join(names)}", flush=True)
        return _spread(stage, config, names)
    item = _find(config, ASSET)
    print(f"[loop] {item.type}  {LEVEL}  scale={item.scale} up={item.axis_up}",
          flush=True)

    # --- reference the source asset, untouched -----------------------------
    prim_path = "/World/subject"
    prim = stage.DefinePrim(Sdf.Path(prim_path))
    _src = (SOURCE_OVERRIDE or (str(item.source) if "://" in str(item.source)
                                else _abs(config, item.source)))
    if SOURCE_OVERRIDE:
        print(f"[loop] source override -> {SOURCE_OVERRIDE}", flush=True)
    prim.GetReferences().AddReference(_src)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    if str(item.axis_up).upper() == "Y":
        xf.AddRotateXOp().Set(90.0)
    xf.AddScaleOp().Set(Gf.Vec3d(item.scale, item.scale, item.scale))

    bb = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_,
                               UsdGeom.Tokens.render]).ComputeWorldBound(prim)
    size = bb.ComputeAlignedRange().GetSize()
    print(f"[loop] envelope {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} m",
          flush=True)
    _ground(stage, max(float(size[0]), float(size[1])) * 9.0 + 60.0)
    _light(stage)

    # --- the two earlier stages, side by side ------------------------------
    # PRISTINE | THICKENED | DESTROYED, so each step of the pipeline is
    # visible rather than inferred. The middle one is the interesting one: it
    # is what `solidify` produced, before a single cut, so an unfilled wall
    # cross-section there is solidify's and not the fracture's.
    step = max(float(size[0]), float(size[1])) * 1.6 + 10.0
    stages = {}
    if os.environ.get("DAMAGE_STAGES", "1") != "0":
        from disaster import mesh_damage as _md
        # PRISTINE | DECIMATED | THICKENED | DESTROYED, left to right, in the
        # order the pipeline actually applies them. `decimated` is its own
        # stage because it is now the input everything downstream sees: the
        # thickened reference below is built FROM it, not from the full-res
        # source, so the row shows what the cutter was really handed.
        _row = ((3, "pristine"), (2, "decimated"), (1, "thickened")) \
            if 0.0 < DECIMATE < 1.0 else ((2, "pristine"), (1, "thickened"))
        for k, label in _row:
            other = stage.DefinePrim(Sdf.Path(f"/World/stage_{label}"))
            other.GetReferences().AddReference(_src)
            oxf = UsdGeom.Xformable(other)
            oxf.ClearXformOpOrder()
            oxf.AddTranslateOp().Set(Gf.Vec3d(-step * k, 0.0, 0.0))
            if str(item.axis_up).upper() == "Y":
                oxf.AddRotateXOp().Set(90.0)
            oxf.AddScaleOp().Set(Gf.Vec3d(item.scale, item.scale, item.scale))
            stages[label] = -step * k
        # Thicken the middle one exactly the way `damage_building` would:
        # subdivide to the same edge length first, because solidify extrudes
        # whatever tessellation it is handed.
        if 0.0 < DECIMATE < 1.0:
            for _lbl in ("decimated", "thickened"):
                _p = stage.GetPrimAtPath(f"/World/stage_{_lbl}")
                if _p and _p.IsValid():
                    _md.decimate_prims(_md.mesh_prims(_p), DECIMATE)
        mid = stage.GetPrimAtPath("/World/stage_thickened")
        prims = _md.mesh_prims(mid)
        for mp in prims:
            _md.subdivide(mp, 4.0, max_points=400_000)
        bb2 = _md.bounds_of(_md.mesh_prims(mid))
        wall = _md.wall_for(mid, _md.material("masonry").wall_m)
        n_th = _md.solidify_prims(_md.mesh_prims(mid), wall, bounds=bb2)
        print(f"[loop] thickened reference: wall_m={wall:.2f} on {n_th} mesh(es)",
              flush=True)
        # CUT IT OPEN. A solidified shell is watertight — measured, 0 open
        # edges of 15,516 — so a lamp inside it is completely enclosed and
        # invisible. The only way to see the interior is to remove a wall, and
        # clipping with the SAME `_clip_by_plane` the fracture uses puts the
        # cap on screen at the same time: one clean cut through a known wall,
        # where an unfilled cross-section is unmissable.
        if os.environ.get("DAMAGE_CUTAWAY", "1") != "0":
            soup, _taken = _md._mesh_soup(_md.mesh_prims(mid))
            c = bb2.center
            cv, cf, cuv, cm = _md._clip_by_plane(
                # NORMAL TOWARD THE CAMERA. `_clip_by_plane` keeps the half
                # behind the normal, so pointing it at the viewer (-y) removes
                # the near wall and the interior faces the lens.
                soup.verts, soup.faces, np.array([0.0, -1.0, 0.0]),
                np.array([float(c[0]), float(c[1]), float(c[2])]),
                soup.uv, soup.fmat, cap=True, cap_mat=None)
            for mp in _md.mesh_prims(mid):
                mp.SetActive(False)
            # Authored directly rather than through `_author_soup`, which
            # wants a root transform and the material bookkeeping a fragment
            # carries; this is a look-inside prop, not a fragment.
            # THROUGH `_author_soup`, so the cutaway carries the same UVs and
            # material bindings a fragment does — authored bare it renders
            # default grey, which says nothing about whether the materials are
            # right. Its `root_inv` is a NUMPY 4x4, not a `Gf.Matrix4d`;
            # identity here because the soup is already in world space, which
            # is also why it goes under /World and not under the translated
            # stage prim (that would apply the row offset twice).
            # `uv_names` IS THE SIXTH ARGUMENT AND IT MATTERS. Dropped, the
            # Soup defaults to no source names and `_author_soup` writes the
            # UVs under `st` alone — but these materials read `uv0`, so every
            # lookup misses and each shape renders as one flat colour. That is
            # the same mismatch that made the fragments untextured.
            _md._author_soup(stage, Sdf.Path("/World/cutaway"),
                             _md.Soup(cv, cuv, cf, cm, soup.mats,
                                      soup.uv_names), cv,
                             np.eye(4))
            print(f"[loop] cutaway: {len(cf)} tris kept of {len(soup.faces)}",
                  flush=True)

    # --- run the damage, right now ----------------------------------------
    # DECIMATE FIRST, on the damaged copy only — the pristine and thickened
    # stages keep their full tessellation, so the picture shows what was given
    # up as well as what it bought.
    if 0.0 < DECIMATE < 1.0:
        from disaster import mesh_damage as _md2
        t_d = time.time()
        before = sum(len(UsdGeom.Mesh(mp).GetFaceVertexCountsAttr().Get() or [])
                     for mp in _md2.mesh_prims(prim))
        got = _md2.decimate_prims(_md2.mesh_prims(prim), DECIMATE)
        after = sum(len(UsdGeom.Mesh(mp).GetFaceVertexCountsAttr().Get() or [])
                    for mp in _md2.mesh_prims(prim))
        print(f"[loop] decimate {time.time() - t_d:4.1f}s  {before} -> {after} "
              f"faces ({after / max(before, 1):.4f}) on {got['meshes']} mesh(es)",
              flush=True)

    t0 = time.time()
    model = kinds.get("earthquake")
    rung = None
    for r in __import__("disaster.levels", fromlist=["levels"]).ladder(
            "earthquake", "structure"):
        if r.name == LEVEL:
            rung = r
    if rung is None:
        raise SystemExit(f"[loop] no rung {LEVEL!r} on the earthquake ladder")
    report = model.damage_archetype(
        stage, prim, LEVEL, seed=SEED, config=config,
        intensity=min(1.0, rung.at + 0.06), source=str(item.source))
    loose = list(report.get("loose") or [])
    t_dmg = time.time() - t0
    print(f"[loop] damage {t_dmg:5.1f}s  cells={report.get('cells')} "
          f"loose={len(loose)} thickened={report.get('thickened')} "
          f"material={report.get('material')}", flush=True)

    # --- settle the wreck, then scatter along where it LANDED --------------
    t_w = time.time()
    fp, ctr = (float(size[0]), float(size[1])), (0.0, 0.0)
    if loose:
        info = S.run(stage, loose, ["/World/ground"], steps=STEPS, kick=0.15,
                     bake_result=True, dt=DT, max_travel_m=MAX_TRAVEL)
        fp, ctr = D.settled_footprint(stage, report.get("paths") or [],
                                     (fp, ctr))
        print(f"[loop] wreck settle {time.time() - t_w:5.1f}s  "
              f"steps={info.get('steps_used')}/{STEPS} "
              f"still_moving={info.get('still_moving')}  "
              f"settled footprint {fp[0]:.0f}x{fp[1]:.0f} m "
              f"(plan was {size[0]:.0f}x{size[1]:.0f})", flush=True)
    wreck = list(report.get("paths") or [])
    loose = []

    # --- and the debris the building shed around itself --------------------
    # THE FRAGMENTS ARE ONLY HALF OF IT. Everything above is the building cut
    # into pieces; this is the separate library of debris ASSETS — planks,
    # rebar, sheeting, mounds of fines — that `disaster.debris` scatters around
    # the base. The live scene path calls it through `mesh_damage._shed_debris`
    # inside `apply_to_stage`, but this loop runs `damage_archetype`, which
    # never does, so the loop has been showing bare fragments the whole time.
    #
    # `share` comes from the PILE, not from the rung: `debris.fallen` counts
    # what actually came free, consumed fragments included, because pulverised
    # material is exactly what should be showing up around the base as fines.
    debris_static = []
    if DEBRIS:
        t_d = time.time()
        import scene_generator as _sg
        from compile_disaster import compile_earthquake
        from disaster import debris as D
        # THE RULES ARE ZERO IN A SCENE PRESET. `compile_disaster` starts every
        # debris knob at 0.0 and only a DISASTER preset's severity fills them
        # in; `urban_v2` is a scene, so `shed_m3_per_m` stays 0, `budget_m3`
        # returns (0, 0) and `plan` returns nothing. That is why the first run
        # here reported "0 placements" with share=1.00 — the pools were fine.
        # So the rung's own severity is compiled into a block and borrowed.
        _dis = _sg._stage(config, "disaster") or {}
        if not float((_dis.get("debris") or {}).get("shed_m3_per_m") or 0.0):
            _dis["debris"] = compile_earthquake(
                float(rung.at), {}, (1000.0, 1000.0))["debris"]
        shed = D.shed(
            # `kind` IS THE STRUCTURE, not the disaster — `debris.for_structure`
            # matches it against the pool's material groups (brick, concrete,
            # steel). Passing "earthquake" here quietly matched the wrong ones.
            stage, config, kind=str(report.get("material") or "masonry"),
            rung=LEVEL,
            centre=ctr,
            footprint_m=fp,
            height_m=float(size[2]),
            parent_path="/World/debris",
            rng=random.Random(SEED + 4703))
        # PIECES FALL, MOUNDS DO NOT. `settle.run` gives every path in `loose`
        # a rigid body and a collider, so the pieces need nothing extra here —
        # they are dropped from `lift` and physics seats them on the ground,
        # on a mound, or on the wreck. Only the mounds need the shrunken
        # proxy, so a piece landing on one beds into it.
        loose.extend(shed["paths"])
        piles = list(shed["statics"])
        debris_static = D.pile_colliders(stage, piles, PILE_COLLIDER)
        print(f"[loop] debris {time.time() - t_d:4.1f}s  "
              f"{len(shed['placements'])} placement(s), "
              f"{len(shed['paths'])} piece(s) dropped + {len(piles)} mound(s), "
              f"r={shed['radius_m']:.1f} m  rung_share={D.rung_share(LEVEL):.2f}",
              flush=True)
        print(f"[loop] mound colliders x{PILE_COLLIDER:.2f} on "
              f"{len(debris_static)} (hidden, convex hull)", flush=True)

    # --- settle the debris onto it -----------------------------------------
    t0 = time.time()
    if loose:
        # THE MOUNDS MUST BE COLLIDERS, and so must the wreck: both are already
        # baked and static, and if they are not in this list the scatter drops
        # straight through them.
        info = S.run(stage, loose,
                     wreck + debris_static + ["/World/ground"], steps=STEPS,
                     kick=0.15, bake_result=True, dt=DT,
                     max_travel_m=MAX_TRAVEL)
        print(f"[loop] dt={DT:.4f}s ({1.0/DT:.0f} Hz) "
              f"sim={info.get('sim_seconds', 0):.1f}s", flush=True)
        print(f"[loop] debris settle {time.time() - t0:5.1f}s  "
              f"steps={info.get('steps_used')}/{STEPS} "
              f"still_moving={info.get('still_moving')} "
              f"drop_median={info.get('drop_median', 0):+.2f} m "
              f"spread_max={info.get('spread_max', 0):.1f} m", flush=True)
    else:
        print("[loop] nothing came loose — no settle", flush=True)

    # --- photograph it -----------------------------------------------------
    os.makedirs(SNAP_DIR, exist_ok=True)
    span = max(float(size[0]), float(size[1]), float(size[2]), 8.0)
    snaps = _snaps()
    # Framed on all three when they are present, so the progression reads
    # left-to-right in one shot.
    # Framed on the whole row so the progression reads left to right in one
    # shot: the eye pulls back with the row's width, not the building's.
    lo = min(list(stages.values()) + [0.0])
    cx = lo / 2.0
    row = abs(lo) + max(float(size[0]), float(size[1]))
    # PULL BACK FOR HEIGHT, NOT JUST FOR THE ROW. `row` is a horizontal extent,
    # so on a 231 m tower with a 42x49 m footprint the camera framed the plan
    # and cut the building off at the top of frame. A vertical field needs the
    # height divided by the aspect before it can be compared with a width.
    # MULTIPLIED BY THE ASPECT, NOT DIVIDED BY IT. The vertical field is the
    # NARROW one on a 16:9 frame, so a metre of height needs more pull-back
    # than a metre of width, not less. Dividing framed the plan and cut the
    # 231 m tower off at the top.
    tall = float(size[2]) * (1280.0 / 720.0)
    wide = max(row, span, tall) * 1.15
    snaps.place_camera(stage,
                       (cx, -wide * 1.05, max(wide * 0.42, float(size[2]) * 0.55)),
                       (cx, 0.0, float(size[2]) * 0.45))
    out = os.path.join(SNAP_DIR, f"{item.type}_{LEVEL}.png")
    snaps.snapshot(out, 50)
    print(f"[loop] DONE -> {out}", flush=True)

    while simulation_app.is_running():
        simulation_app.update()
    return 0


def _abs(config, rel):
    root = str(config.get("asset_root") or "")
    return root.rstrip("/") + "/" + str(rel).lstrip("/") if root else str(rel)


if __name__ == "__main__":
    sys.exit(main())
