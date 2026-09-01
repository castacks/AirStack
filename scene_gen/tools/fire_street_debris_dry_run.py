#!/usr/bin/env python3
"""fire_street_debris_dry_run.py — host-side, no Kit, no GPU: runs the REAL
fire-side debris apron (`disaster.fire_assembly_lib.fire_apron_pass`) against
the REAL 39-record manifest and the REAL sidecars in `city_138`, and reports
a per-building piece count plus a total prim/geometry-cost estimate — the
verification the "debris needs to increase" round asked for, run BEFORE
paying for a relaunch to see it.

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        python3 tools/fire_street_debris_dry_run.py
    tools/fire_street_debris_dry_run.py --scale 1.5
    tools/fire_street_debris_dry_run.py \\
        --manifest _plans/fire_city_500m_39.json \\
        --bakes ~/docker/isaac-sim/cache/main/fire_bakes/city_138

WHY IT DOES NOT IMPORT THE LAUNCHER. `urban_fire_city_launch_script.py`
builds a `SimulationApp` at module scope — a second Kit app in one process is
a segfault (that launcher's own header). This tool needs none of its Kit-side
machinery (cell resolution, placement, Flow) — only the manifest -> stem ->
sidecar match, which `tools/fire_city_manifest.py` (`fcm`) already does
identically to the launcher's own `bake_paths` (verified against it in
`fire_flow_dry_run.py`'s own `load_rows`, the same idiom reused here) — and
`disaster.fire_assembly_lib.fire_apron_pass` itself, which IS safely
importable (only needs `pxr`, via `usd-core` — no `SimulationApp`).

THE ONE APPROXIMATION. `build_fire_apron` wants `r["bbox"]` only for its
`z0 = box[2]` (ground contact height) — in a real launch that is `fal.bbox()`
measured on the COMPOSED, PLACED holder. Here it is the bake's own sidecar
`doc["bbox"]` (bake-LOCAL, pre-placement) instead: a city yaw is a rotation
about Z (never moves Z) and every building's own city `z` in this manifest is
0.0, so the local bbox's own z_min equals the world one — this is the same
reasoning `fire_assembly_lib.build_fire_apron`'s own docstring gives for why
Z is the caller's concern, not `world_masses`'s. `x`/`y`/`yaw` come straight
from the manifest record (`fal.fire_apron_pass` never reads geometry off the
stage — every seat is analytic, off `masses` + `x`/`y`/`yaw`), so the seats
this tool reports are pixel-for-pixel what a real launch will author.

Exit 0 clean, 1 if fewer than 39 records get a bake match (a manifest/bake
mismatch this tool exists to catch before a relaunch, not after).
"""
import argparse
import glob
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
_TOOLS = os.path.join(_SCENE_GEN, "tools")
for _p in (_SCENE_GEN, _TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from disaster import fire_assembly_lib as fal              # noqa: E402
from disaster import fire_bake as fb                        # noqa: E402
import fire_city_manifest as fcm                            # noqa: E402
from pxr import Usd, UsdGeom                                 # noqa: E402

DEFAULT_MANIFEST = os.path.join(_SCENE_GEN, "_plans", "fire_city_500m_39.json")
DEFAULT_BAKES = os.path.expanduser(
    "~/docker/isaac-sim/cache/main/fire_bakes/city_138")


def load_placed_rows(manifest_path, bakes_dir):
    """`(seed, placed_rows, missing)` — `placed_rows` in exactly
    `FireCityApp.placed`'s own shape (`i`/`stem`/`x`/`y`/`yaw`/`bbox`/`doc`/
    `masses`), built from the REAL manifest + REAL sidecars, no stage.
    Mirrors `fire_flow_dry_run.load_rows`'s stem match (`fcm.
    build_entry_and_stem`, identical to the launcher's own `bake_paths`).
    """
    top_seed, records = fcm.load_manifest(manifest_path)
    seed = fcm.resolve_city_seed(manifest_path, top_seed)
    by_stem = {}
    for jf in sorted(glob.glob(os.path.join(bakes_dir, "*.json"))):
        by_stem[os.path.splitext(os.path.basename(jf))[0]] = jf

    placed, missing = [], []
    for i, rec in enumerate(records):
        try:
            _entry, stem = fcm.build_entry_and_stem(rec, i)
        except Exception as exc:
            missing.append((i, None, "not a valid bake entry: {0}".format(exc)))
            continue
        jf = by_stem.get(stem)
        if not jf:
            missing.append((i, stem, "no bake on disk for this stem"))
            continue
        doc, masses, _events = fb.load_for_assembly(jf)
        placed.append({
            "i": i, "stem": stem,
            "x": float(rec.get("x", 0.0)), "y": float(rec.get("y", 0.0)),
            "yaw": float(rec.get("yaw_deg", 0.0)),
            "bbox": doc.get("bbox"), "doc": doc, "masses": masses,
        })
    return seed, placed, missing


def approx_mesh_bytes(n_points, n_faces):
    """Documented, rough byte estimate for one authored Mesh's own
    points + quad topology — 12 B/point (float3), 4 B/face-vertex-count,
    4 B/face-vertex-index x4 (quads). Not a Hydra/GPU number — the true VRAM
    figure needs a real render (`fal.vram_mb`, the launcher's own
    `nvidia-smi` reading, gated by `FC_FIRE_APRON`)."""
    return 12 * n_points + 4 * n_faces + 4 * n_faces * 4


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--manifest", default=DEFAULT_MANIFEST)
    ap.add_argument("--bakes", default=DEFAULT_BAKES)
    ap.add_argument("--seed", type=int, default=7,
                    help="FA_SEED — the apron's own per-building rng seed")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="FA_APRON_SCALE — density multiplier on top of "
                         "fal.APRON_DENSITY")
    args = ap.parse_args()

    seed, placed, missing = load_placed_rows(args.manifest, args.bakes)
    print("[fsd] manifest {0}: {1} record(s) matched a bake in {2} (city "
          "seed {3})".format(args.manifest, len(placed), args.bakes, seed))
    if missing:
        print("[fsd] {0} record(s) with NO bake match (excluded below):"
              .format(len(missing)))
        for i, stem, why in missing:
            print("[fsd]    record {0:<3} stem={1!r:<40} {2}".format(
                i, stem, why))

    stage = Usd.Stage.CreateInMemory()
    # `/World/fire` already exists by the time a REAL launch calls
    # `fire_debris_apron()` (many earlier fire passes author under it) — pre-
    # authoring the same two ancestors here means the prim delta measured
    # below is the apron pass's OWN incremental cost, not an artefact of
    # this tool building a bare stage from nothing.
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/fire")
    n_before = sum(1 for _ in stage.Traverse())
    rows = fal.fire_apron_pass(stage, "/World/fire", placed,
                               seed=args.seed, scale=args.scale)

    print()
    print("--- PER-BUILDING (FA_APRON_SCALE={0}) "
          "{1}".format(args.scale, "-" * 40))
    print("  {0:<4} {1:<34} {2:<5} {3:<8} {4}".format(
        "#", "stem", "lvl", "lumps", "sides"))
    total_lumps, total_bytes, n_meshes = 0, 0, 0
    level_tally = {}
    for r in sorted(rows, key=lambda r: r["i"]):
        lvl = r.get("level") or "?"
        level_tally.setdefault(lvl, [0, 0])
        level_tally[lvl][0] += 1
        if not r.get("prim"):
            print("  {0:<4} {1:<34} {2:<5} {3:<8} {4}".format(
                r["i"], r["stem"], lvl, "-", r.get("note", "")))
            continue
        level_tally[lvl][1] += r["n"]
        n_meshes += 1
        total_lumps += r["n"]
        prim = stage.GetPrimAtPath(r["prim"])
        me = UsdGeom.Mesh(prim)
        n_pts = len(me.GetPointsAttr().Get() or [])
        n_faces = len(me.GetFaceVertexCountsAttr().Get() or [])
        total_bytes += approx_mesh_bytes(n_pts, n_faces)
        print("  {0:<4} {1:<34} {2:<5} {3:<8} {4}".format(
            r["i"], r["stem"], lvl, r["n"], "/".join(r["sides"])))

    n_gated_out = len(rows) - n_meshes
    print()
    print("--- BY LEVEL ---------------------------------------------------")
    for lvl in fal.APRON_LEVELS + ("F5c", "F6", "F0"):
        if lvl not in level_tally:
            continue
        n_b, n_l = level_tally[lvl]
        print("  {0:<5} {1:>3} building(s)  {2:>5} lump(s)  "
              "(avg {3:.1f}/building)".format(
                  lvl, n_b, n_l, (n_l / n_b) if n_b else 0.0))

    # Prim cost: 1 Mesh + <=2 materialBind GeomSubsets per building that got
    # an apron, plus a FIXED one-time cost of 6 (the <root>/apron and
    # <root>/DebrisLooks scope ancestors + 2 shared debris materials, each a
    # Material + Shader pair) for the whole city — see `fire_city_dressing_
    # probe.py`'s own accounting, which this mirrors.
    prim_bound = 3 * n_meshes + 6
    n_after = sum(1 for _ in stage.Traverse())

    print()
    print("--- TOTALS -------------------------------------------------------")
    print("  {0} of {1} manifest record(s) got an apron mesh ({2} gated out: "
          "F5c/F6/F0 or no venting sides recorded, {3} had no bake match)"
          .format(n_meshes, len(rows), n_gated_out, len(missing)))
    print("  {0} lump(s) total, {1:.1f} avg lumps/apron building".format(
          total_lumps, (total_lumps / n_meshes) if n_meshes else 0.0))
    print("  new prims this pass authors: {0} (bound: 3/building + 6 fixed "
          "= {1})".format(n_after - n_before, prim_bound))
    print("  raw point+topology data: ~{0:.0f} KiB across {1} mesh(es) "
          "(~{2:.1f} KiB/mesh avg) — NOT a Hydra/GPU number, see this "
          "tool's own docstring".format(
              total_bytes / 1024.0, n_meshes,
              (total_bytes / 1024.0 / n_meshes) if n_meshes else 0.0))

    problems = []
    if len(missing) > 0:
        problems.append("{0} manifest record(s) have no bake on disk in "
                        "{1}".format(len(missing), args.bakes))
    if (n_after - n_before) > prim_bound:
        problems.append("prim count {0} exceeds the documented bound {1}"
                        .format(n_after - n_before, prim_bound))

    print()
    if problems:
        print("FIRE STREET DEBRIS DRY RUN *** PROBLEM ***")
        for p in problems:
            print("  - " + p)
        return 1
    print("FIRE STREET DEBRIS DRY RUN OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
