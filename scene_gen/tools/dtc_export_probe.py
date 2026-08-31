#!/usr/bin/env python
"""dtc_export_probe — the FULL bake chain on a merged asset, minus Kit.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/dtc_export_probe.py dtc:Building_12 F3"

Bare `pxr` only — no `SimulationApp`, no GPU, no physics. It is
`fire_bake_launch_script.main()` with the settle and the Flow-less Kit
plumbing removed and NOTHING ELSE changed: same `/World` + `/World/bake/<tag>`
hierarchy, same `gac_fire.burn_gac`, same `fire_bake.rehome_for_export` ->
drop `<cell>/src` -> `strip_physics` -> stray-root sweep -> ROOT-LAYER export
-> `fire_bake.verify_export` sequence, same sidecar.

WHY IT EXISTS
-------------
The export is where a new PACK breaks, and it breaks silently. A sliced piece
binds a Material prim that lives inside `<cell>/src`; drop the source and the
piece renders WHITE with its geometry and its UVs perfectly intact
(`gac_slice.rehome_materials`, `fire_bake.rehome_for_export`). GAC survives
that because every one of its materials is a separate `Materials/*_Inst.usd`
the rehome can reference. **A downtowncity block has no such file** — its
33-88 materials are authored INLINE in the same `.usdc` as the mesh — so
`gac_slice._material_source` had to learn to fall back to the asset's own
layer before `rehome_for_export` could re-anchor anything at all. Without
that, every material comes back FAILED, the launcher keeps `<cell>/src`
(`src_kept: true`) and every bake ships the whole merged tower invisibly
inside it.

This probe is how that is checked without burning a GPU hour: it exports for
real and then reopens the file COLD and verifies it — every material's base
map resolves, nothing still points into the dropped subtree, no physics API
survived, no Flow prim shipped.

Env: `DEP_OUT` (default /isaac-sim/.cache/fire_bakes_test), `DEP_SEED` (7),
`DEP_KEEP` (1 = keep the exported .usd, the default).
"""
import os
import random
import resource
import sys
import time
import traceback

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Sdf, Usd, UsdGeom                              # noqa: E402
from disaster import fire_bake as fb                           # noqa: E402
from disaster import fracture                                  # noqa: E402
from disaster import gac_fire as gcf                           # noqa: E402
from disaster import soot_plume as spl                         # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402

NAME = sys.argv[1] if len(sys.argv) > 1 else "dtc:Building_12"
LEVEL = sys.argv[2] if len(sys.argv) > 2 else "F3"
OUT_DIR = (os.environ.get("DEP_OUT") or "").strip() or "/isaac-sim/.cache/fire_bakes_test"
SEED = int((os.environ.get("DEP_SEED") or "7").strip() or 7)
TEX_DIR = os.path.join(OUT_DIR, "textures")

KIND, ASSET = gcf.split_kind(NAME)
ENTRY = {"kind": KIND, "name": ASSET, "level": LEVEL, "seed": SEED,
         "origin": None, "sides": None, "index": 0}
TAG = fb.bake_tag(ENTRY)
CELL = "{0}/{1}".format(fb.BAKE_ROOT, TAG)


def main():
    t0 = time.time()
    os.makedirs(TEX_DIR, exist_ok=True)
    # THE SOOT PNGS TRAVEL WITH THE BAKE — the same module global the real
    # launcher re-points, read at call time by every writer.
    spl.OUT_DIR = TEX_DIR

    fracture.ensure_vtk(verbose=False)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path(fb.DEFAULT_PRIM))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path(fb.BAKE_ROOT))
    UsdGeom.Xform.Define(stage, Sdf.Path(CELL))

    mats = uf.materials(stage, fb.BAKE_ROOT)
    t_b = time.time()
    bctx = gcf.burn_gac(stage, CELL, NAME, LEVEL, random.Random(SEED),
                        np.random.default_rng(SEED), mats, TAG,
                        flow_root=None, mat_cache={}, ssf=1.0, verbose=True)
    build_s = time.time() - t_b
    for n in bctx["notes"]:
        print("[dep]     " + n)
    doomed = [CELL + "/src"]

    # -- the export, step for step as `fire_bake_launch_script.main` does it -
    t_e = time.time()
    looks = fb.BAKE_ROOT + "/Looks"
    rh = fb.rehome_for_export(stage, fb.BAKE_ROOT, doomed, looks, verbose=True)
    src_kept = bool(rh["failed"])
    if not src_kept:
        for d in doomed:
            if stage.GetPrimAtPath(Sdf.Path(d)).IsValid():
                stage.RemovePrim(Sdf.Path(d))
        print("[dep] dropped the merged source subtree ({0})".format(
            ", ".join(doomed)))
    else:
        print("[dep] *** KEEPING {0} *** — {1} material(s) could not be "
              "rehomed: {2}".format(", ".join(doomed), rh["failed"],
                                    ", ".join(rh["failed_paths"][:6])))

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                           useExtentsHint=False)
    r = bc.ComputeWorldBound(stage.GetPrimAtPath(Sdf.Path(CELL))).ComputeAlignedRange()
    bbox = None if r.IsEmpty() else [float(v) for v in
                                     (r.GetMin()[0], r.GetMin()[1], r.GetMin()[2],
                                      r.GetMax()[0], r.GetMax()[1], r.GetMax()[2])]
    top_z = bbox[5] if bbox else None

    fb.strip_physics(stage, root=None, remove_prims=fb.STRIP_PRIMS, verbose=True)
    for p in list(stage.GetPseudoRoot().GetChildren()):
        if p.GetName() != "World":
            print("[dep] removing stray root prim {0}".format(p.GetPath()))
            stage.RemovePrim(p.GetPath())
    stage.SetDefaultPrim(stage.GetPrimAtPath(Sdf.Path(fb.DEFAULT_PRIM)))

    os.makedirs(OUT_DIR, exist_ok=True)
    out_usd, out_json = fb.out_paths(ENTRY, OUT_DIR)
    # ROOT LAYER ONLY. Never `stage.Export()` / `stage.Flatten()`.
    stage.GetRootLayer().Export(out_usd)
    export_s = time.time() - t_e
    mb = os.path.getsize(out_usd) / 1e6
    print("[dep] exported {0:.1f} MB in {1:.0f} s -> {2}".format(
        mb, export_s, out_usd))

    events = list(bctx["fire"].get("events") or [])
    masses = {}
    for ev in events:
        for op in (ev.get("ops") or []):
            tagm = op.get("mass") or "main"
            if tagm not in masses and op.get("m") is not None:
                masses[tagm] = op["m"]
    for k, v in (bctx["info"]["masses"] or {}).items():
        masses.setdefault(k, v)
    gac = bctx.get("gac") or {}
    doc = fb.sidecar(
        ENTRY, bctx["fire"], masses, events, bbox, top_z,
        {"interior": [], "roof": []}, bctx["notes"],
        {"build_s": round(build_s, 1), "export_s": round(export_s, 1),
         "total_s": round(time.time() - t0, 1)},
        {"loose": len(bctx["loose"]), "static": len(bctx["static_extra"]),
         "authored": len(bctx["authored"]),
         "pieces": int(gac.get("n_pieces") or 0),
         "atlases": int(gac.get("n_atlases") or 0), "usd_mb": round(mb, 2)},
        settle_info={"note": "no settle — dtc_export_probe is bare USD"},
        usd=out_usd, textures_dir=TEX_DIR, src_kept=src_kept,
        extra={"rehome": {k: v for k, v in rh.items() if k != "failed_paths"}})
    fb.write_sidecar(out_json, doc)
    print("[dep] sidecar -> {0}".format(out_json))

    info = fb.verify_export(out_usd, doomed=("/src",), expect_root=fb.BAKE_ROOT,
                            check_remote=False)
    print("\n" + "=" * 74)
    print("DTC EXPORT PROBE  {0} {1}".format(NAME, LEVEL))
    print("  pieces     {0},  atlases {1},  glass {2}".format(
        gac.get("n_pieces"), gac.get("n_atlases"), gac.get("n_glass")))
    print("  rehome     needed {0}, rehomed {1}, failed {2}, rebound {3}, "
          "pruned {4}  ->  src_kept={5}".format(
              rh["needed"], rh["rehomed"], rh["failed"], rh["rebound"],
              rh.get("pruned"), src_kept))
    print("  export     {0:.1f} MB, {1} prim(s), {2} mesh(es), {3} material(s)"
          .format(mb, info.get("prims"), info.get("meshes"),
                  info.get("materials")))
    print("  verify     {0}   (textures checked {1}, missing {2}, remote {3}; "
          "doomed prims {4}, arcs {5}; physics {6}; flow {7})".format(
              "OK" if info.get("ok") else "*** PROBLEM ***",
              info.get("textures_checked"), info.get("n_textures_missing"),
              info.get("n_textures_remote"), info.get("n_doomed_prims"),
              info.get("n_doomed_arcs"), info.get("n_physics_prims"),
              info.get("n_flow_prims")))
    print("  peak rss   {0:.0f} MB,  total {1:.0f} s".format(
        resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0,
        time.time() - t0))
    print("=" * 74)
    print("DTC EXPORT PROBE " + ("DONE" if info.get("ok") else "PROBLEM"))
    return 0 if info.get("ok") else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        traceback.print_exc()
        print("DTC EXPORT PROBE FAILED: {0}".format(exc))
        sys.exit(2)
