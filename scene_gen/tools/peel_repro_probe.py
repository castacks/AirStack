#!/usr/bin/env python
"""peel_repro_probe — rebuild ONE kit building's fire offline and measure its
`render_peel` stamps, the same two ways `tools/peel_backing_probe.py`
measures a shipped bake.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/peel_repro_probe.py \\
        brownstone_row F4 E,N --seed 438"

Defaults reproduce the building the user reviewed on the live 500 m fire
city (2026-08-31): `kit_brownstone_row_F4_o4_EN_s438`, whose
`bake/k14/peel_k14_166` was "not on the building, it's floating" and wearing
a material that "doesn't match the building it's on".

The build chain is `tools/kit_burn_probe.py`'s, call for call (that file is
the gate on this path, so the repro must not diverge from it): build the
placed kit -> `apply_placements` -> `apply_glass_tint` -> `burn_building`.
Nothing is written to disk. Unlike the shipped bake, the kit modules
COMPOSE here, so the backing measurement is real: the wall the peel is
supposed to be on is on the stage.
"""
import argparse
import os
import sys
import time
import traceback

sys.path.insert(0, os.environ.get("SCENE_GEN", "/isaac-sim/AirStack/scene_gen"))

DEFAULT = ("brownstone_row", "F4", "E,N", 438)


def _sides(spec):
    if not spec:
        return None
    out = tuple(q.strip().upper()[:1]
                for q in spec.replace("/", ",").split(",") if q.strip())
    return out or None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("style", nargs="?", default=DEFAULT[0])
    ap.add_argument("level", nargs="?", default=DEFAULT[1])
    ap.add_argument("sides", nargs="?", default=DEFAULT[2])
    ap.add_argument("--seed", type=int, default=DEFAULT[3])
    ap.add_argument("--reach", type=float, default=0.35)
    ap.add_argument("--out", default="", help="also export the stage here")
    a = ap.parse_args(argv)

    import random
    import numpy as np
    from pxr import Usd, UsdGeom

    import scene_generator as sg
    from detail import urban_building as ub
    from disaster import fracture, quake_flow as qf, urban_fire as uf
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import peel_backing_probe as pbp

    fracture.ensure_deps(verbose=False)
    fracture.ensure_vtk(verbose=False)

    t0 = time.time()
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/W"))
    UsdGeom.Scope.Define(stage, "/W/bench")
    cell = "/W/bench/k0"
    UsdGeom.Xform.Define(stage, cell)
    mats = uf.materials(stage, "/W/bench")
    rng = random.Random(a.seed)
    nrng = np.random.default_rng(a.seed)

    pls = ub.build_building(a.style, 0.0, 0.0, 0.0, random.Random(a.seed + 7))
    sg.apply_placements(stage, pls, cell + "/parts", 1.0)
    ub.apply_glass_tint(stage, pls)
    specs = qf._mass_specs(a.style, 0.0, 0.0, 0.0)
    main_spec = max(specs, key=lambda m: len(m["levels"]))
    n_st = max(1, len(main_spec["levels"]))
    origin = max(0, min(n_st - 1, int(round(0.25 * (n_st - 1)))))
    ctx = uf.burn_building(stage, cell, a.style, pls, 0.0, 0.0, 0.0, a.level,
                           rng, nrng, mats, "k0", flow_root=None,
                           origin=origin, sides=_sides(a.sides),
                           mat_cache={})
    for n in ctx["notes"]:
        if "peel" in n or "smoke" in n:
            print("   note:", n[:260])
    print("[peel_repro] {0} {1} sides={2} seed={3}: {4:.0f}s".format(
        a.style, a.level, ",".join(ctx["fire"].get("sides") or ()), a.seed,
        time.time() - t0))

    if a.out:
        stage.Export(a.out)
        print("[peel_repro] exported", a.out)

    r = pbp.probe_stage(stage, reach=a.reach, label="{0}_{1}".format(
        a.style, a.level), root=cell)
    return 0 if r["n"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(1)
