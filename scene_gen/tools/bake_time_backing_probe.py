#!/usr/bin/env python
"""bake_time_backing_probe -- reproduce the STATE `disaster.fire_bake.
_judge_candidates` actually sees at BAKE TIME (right after `gac_fire.
burn_gac` builds the building, `<cell>/src` is composed and marked
INVISIBLE but NOT YET REMOVED) and run the judge on it directly, printing
`backing_path` for every `spall`/`spallhalo`/`crack` candidate that comes
back `contact=True` via the new backing test.

WHY. `fire_bake_launch_script.py`'s own log for a real bake
(`gac_SM_Building_26_F4_s162.log`, 2026-08-31) shows the LIVE judge (running
inside Kit, post-settle, pre-export) deactivating 31 `spall` + 27
`spallhalo` — the backing test IS running and IS working for most of them —
but 14 more of the same families are STILL ACTIVE in the exported file, and
a COLD re-open of that exact file (`tools/airborne_probe.py`) correctly
judges all 14 as unbacked. The one thing removed between the live judge
(line ~494 of the launch script) and the export is `<cell>/src` (dropped a
few lines later, once materials are rehomed) -- composed and INVISIBLE the
whole time up to that point. If ANY of `_judge_candidates`' machinery lets
an invisible `<cell>/src` mesh count as backing, this is exactly the
mismatch: the live judge sees it (wrongly keeps the stamp), the cold re-open
of the exported file never can (it is gone), and correctly flags it.

This script builds the SAME bake up to (and including) that exact window
-- `gac_fire.burn_gac`, no Kit, no physics, `<cell>/src` composed and
invisible, nothing removed yet -- and calls `fire_bake._judge_candidates`
on it directly. No physics settle is needed to test THIS mechanism: spall/
spallhalo/crack stamps are authored directly by `_face_polygon`/`_scar` at
a fixed world position and are never given to `settle.run` as loose bodies,
and `<cell>/src`'s own position and visibility are fixed the moment
`burn_gac` composes it -- both are exactly what they will still be at the
real judge's call site, physics or no physics.

    docker exec isaac-sim bash -c \
        "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
         /isaac-sim/AirStack/scene_gen/tools/bake_time_backing_probe.py \
         SM_Building_26 F4 162 g5"

No Kit, no physics, no GPU -- safe beside a live session. Never writes
anything (in-memory stage only).
"""
import random
import sys
import time

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")

from pxr import Sdf, Usd, UsdGeom                              # noqa: E402
from disaster import fire_bake as fb, fracture                 # noqa: E402
from disaster import gac_fire as gf, urban_fire as uf           # noqa: E402


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "SM_Building_26"
    level = sys.argv[2] if len(sys.argv) > 2 else "F4"
    seed = int(sys.argv[3]) if len(sys.argv) > 3 else 162
    tag = sys.argv[4] if len(sys.argv) > 4 else "g5"

    fracture.ensure_vtk(verbose=False)
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/World")
    st.SetDefaultPrim(st.GetPrimAtPath("/World"))
    UsdGeom.Xform.Define(st, Sdf.Path(fb.BAKE_ROOT))
    cell = "{0}/{1}".format(fb.BAKE_ROOT, tag)
    UsdGeom.Xform.Define(st, Sdf.Path(cell))
    mats = uf.materials(st, fb.BAKE_ROOT)
    rng = random.Random(seed)
    nrng = np.random.default_rng(seed)

    t0 = time.time()
    bctx = gf.burn_gac(st, cell, name, level, rng, nrng, mats, tag,
                       flow_root=None, mat_cache={}, ssf=1.0, origin=None,
                       sides=None, use_baked_kit=True, verbose=True)
    print("[probe] built in {0:.0f}s: {1} loose, {2} static, {3} authored"
          .format(time.time() - t0, len(bctx["loose"]),
                  len(bctx["static_extra"]), len(bctx["authored"])))

    src = st.GetPrimAtPath(Sdf.Path(cell + "/src"))
    print("[probe] <cell>/src: valid={0} active={1} visibility={2}".format(
        src.IsValid() if src else False,
        src.IsActive() if src and src.IsValid() else None,
        UsdGeom.Imageable(src).ComputeVisibility() if src and src.IsValid()
        else None))

    # THE JUDGE, exactly as the launch script calls it (no physics has run,
    # but see the module docstring for why that does not matter here).
    info = fb._judge_candidates(st, fb.BAKE_ROOT, gap_m=1.0, verbose=False)
    judged = info["judged"]
    decal = [j for j in judged if j["prefix"] in fb._WALL_DECAL_FAMILIES]
    kept = [j for j in decal if not j["deactivate"]]
    flagged = [j for j in decal if j["deactivate"]]
    print("[probe] {0} candidate(s) judged, {1} decal-family, {2} kept, "
          "{3} would-deactivate (pre-settle, pre-export)".format(
              len(judged), len(decal), len(kept), len(flagged)))

    src_str = str(src.GetPath()) if src and src.IsValid() else None
    n_src_backed = 0
    print("\n  KEPT decal-family candidates and what backs them:")
    for j in sorted(kept, key=lambda j: (j["backing_m"] is None,
                                         j["backing_m"] or 0.0)):
        bp = j["backing_path"] or ""
        under_src = bool(src_str and (bp == src_str or
                                      bp.startswith(src_str + "/")))
        if under_src:
            n_src_backed += 1
        flag = "  <-- UNDER <cell>/src (INVISIBLE at judge time)" if under_src else ""
        print("    {0:<40} backing_m={1!s:<8} backing_path={2}{3}".format(
            j["path"].rsplit("/", 1)[-1], j["backing_m"], bp, flag))

    print("\n[probe] {0}/{1} KEPT decal-family candidate(s) are backed by "
          "something living under the invisible <cell>/src subtree"
          .format(n_src_backed, len(kept)))
    if n_src_backed:
        print("[probe] *** MECHANISM CONFIRMED: the backing test's locator "
              "is NOT excluding <cell>/src's invisible geometry ***")
    else:
        print("[probe] no src-backed decal found in THIS pre-settle "
              "reproduction -- either src invisibility is respected here, "
              "or the real discrepancy needs the settle step too "
              "(loose collapse debris moving OUT of backing range, or "
              "vice versa) to show up.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
