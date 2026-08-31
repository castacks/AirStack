#!/usr/bin/env python
"""soot_harden_probe — verify `disaster.soot_bake.harden_baked_materials`
end to end, on a REAL bake, with NEITHER production call site touched.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/soot_harden_probe.py dtc:Amar_Tower F5c /tmp/soot_harden_probe"

WHY THIS PROBE EXISTS, AND WHY IT IS NOT JUST `gac_burn_probe.py`. The fix
for "the sooted copy on a curtain-wall tower stays mirror-reflective"
(`disaster/soot_bake.harden_sooted_shader`, fire_dtc3 bench review,
2026-08-30: dtc Amar_Tower) lives entirely in `disaster/soot_bake.py` — the
one file this change was scoped to. The two places that actually BIND a
sooted copy (`urban_fire._bind_soot`, `gac_fire.bake_atlases`, both via
`soot_plume.piece_material_like`) are out of scope, so neither has been
wired to call the new hardening. `soot_bake.harden_baked_materials` is the
zero-call-site-change integration point instead: it sweeps a stage for
`sootbake_<digest>.png`-named copies and mattes the ones `bake_module`
already logged (as a side effect, keyed by that same digest) as
significantly sooted. This probe demonstrates that end to end on a REAL
building, in the SAME process the burn ran in (the coverage registry is
in-memory only):

  1. run `gac_fire.burn_gac` exactly as `gac_burn_probe.py` does (same
     fixed seeds) for `name`/`level`;
  2. export the untouched stage ("baseline.usd");
  3. call `soot_bake.harden_baked_materials` on the SAME stage;
  4. export again ("hardened.usd").

Then run `tools/piece_soot_probe.py <baseline|hardened>.usd <regex>` on
both and compare — the RECOMMENDED permanent integration (a one-line swap
of `soot_plume.piece_material_like(...)` for `soot_bake.
make_hardened_material(..., coverage=...)` at each of the two call sites,
so this sweep is not needed at all) is documented on `make_hardened_
material` itself.
"""
import os
import random
import sys
import time
import traceback

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom                                  # noqa: E402
from disaster import fracture, gac_fire as gf, soot_bake as sb, urban_fire as uf  # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "dtc:Amar_Tower"
level = sys.argv[2] if len(sys.argv) > 2 else "F5c"
out_dir = sys.argv[3] if len(sys.argv) > 3 else "/tmp/soot_harden_probe"
os.makedirs(out_dir, exist_ok=True)

t0 = time.time()
fracture.ensure_vtk(verbose=False)
st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/W")
st.SetDefaultPrim(st.GetPrimAtPath("/W"))
UsdGeom.Scope.Define(st, "/W/bench")
cell = "/W/bench/g0"
UsdGeom.Xform.Define(st, cell)
mats = uf.materials(st, "/W/bench")
rng = random.Random(7)
nrng = np.random.default_rng(7)
try:
    ctx = gf.burn_gac(st, cell, name, level, rng, nrng, mats, "g0",
                      flow_root=None, mat_cache={}, ssf=1.0, origin=None,
                      verbose=True)
except Exception:
    traceback.print_exc()
    sys.exit(1)

print("[soot_harden_probe] burn done in {0:.0f}s -- bake_module recorded "
      "{1} coverage digest(s) as a side effect".format(
          time.time() - t0, len(sb._COVERAGE_BY_DIGEST)))

baseline_path = os.path.join(out_dir, "baseline.usd")
st.GetRootLayer().Export(baseline_path)
print("[soot_harden_probe] wrote baseline (pre-harden)  -> " + baseline_path)

n_changed = sb.harden_baked_materials(st, verbose=True)
print("[soot_harden_probe] harden_baked_materials changed {0} "
      "material(s) (threshold coverage >= {1})".format(
          n_changed, sb.SOOT_HARDEN_MIN))

hardened_path = os.path.join(out_dir, "hardened.usd")
st.GetRootLayer().Export(hardened_path)
print("[soot_harden_probe] wrote hardened (post-harden) -> " + hardened_path)
