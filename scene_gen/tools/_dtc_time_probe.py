"""_dtc_time_probe — where does `gac_fire.prepare` spend its time on a
downtowncity block? Times each stage and, inside `bake_atlases`, every
`soot_bake.uv_position_map` call (which is a per-triangle PYTHON loop)."""
import sys, time, collections
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
import random
import numpy as np
from pxr import Usd, UsdGeom
from disaster import soot_bake as sb
from disaster import gac_fire as gf

NAME = sys.argv[1] if len(sys.argv) > 1 else "dtc:Building_12"
LEVEL = sys.argv[2] if len(sys.argv) > 2 else "F3"

STATS = collections.Counter()
CALLS = collections.Counter()
_orig = sb.uv_position_map
def timed(*a, **kw):
    px = kw.get("px")
    t = time.time()
    out = _orig(*a, **kw)
    dt = time.time() - t
    STATS["px%s" % px] += dt
    CALLS["px%s" % px] += 1
    STATS["_total"] += dt
    return out
sb.uv_position_map = timed
gf.__dict__.setdefault("_", None)

st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/W")
st.SetDefaultPrim(st.GetPrimAtPath("/W"))
UsdGeom.Xform.Define(st, "/W/g0")
t0 = time.time()
pre = gf.prepare(st, "/W/g0", NAME, LEVEL, random.Random(7), "d0",
                 out_dir="/isaac-sim/.cache/dtc_timeprobe", verbose=True)
tot = time.time() - t0
print("\nprepare TOTAL %.1f s" % tot)
print("uv_position_map: %.1f s (%.0f%% of prepare) across %d call(s)"
      % (STATS["_total"], 100.0 * STATS["_total"] / max(tot, 1e-9),
         sum(CALLS.values())))
for k in sorted(CALLS):
    print("   %-10s %5d call(s)  %8.1f s" % (k, CALLS[k], STATS[k]))
print("atlases sooted: %d, tiled: %d"
      % (len(set(id(v) for k, v in pre["sooted"].items() if k != "_png")),
         len(pre["sooted"].get("_tiled") or ())))
