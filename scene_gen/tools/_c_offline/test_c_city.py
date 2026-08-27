"""Offline check of the CITY half: `_c_plate_bounds`, `_c_mass`, `_tilt_prim`
+ its ground response on a mock stage, and the plate clamp."""
import os, random, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, "/home/krrishjain/SEI-COA/disaster-dataset/scene_gen")
import mockpxr; mockpxr.install()
from disaster import damage, quake_flow as qf
damage._pbr = lambda *a, **k: object()
from disaster import quake as qk

fails = []
def chk(n, c, extra=""):
    print(("  ok   " if c else "  FAIL ") + n + ("  " + extra if extra else ""))
    if not c: fails.append(n)

cfg = {"layout": {"region_m": [250, 250]}}
b = qk._c_plate_bounds(cfg)
chk("plate bounds inset by 2 m: {0}".format(tuple(round(q, 1) for q in b)),
    b == (-123.0, -123.0, 123.0, 123.0))
b2 = qk._c_plate_bounds(cfg, ssf=1.0, margin=6.0)
chk("margin knob", b2 == (-119.0, -119.0, 119.0, 119.0))

rec = {"W": 24.0, "D": 16.0, "H": 21.0}
p = {"x_m": 118.0, "y_m": -40.0, "z_m": 0.0, "yaw_deg": 33.0, "prim_path": "/World/g/b7"}
m = qk._c_mass(p, rec)
chk("mass frame from a placement", m["W"] == 24.0 and m["cx"] == 118.0 and m["yaw"] == 33.0)

st = mockpxr.Stage()
st.define("/World/g/b7")
prim = st.GetPrimAtPath("/World/g/b7")
qf.materials = lambda *a, **k: {k2: object() for k2 in (
    "soil", "concrete", "dark_concrete", "brick", "crack", "rebar", "plaster",
    "mortar", "timber", "glass")}
g = qk._tilt_prim(st, prim, p, rec, 7.0, 0.9, random.Random(4), 1.0, bounds=b)
chk("tilt_prim leans toward {0}, drop {1:.2f}, rise {2:.2f}".format(
    g["low"], g["drop"], g["rise"]), g["rise"] >= qf.C_MIN_RISE_M - 1e-9)
chk("it authored ground ({0} prims)".format(len(st.prims) - 1), len(st.prims) > 60)
chk("under <scope>/quake_tilt",
    any(q.startswith("/World/g/quake_tilt/") for q in st.prims))
bad = [q.path for q in st.prims.values() for kind, v in q.ops
       if kind == "translate" and not (-123.01 <= v[0] <= 123.01 and -123.01 <= v[1] <= 123.01)]
chk("nothing off the plate", not bad, str(bad[:2]))
chk("the building prim itself moved", len(prim.ops) == 3)

print("\n{0}".format("ALL OK" if not fails else "FAILED: " + ", ".join(fails)))
sys.exit(1 if fails else 0)
