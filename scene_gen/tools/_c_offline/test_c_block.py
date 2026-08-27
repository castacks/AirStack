"""Offline check of the agent-C ground-response block: sign of the tilt, the
measured low/high sides, and a smoke run of every authoring helper."""
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, "/home/krrishjain/SEI-COA/disaster-dataset/scene_gen")

import mockpxr                                                  # noqa: E402
mockpxr.install()

from disaster import quake_flow as qf                           # noqa: E402

BLOCK = os.environ.get("C_BLOCK", os.path.join(HERE, "c_block.py"))
if os.path.exists(BLOCK) and not hasattr(qf, "_c_ground_response"):
    exec(compile(open(BLOCK).read(), BLOCK, "exec"), qf.__dict__)

fails = []


def chk(name, cond, extra=""):
    print(("  ok   " if cond else "  FAIL ") + name + ("  " + extra if extra else ""))
    if not cond:
        fails.append(name)


def mass(W=27.0, D=17.0, H=33.0, yaw=0.0, cx=0.0, cy=0.0):
    return {"cx": cx, "cy": cy, "yaw": yaw, "W": W, "D": D, "z0": 0.0,
            "top": H, "levels": [0.0, 3.5, 7.0], "module": 4.0}


def ctx(stage, tag="t"):
    return qf._c_ctx(stage, "/World/g", {k: object() for k in (
        "soil", "concrete", "dark_concrete", "brick", "crack", "rebar",
        "plaster", "mortar", "timber", "glass")}, random.Random(4), tag)


print("1) tilt matrix sign — the building must lean TOWARD low_side")
for yaw in (0.0, 37.0, 90.0):
    for side in ("S", "E", "N", "W"):
        m = mass(yaw=yaw)
        M, g = qf._c_tilt_matrix(m, side, 8.0, 1.0)
        low, drop, rise = qf._c_read_M(m, M)
        chk("yaw {0:.0f} side {1}: measured low = {2}".format(yaw, side, low),
            low == side, "drop {0:.2f}/{1:.2f} rise {2:.2f}/{3:.2f}".format(
                drop, g["drop"], rise, g["rise"]))
        chk("  drop matches", abs(drop - g["drop"]) < 1e-6)
        chk("  rise matches", abs(rise - g["rise"]) < 1e-6)

print("2) the OLD edge-pivot / -theta convention heaved the WRONG side")
m = mass()
side = "S"
ox, oy = qf._outward(m, side)
edge = (m["D"] if side in ("S", "N") else m["W"]) / 2.0
px, py = qf._to_world(m, ox * edge if side in ("E", "W") else 0.0,
                      oy * edge if side in ("S", "N") else 0.0)
Mold = qf._rot_about((px, py, 0.0), (-oy, ox, 0.0), -8.0) * qf._translate(0, 0, -1.0)
low, drop, rise = qf._c_read_M(m, Mold)
chk("old tilt_sink(side=S) actually leans toward " + low, low == "N",
    "drop {0:.2f} rise {1:.2f}".format(drop, rise))
Mov = qf._rot_about((px, py, 0.0), (-oy, ox, 0.0), -80.0)
chk("old overturn(side=S) actually falls toward " + qf._c_fall_side(m, Mov),
    qf._c_fall_side(m, Mov) == "N")

print("3) the new tilt clears the high side out of the ground")
for W, D, deg, sink in ((27, 17, 8, 1.0), (22, 14, 5, 0.9), (32, 16, 20, 0.8)):
    m = mass(W=W, D=D)
    M, g = qf._c_tilt_matrix(m, "S", deg, sink)
    chk("W{0} D{1} {2}deg sink{3}: rise {4:+.2f} m, drop {5:.2f} m".format(
        W, D, deg, sink, g["rise"], g["drop"]), g["rise"] >= qf.C_MIN_RISE_M - 1e-9)
M, g = qf._c_tilt_matrix(mass(W=40, D=30), "E", 9.0, 1.4, max_drop_m=2.4)
chk("max_drop_m caps the low corner", g["drop"] <= 2.41, "{0:.2f}".format(g["drop"]))

print("4) smoke: every helper authors without raising")
st = mockpxr.Stage()
c = ctx(st, "b0")
m = mass()
M, g = qf._c_tilt_matrix(m, "E", 12.0, 1.1)
out = qf._c_ground_response(c, m, low_side=g["low"], drop_m=g["drop"],
                            rise_m=g["rise"], tag="tilt")
n_tilt = len(st.prims)
chk("tilt ground authored {0} prims".format(n_tilt), 60 < n_tilt < 2200)

c2 = ctx(st, "b1")
qf._c_ground_response(c2, mass(cx=90.0, cy=40.0), sink_m=0.9, tag="settle")
n_set = len(st.prims) - n_tilt
chk("settlement ring authored {0} prims".format(n_set), 60 < n_set < 2200)

c3 = ctx(st, "b2")
m3 = mass(cx=-90.0, cy=-40.0)
Mov = qf._rot_about((m3["cx"], m3["cy"] - m3["D"] / 2.0, 0.0), (1.0, 0.0, 0.0), -78.0)
r = qf._c_overturn_ground(c3, m3, Mov, angle_deg=78.0, tag="ovg")
n_ov = len(st.prims) - n_tilt - n_set
chk("overturn ground authored {0} prims, fall {1} hinge {2}".format(
    n_ov, r["fall"], r["hinge"]), n_ov > 60 and r["fall"] == "N")

print("5) bounds clamp: nothing authored outside the plate")
st2 = mockpxr.Stage()
c4 = ctx(st2, "b3")
m4 = mass(cx=118.0, cy=0.0)          # hard against the +x edge of a 250 m plate
M4, g4 = qf._c_tilt_matrix(m4, "E", 10.0, 1.0)
qf._c_ground_response(c4, m4, low_side=g4["low"], drop_m=g4["drop"],
                      rise_m=g4["rise"], tag="edge",
                      bounds=(-123.0, -123.0, 123.0, 123.0))
bad = []
for p in st2.prims.values():
    for kind, v in p.ops:
        if kind == "translate" and not (-123.01 <= v[0] <= 123.01
                                        and -123.01 <= v[1] <= 123.01):
            bad.append((p.path, tuple(round(q, 1) for q in v)))
    for q in p.attrs.get("PointsAttr", []) or []:
        if not (-123.01 <= q[0] <= 123.01 and -123.01 <= q[1] <= 123.01):
            bad.append((p.path, tuple(round(x, 1) for x in q)))
chk("no prim off the plate ({0} prims)".format(len(st2.prims)), not bad,
    str(bad[:3]))

print("6) yawed buildings: the wedge follows the building, not the world")
st3 = mockpxr.Stage()
c5 = ctx(st3, "b4")
m5 = mass(yaw=41.0, cx=20.0, cy=-11.0)
M5, g5 = qf._c_tilt_matrix(m5, "N", 7.0, 0.8)
qf._c_ground_response(c5, m5, low_side=g5["low"], drop_m=g5["drop"],
                      rise_m=g5["rise"], tag="yaw")
far = 0.0
for p in st3.prims.values():
    for kind, v in p.ops:
        if kind == "translate":
            far = max(far, ((v[0] - 20.0) ** 2 + (v[1] + 11.0) ** 2) ** 0.5)
chk("everything within footprint + 10 m ({0:.1f} m)".format(far), far < 30.0)

print("\n{0}".format("ALL OK" if not fails else "FAILED: " + ", ".join(fails)))
sys.exit(1 if fails else 0)
