"""Integration check: run the three foundation recipes end to end on REAL kit
placements with a mock stage (no Isaac). Catches NameErrors and bad frames in
the rewritten r_tilt_sink / r_settlement / r_tilt_severe / r_overturn."""
import os, random, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, "/home/krrishjain/SEI-COA/disaster-dataset/scene_gen")
import mockpxr; mockpxr.install()
import numpy as np
from detail import urban_building as ub
from disaster import damage, quake_flow as qf
damage._pbr = lambda *a, **k: object()          # shader graphs are not the point
damage.bound_texture = lambda *a, **k: None

fails = []
def chk(name, cond, extra=""):
    print(("  ok   " if cond else "  FAIL ") + name + ("  " + extra if extra else ""))
    if not cond:
        fails.append(name)

MATS = {k: object() for k in ("soil", "concrete", "dark_concrete", "brick",
                              "crack", "rebar", "plaster", "mortar", "timber",
                              "glass")}

def run(style, recipe, seed=4, yaw=0.0, cx=0.0, cy=0.0):
    st = mockpxr.Stage()
    rng = random.Random(seed)
    pls = ub.build_building(style, cx, cy, yaw, random.Random(seed))
    for i, p in enumerate(pls):
        pp = "/World/g/p{0}".format(i)
        st.define(pp)
        p["prim_path"] = pp
    qf.fit_interior = lambda *a, **k: {"slabs": {}, "columns": {},
                                       "partitions": [], "props": {}, "all": []}
    qf.dress_roof = lambda *a, **k: []
    # the fracture path needs trimesh/vtk, which the host has not got: the
    # point here is the RIGID transform and the ground, not the fragments
    qf._break = lambda *a, **k: ([], [])
    qf._roof_box = lambda *a, **k: None
    res = qf.wreck_building(st, "/World/g", style, pls, cx, cy, yaw,
                            [(recipe, {})], rng, np.random.default_rng(seed),
                            MATS, "b0")
    return st, res


def spread(st, cx, cy):
    """Farthest AUTHORED prim from the building centre. The kit placements
    (`/World/g/pN`) are excluded: the mock never ran `apply_placements`, so
    their world transform starts at the identity and `_transform_prims` lands
    them at the origin — a mock artefact that would mask a real yaw bug."""
    import re
    far = 0.0
    for pr in st.prims.values():
        if re.match(r"^/World/g/p\d+$", pr.path):
            continue
        for kind, v in pr.ops:
            if kind == "translate":
                far = max(far, ((v[0] - cx) ** 2 + (v[1] - cy) ** 2) ** 0.5)
    return far

import itertools
SEEDS = [int(q) for q in os.environ.get("SEEDS", "4").split(",")]
for style, seed in itertools.product(
        ("apartment_tall", "office", "brownstone", "tower", "walkup",
         "department_store"), SEEDS):
    for recipe in ("tilt_sink", "settlement", "tilt_severe", "overturn"):
        try:
            st, res = run(style, recipe, seed)
            chk("{0}/{1}/s{4}: {2} prims, {3} authored".format(
                style, recipe, len(st.prims), len(res["authored"]), seed),
                len(res["authored"]) > 40, "; ".join(res["notes"]))
        except Exception as exc:
            import traceback; traceback.print_exc()
            chk("{0}/{1}/s{2}".format(style, recipe, seed), False, repr(exc))

print("YAW: the ground must follow the BUILDING, not the world axes")
for style in ("apartment_tall", "office"):
    for recipe in ("tilt_sink", "tilt_severe", "overturn"):
        far = {}
        for yaw in (0.0, 90.0, 45.0):
            st, res = run(style, recipe, 4, yaw=yaw, cx=60.0, cy=-25.0)
            far[yaw] = spread(st, 60.0, -25.0)
        lim = 1.30 * far[0.0] + 2.0
        chk("{0}/{1}: spread {2:.1f} / {3:.1f} / {4:.1f} m at yaw 0/90/45".format(
            style, recipe, far[0.0], far[90.0], far[45.0]),
            far[90.0] < lim and far[45.0] < lim, "limit {0:.1f}".format(lim))

print("\n{0}".format("ALL OK" if not fails else "FAILED: " + ", ".join(fails)))
sys.exit(1 if fails else 0)
