#!/usr/bin/env python3
"""test_urban_fire_sliced_dressing.py — the two SLICED-ONLY dressing changes
of the fire_dtc3 review (2026-08-30) do what they say, and the KIT path they
are gated away from does not move a single draw.

    python3 scene_gen/tests/test_urban_fire_sliced_dressing.py
    pytest -q scene_gen/tests/test_urban_fire_sliced_dressing.py

    # the USD-dependent tests need pxr — run under the container:
    docker exec isaac-sim bash -c \
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
       /isaac-sim/AirStack/scene_gen/tests/test_urban_fire_sliced_dressing.py"

WHY THIS EXISTS
---------------
A. `urban_fire.r_street_debris` now draws (50, 6.5, 0.58) instead of
   (24, 4.5, 0.5) when `ctx["soot_prebaked"]` is a set — the sliced path —
   because 24 lumps in a 4.5 m apron read as a sprinkle under a 28 m GAC
   elevation. The MCE kit look is FROZEN, so the kit branch has to keep the
   original constants AND the original rng stream: the count, every lump's
   placement and every draw `quake_flow._a_lump` makes after them. This
   replays the pre-change body verbatim against the live one on the same
   seed and demands identical geometry, which is a stronger statement than
   "the count matches" — `_a_lump` alone spends 29 draws per lump, so a
   single extra or reordered draw anywhere shifts every later lump.

B. `quake_flow._inside_inset` is the convex point-in-polygon test the new
   `fit_interior(footprint=...)` clamps the column grid with (the grid is
   laid on the mass's `W x D` bounding box and its corners poke out through
   an irregular façade). Pure numeric, and it has to be winding-agnostic —
   `gac_fire._storey_footprints`' hulls are not guaranteed CCW — and it has
   to answer True for a degenerate polygon, because "no footprint measured"
   must mean "clamp nothing", which is what every pre-kwarg caller did.

C. `fit_interior`'s new kwarg defaults to None. The earthquake session
   shares that function; a default that clamped anything would change its
   scenes silently.
"""

import inspect
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)

from disaster import quake_flow as qf                          # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402

try:
    from pxr import Usd, UsdGeom, UsdShade                      # noqa: E402
    HAVE_USD = True
except Exception:                                              # pragma: no cover
    HAVE_USD = False

SEED = 20260830


# ---------------------------------------------------------------------------
# B. _inside_inset — pure numeric, no USD
# ---------------------------------------------------------------------------
_SQ_CCW = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
_SQ_CW = list(reversed(_SQ_CCW))


def test_inside_inset_centre_is_inside_either_winding():
    for poly in (_SQ_CCW, _SQ_CW):
        assert qf._inside_inset(poly, 5.0, 5.0, 0.35)


def test_inside_inset_rejects_outside_and_the_margin_band():
    for poly in (_SQ_CCW, _SQ_CW):
        assert not qf._inside_inset(poly, -1.0, 5.0, 0.35)   # outside
        assert not qf._inside_inset(poly, 0.2, 5.0, 0.35)    # inside, too close
        assert qf._inside_inset(poly, 0.4, 5.0, 0.35)        # clear of the edge


def test_inside_inset_margin_is_the_distance_to_the_nearest_edge():
    # a corner is `inset` from TWO edges at once: 0.5 in from each clears a
    # 0.35 margin, 0.3 in from each does not
    assert qf._inside_inset(_SQ_CCW, 0.5, 0.5, 0.35)
    assert not qf._inside_inset(_SQ_CCW, 0.3, 0.3, 0.35)


def test_inside_inset_degenerate_polygon_clamps_nothing():
    assert qf._inside_inset([], 0.0, 0.0, 0.35)
    assert qf._inside_inset([(0.0, 0.0), (1.0, 0.0)], 99.0, 99.0, 0.35)


# ---------------------------------------------------------------------------
# C. the fit_interior kwarg is opt-in
# ---------------------------------------------------------------------------
def test_fit_interior_footprint_defaults_to_none():
    sig = inspect.signature(qf.fit_interior)
    assert "footprint" in sig.parameters
    assert sig.parameters["footprint"].default is None


# ---------------------------------------------------------------------------
# A. r_street_debris — kit stream frozen, sliced apron bigger
# ---------------------------------------------------------------------------
def _ctx(stage, sliced):
    """The smallest ctx `r_street_debris` reads, on a 14 x 28 m mass."""
    parent = "/W"
    mats = {}
    for key in ("fire_glass", "char_concrete", "soot_light"):
        mats[key] = UsdShade.Material.Define(stage, parent + "/M/" + key)
    return {
        "stage": stage, "parent": parent, "tag": "t", "rng": random.Random(SEED),
        "authored": [], "notes": [], "loose": [], "mats": mats,
        "soot_prebaked": (set() if sliced else False),
        "fire": {"sides": ["E"]},
        "info": {"masses": {"main": {
            "W": 14.35, "D": 28.37, "cx": 0.0, "cy": 0.0, "yaw": 0.0,
            "z0": 0.0, "top": 69.2, "module": 4.0, "levels": [0.0]}}},
    }


def _orig_street_debris(ctx, density=1.0):
    """`r_street_debris` EXACTLY as it stood before the fire_dtc3 change —
    the reference stream the kit path must still reproduce."""
    f, rng = ctx["fire"], ctx["rng"]
    m = ctx["info"]["masses"]["main"]
    n = 0
    for side in f["sides"]:
        nx, ny = qf._outward(m, side)
        span = m["W"] if side in ("S", "N") else m["D"]
        half = (m["D"] if side in ("S", "N") else m["W"]) / 2.0
        count = int(24 * density * rng.uniform(0.7, 1.3))
        for _ in range(count):
            t = rng.uniform(-0.5, 0.5) * span
            d = half + rng.uniform(0.4, 4.5)
            if side in ("S", "N"):
                lx, ly = t, math.copysign(d, ny or 1.0)
            else:
                lx, ly = math.copysign(d, nx or 1.0), t
            wx, wy = qf._to_world(m, lx, ly)
            s = rng.uniform(0.07, 0.34)
            r = rng.random()
            mat = (ctx["mats"]["fire_glass"] if r < 0.42 else
                   ctx["mats"]["char_concrete"] if r < 0.7 else
                   ctx["mats"]["soot_light"])
            path = "{0}/sdeb_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                             qf._uid(ctx))
            qf._a_lump(ctx["stage"], path, wx, wy, m["z0"] + s * 0.18, s, rng,
                       mat, jitter=0.45)
            ctx["authored"].append(path)
            n += 1
    return n


def _dump(stage, ctx):
    """Every authored lump as (translate, points-checksum) — a fingerprint of
    the whole rng stream, not just of the count."""
    out = []
    for p in ctx["authored"]:
        pr = stage.GetPrimAtPath(p)
        xf = UsdGeom.Xformable(pr).GetLocalTransformation()
        pts = UsdGeom.Mesh(pr).GetPointsAttr().Get()
        out.append((round(float(xf[3][0]), 9), round(float(xf[3][1]), 9),
                    round(float(xf[3][2]), 9),
                    round(sum(abs(v[0]) + abs(v[1]) + abs(v[2])
                              for v in pts), 9)))
    return out


def _run(fn, sliced):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    ctx = _ctx(stage, sliced)
    fn(ctx)
    return ctx, _dump(stage, ctx)


def test_kit_street_debris_is_byte_identical():
    if not HAVE_USD:                                           # pragma: no cover
        return
    _, live = _run(uf.r_street_debris, sliced=False)
    _, ref = _run(_orig_street_debris, sliced=False)
    assert live == ref, "the kit path's debris stream moved"
    assert 16 <= len(live) <= 31, len(live)        # int(24 * u(0.7, 1.3))


def test_sliced_street_debris_is_denser_and_wider():
    if not HAVE_USD:                                           # pragma: no cover
        return
    _, kit = _run(uf.r_street_debris, sliced=False)
    _, gac = _run(uf.r_street_debris, sliced=True)
    assert len(gac) >= 1.9 * len(kit), (len(gac), len(kit))
    # the apron: E side, so |x| beyond the half-width, and the run along the
    # elevation is |y| against the 28.37 m span
    half = 14.35 / 2.0
    assert max(abs(r[0]) for r in gac) - half > \
        max(abs(r[0]) for r in kit) - half
    assert max(abs(r[1]) for r in gac) > max(abs(r[1]) for r in kit)


if __name__ == "__main__":
    print("pxr: " + ("present" if HAVE_USD else "ABSENT -- USD tests skipped"))
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)
