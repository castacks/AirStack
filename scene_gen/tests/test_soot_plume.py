#!/usr/bin/env python3
"""test_soot_plume.py — is the soot WHERE THE FIRE IS?

    python3 scene_gen/tests/test_soot_plume.py
    pytest -q scene_gen/tests/test_soot_plume.py

WHY THIS EXISTS
---------------
`disaster/soot_plume.py` exists to make the scorch on a burning building come
OUT OF the openings the fire vents through — the same openings `r_flames`
lights — instead of being a random wash that happens to share a storey band
with the flames. That is a claim about arithmetic, and it is checked here on
REAL kit buildings (`urban_building.build_building` + `quake_flow.describe`,
pure placement math, no USD, no Kit, no Isaac Sim), through the actual
`plan_fire` -> `plan_events` -> `skin` path a scene takes:

  * `urban_fire.check_soot_events` — every event on the band and sides; flame
    only while the ladder says alight and never past the Flow budget; the skin
    clean two storeys under the origin; dark at every event's own head; an F5
    band near-saturated and still not one flat colour;
  * the crop that reaches a prim addresses the right pixels (a synthetic skin
    with a marker in it, cropped back — with the row flip and the perimeter
    wrap both exercised). NOTE: since 2026-08-30 this crop is only a
    PREFILTER ("does any soot reach this module's rectangle at all?") and a
    preview device (`soot_png.py`, `soot_elevation.py`) — it is not the sim
    path any more. What actually lands on a prim goes through the module's
    own UVs (`disaster/soot_bake.py`), and that path is checked in
    `test_soot_bake.py`, not here;
  * every module of a burning mass maps to a crop inside the canvas;
  * deposits from two events at one place ADD (the pair is darker than either);
  * the same seed gives the same skin.

It runs host-side in well under a minute.
"""

import os
import random
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from detail import urban_building as ub          # noqa: E402
from disaster import quake_flow as qf            # noqa: E402
from disaster import soot_plume as sp            # noqa: E402
from disaster import urban_fire as uf            # noqa: E402


def _ctx(style, level, seed, sides=("S", "E")):
    rng = random.Random(seed)
    nrng = np.random.default_rng(seed)
    placements = ub.build_building(style, 0.0, 0.0, 0.0, rng)
    info = qf.describe(style, placements, 0.0, 0.0, 0.0)
    ctx = {"info": info, "rng": rng, "nrng": nrng, "notes": []}
    mtag = max(info["masses"].items(),
               key=lambda kv: (len(kv[1]["levels"]), kv[0] == "main"))[0]
    n = len(info["masses"][mtag]["levels"])
    origin = max(1, min(n - 2, int(round(0.3 * (n - 1)))))
    ctx["fire"] = uf.plan_fire(info, level, rng, origin=origin, sides=sides)
    ctx["fire"]["events"] = sp.plan_events(ctx, uf._severity, rng)
    return ctx, mtag


def test_soot_is_where_the_fire_is():
    bad = uf.check_soot_events(verbose=False)
    assert not bad, "\n".join(bad)


def test_piece_crop_addresses_the_right_pixels():
    h, w = 200, 1000                       # 20 m tall, 100 m round, 10 px/m
    rgba = np.zeros((h, w, 4), dtype=np.float32)
    sk = {"rgba": rgba, "ppm": 10.0, "per": 100.0, "H": 20.0, "z0": 0.0,
          "offsets": {"S": 0.0, "E": 30.0, "N": 50.0, "W": 80.0}}
    # a marker on the E side, u 5..7 m, z 10..12 m: row 0 is the TOP, so
    # z 10..12 is rows 80..100; E starts 30 m round, so u 5..7 is cols 350..370
    rgba[80:100, 350:370, 3] = 1.0
    hit = sp.piece_crop(sk, "E", 5.0, 7.0, 10.0, 12.0)
    assert hit.shape[:2] == (20, 20) and float(hit[..., 3].min()) == 1.0
    assert float(sp.piece_crop(sk, "E", 7.0, 9.0, 10.0, 12.0)[..., 3].max()) == 0.0
    assert float(sp.piece_crop(sk, "E", 5.0, 7.0, 12.0, 14.0)[..., 3].max()) == 0.0
    assert float(sp.piece_crop(sk, "E", 5.0, 7.0, 8.0, 10.0)[..., 3].max()) == 0.0
    # a flipped row mapping would put the marker at z 8..10 — the exact
    # "stain under the sill instead of over the head" failure
    # the perimeter wraps: W's last two metres and S's first two are one
    rgba[:, :] = 0.0
    rgba[0:10, 990:1000, 3] = 1.0
    rgba[0:10, 0:10, 3] = 1.0
    wrap = sp.piece_crop(sk, "W", 19.0, 21.0, 19.0, 20.0)
    assert wrap.shape[1] == 20 and float(wrap[..., 3].min()) == 1.0


def test_every_module_of_the_burning_mass_crops_inside_the_skin():
    ctx, mtag = _ctx("commercial_mid", "F3", 3)
    sk = sp.skin(ctx, ctx["fire"]["events"], ctx["nrng"])
    m = ctx["info"]["masses"][mtag]
    n = 0
    for e in qf._els(ctx, mass=mtag,
                     role=("wall", "corner", "parapet", "parapet_corner")):
        fe = qf._piece_frame(e)
        if fe is None:
            continue
        u0, u1 = sp.piece_span(e, fe, m, e["side"])
        L = sp.side_length(m, e["side"])
        assert -0.6 * fe[3] <= u0 < u1 <= L + 0.6 * fe[3], (e["name"], e["side"], u0, u1, L)
        crop = sp.piece_crop(sk, e["side"], u0, u1, float(e["z"]),
                             float(e["z"]) + float(fe[4]))
        assert crop.shape[0] >= 1 and crop.shape[1] >= 1 and crop.shape[2] == 4
        n += 1
    assert n > 20


def test_two_events_at_one_place_add_up():
    ctx, _ = _ctx("commercial_mid", "F2", 5, sides=("S",))
    ev = [e for e in ctx["fire"]["events"] if e["state"] == "flame"][0]
    # the RAW deposit: the noised one is drawn from a generator the second
    # event has advanced, so the two runs' noise fields are not comparable
    one = sp.skin(ctx, [ev], np.random.default_rng(1))["dep_raw"]
    two = sp.skin(ctx, [ev, dict(ev)], np.random.default_rng(1))["dep_raw"]
    assert np.all(two >= one - 1e-5)
    lit = one > 0.05 * float(one.max())
    ratio = float(two[lit].sum() / max(1e-9, one[lit].sum()))
    assert 1.6 < ratio < 2.4, ratio


def test_same_seed_same_skin():
    a, _ = _ctx("apartment", "F3", 11)
    b, _ = _ctx("apartment", "F3", 11)
    sa = sp.skin(a, a["fire"]["events"], np.random.default_rng(11))["rgba"]
    sb = sp.skin(b, b["fire"]["events"], np.random.default_rng(11))["rgba"]
    assert sa.shape == sb.shape and np.array_equal(sa, sb)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)
