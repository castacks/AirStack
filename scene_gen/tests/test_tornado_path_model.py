"""test_tornado_path_model.py — the tornado TRACK model, pinned and extended.

Stream P of `_plans/urban_tornado_plan.md` (see sections 0, 1, 2.1, 2.2, 2.4,
3, 5). This file owns two jobs that have to live in the same place because
the second is only trustworthy if the first passes:

  1. THE SAFETY NET. Every existing caller of `disaster.tornado`  — the
     suburb tornado presets, `tools/tornado_png.py`, `scour_relief` via
     `from_track` — has to produce BYTE-IDENTICAL output after this file's
     other job below. `test_suburb_presets_unchanged` and
     `test_compile_tornado_suburb_spec_unchanged` freeze the pre-change
     behaviour of the two presets named in the plan
     (`suburb_tornado_1000`, `suburb_tornado_1000_l2`) and of
     `compile_disaster.compile_tornado` on the `suburb_tornado` spec into
     `tests/fixtures/tornado_field_snapshot.json`, captured BEFORE any of
     the new knobs below existed. If this drifts, the fix is always in the
     new code, never in the fixture.

  2. THE NEW PATH KNOBS (plan 2.2): `curvature_deg_per_km` (a gentle arc,
     implemented inside `_wobble` so `frame`/`from_track` stay exact
     inverses), `touchdown_m` / `liftoff_m` / `ramp_m` (a finite track that
     ramps up and ropes out instead of spanning the whole plate), and
     `wind_at` (plan 2.4: the near-surface wind DIRECTION a building sees,
     built on the SAME cross-track profile `intensity_field` uses).

REGENERATING THE FIXTURE — only ever run this against code where job 1 is
still true (i.e. before editing `tornado.py`/`compile_disaster.py`, or after
confirming a change is a deliberate, reviewed behaviour change):

    cd scene_gen && python3 tests/test_tornado_path_model.py --snapshot

RUNNING THE TESTS (per the plan: run ONLY this file):

    cd scene_gen && python3 -m pytest -q tests/test_tornado_path_model.py
"""

import json
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import numpy as np                                     # noqa: E402
import pytest                                           # noqa: E402
import yaml                                              # noqa: E402

import compile_disaster as cd                            # noqa: E402
from disaster import tornado as tn                       # noqa: E402

FIXTURE_PATH = os.path.join(_HERE, "fixtures", "tornado_field_snapshot.json")

# The two presets the plan names as the ones that must not move:
# `suburb_tornado_1000` and `suburb_tornado_1000_l2`.
PRESETS = ("suburb_tornado_1000", "suburb_tornado_1000_l2")

GRID_N = 41
N_TRACK_POINTS = 50
N_THROW_DRAWS = 20


# ---------------------------------------------------------------------------
# pure computation shared by the snapshot generator and the parity test
# ---------------------------------------------------------------------------

def _fixed_xy_points(region, n=N_TRACK_POINTS, seed=90210, pad=1.3):
    """`n` deterministic `(x, y)` world points spanning a padded `region`.

    Padded past the plate edge on purpose — `to_track` has no notion of a
    boundary, and a point outside the region still has to round-trip through
    `from_track` exactly, including once `curvature_deg_per_km` is in play.
    """
    x0, y0, x1, y1 = region
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    hw, hh = (x1 - x0) / 2.0 * pad, (y1 - y0) / 2.0 * pad
    rng = np.random.default_rng(seed)
    xs = cx + rng.uniform(-hw, hw, n)
    ys = cy + rng.uniform(-hh, hh, n)
    return [[float(x), float(y)] for x, y in zip(xs, ys)]


def _fixed_ac_points(n=N_TRACK_POINTS, seed=90211, a_span=1500.0,
                     c_span=400.0):
    """`n` deterministic `(along_m, cross_m)` points for `from_track`."""
    rng = np.random.default_rng(seed)
    a = rng.uniform(-a_span, a_span, n)
    c = rng.uniform(-c_span, c_span, n)
    return [[float(x), float(y)] for x, y in zip(a, c)]


def _snapshot_for_preset(name):
    """Everything STEP 1 of the brief asks to pin for one preset."""
    cfg = cd.load_scene_config(name)
    tcfg = tn.resolve_cfg(cfg)
    region_m = (cfg.get("layout") or {}).get("region_m") or [500.0, 500.0]
    rw, rh = float(region_m[0]), float(region_m[1])
    region = (-rw / 2.0, -rh / 2.0, rw / 2.0, rh / 2.0)
    seed = int(cfg.get("seed", 0))

    inten = tn.intensity_field(tcfg, region, np.random.default_rng(seed + 23))
    xs = np.linspace(region[0], region[2], GRID_N)
    ys = np.linspace(region[1], region[3], GRID_N)
    grid = [[float(inten(float(x), float(y))) for x in xs] for y in ys]

    to_track, (ux, uy), (vx, vy) = tn.frame(tcfg)
    from_world = tn.from_track(tcfg)

    xy_pts = _fixed_xy_points(region)
    to_track_out = [list(to_track(x, y)) for (x, y) in xy_pts]

    ac_pts = _fixed_ac_points()
    from_track_out = [list(from_world(a, c)) for (a, c) in ac_pts]

    throw = tn.throw_field(tcfg)
    trng = random.Random(seed + 71)
    intensities = [k / float(N_THROW_DRAWS - 1) for k in range(N_THROW_DRAWS)]
    throw_out = [list(throw(i, trng)) for i in intensities]

    return {
        "seed": seed,
        "region": list(region),
        "tcfg": tcfg,
        "axes": {"origin": list((tcfg["origin_m"][0], tcfg["origin_m"][1])),
                "u": [ux, uy], "v": [vx, vy]},
        "grid": {"xs": [float(x) for x in xs], "ys": [float(y) for y in ys],
                "values": grid},
        "to_track": {"points": xy_pts, "out": to_track_out},
        "from_track": {"points": ac_pts, "out": from_track_out},
        "throw": {"intensities": intensities, "out": throw_out},
    }


def _compile_tornado_suburb_snapshot():
    """`compile_tornado` called directly on `suburb_tornado.yaml`'s raw
    high-level spec — STEP 2c's "compile the suburb spec before/after"."""
    path = cd.resolve_config_path("suburb_tornado")
    with open(path) as f:
        spec = yaml.safe_load(f)
    sev = float(spec.get("severity", 1.0))
    region = tuple(float(v) for v in spec["region_m"])
    result = cd.compile_tornado(sev, spec, region)
    return {"path": os.path.relpath(path, _SCENE_GEN), "sev": sev,
            "region": list(region), "result": result}


def _full_snapshot():
    return {
        "schema": "tornado_field_snapshot.v1",
        "presets": {name: _snapshot_for_preset(name) for name in PRESETS},
        "compile_tornado_suburb": _compile_tornado_suburb_snapshot(),
    }


# ---------------------------------------------------------------------------
# recursive "equal to 1e-9" — deliberately NOT pytest.approx, whose default
# relative tolerance (1e-6) is looser than the brief asks for.
# ---------------------------------------------------------------------------

def _assert_close(expected, actual, path="$", tol=1e-9):
    if isinstance(expected, dict):
        assert isinstance(actual, dict), f"{path}: expected a dict, got {actual!r}"
        missing = set(expected) - set(actual)
        assert not missing, f"{path}: missing keys {missing}"
        for k in expected:
            _assert_close(expected[k], actual[k], f"{path}.{k}", tol)
    elif isinstance(expected, (list, tuple)):
        assert isinstance(actual, (list, tuple)), (
            f"{path}: expected a list, got {actual!r}")
        assert len(expected) == len(actual), (
            f"{path}: length {len(expected)} != {len(actual)}")
        for i, (e, a) in enumerate(zip(expected, actual)):
            _assert_close(e, a, f"{path}[{i}]", tol)
    elif isinstance(expected, bool) or isinstance(actual, bool):
        assert expected == actual, f"{path}: {expected!r} != {actual!r}"
    elif isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
        assert math.isclose(float(expected), float(actual), rel_tol=0.0,
                            abs_tol=tol), f"{path}: {expected!r} != {actual!r}"
    else:
        assert expected == actual, f"{path}: {expected!r} != {actual!r}"


def _subset_close(expected_subset, actual_superset, path="$", tol=1e-9):
    """Like `_assert_close`, but `actual_superset` may carry EXTRA keys.

    Used only for the resolved `tcfg` dict: STEP 2 adds new `DEFAULTS` keys
    (`curvature_deg_per_km`, `touchdown_m`, ...) that did not exist when the
    fixture was captured, and their presence with a neutral default is not a
    behaviour change — the byte-identical requirement is on what the field
    FUNCTIONS produce, pinned separately by the grid/track/throw checks.
    """
    assert isinstance(actual_superset, dict), path
    for k, v in expected_subset.items():
        assert k in actual_superset, f"{path}.{k}: key dropped"
        _assert_close(v, actual_superset[k], f"{path}.{k}", tol)


# ---------------------------------------------------------------------------
# fixture access
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def snapshot():
    with open(FIXTURE_PATH) as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# 1. the safety net
# ---------------------------------------------------------------------------

def test_suburb_presets_unchanged(snapshot):
    """`suburb_tornado_1000` / `suburb_tornado_1000_l2` field outputs are
    byte-identical (to 1e-9) to the pre-change fixture."""
    for name in PRESETS:
        stored = snapshot["presets"][name]
        fresh = _snapshot_for_preset(name)

        assert fresh["seed"] == stored["seed"], name
        _assert_close(stored["region"], fresh["region"], f"{name}.region")
        _subset_close(stored["tcfg"], fresh["tcfg"], f"{name}.tcfg")
        _assert_close(stored["grid"], fresh["grid"], f"{name}.grid")
        _assert_close(stored["to_track"], fresh["to_track"],
                     f"{name}.to_track")
        _assert_close(stored["from_track"], fresh["from_track"],
                     f"{name}.from_track")
        _assert_close(stored["throw"], fresh["throw"], f"{name}.throw")


def test_compile_tornado_suburb_spec_unchanged(snapshot):
    """`compile_tornado` on the `suburb_tornado` preset's raw spec is
    dict-equal before/after — the additive pass-through never fires for a
    spec that does not set the new top-level keys."""
    stored = snapshot["compile_tornado_suburb"]
    fresh = _compile_tornado_suburb_snapshot()
    assert fresh["sev"] == stored["sev"]
    _assert_close(stored["region"], fresh["region"])
    _assert_close(stored["result"], fresh["result"], "compile_tornado_suburb")


# ---------------------------------------------------------------------------
# 2. to_track / from_track stay exact inverses with curvature + wobble on
# ---------------------------------------------------------------------------

def test_round_trip_with_curvature_and_wobble():
    cfg = dict(tn.DEFAULTS, curvature_deg_per_km=12.0, wobble_m=22.0,
              wobble_period_m=340.0, origin_m=[13.0, -47.0],
              heading_deg=71.0)
    to_track, _u, _v = tn.frame(cfg)
    from_world = tn.from_track(cfg)

    rng = np.random.default_rng(4477)
    xs = rng.uniform(-900.0, 900.0, 200)
    ys = rng.uniform(-900.0, 900.0, 200)
    for x, y in zip(xs, ys):
        a, c = to_track(float(x), float(y))
        x2, y2 = from_world(a, c)
        assert math.isclose(x2, float(x), rel_tol=0.0, abs_tol=1e-9)
        assert math.isclose(y2, float(y), rel_tol=0.0, abs_tol=1e-9)


def test_curvature_default_zero_matches_no_curvature_key():
    """`curvature_deg_per_km` defaults to 0.0 -- a cfg that never mentions it
    and one that sets it to 0.0 explicitly must agree exactly."""
    base = dict(tn.DEFAULTS)
    explicit = dict(tn.DEFAULTS, curvature_deg_per_km=0.0)
    from_a = tn.from_track(base)
    from_b = tn.from_track(explicit)
    for a in (-400.0, -1.0, 0.0, 1.0, 733.0):
        for c in (-80.0, 0.0, 120.0):
            xa, ya = from_a(a, c)
            xb, yb = from_b(a, c)
            assert xa == xb and ya == yb, (a, c)


# ---------------------------------------------------------------------------
# 3. curvature sign and magnitude
# ---------------------------------------------------------------------------

def test_curvature_sign_and_magnitude():
    """`curvature_deg_per_km = +10` turns the track LEFT of travel as `along`
    increases, so a point on the ORIGINAL straight chord 800 m downtrack now
    has NEGATIVE `cross` (the centreline moved left of it, so the chord point
    reads as being to the right of the moved centreline)."""
    heading = 0.0
    cfg = dict(tn.DEFAULTS, curvature_deg_per_km=10.0, wobble_m=0.0,
              origin_m=[0.0, 0.0], heading_deg=heading)
    to_track, (ux, uy), (vx, vy) = tn.frame(cfg)

    a = 800.0
    # The point on the STRAIGHT chord through the origin at heading 0,
    # 800 m along it: (a, 0) in world coordinates when heading is 0.
    x, y = ux * a, uy * a
    a_out, c_out = to_track(x, y)

    R = 1000.0 / (10.0 * math.pi / 180.0)
    expected_offset = R - math.sqrt(R * R - a * a)   # > 0 (curve moved left)

    assert math.isclose(a_out, a, rel_tol=0.0, abs_tol=1e-9)
    assert c_out < 0.0
    assert math.isclose(c_out, -expected_offset, rel_tol=0.0, abs_tol=1e-6)


def test_curvature_sign_flips_with_sign():
    """Negative curvature turns right, giving the mirror-image offset."""
    cfg_pos = dict(tn.DEFAULTS, curvature_deg_per_km=7.0, wobble_m=0.0,
                   origin_m=[0.0, 0.0], heading_deg=0.0)
    cfg_neg = dict(tn.DEFAULTS, curvature_deg_per_km=-7.0, wobble_m=0.0,
                   origin_m=[0.0, 0.0], heading_deg=0.0)
    tp, _, _ = tn.frame(cfg_pos)
    tn_, _, _ = tn.frame(cfg_neg)
    _a1, c_pos = tp(600.0, 0.0)
    _a2, c_neg = tn_(600.0, 0.0)
    assert math.isclose(c_pos, -c_neg, rel_tol=0.0, abs_tol=1e-9)


def test_curvature_huge_radius_matches_parabola_limit():
    """A tiny curvature (huge R) is the numerically-stable parabola branch,
    and it has to agree with the exact circle formula in the regime both are
    valid (small a/R) -- otherwise the two branches disagree at the seam."""
    tiny = 0.0005     # deg/km -> R in the tens of millions of metres
    cfg = dict(tn.DEFAULTS, curvature_deg_per_km=tiny, wobble_m=0.0,
              origin_m=[0.0, 0.0], heading_deg=0.0)
    to_track, (ux, uy), _ = tn.frame(cfg)
    R = 1000.0 / (tiny * math.pi / 180.0)
    for a in (10.0, 250.0, 900.0):
        x, y = ux * a, uy * a
        _a_out, c_out = to_track(x, y)
        parabola = -(a * a) / (2.0 * R)
        # Both branches are approximations near their own seam; require they
        # agree to a few mm at this scale (R ~ 5.7e7 m for `tiny`).
        assert math.isclose(c_out, parabola, rel_tol=1e-3, abs_tol=1e-3)


# ---------------------------------------------------------------------------
# 4. touchdown / liftoff / ramp
# ---------------------------------------------------------------------------

def _finite_track_cfg(**extra):
    cfg = dict(tn.DEFAULTS, touchdown_m=-400.0, liftoff_m=400.0,
              ramp_m=120.0, wobble_m=0.0, width_min=1.0, along_min=1.0,
              origin_m=[0.0, 0.0], heading_deg=0.0)
    cfg.update(extra)
    return cfg


def test_taper_zero_outside_window():
    cfg = _finite_track_cfg()
    region = (-800.0, -800.0, 800.0, 800.0)
    inten = tn.intensity_field(cfg, region, np.random.default_rng(1))
    assert inten(-401.0, 0.0) == 0.0
    assert inten(401.0, 0.0) == 0.0
    # ...and comfortably inside it, it is not.
    assert inten(0.0, 0.0) > 0.0


def test_taper_matches_untapered_at_along_zero():
    tapered = _finite_track_cfg()
    untapered = _finite_track_cfg(touchdown_m=None, liftoff_m=None)
    region = (-800.0, -800.0, 800.0, 800.0)
    i_tap = tn.intensity_field(tapered, region, np.random.default_rng(1))
    i_un = tn.intensity_field(untapered, region, np.random.default_rng(1))
    for c in (0.0, 15.0, -40.0):
        assert math.isclose(i_tap(0.0, c), i_un(0.0, c), rel_tol=0.0,
                            abs_tol=1e-12)


def test_taper_shrinks_half_width_near_liftoff():
    """At `along = 350` (70 m, or 0.583 of a 120 m ramp, before a `liftoff_m
    = 400`), the half-width is between 0.35 and 0.6 of its along-0 value —
    the rope-out band from STEP 2b. `width_min=1.0`/`along_min=1.0` in
    `_finite_track_cfg` hold the ordinary along-track BREATHING flat, so what
    is measured here is only the new taper, not a mix of the two."""
    cfg = _finite_track_cfg()
    region = (-800.0, -800.0, 800.0, 800.0)
    inten = tn.intensity_field(cfg, region, np.random.default_rng(1))

    def half_width_at(a, step=0.25, limit=200.0):
        c = 0.0
        while c < limit:
            if inten(a, c) <= 0.0:
                return c
            c += step
        raise AssertionError(f"never found the edge at along={a}")

    hw0 = half_width_at(0.0)
    hw350 = half_width_at(350.0)
    ratio = hw350 / hw0
    assert 0.30 <= ratio <= 0.65, (hw0, hw350, ratio)


def test_taper_none_none_untouched():
    """`touchdown_m`/`liftoff_m` both `None` (the default) must produce a
    field that is exactly the pre-STEP-2 arithmetic -- pinned generically
    here (not only on the two suburb presets)."""
    cfg = dict(tn.DEFAULTS)
    region = (-250.0, -250.0, 250.0, 250.0)
    inten_a = tn.intensity_field(cfg, region, np.random.default_rng(7))
    inten_b = tn.intensity_field(cfg, region, np.random.default_rng(7))
    for x in (-200.0, -10.0, 0.0, 60.0, 210.0):
        for y in (-180.0, 0.0, 155.0):
            assert inten_a(x, y) == inten_b(x, y)


# ---------------------------------------------------------------------------
# 5. wind_at
# ---------------------------------------------------------------------------

def _wind_cfg(**extra):
    cfg = dict(tn.DEFAULTS, wobble_m=0.0, origin_m=[0.0, 0.0],
              heading_deg=0.0, core_frac=0.30, peak=0.92,
              translation_frac=0.25, inflow_frac=0.30, over_frac=0.18)
    cfg.update(extra)
    return cfg


def _point_at_cross(cfg, cross_m, along_m=0.0):
    """A world `(x, y)` at `cross_m` metres left of the centreline.

    Goes through `from_track` rather than assuming `heading_deg == 0`, so it
    stays correct even if a caller overrides heading in `**extra`.
    """
    from_world = tn.from_track(cfg)
    return from_world(along_m, cross_m)


def test_wind_at_right_flank_blows_forward():
    cfg = _wind_cfg()
    hw = 0.5 * float(cfg["width_m"])
    x, y = _point_at_cross(cfg, -0.5 * hw)
    w = tn.wind_at(cfg, x, y)
    delta = abs(((w["bearing_deg"] - cfg["heading_deg"] + 180.0) % 360.0)
               - 180.0)
    assert delta <= 35.0, w


def test_wind_at_left_flank_blows_backward():
    cfg = _wind_cfg()
    hw = 0.5 * float(cfg["width_m"])
    x, y = _point_at_cross(cfg, 0.5 * hw)
    w = tn.wind_at(cfg, x, y)
    target = (cfg["heading_deg"] + 180.0) % 360.0
    delta = abs(((w["bearing_deg"] - target + 180.0) % 360.0) - 180.0)
    assert delta <= 35.0, w


def test_wind_at_over_flag():
    cfg = _wind_cfg()
    hw = 0.5 * float(cfg["width_m"])
    x0, y0 = _point_at_cross(cfg, 0.0)
    w0 = tn.wind_at(cfg, x0, y0)
    assert w0["over"] is True

    x1, y1 = _point_at_cross(cfg, 0.5 * hw)
    w1 = tn.wind_at(cfg, x1, y1)
    assert w1["over"] is False


def test_wind_at_speed_asymmetric_right_stronger():
    cfg = _wind_cfg()
    hw = 0.5 * float(cfg["width_m"])
    for frac in (0.35, 0.5, 0.7):
        xr, yr = _point_at_cross(cfg, -frac * hw)
        xl, yl = _point_at_cross(cfg, frac * hw)
        wr = tn.wind_at(cfg, xr, yr)
        wl = tn.wind_at(cfg, xl, yl)
        assert wr["speed_frac"] > wl["speed_frac"], (frac, wr, wl)


def test_wind_at_inflow_points_toward_centreline():
    """The wind vector's component along the LEFT axis comes entirely from
    the inflow term (the tangential and translation terms are both purely
    along-heading when heading_deg=0) — so its sign tells which way the
    inflow points. Positive `cross` (left flank) must inflow NEGATIVE (toward
    the centre, i.e. to the right); negative `cross` (right) must inflow
    POSITIVE."""
    cfg = _wind_cfg()
    hw = 0.5 * float(cfg["width_m"])

    xl, yl = _point_at_cross(cfg, 0.4 * hw)
    wl = tn.wind_at(cfg, xl, yl)
    # heading_deg = 0 -> world +y IS the left axis, so the wind's own wy is
    # the left-axis component directly.
    a_l, c_l = tn.frame(cfg)[0](xl, yl)
    assert c_l > 0.0
    speed = wl["speed_frac"] * (1.0 + float(cfg["translation_frac"])
                                * float(cfg["peak"]))
    wy_l = speed * math.sin(math.radians(wl["bearing_deg"]))
    assert wy_l < 0.0, wl

    xr, yr = _point_at_cross(cfg, -0.4 * hw)
    wr = tn.wind_at(cfg, xr, yr)
    speed_r = wr["speed_frac"] * (1.0 + float(cfg["translation_frac"])
                                  * float(cfg["peak"]))
    wy_r = speed_r * math.sin(math.radians(wr["bearing_deg"]))
    assert wy_r > 0.0, wr


def test_wind_at_outside_corridor_is_translation_only():
    cfg = _wind_cfg()
    x, y = _point_at_cross(cfg, 5000.0)
    w = tn.wind_at(cfg, x, y)
    assert w["speed_frac"] == 0.0
    assert math.isclose(w["bearing_deg"] % 360.0, cfg["heading_deg"] % 360.0,
                        rel_tol=0.0, abs_tol=1e-6)
    assert w["over"] is False


def test_wind_at_matches_intensity_field_cross_track_profile():
    """`wind_at`'s `speed_frac` has to fall to 0 at exactly the same edge
    `intensity_field` does (no edge noise in either — `edge_noise_m=0` here
    so they read the identical noiseless profile)."""
    cfg = _wind_cfg(edge_noise_m=0.0)
    region = (-800.0, -800.0, 800.0, 800.0)
    inten = tn.intensity_field(cfg, region, np.random.default_rng(3))
    hw = 0.5 * float(cfg["width_m"])
    for frac in (0.0, 0.5, 0.95, 1.05, 2.0):
        x, y = _point_at_cross(cfg, frac * hw)
        w = tn.wind_at(cfg, x, y)
        i = inten(x, y)
        if i <= 0.0:
            assert w["speed_frac"] == 0.0
        else:
            assert w["speed_frac"] > 0.0


if __name__ == "__main__":
    if "--snapshot" in sys.argv:
        os.makedirs(os.path.dirname(FIXTURE_PATH), exist_ok=True)
        snap = _full_snapshot()
        with open(FIXTURE_PATH, "w") as f:
            json.dump(snap, f, indent=1, sort_keys=True)
            f.write("\n")
        print("wrote {0}".format(FIXTURE_PATH))
    else:
        raise SystemExit(
            "run under pytest, or pass --snapshot to regenerate the fixture "
            "(only against code where the safety net is known to hold)")
