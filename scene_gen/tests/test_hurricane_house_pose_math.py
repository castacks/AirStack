"""Pure-python coverage for the geometry math behind `hurricane_flow`'s
pose-authoring of `roof_collapsed` / `partial_collapse` / `leveled` and the
rafter lattice for `cover_lost` / `deck_panels_lost` / `roof_stripped`.

`hurricane_flow.py` imports `pxr` only inside the functions that need it
(the module docstring is explicit about this), and `_yaw_pitch_for_direction`
/ `_rafter_specs_for_bay` are two of the functions that do not — they take
and return plain numbers/dicts, matching the SKILL's "Test idiom" section:
"a hurricane module must keep its scatter/field/ladder maths in pure-Python
functions". This is the offline gate for the rafter lattice; the USD-level
authoring itself (`_ridge_info`, `author_rafters`, the hinge poses) is
verified separately with bare pxr through `usd_python.sh` since it needs a
real mesh to read points from.
"""
import math
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_HERE)
sys.path.insert(0, _SCENE_GEN_DIR)

from disaster import hurricane_flow as hf  # noqa: E402

# The cottage roof bay's measured geometry (bare pxr, on the pod, against
# `house_cottage_pristine.usd`'s `house_roof_7_12`): ridge at x=0 spanning
# y in [-5.708, 5.708], eaves at x=+-5.557, base z=3.23, peak z=7.877.
_COTTAGE_RIDGE = dict(
    ridge_point=lambda run: (0.0, run, 7.877),
    eave_point=lambda run, e: (e, run, 3.23),
    run_lo=-5.708, run_hi=5.708, eave_lo=-5.557, eave_hi=5.557,
    ridge_coord=0.0, z_ridge=7.877, z_eave=3.23)


def test_yaw_pitch_flat_x_direction():
    """A purely horizontal +X direction: no pitch, yaw along +X."""
    yaw, pitch = hf._yaw_pitch_for_direction(1.0, 0.0, 0.0)
    assert yaw == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)


def test_yaw_pitch_straight_down():
    """Pure -Z: pitch is 90 (straight down), yaw is irrelevant/0 by
    convention (atan2(0, 0) == 0)."""
    yaw, pitch = hf._yaw_pitch_for_direction(0.0, 0.0, -1.0)
    assert pitch == pytest.approx(90.0)


def test_yaw_pitch_roundtrips_through_box_rotation_convention():
    """`planks._box`'s own matrix at roll=0: local +X maps to world
    `(cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), -sin(pitch))`. Feed a random
    direction through `_yaw_pitch_for_direction` and rebuild that same
    formula -- it must reproduce the direction (normalised) to float
    precision, or a rafter box would point the wrong way in the actual
    authored mesh."""
    import random
    rng = random.Random(11)
    for _ in range(200):
        d = (rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-3, 3))
        length = math.sqrt(sum(c * c for c in d))
        if length < 1e-6:
            continue
        ux, uy, uz = (c / length for c in d)
        yaw, pitch = hf._yaw_pitch_for_direction(*d)
        yr, pr = math.radians(yaw), math.radians(pitch)
        rx = math.cos(yr) * math.cos(pr)
        ry = math.sin(yr) * math.cos(pr)
        rz = -math.sin(pr)
        assert rx == pytest.approx(ux, abs=1e-6)
        assert ry == pytest.approx(uy, abs=1e-6)
        assert rz == pytest.approx(uz, abs=1e-6)


def test_rafter_specs_span_the_whole_ridge():
    """Rafters must reach both ends of the measured ridge run, not stop
    short of it -- a lattice that only covers the middle third of a bay
    reads as worse than no lattice (a gap where the roof plane should be)."""
    specs = hf._rafter_specs_for_bay(_COTTAGE_RIDGE)
    rafters = [s for s in specs if abs(s["l"] - 11.416) > 0.01]  # exclude ridge board
    ys = [s["y"] for s in rafters]
    assert min(ys) <= _COTTAGE_RIDGE["run_lo"] + 0.05
    assert max(ys) >= _COTTAGE_RIDGE["run_hi"] - 0.05


def test_rafter_specs_two_slopes_for_a_full_gable():
    """A full gable (both eaves present, on either side of the ridge) must
    produce rafters on BOTH sides -- one slope only reads as a lean-to."""
    specs = hf._rafter_specs_for_bay(_COTTAGE_RIDGE)
    xs = [s["x"] for s in specs if abs(s["z"] - _COTTAGE_RIDGE["z_ridge"]) > 0.1]
    assert any(x < -0.5 for x in xs), "no rafters on the negative-x slope"
    assert any(x > 0.5 for x in xs), "no rafters on the positive-x slope"


def test_rafter_specs_half_gable_gets_one_slope_only():
    """A half-gable bay (one eave coincides with the ridge, per
    `modular_house.ROOF_HALF_GABLE`'s own 'half width' shape) must not
    fabricate a slope on the missing side."""
    half = dict(_COTTAGE_RIDGE, eave_lo=0.0)  # eave collapsed onto the ridge
    specs = hf._rafter_specs_for_bay(half)
    xs = [s["x"] for s in specs if abs(s["z"] - half["z_ridge"]) > 0.1]
    assert all(x >= -0.05 for x in xs), (
        "a half-gable with no negative-side eave produced negative-side "
        "rafters anyway")
    assert any(x > 0.5 for x in xs)


def test_rafter_ridge_board_spans_the_run():
    """The ridge board (identified here as the longest spec -- it always
    equals the full run length, while every rafter is shorter, being the
    diagonal eave-to-ridge distance over a fraction of the bay's depth) must
    span the measured run, not a fixed/wrong length."""
    specs = hf._rafter_specs_for_bay(_COTTAGE_RIDGE)
    ridge_board = max(specs, key=lambda s: s["l"])
    expected = _COTTAGE_RIDGE["run_hi"] - _COTTAGE_RIDGE["run_lo"]
    assert ridge_board["l"] == pytest.approx(expected, abs=0.01)


def test_rafter_spacing_is_honoured():
    """Rafters land at roughly `RAFTER_SPACING_M` centres along the ridge,
    not some other arbitrary count."""
    specs = hf._rafter_specs_for_bay(_COTTAGE_RIDGE)
    span = _COTTAGE_RIDGE["run_hi"] - _COTTAGE_RIDGE["run_lo"]
    n_expected = max(2, round(span / hf.RAFTER_SPACING_M)) + 1
    # one slope's worth of rafter (not ridge-board) positions
    per_slope = [s for s in specs
                if s["x"] < -0.5 and abs(s["l"] - span) > 0.01]
    assert abs(len(per_slope) - n_expected) <= 1


# ---------------------------------------------------------------------------
# JOB A (2026-08-31 second pass) -- lock the collapse-pose ratios
# ---------------------------------------------------------------------------
# These are plain module constants (no pxr needed to read them) -- locking
# them here means a future retune of `hurricane_flow`'s collapse poses fails
# an offline test instead of silently drifting the look back toward the
# "propped-open lid" `~/hurricane_previews/ROUND1_L3/collapsed_house_obl.png`
# measured and the module docstring's "JOB A tuning" section explains.
def test_roof_collapse_scale_range_flattens_the_pitch():
    """`roof_collapsed` must SHRINK a roof bay's own height range toward its
    eave (both factors < 1) -- a scale >= 1 would not flatten anything, and
    the whole point of JOB A was replacing a pure hinge (which cannot
    flatten a pitch at all) with a scale that does."""
    lo, hi = hf.ROOF_COLLAPSE_SCALE_RANGE
    assert 0.0 < lo < hi < 1.0
    assert (lo, hi) == (0.30, 0.50)


def test_roof_collapse_tilt_is_small_relative_to_the_hinge():
    """The tilt is COSMETIC, not the mechanism -- its nominal range must stay
    well under a quarter turn, and `ROOF_COLLAPSE_HEIGHT_FRAC` must be
    strictly under 1.0 (a cap of 1.0 or more would let a bay stay at its
    full pristine ridge height, which is exactly the bug JOB A fixed)."""
    lo, hi = hf.ROOF_COLLAPSE_TILT_RANGE_DEG
    assert 0.0 < lo < hi <= 20.0
    assert 0.0 < hf.ROOF_COLLAPSE_HEIGHT_FRAC < 1.0
    assert hf.ROOF_COLLAPSE_HEIGHT_FRAC == pytest.approx(0.75)


def test_roof_collapse_drop_stays_within_wall_top_band():
    """The settle target is "within [wall_top - 0.6, wall_top]" -- the drop
    range's own span IS that 0.6 m, starting at zero (never lifts the eave
    ABOVE its own pristine wall-top rest height)."""
    lo, hi = hf.ROOF_COLLAPSE_DROP_RANGE_M
    assert lo == 0.0
    assert hi == pytest.approx(0.6)


def test_leveled_roof_scale_is_flatter_than_roof_collapsed():
    """`leveled` is a WORSE state than `roof_collapsed` and must flatten the
    roof MORE (a smaller scale ceiling) -- JOB A's "z-scaled <= 0.3"."""
    assert hf.LEVELED_ROOF_SCALE_RANGE[1] <= 0.30
    assert hf.LEVELED_ROOF_SCALE_RANGE[1] <= hf.ROOF_COLLAPSE_SCALE_RANGE[0]


def test_leveled_roof_and_wall_caps_are_absolute_and_small():
    """`leveled` reads as a full pancake collapse near the true GROUND, so
    both caps are small ABSOLUTE numbers (not fractions of the original
    ridge/wall height the way `roof_collapsed`'s cap is) -- JOB A's "max z
    <= 2.5 m" / "max z <= 0.6 m"."""
    assert hf.LEVELED_ROOF_MAX_Z_M == pytest.approx(2.5)
    assert hf.LEVELED_WALL_MAX_Z_M == pytest.approx(0.6)
    # every style's single-storey wall height (~3.5 m) and two-storey
    # height (~7 m) are both well above this cap -- confirms it is doing
    # real flattening work, not a no-op ceiling.
    assert hf.LEVELED_WALL_MAX_Z_M < 3.5


def test_partial_wall_rack_angle_matches_leveled_or_less():
    """`partial_collapse` racks its windward-row walls "outward" like
    `leveled` does, but is a LESS severe rung -- its angle range's floor
    must not exceed `leveled`'s (both ranges top out near-vertical-to-flat,
    but `partial_collapse`'s floor is allowed to be shallower)."""
    p_lo, p_hi = hf.PARTIAL_WALL_RACK_RANGE_DEG
    l_lo, l_hi = hf.LEVELED_WALL_RACK_RANGE_DEG
    assert p_lo <= l_lo
    assert p_hi <= l_hi <= 95.0


def test_partial_wall_cap_is_looser_than_leveled():
    """A `partial_collapse` racked wall is allowed to stand TALLER than a
    `leveled` one (1.2 m vs 0.6 m) -- `leveled` is the more total failure."""
    assert hf.PARTIAL_WALL_MAX_Z_M > hf.LEVELED_WALL_MAX_Z_M
    assert hf.PARTIAL_WALL_MAX_Z_M == pytest.approx(1.2)


def test_partial_row_tolerance_covers_the_measured_villa_and_terrace_gap():
    """`villa` (single hip-roof mesh) and `terrace` (row-house) both measured
    a 0.625-0.626 m gap between their own roof footprint and the house's
    wall-derived min-Y -- the OLD 0.5 m tolerance missed both, leaving their
    `partial_collapse` roof untouched at the racked end. The new tolerance
    must clear that measured gap with room, but stay well under the next
    tier the 8-style measurement found (`terrace`'s second roof piece, at
    1.334 m) so it cannot accidentally pull in the wrong row."""
    assert hf.PARTIAL_ROW_TOL_M > 0.626
    assert hf.PARTIAL_ROW_TOL_M < 1.334


# ---------------------------------------------------------------------------
# JOB B -- windward_variant
# ---------------------------------------------------------------------------
def test_windward_variant_four_cardinal_cases():
    """At street yaw 0 (unrotated, local == world), each cardinal wind
    bearing must pick the LOCAL side whose outward face is directly hit by
    that wind -- wind toward north (0) hits the south-facing (`'s'`) side,
    toward east (90) hits west (`'w'`), toward south (180) hits north
    (`'n'`), toward west (270) hits east (`'e'`)."""
    assert hf.windward_variant(0.0, 0.0) == "s"
    assert hf.windward_variant(0.0, 90.0) == "w"
    assert hf.windward_variant(0.0, 180.0) == "n"
    assert hf.windward_variant(0.0, 270.0) == "e"


def test_windward_variant_rotates_with_the_house_yaw():
    """A house placed at street yaw 90 (rotated a quarter turn) must pick a
    DIFFERENT local side than the same wind bearing would at yaw 0 -- the
    whole point of the function is correcting for the house's own facing,
    not just reading the wind bearing directly."""
    at_yaw0 = hf.windward_variant(0.0, 0.0)
    at_yaw90 = hf.windward_variant(90.0, 0.0)
    assert at_yaw0 != at_yaw90
    # Hand-derived: wind toward 0 (from due south, world bearing 180) hits
    # whichever local side's own base bearing, after a +90 deg house
    # rotation, equals 180 -- `_SIDE_BASE_BEARING["w"] == 270`, and
    # `(270 - 90) % 360 == 180` -- so the local WEST face (`'w'`) takes it.
    assert at_yaw90 == "w"


def test_windward_variant_two_diagonal_cases():
    """Two non-cardinal (yaw, wind) pairs, hand-derived from the same
    `(target + yaw) % 360` algebra `windward_variant` itself documents,
    each landing unambiguously closer to one side than any other (no
    tie)."""
    # yaw=30, wind toward 200 (from world bearing 20):
    # needed_b0 = (20 + 30) % 360 = 50 -- closest of {0,90,180,270} is
    # 90 ("e", |50-90|=40) over 0 ("n", |50-0|=50).
    assert hf.windward_variant(30.0, 200.0) == "e"
    # yaw=200, wind toward 10 (from world bearing 190):
    # needed_b0 = (190 + 200) % 360 = 30 -- closest of {0,90,180,270} is 0 ("n").
    assert hf.windward_variant(200.0, 10.0) == "n"


def test_windward_variant_always_one_of_the_four_sides():
    """A sweep of yaw/wind combinations, including values outside
    [0, 360), must always resolve to one of the four `VARIANTS` -- never
    raise, never return something else."""
    import random
    rng = random.Random(5)
    for _ in range(50):
        yaw = rng.uniform(-400.0, 760.0)
        wind = rng.uniform(-400.0, 760.0)
        assert hf.windward_variant(yaw, wind) in hf.VARIANTS


def test_variant_seed_dir_matches_the_bake_time_algebra():
    """`_VARIANT_SEED_DIR[side]` is what `strip_roof`'s own `seed_dir` must
    be at BAKE TIME (house at yaw 0) for the dropped bays to land on that
    LOCAL side -- `(seed_dir + 180) % 360` must equal the side's own base
    bearing (`_SIDE_BASE_BEARING`), by `strip_roof`'s documented
    convention."""
    for side, b0 in hf._SIDE_BASE_BEARING.items():
        seed_dir = hf._VARIANT_SEED_DIR[side]
        assert (seed_dir + 180.0) % 360.0 == pytest.approx(b0 % 360.0)


def test_levels_with_variants_excludes_roof_stripped():
    """`roof_stripped` drops EVERY bay (`_ROOF_FRAC["roof_stripped"] ==
    (1.00, 0.45)`), so it has no windward SIDE left to place a variant on --
    only the two partial-loss rungs get cardinal variants."""
    assert set(hf.LEVELS_WITH_VARIANTS) == {"cover_lost", "deck_panels_lost"}
    assert hf._ROOF_FRAC["roof_stripped"][0] == pytest.approx(1.0)
    assert set(hf.VARIANTS) == {"n", "e", "s", "w"}



# ---------------------------------------------------------------------------
# JOB A review fix (2026-08-31 third pass) -- the upper-storey FLOOR slab
# ---------------------------------------------------------------------------
# Coordinator review, measured on `house_l_family_leveled.usd`: every wall
# was correctly flattened to `LEVELED_WALL_MAX_Z_M`, but `house_floor` still
# spanned up to 3.51 m -- the upper-storey slab those walls used to hold up
# was never posed at all and hung in mid-air with nothing left under it.
def test_floor_caps_are_absolute_and_under_1_2m():
    """Both `PARTIAL_FLOOR_MAX_Z_M` and `LEVELED_FLOOR_MAX_Z_M` are the
    coordinator's own "max z <= 1.2 m" figure, and BOTH are absolute (not
    relative to a still-standing floor the way `PARTIAL_WALL_MAX_Z_M` is) --
    an upper floor that needs posing at all has, by definition, lost its
    support, so there is no intact reference storey left to be relative
    to."""
    assert hf.PARTIAL_FLOOR_MAX_Z_M == pytest.approx(1.2)
    assert hf.LEVELED_FLOOR_MAX_Z_M == pytest.approx(1.2)
    assert hf.PARTIAL_FLOOR_MAX_Z_M <= 1.2 + 1e-9
    assert hf.LEVELED_FLOOR_MAX_Z_M <= 1.2 + 1e-9


def test_floor_tilt_matches_the_coordinators_3_to_8_degree_band():
    """The coordinator specified "tilted 3-8 degrees" for the dropped
    floor slabs -- both levels' tilt ranges must match exactly, and stay
    well under a quarter turn the same way the roof tilt ranges do."""
    assert hf.PARTIAL_FLOOR_TILT_RANGE_DEG == (3.0, 8.0)
    assert hf.LEVELED_FLOOR_TILT_RANGE_DEG == (3.0, 8.0)


def test_floor_row_tolerance_clears_the_measured_5m_grid_gap():
    """`house_floor` is laid out on a coarser grid than the wall/roof
    pieces -- measured 1.334 m from the house's own min-Y to the nearest
    floor block's min-Y on every multi-storey style (`l_family`,
    `two_storey`, `wide_house`, `terrace`), well past `PARTIAL_ROW_TOL_M`
    (0.7 m, sized for the WALL/ROOF grid). `PARTIAL_FLOOR_ROW_TOL_M` must
    clear that gap with room, but stay under the next tier the same
    8-style measurement found (6.334 m) so it cannot pull in a floor block
    from the wrong end of the house."""
    assert hf.PARTIAL_FLOOR_ROW_TOL_M > 1.334
    assert hf.PARTIAL_FLOOR_ROW_TOL_M < 6.334


def test_leveled_floor_scale_shrinks_like_the_roof_and_wall_do():
    """The floor squash must actually shrink (both factors < 1), same
    invariant `test_roof_collapse_scale_range_flattens_the_pitch` locks
    for the roof -- a scale >= 1 would not be a collapse pose at all."""
    for lo, hi in (hf.PARTIAL_FLOOR_SCALE_RANGE, hf.LEVELED_FLOOR_SCALE_RANGE):
        assert 0.0 < lo < hi < 1.0


# ---------------------------------------------------------------------------
# RAGGED RAFTERS -- break the perfect cage (STREAM Q, 2026-08-31)
# ---------------------------------------------------------------------------
# The coordinator's review of the shipped library found `roof_stripped` /
# `deck_panels_lost` houses wearing a COMPLETE, perfectly regular rafter
# cage -- "every rafter present, no sheathing remnants ... reads as
# under-construction rather than storm-stripped". These tests lock the
# fix's invariants: seeded determinism, fractions in range, at least one
# rafter always survives, and the snapped-rafter geometry stays physically
# sane (shorter, and tilted MORE downward, pivoting from its own ridge end).
def _dummy_rafters(n, rng=None):
    """`n` synthetic common-rafter specs (no ridge board mixed in), spanning
    a ridge along local Y at x=0 -- enough shape for `_raggedize_rafters`/
    `_snap_rafter` to operate on without needing a real `_ridge_info`."""
    rng = rng or __import__("random").Random(0)
    specs = []
    for i in range(n):
        y = -5.0 + 10.0 * i / max(1, n - 1)
        dx, dy, dz = 5.5, 0.0, -4.5   # ridge -> eave, matching the cottage
        yaw, pitch = hf._yaw_pitch_for_direction(dx, dy, dz)
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        specs.append({
            "class": "rafter", "x": 2.75, "y": y, "z": 5.7 - 2.25,
            "l": length, "w": hf.RAFTER_W_M, "t": hf.RAFTER_T_M,
            "yaw": yaw, "pitch": pitch, "roll": 0.0})
    return specs


def test_raggedize_rafters_is_deterministic_for_a_given_seed():
    """The SAME seed must produce the SAME break pattern (which indices are
    removed/snapped, and the snapped pieces' own new geometry) -- two
    independent `Random` objects seeded identically, fed the identical
    input list, must agree exactly."""
    import random
    specs = _dummy_rafters(40)
    out1, stats1 = hf._raggedize_rafters(specs, random.Random(42))
    out2, stats2 = hf._raggedize_rafters(specs, random.Random(42))
    assert stats1 == stats2
    assert len(out1) == len(out2)
    for a, b in zip(out1, out2):
        assert a == b


def test_raggedize_rafters_different_seeds_usually_differ():
    """Sanity check the determinism test isn't vacuously true because the
    function ignores `rng` -- a different seed must (with overwhelming
    likelihood on 40 rafters) produce a different pattern."""
    import random
    specs = _dummy_rafters(40)
    out1, _ = hf._raggedize_rafters(specs, random.Random(1))
    out2, _ = hf._raggedize_rafters(specs, random.Random(2))
    assert out1 != out2


def test_raggedize_rafters_fractions_in_range():
    """Removed/snapped SHARES must land in the documented ranges -- run
    enough trials on a large enough rafter count that rounding at small n
    cannot dominate the check."""
    import random
    rng = random.Random(7)
    for _ in range(200):
        n = rng.randint(20, 60)
        specs = _dummy_rafters(n)
        _out, stats = hf._raggedize_rafters(specs, rng)
        assert stats["kept"] + stats["removed"] + stats["snapped"] == n
        remove_frac = stats["removed"] / n
        snap_frac = stats["snapped"] / n
        # +-1 rafter of slack for the rounding `int(round(frac * n))` does,
        # expressed as a fraction of n.
        slack = 1.5 / n
        lo, hi = hf.RAFTER_REMOVE_FRAC_RANGE
        assert lo - slack <= remove_frac <= hi + slack
        lo, hi = hf.RAFTER_SNAP_FRAC_RANGE
        assert lo - slack <= snap_frac <= hi + slack


def test_raggedize_rafters_never_empties_a_small_bay():
    """A tiny bay (n=1 or 2) must never lose every rafter -- a hole with
    NOTHING in it reads as a bug in the lattice generator, not storm
    damage."""
    import random
    rng = random.Random(3)
    for n in (1, 2, 3):
        for _ in range(50):
            specs = _dummy_rafters(n)
            out, stats = hf._raggedize_rafters(specs, rng)
            assert len(out) >= 1
            assert stats["kept"] + stats["snapped"] >= 1


def test_raggedize_rafters_never_mutates_its_input():
    specs = _dummy_rafters(10)
    import copy
    before = copy.deepcopy(specs)
    hf._raggedize_rafters(specs, __import__("random").Random(9))
    assert specs == before


def test_snap_rafter_shortens_and_tilts_further_down():
    """A snapped rafter must come back SHORTER than the original (within
    `RAFTER_SNAP_KEEP_RANGE`) and pitched MORE steeply downward (per
    `_yaw_pitch_for_direction`'s convention, a larger pitch means more
    downward), never less."""
    import random
    rng = random.Random(11)
    spec = _dummy_rafters(1)[0]
    for _ in range(30):
        snapped = hf._snap_rafter(spec, rng)
        lo, hi = hf.RAFTER_SNAP_KEEP_RANGE
        assert lo * spec["l"] - 1e-6 <= snapped["l"] <= hi * spec["l"] + 1e-6
        assert snapped["pitch"] > spec["pitch"]
        assert snapped["yaw"] == pytest.approx(spec["yaw"])


def test_snap_rafter_keeps_its_ridge_end_fixed():
    """The snapped piece must still hinge from the SAME ridge-end point the
    original rafter had -- a snap that also slides the attachment point
    around would read as the rafter teleporting, not breaking."""
    import random
    rng = random.Random(13)
    spec = _dummy_rafters(1)[0]

    def _ridge_end(s):
        yr, pr = math.radians(s["yaw"]), math.radians(s["pitch"])
        dirx = math.cos(yr) * math.cos(pr)
        diry = math.sin(yr) * math.cos(pr)
        dirz = -math.sin(pr)
        return (s["x"] - 0.5 * s["l"] * dirx, s["y"] - 0.5 * s["l"] * diry,
               s["z"] - 0.5 * s["l"] * dirz)

    orig_end = _ridge_end(spec)
    for _ in range(10):
        snapped = hf._snap_rafter(spec, rng)
        new_end = _ridge_end(snapped)
        for a, b in zip(orig_end, new_end):
            assert a == pytest.approx(b, abs=1e-6)


def _far_end(s):
    """The end of spec *s* OPPOSITE its ridge pivot (`+0.5*l*dir`, the
    mirror of `test_snap_rafter_keeps_its_ridge_end_fixed`'s `_ridge_end`)
    -- the ORIGINAL rafter's own eave point, or a snapped rafter's own free
    (drooping) end, depending on which spec is passed in."""
    yr, pr = math.radians(s["yaw"]), math.radians(s["pitch"])
    dirx = math.cos(yr) * math.cos(pr)
    diry = math.sin(yr) * math.cos(pr)
    dirz = -math.sin(pr)
    return (s["x"] + 0.5 * s["l"] * dirx, s["y"] + 0.5 * s["l"] * diry,
           s["z"] + 0.5 * s["l"] * dirz)


def test_snap_rafter_never_drops_more_than_max_overshoot_below_original_eave():
    """The measured bug: `terrace/cover_lost`'s `n`/`e` cardinal variants
    (a SHALLOW-pitched style) put a snapped rafter's free end at -0.35 to
    -0.55 m -- below the house's own ground floor -- when a high `keep`
    draw and a large tilt draw compounded. The free end must never sit more
    than `RAFTER_SNAP_MAX_OVERSHOOT_M` below where the ORIGINAL, un-snapped
    rafter's own eave end was, on EITHER a normal-pitch rafter (the cottage-
    like dummy) or a shallow one (~9.5 deg, closer to the style that
    actually broke) -- enough trials to exercise the full (keep, tilt) draw
    space, including its extremes."""
    import random
    rng = random.Random(17)
    normal = _dummy_rafters(1)[0]
    dx, dy, dz = 6.0, 0.0, -1.0   # ~9.5 deg -- a shallow roof pitch
    yaw, pitch = hf._yaw_pitch_for_direction(dx, dy, dz)
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    shallow = {"class": "rafter", "x": 0.0, "y": 0.0, "z": 3.0, "l": length,
              "w": hf.RAFTER_W_M, "t": hf.RAFTER_T_M, "yaw": yaw,
              "pitch": pitch, "roll": 0.0}

    for spec in (normal, shallow):
        original_eave_z = _far_end(spec)[2]
        for _ in range(300):
            snapped = hf._snap_rafter(spec, rng)
            free_end_z = _far_end(snapped)[2]
            assert free_end_z >= (original_eave_z
                                  - hf.RAFTER_SNAP_MAX_OVERSHOOT_M - 1e-6)


def test_rafter_specs_for_bay_rng_none_is_byte_identical_to_before():
    """Regression guard for the `_rafter_and_ridge_specs` refactor: calling
    without `rng` must reproduce EXACTLY what this function returned before
    raggedness existed."""
    specs = hf._rafter_specs_for_bay(_COTTAGE_RIDGE)
    common, ridge = hf._rafter_and_ridge_specs(_COTTAGE_RIDGE)
    assert specs == common + ([ridge] if ridge is not None else [])


def test_rafter_specs_for_bay_with_rng_drops_the_removed_share():
    """With `rng`, the returned list must be shorter than the full cage by
    exactly the number of rafters `_raggedize_rafters` removed (snapped
    ones stay in the list, just transformed) -- and the ridge board must
    still be present."""
    import random
    full = hf._rafter_specs_for_bay(_COTTAGE_RIDGE)
    ragged = hf._rafter_specs_for_bay(_COTTAGE_RIDGE, rng=random.Random(21))
    assert len(ragged) < len(full)
    ridge_full = max(full, key=lambda s: s["l"])
    assert any(s["l"] == pytest.approx(ridge_full["l"]) for s in ragged), (
        "the ridge board must survive raggedizing intact")


# ---------------------------------------------------------------------------
# Ragged sheathing patches / ridge shreds placement (pure geometry)
# ---------------------------------------------------------------------------
def _unit(v):
    return math.sqrt(sum(c * c for c in v))


def test_patch_placement_axes_are_orthonormal():
    """`u_axis`/`v_axis`/`n_axis` must each be unit length and mutually
    perpendicular -- `_ragged_patch_points` relies on this basis being
    orthonormal to place vertices in the right plane."""
    import random
    rng = random.Random(5)
    for _ in range(50):
        placement = hf._patch_placement_on_bay(_COTTAGE_RIDGE, rng)
        assert placement is not None
        u, v, n = placement["u_axis"], placement["v_axis"], placement["n_axis"]
        for axis in (u, v, n):
            assert _unit(axis) == pytest.approx(1.0, abs=1e-6)
        assert sum(a * b for a, b in zip(u, v)) == pytest.approx(0.0, abs=1e-6)
        assert sum(a * b for a, b in zip(u, n)) == pytest.approx(0.0, abs=1e-6)
        assert sum(a * b for a, b in zip(v, n)) == pytest.approx(0.0, abs=1e-6)


def test_patch_placement_center_lies_between_ridge_and_eave():
    """The anchor point must lie strictly between the ridge and the eave it
    was drawn against (t in (0, 1)), never past either end."""
    import random
    rng = random.Random(6)
    for _ in range(50):
        placement = hf._patch_placement_on_bay(_COTTAGE_RIDGE, rng,
                                               t_range=(0.15, 0.85))
        cz = placement["center"][2]
        assert _COTTAGE_RIDGE["z_eave"] < cz < _COTTAGE_RIDGE["z_ridge"]


def test_patch_placement_ridge_shred_range_stays_near_the_ridge():
    """A ridge-shred `t_range` (near 0) must land the anchor much closer to
    the ridge z than to the eave z -- otherwise a "shred on the ridge
    board" would actually land halfway down the slope."""
    import random
    rng = random.Random(8)
    span = _COTTAGE_RIDGE["z_ridge"] - _COTTAGE_RIDGE["z_eave"]
    for _ in range(50):
        placement = hf._patch_placement_on_bay(_COTTAGE_RIDGE, rng,
                                               t_range=(0.0, 0.12))
        cz = placement["center"][2]
        assert cz > _COTTAGE_RIDGE["z_ridge"] - 0.12 * span - 1e-6


def test_ragged_patch_points_shape_and_size():
    """`_ragged_patch_points` must return `n_verts` top/bottom points each,
    no NaN/Inf, roughly the requested radius (within the jag budget), and
    the top/bottom pair at each vertex separated by exactly the requested
    thickness along the normal."""
    import random
    rng = random.Random(9)
    placement = hf._patch_placement_on_bay(_COTTAGE_RIDGE, rng)
    radius_u, radius_v, half_t, n_verts = 0.6, 0.4, 0.01, 8
    top, bottom = hf._ragged_patch_points(
        placement, radius_u, radius_v, half_t, n_verts, 0.35, rng)
    assert len(top) == len(bottom) == n_verts
    center = placement["center"]
    n_axis = placement["n_axis"]
    for t, b in zip(top, bottom):
        for c in t + b:
            assert not math.isnan(c) and not math.isinf(c)
        # top - bottom must be (2*half_t) along n_axis
        diff = tuple(t[k] - b[k] for k in range(3))
        assert _unit(diff) == pytest.approx(2 * half_t, abs=1e-6)
        for k in range(3):
            assert diff[k] == pytest.approx(2 * half_t * n_axis[k], abs=1e-6)
        # distance from the ANCHOR CENTRE (not the offset top/bottom plane)
        # must stay within the jag budget of the larger nominal radius
        dist = math.sqrt(sum((t[k] - center[k]) ** 2 for k in range(3)))
        assert dist <= max(radius_u, radius_v) * 1.4
