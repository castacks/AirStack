#!/usr/bin/env python3
"""test_review_points_cameras.py — STREAM C's offline verification for the
hurricane review cameras (`.agents/skills/build-hurricane-scenes/
SESSION_2026-08-31.md` S3/S4, "review cameras frame the wrong things, and
there are no close views").

    cd scene_gen && python -m pytest tests/test_review_points_cameras.py -q

Pure Python, host-side. `disaster.surge`/`disaster.hurricane`/`disaster.
washaway` import no `pxr` at module scope. `simulation/isaac-sim/utils/
snapshots_rp.py` DOES (`import carb`, `from pxr import Gf, Sdf, UsdGeom` —
it exists to run inside Kit), so it is reached here only through `scene_gen/
tools/hurricane_cameras_png.py`, which stubs both the `tornado_png.py:40-56`
way (see that module's own docstring for the full argument) before
importing it. No Isaac Sim anywhere in this file.

WHAT IS PINNED HERE

  deliverable A   `surge.review_points` keeps its three subjects
                  (`shoreline`/`deep_water`/`dry_inland`) >= `edge_margin_m`
                  (default 70 m) inside the plate on EVERY side — measured
                  against the REAL V2_L2/V2_L3 ground truth, where the OLD
                  behaviour put `deep_water`/`dry_inland` 5.2 m from a
                  corner (see the regression check below); `_clear_azimuth`
                  solves tree-avoidance and the "look toward the plate
                  centre" constraint JOINTLY, not one after the other, and
                  stays backward compatible when `region`/`avoid` are not
                  given; `_cap_oblique_range` shrinks the oblique's range
                  when the frame's top edge would otherwise ray-trace past
                  the plate.
  deliverable B   `windward_azimuth_deg`'s bake-convention algebra;
                  `densest_cluster`'s grid-binning; `flooded_street_point`'s
                  house-pair heuristic; `select_review_subjects` end to end
                  on the real V2_L2 (which is missing several damage levels
                  outright — the SESSION doc's "still wrong" #2) and V2_L3
                  ground truth, including that every subject it emits stays
                  clear of the plate edge after the full camera-geometry
                  solve (`still_bad is False`).
  deliverable C   `_offplate_fraction`'s colour-band gate and
                  `_flag_offplate`'s geometric "aimed off-plate" gate, both
                  ways (fires / does not fire).
"""
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
_TOOLS_DIR = os.path.join(_SCENE_GEN, "tools")
sys.path.insert(0, _SCENE_GEN)
sys.path.insert(0, _TOOLS_DIR)

import pytest                                                  # noqa: E402

from disaster import surge as sgw                              # noqa: E402
from disaster import hurricane as hu                           # noqa: E402

import hurricane_cameras_png as hcp                             # noqa: E402
hcp._ensure_plot_deps()                                          # noqa: E402
srp = hcp.srp                                                   # snapshots_rp, stubbed pxr/carb

_HOME = os.path.expanduser("~")
_GT_L3 = os.path.join(_HOME, "hurricane_previews", "V2_L3", "GT_hurricane.json")
_GT_L2 = os.path.join(_HOME, "hurricane_previews", "V2_L2", "GT_hurricane.json")


def _load_gt(path):
    if not os.path.isfile(path):
        pytest.skip("{0} not present on this machine".format(path))
    with open(path) as fh:
        return json.load(fh)


# ---------------------------------------------------------------------------
# deliverable A -- review_points margin + _clear_azimuth + _cap_oblique_range
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("gt_path", [_GT_L3, _GT_L2])
def test_review_points_keeps_margin_from_every_edge(gt_path):
    gt = _load_gt(gt_path)
    region = tuple(gt["region"])
    scfg = gt["surge"]
    pts = sgw.review_points(scfg, region)
    for name in ("shoreline", "deep_water", "dry_inland"):
        assert name in pts, name
        m = hcp.edge_margin(region, *pts[name])
        # a hair under 70 for grid-discretisation slack (48x48 lattice over
        # an inset span -- see `review_points`'s own docstring)
        assert m >= 69.0, "{0}: margin {1:.1f}m < 70m ({2})".format(
            name, m, gt_path)


def test_review_points_margin_is_a_real_fix_not_a_no_op():
    """The regression this whole deliverable exists for: the OLD behaviour
    (`edge_margin_m=0`) put `deep_water`/`dry_inland` 5.2 m from the V2_L3
    plate's corner -- verified against the real ground truth, not a
    synthetic case, so a change that silently no-ops the margin (the
    "verify the control first" lesson in this repo's own method notes)
    cannot pass this test by accident."""
    gt = _load_gt(_GT_L3)
    region = tuple(gt["region"])
    scfg = gt["surge"]
    old = sgw.review_points(scfg, region, edge_margin_m=0.0)
    new = sgw.review_points(scfg, region)
    for name in ("deep_water", "dry_inland"):
        old_m = hcp.edge_margin(region, *old[name])
        new_m = hcp.edge_margin(region, *new[name])
        assert old_m < 10.0, "fixture assumption broke: {0}".format(name)
        assert new_m >= 69.0
        assert new_m > old_m + 50.0


def test_review_points_respects_region_bounds_and_caps_margin():
    """A small region must not invert the inset (49% cap): a 40 m region
    asked for a 70 m margin still returns points INSIDE the region, close
    to the capped 19.6 m (49% of the 20 m half-extent), not an empty or
    inverted search box."""
    region = (-20.0, -20.0, 20.0, 20.0)
    scfg = sgw.resolve_cfg({})
    pts = sgw.review_points(scfg, region, edge_margin_m=70.0)
    assert pts                                        # the search box must not invert
    for name, (x, y) in pts.items():
        assert region[0] <= x <= region[2]
        assert region[1] <= y <= region[3]
        m = hcp.edge_margin(region, x, y)
        assert m >= 0.49 * (region[2] - region[0]) - 1.0


def test_clear_azimuth_backward_compatible_with_no_region():
    """`region=None` must reproduce the pre-existing tree-avoidance-only
    contract EXACTLY -- every non-hurricane `views_around` caller passes no
    `region` and must see no behaviour change."""
    assert srp._clear_azimuth(0.0, 0.0, [], 45.0, 1.0, 225.0) == 225.0
    assert srp._clear_azimuth(0.0, 0.0, None, 45.0, 1.0, 225.0) == 225.0
    # one tree sitting on the preferred bearing -- must rotate off it and
    # clear by `_CLEAR_M`
    az = srp._clear_azimuth(0.0, 0.0, [(45.0 * math.cos(math.radians(225.0)),
                                       45.0 * math.sin(math.radians(225.0)))],
                            45.0, 1.0, 225.0)
    assert az != 225.0
    a = math.radians(az)
    cx, cy = 45.0 * math.cos(a), 45.0 * math.sin(a)
    d = math.hypot(cx - 45.0 * math.cos(math.radians(225.0)),
                   cy - 45.0 * math.sin(math.radians(225.0)))
    assert d >= srp._CLEAR_M - 1e-6


def test_blank_frame_retry_poses_escape_exact_oblique_position():
    eye = (10.0, -20.0, 8.0)
    target = (0.0, 0.0, 1.0)
    poses = srp._retry_camera_poses(eye, target)
    assert len(poses) == 3
    assert len({tuple(round(v, 6) for v in p[0]) for p in poses}) == 3
    for retry_eye, retry_target in poses:
        assert retry_target == target
        assert retry_eye != eye
        assert retry_eye[2] > eye[2]


def test_blank_frame_retry_poses_keep_nadir_near_subject():
    eye = (4.0, 9.0, 100.0)
    target = (4.0, 9.0, 0.0)
    for retry_eye, retry_target in srp._retry_camera_poses(eye, target):
        assert retry_target == target
        assert math.hypot(retry_eye[0] - eye[0],
                          retry_eye[1] - eye[1]) <= 1.01
        assert retry_eye[2] > eye[2]


def test_clear_azimuth_looks_inward_after_tree_avoidance():
    """A subject near the NE corner: the joint solve must land within the
    +/-60 deg inward cone even though a tree sits squarely on the naive
    "closest to preferred" candidate -- i.e. inward-ness is not sacrificed
    just because SOME candidate clears the tree."""
    region = (-250.0, -250.0, 250.0, 250.0)
    x, y = 200.0, 200.0
    inward = srp._inward_bearing_deg(x, y, region)
    assert abs(inward - 45.0) < 1e-6
    # a tree parked right on the inward bearing's own camera slot
    a = math.radians(inward)
    tree = (x + 45.0 * math.cos(a), y + 45.0 * math.sin(a))
    az = srp._clear_azimuth(x, y, [tree], 45.0, 1.0, 225.0, region=region)
    assert srp._ang_dist(az, inward) <= 60.0 + 1e-6, (
        "az {0} is not within the inward cone of {1}".format(az, inward))


def test_clear_azimuth_prefers_lighting_when_both_constraints_satisfied():
    """When `preferred` already satisfies both constraints, it must win
    outright (no gratuitous rotation away from the lighting-preserving
    bearing)."""
    region = (-250.0, -250.0, 250.0, 250.0)
    az = srp._clear_azimuth(0.0, 0.0, [], 45.0, 1.0, 225.0, region=region)
    assert az == 225.0


def test_cap_oblique_range_shrinks_when_top_ray_overshoots():
    region = (-250.0, -250.0, 250.0, 250.0)
    x, y = 235.0, 0.0                 # 15 m from the +X edge
    az = 180.0                       # camera to the west, looking EAST -- outward
    d_use, capped, still_bad = srp._cap_oblique_range(
        x, y, az, 28.0, 21.1, 1.5, region)
    assert capped
    assert d_use < 28.0
    assert d_use >= 0.3 * 28.0 - 1e-6


def test_cap_oblique_range_noop_when_already_inside():
    region = (-250.0, -250.0, 250.0, 250.0)
    d_use, capped, still_bad = srp._cap_oblique_range(
        0.0, 0.0, 225.0, 45.0, 22.0, 1.0, region)
    assert not capped
    assert not still_bad
    assert d_use == 45.0


def test_cap_oblique_range_is_a_noop_with_no_region():
    d_use, capped, still_bad = srp._cap_oblique_range(
        235.0, 0.0, 180.0, 28.0, 21.1, 1.5, None)
    assert d_use == 28.0 and not capped and not still_bad


def test_half_vfov_matches_horizontal_since_frame_is_square():
    # RES is square, so the vertical FOV a render actually shows equals the
    # horizontal one computed from the one aperture `place_camera` sets.
    half = srp._half_vfov_deg(18.0)
    expect = math.degrees(math.atan(20.955 / 36.0))
    assert abs(half - expect) < 1e-9


def test_top_ray_ground_point_none_when_above_horizon():
    # the ORIGINAL default framing (45 m / 22 m / aim 1 m) points its top
    # edge above the horizon at an 18mm lens -- nothing to cap.
    half = srp._half_vfov_deg(18.0)
    pt = srp._top_ray_ground_point(0.0, 0.0, 225.0, 45.0, 22.0, 1.0, half)
    assert pt is None


# ---------------------------------------------------------------------------
# deliverable B -- the 5 close subjects
# ---------------------------------------------------------------------------

def test_windward_azimuth_deg_matches_the_bake_algebra():
    # `_bearing_of(yaw) = (180 - yaw) % 360` (compass) converts to this
    # module's math convention as `(yaw - 90) % 360` -- spelled out for the
    # four cardinal placements the bake docstring itself uses.
    assert hcp.windward_azimuth_deg(0.0) == 270.0
    assert hcp.windward_azimuth_deg(90.0) == 0.0
    assert hcp.windward_azimuth_deg(180.0) == 90.0
    assert hcp.windward_azimuth_deg(270.0) == 180.0
    assert hcp.windward_azimuth_deg(-40.0) == (-40.0 - 90.0) % 360.0
    # per-house cardinal variants (H2b): the damaged side is `variant`, not -Y
    assert hcp.windward_azimuth_deg(0.0, "cover_lost", "n") == 90.0    # +Y face -> camera north of it
    assert hcp.windward_azimuth_deg(0.0, "cover_lost", "s") == 270.0   # same as the bake algebra
    assert hcp.windward_azimuth_deg(90.0, "deck_panels_lost", "e") == 90.0
    assert hcp.windward_azimuth_deg(0.0, "roof_collapsed", "e") == 270.0  # variants only for the two bay-drop levels
    assert hcp.windward_azimuth_deg(0.0, "cover_lost", None) == 270.0     # older GT without a variant


def test_elevation_to_height_inverts_top_ray_pitch():
    dist, elev, aim = 28.0, 35.0, 1.5
    h = hcp.elevation_to_height(dist, elev, aim)
    theta = math.degrees(math.atan2(h - aim, dist))
    assert abs(theta - elev) < 1e-9


def test_densest_cluster_finds_the_tight_group():
    import random
    rng = random.Random(7)
    tight = [(10.0 + rng.uniform(-0.3, 0.3), 10.0 + rng.uniform(-0.3, 0.3))
            for _ in range(8)]
    background = [(rng.uniform(-200, 200), rng.uniform(-200, 200))
                 for _ in range(20)]
    result = hcp.densest_cluster(tight + background, cell_m=6.0)
    assert result is not None
    cx, cy, n = result
    assert n >= 6
    assert math.hypot(cx - 10.0, cy - 10.0) < 3.0


def test_densest_cluster_empty():
    assert hcp.densest_cluster([]) is None


def test_flooded_street_point_finds_the_facing_pair():
    # two houses 20 m apart, yaw 180 deg apart (facing each other across a
    # street), midpoint depth inside [0.3, 0.8]; plus decoys that must lose:
    # a too-far pair, a not-aligned pair, and a dry pair.
    houses = [
        (-10.0, 0.0, 0.0), (10.0, 0.0, 180.0),      # the real pair
        (-10.0, 100.0, 0.0), (10.0, 100.0, 90.0),   # not aligned (90 deg)
        (-10.0, 200.0, 0.0), (200.0, 200.0, 180.0), # too far apart
    ]

    def depth_fn(x, y):
        if abs(x - 0.0) < 1e-6 and abs(y - 0.0) < 1e-6:
            return 0.5
        return 0.0                     # everywhere else is dry

    result = hcp.flooded_street_point(houses, depth_fn)
    assert result is not None
    mx, my, bearing = result
    assert abs(mx - 0.0) < 1e-6 and abs(my - 0.0) < 1e-6


def test_flooded_street_point_none_when_nothing_qualifies():
    houses = [(-10.0, 0.0, 0.0), (10.0, 0.0, 90.0)]     # not aligned

    def depth_fn(x, y):
        return 0.5

    assert hcp.flooded_street_point(houses, depth_fn) is None


@pytest.mark.parametrize("gt_path", [_GT_L3, _GT_L2])
def test_select_review_subjects_end_to_end(gt_path):
    gt = _load_gt(gt_path)
    region = tuple(gt["region"])
    trees = gt.get("trees") or []
    subs = hcp.select_review_subjects(gt)

    # the 6 pre-existing subjects are untouched in NAME (positions may have
    # shifted under the margin fix -- covered by the margin test above)
    for name in ("shoreline", "deep_water", "dry_inland"):
        assert name in subs

    # the 5 NEW close subjects: on this ground truth each level bucket
    # exists somewhere on the plate (verified against the SESSION doc's own
    # tallies), so all five must resolve -- a caller running against a
    # thinner damage ladder gets documented fallbacks instead (see the
    # `note` field, exercised by the level-filter tests below), never a
    # silent drop.
    for name in ("stripped_roof_house", "collapsed_house", "raft_field",
                "fallen_tree", "flooded_street"):
        assert name in subs, "{0} missing for {1}".format(name, gt_path)
        s = subs[name]
        assert region[0] <= s["x"] <= region[2]
        assert region[1] <= s["y"] <= region[3]
        assert s["obl_dist"] > 0.0
        assert 0.0 <= s["azimuth_deg"] < 360.0

    # every subject's chosen azimuth, after the full clear+cap solve, keeps
    # the frame's top edge on the plate -- the whole point of deliverable A
    # applied to subjects `review_points` does not itself choose.
    avoid = [(t["x"], t["y"]) for t in trees
            if t.get("level") not in ("fallen", "snapped")]
    for name, s in subs.items():
        dist = float(s.get("obl_dist", 45.0))
        obl_h = float(s.get("obl_h", 22.0))
        aim_h = float(s.get("aim_h", 1.0))
        preferred = float(s.get("azimuth_deg", 225.0))
        az = srp._clear_azimuth(s["x"], s["y"], avoid, dist, 1.0, preferred,
                                region=region)
        _, _, still_bad = srp._cap_oblique_range(
            s["x"], s["y"], az, dist, obl_h, aim_h, region)
        assert not still_bad, "{0} ({1}) still off-plate after capping".format(
            name, gt_path)


def test_select_review_subjects_degrades_gracefully_with_no_severe_damage():
    """A synthetic plate with ONLY `pristine` houses/trees and a dry surge
    must not raise, and must simply omit the subjects nothing qualifies
    for."""
    region = [-100.0, -100.0, 100.0, 100.0]
    scfg = dict(sgw.resolve_cfg({}))
    scfg["surge_m"] = 0.30            # the floor -- effectively dry
    gt = {
        "region": region, "surge": scfg,
        "houses": [{"x": 0.0, "y": 0.0, "yaw_deg": 0.0, "level": "pristine",
                   "water_depth_m": 0.0}],
        "trees": [{"x": 5.0, "y": 5.0, "yaw_deg": 0.0, "level": "pristine"}],
    }
    subs = hcp.select_review_subjects(gt)
    assert "stripped_roof_house" not in subs
    assert "collapsed_house" not in subs
    assert "raft_field" not in subs
    assert "flooded_street" not in subs
    # a pristine tree is still the "most severe available" fallback
    assert "fallen_tree" not in subs or subs["fallen_tree"].get("note")


def test_house_level_fallback_ladder_is_documented():
    """When the primary level bucket is empty, the fallback used must be
    named in `note` -- never a silent substitution."""
    region = [-200.0, -200.0, 200.0, 200.0]
    scfg = dict(sgw.resolve_cfg({}))
    scfg["surge_m"] = 1.0
    gt = {
        "region": region, "surge": scfg,
        "houses": [
            {"x": 0.0, "y": 0.0, "yaw_deg": 30.0, "level": "shingles_lost",
             "water_depth_m": 0.0},
        ],
        "trees": [],
    }
    subs = hcp.select_review_subjects(gt)
    assert "stripped_roof_house" in subs
    assert "shingles_lost" in subs["stripped_roof_house"].get("note", "")


# ---------------------------------------------------------------------------
# deliverable C -- the background-dominated sanity gate
# ---------------------------------------------------------------------------

def test_offplate_fraction_flags_a_uniform_hdri_ground_frame():
    import numpy as np
    arr = np.zeros((32, 32, 4), dtype="uint8")
    arr[..., 0], arr[..., 1], arr[..., 2] = 103, 114, 75    # the calibrated mean
    frac = srp._offplate_fraction(arr)
    assert frac == 1.0
    assert frac > srp._HDRI_GROUND_FRAC


def test_offplate_fraction_does_not_flag_a_varied_scene_frame():
    import numpy as np
    rng = np.random.default_rng(3)
    # half sky-blue, half a saturated red roof -- neither is inside the
    # calibrated band, so the fraction must be far under threshold.
    arr = np.zeros((32, 32, 4), dtype="uint8")
    arr[:16, :, 0], arr[:16, :, 1], arr[:16, :, 2] = 120, 170, 230   # sky
    arr[16:, :, 0], arr[16:, :, 1], arr[16:, :, 2] = 180, 40, 30     # roof
    frac = srp._offplate_fraction(arr)
    assert frac < srp._HDRI_GROUND_FRAC


def test_flag_offplate_geometric_signal_fires(capsys):
    import numpy as np
    arr = np.zeros((8, 8, 4), dtype="uint8")
    arr[..., 0], arr[..., 1], arr[..., 2] = 120, 170, 230   # not the HDRI band
    srp._flag_offplate(arr, "/tmp/x/deep_water_obl.png",
                       region=(-250.0, -250.0, 250.0, 250.0),
                       target=(9000.0, 9000.0))
    out = capsys.readouterr().out
    assert "OFF-PLATE" in out
    assert "deep_water_obl" in out


def test_flag_offplate_silent_when_target_inside_and_colour_clean(capsys):
    import numpy as np
    arr = np.zeros((8, 8, 4), dtype="uint8")
    arr[..., 0], arr[..., 1], arr[..., 2] = 120, 170, 230
    srp._flag_offplate(arr, "/tmp/x/shoreline_obl.png",
                       region=(-250.0, -250.0, 250.0, 250.0),
                       target=(0.0, 0.0))
    out = capsys.readouterr().out
    assert out == ""


def test_flag_offplate_skips_geometric_check_with_no_bounds(capsys):
    import numpy as np
    arr = np.zeros((8, 8, 4), dtype="uint8")
    arr[..., 0], arr[..., 1], arr[..., 2] = 120, 170, 230
    srp._flag_offplate(arr, "/tmp/x/anything_obl.png", region=None, target=None)
    out = capsys.readouterr().out
    assert out == ""


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
