"""test_people_json_to_annotations.py — pins the tornado_people PEOPLE_JSON ->
raven_nav ground-truth annotation conversion (`tools/people_json_to_annotations.py`).

Pure Python, no `pxr`, no Isaac Sim, no scene build: every function under
test takes plain dicts/numbers and returns plain dicts/numbers.

    python3 -m pytest scene_gen/tests/test_people_json_to_annotations.py -q
    python3 scene_gen/tests/test_people_json_to_annotations.py
"""
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
_TOOLS = os.path.join(_SCENE_GEN, "tools")
for _p in (_SCENE_GEN, _TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import people_json_to_annotations as pjta               # noqa: E402


def _rec(x=0.0, y=0.0, z=0.2, body_axis_deg=0.0, occlusion="none",
        visibility="full", covered_frac=0.0, **extra):
    r = {"x": x, "y": y, "z": z, "body_axis_deg": body_axis_deg,
        "occlusion": occlusion, "visibility": visibility,
        "covered_frac": covered_frac}
    r.update(extra)
    return r


# ---------------------------------------------------------------------------
# `_rotated_aabb_xy` — the geometry the whole tool hinges on
# ---------------------------------------------------------------------------

def test_axis_aligned_at_zero_heading():
    """body_axis_deg=0 (pointing +X): the 0.6 m WIDTH axis is +Y, the 1.8 m
    LENGTH axis is +X, so the AABB is exactly (1.8, 0.6) with no rotation
    slop."""
    sx, sy = pjta._rotated_aabb_xy(0.6, 1.8, 0.0)
    assert abs(sx - 1.8) < 1e-9
    assert abs(sy - 0.6) < 1e-9


def test_axis_aligned_at_90_heading():
    """Rotated a quarter turn: length and width swap which world axis they
    occupy."""
    sx, sy = pjta._rotated_aabb_xy(0.6, 1.8, 90.0)
    assert abs(sx - 0.6) < 1e-6
    assert abs(sy - 1.8) < 1e-6


def test_diagonal_heading_is_between_the_two_axis_aligned_extents():
    """At 45 degrees the rotated rectangle's AABB in each world axis must sit
    strictly between the box's own width (0.6, the narrowest it can ever be)
    and its length (1.8, the widest) — and by the box's symmetry at a 45
    degree heading, the AABB is exactly square (sx == sy)."""
    sx, sy = pjta._rotated_aabb_xy(0.6, 1.8, 45.0)
    assert 0.6 < sx < 1.8
    assert 0.6 < sy < 1.8
    assert abs(sx - sy) < 1e-9         # symmetric at 45 degrees


def test_heading_is_periodic_every_180_degrees():
    """A body's box has no notion of head-vs-foot, so +180 (or -180) must
    give the identical footprint as the original heading."""
    for base in (0.0, 37.0, 90.0, 123.4):
        a = pjta._rotated_aabb_xy(0.6, 1.8, base)
        b = pjta._rotated_aabb_xy(0.6, 1.8, base + 180.0)
        assert abs(a[0] - b[0]) < 1e-9
        assert abs(a[1] - b[1]) < 1e-9


def test_aabb_within_the_box_own_bounds():
    """A rotated rectangle's AABB can never be narrower than its own
    shortest edge (0.6) — a sanity floor that would catch a sign error in
    the corner-projection math — nor wider than its own DIAGONAL
    (`hypot(0.6, 1.8)`, reached when a corner points straight down an axis),
    which is looser than the long edge (1.8): the diagonal case is exactly
    what the earlier, wrong version of this test missed."""
    diag = math.hypot(0.6, 1.8)
    for deg in range(0, 360, 13):
        sx, sy = pjta._rotated_aabb_xy(0.6, 1.8, float(deg))
        assert 0.6 - 1e-6 <= sx <= diag + 1e-6
        assert 0.6 - 1e-6 <= sy <= diag + 1e-6


# ---------------------------------------------------------------------------
# `person_to_annotation` — one record
# ---------------------------------------------------------------------------

def test_class_is_person():
    out = pjta.person_to_annotation(_rec())
    assert out["class"] == "person"


def test_center_is_the_record_xyz():
    out = pjta.person_to_annotation(_rec(x=12.5, y=-7.25, z=0.31))
    c = out["bbox_world"]["center_xyz_m"]
    assert abs(c[0] - 12.5) < 1e-6
    assert abs(c[1] - (-7.25)) < 1e-6
    assert abs(c[2] - 0.31) < 1e-6


def test_size_height_is_fixed_regardless_of_heading():
    for deg in (0.0, 45.0, 90.0, 271.0):
        out = pjta.person_to_annotation(_rec(body_axis_deg=deg))
        assert out["bbox_world"]["size_xyz_m"][2] == pjta._BOX_H_M


def test_missing_body_axis_defaults_to_zero_not_a_crash():
    rec = {"x": 1.0, "y": 2.0, "z": 0.2}
    out = pjta.person_to_annotation(rec)
    sx, sy = out["bbox_world"]["size_xyz_m"][0], out["bbox_world"]["size_xyz_m"][1]
    exp_sx, exp_sy = pjta._rotated_aabb_xy(pjta._BOX_W_M, pjta._BOX_L_M, 0.0)
    assert abs(sx - exp_sx) < 1e-6
    assert abs(sy - exp_sy) < 1e-6


def test_occlusion_visibility_covered_frac_pass_through():
    out = pjta.person_to_annotation(_rec(occlusion="midriff", visibility="full",
                                         covered_frac=0.28))
    assert out["occlusion"] == "midriff"
    assert out["visibility"] == "full"
    assert out["covered_frac"] == 0.28


def test_passthrough_fields_absent_when_not_in_record():
    out = pjta.person_to_annotation({"x": 0.0, "y": 0.0, "z": 0.0})
    assert "occlusion" not in out
    assert "visibility" not in out
    assert "covered_frac" not in out


def test_compare_to_groundtruth_only_needs_class_and_bbox_world():
    """The annotation entry must be a strict superset of what
    `compare_to_groundtruth._load_gt` reads (`class`, `bbox_world.
    center_xyz_m`, `bbox_world.size_xyz_m`) — extra keys are fine (it
    accesses everything through `.get`), missing ones are not."""
    out = pjta.person_to_annotation(_rec())
    assert "class" in out
    assert "center_xyz_m" in out["bbox_world"]
    assert "size_xyz_m" in out["bbox_world"]
    assert len(out["bbox_world"]["center_xyz_m"]) == 3
    assert len(out["bbox_world"]["size_xyz_m"]) == 3


# ---------------------------------------------------------------------------
# `convert` — the whole list
# ---------------------------------------------------------------------------

def test_convert_preserves_count_and_order():
    people = [_rec(x=float(i), y=0.0) for i in range(5)]
    out = pjta.convert(people)
    assert len(out) == 5
    for i, o in enumerate(out):
        assert abs(o["bbox_world"]["center_xyz_m"][0] - float(i)) < 1e-6


def test_convert_empty_list():
    assert pjta.convert([]) == []


# ---------------------------------------------------------------------------
# `main` — the CLI, end to end, on a tiny file (tmp_path fixture)
# ---------------------------------------------------------------------------

def test_main_reads_meta_wrapped_file_and_writes_flat_list(tmp_path):
    people_path = tmp_path / "humans_1.json"
    out_path = tmp_path / "annotations" / "TestScene.json"
    people_path.write_text(json.dumps({
        "meta": {"seed": 1},
        "people": [
            _rec(x=1.0, y=2.0, z=0.2, body_axis_deg=0.0, occlusion="none"),
            _rec(x=-3.0, y=4.0, z=0.3, body_axis_deg=90.0, occlusion="midriff",
                covered_frac=0.28),
        ],
    }))
    rc = pjta.main(["--people", str(people_path), "--out", str(out_path)])
    assert rc == 0
    assert out_path.exists()
    data = json.loads(out_path.read_text())
    assert isinstance(data, list)
    assert len(data) == 2
    assert all(e["class"] == "person" for e in data)
    assert data[1]["occlusion"] == "midriff"


def test_main_accepts_a_bare_list_too():
    """Some callers may hand a bare `[...]` (no `{"meta":..., "people":...}`
    wrapper) — `convert`'s caller in `main` falls back to treating the whole
    payload as the people list."""
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        people_path = os.path.join(d, "bare.json")
        out_path = os.path.join(d, "out.json")
        with open(people_path, "w") as f:
            json.dump([_rec(x=5.0, y=5.0)], f)
        rc = pjta.main(["--people", people_path, "--out", out_path])
        assert rc == 0
        with open(out_path) as f:
            data = json.load(f)
        assert len(data) == 1
        assert data[0]["class"] == "person"


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
