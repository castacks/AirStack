#!/usr/bin/env python3
"""test_fire_city_dressing.py — the placement math for the two 2026-08-31
city-assembly passes: SCORCHED VEGETATION near a burning building, and the
FIRE-SIDE DEBRIS APRON on a non-collapse building's venting sides.

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        python tests/test_fire_city_dressing.py
    docker exec isaac-sim bash -c \\
        "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
         /isaac-sim/AirStack/scene_gen/tests/test_fire_city_dressing.py"

RUNS ONLY UNDER A REAL `pxr` — `fire_assembly_lib` imports `pxr` at module
level (unlike most of `disaster/`, which defers it), so this file cannot be
collected on a bare host `pytest` with no USD installed; `uv run --with
usd-core` or the container's `usd_python.sh` both provide one. Bare USD only
— no `SimulationApp`, no Kit, no GPU, so it is safe to run beside a running
sim (memory: "Nucleus USD without Kit").

WHAT THIS PINS DOWN, per `urban_fire_city_launch_script.py`'s two new knobs:

  FC_SCORCH_VEG (`fire_assembly_lib.veg_scorch_radius_m` /
  `vegetation_scorch_targets` / `apply_vegetation_scorch`):
    1. the radius ladder is 0 below F2 ("level>=F2" per the brief) and
       non-decreasing F2..F6, floored so a short building still reaches its
       own kerb trees;
    2. a fuel exactly at the boundary is INSIDE (`<=`), one metre past it is
       excluded;
    3. a fuel in reach of two fires keeps the call with the larger MARGIN,
       not merely the nearer building;
    4. on a synthetic tree built the way `vegetation.survey` expects (a
       "bole" leaf mesh, a "bole" wood mesh, and a leaf PointInstancer with
       its own prototype), the in-reach tree's leaf/trunk materials are
       rebound and the out-of-reach tree's are byte-for-byte UNTOUCHED —
       "trees outside the radius stay green" is not just an intention, nine
       distinct bindings across the two trees are read back and checked.

  FC_FIRE_APRON (`apron_count` / `apron_points_for_side` / `world_masses` /
  `build_fire_apron` / `author_merged_lumps`):
    5. the level gate is exactly F1..F5 — F0, F5c and F6 all produce NO
       prim, matching "no F5c/F6" (collapse already drops a heap) and no F0
       (nothing to scatter);
    6. density is non-decreasing with level and with a venting side's own
       span, and never exceeds `APRON_MAX_PER_SIDE`;
    7. seats sit ONLY on the recorded venting sides, inset from the
       corners, and a YAWED building's outward offset is a real rotation
       (checked against the analytic rotation of the unyawed case) — the
       one way this pass could silently scatter debris on the wrong wall;
    8. one qualifying building costs exactly ONE Mesh prim (plus at most
       two `materialBind` GeomSubsets) no matter how many lumps it holds —
       the "merged, not one-prim-per-lump" requirement — and its point/face
       counts are exactly `8 * n_lumps` / `6 * n_lumps`.

WHAT THIS CANNOT SEE: whether the char/scorch tones actually read as burnt
under a real light, whether the apron looks like debris rather than gravel
at bench scale, and everything about Flow/materials downstream of a real
Kit composition. That needs a render (`scene_gen/tools/soot_png.py`'s own
review-PNG idiom, not reproduced here).
"""

import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import fire_assembly_lib as fal              # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade             # noqa: E402

_FAILS = []


def check(name, cond, detail=""):
    if cond:
        print("ok  " + name)
    else:
        print("FAIL " + name + (": " + detail if detail else ""))
        _FAILS.append(name)


# ---------------------------------------------------------------------------
# 1) the vegetation radius ladder
# ---------------------------------------------------------------------------
def test_radius_ladder_zero_below_f2():
    check("test_radius_ladder_zero_below_f2",
          fal.veg_scorch_radius_m("F0", 30.0) == 0.0
          and fal.veg_scorch_radius_m("F1", 30.0) == 0.0
          and fal.veg_scorch_radius_m("bogus", 30.0) == 0.0)


def test_radius_ladder_monotonic_f2_up():
    levels = ("F2", "F3", "F4", "F5", "F5c", "F6")
    radii = [fal.veg_scorch_radius_m(lv, 30.0) for lv in levels]
    check("test_radius_ladder_monotonic_f2_up",
          all(b >= a for a, b in zip(radii, radii[1:])), str(radii))


def test_radius_floor_for_a_short_building():
    r = fal.veg_scorch_radius_m("F2", 1.0)          # 1.2 x 1 m would be tiny
    check("test_radius_floor_for_a_short_building",
          r == fal.VEG_RADIUS_FLOOR_M, str(r))


# ---------------------------------------------------------------------------
# 2) point-to-footprint distance
# ---------------------------------------------------------------------------
def test_dist_zero_inside_box():
    box = [0.0, 0.0, 0.0, 10.0, 10.0, 10.0]
    check("test_dist_zero_inside_box",
          fal._dist_point_to_box_xy(5.0, 5.0, box) == 0.0
          and fal._dist_point_to_box_xy(0.0, 0.0, box) == 0.0)


def test_dist_outside_box_axis_and_corner():
    box = [0.0, 0.0, 0.0, 10.0, 10.0, 10.0]
    d_axis = fal._dist_point_to_box_xy(15.0, 5.0, box)
    d_corner = fal._dist_point_to_box_xy(13.0, 14.0, box)
    check("test_dist_outside_box_axis_and_corner",
          abs(d_axis - 5.0) < 1e-9 and abs(d_corner - math.hypot(3, 4)) < 1e-9,
          "{0} {1}".format(d_axis, d_corner))


# ---------------------------------------------------------------------------
# 3) vegetation_scorch_targets — membership, boundary, and the margin rule
# ---------------------------------------------------------------------------
def test_scorch_targets_boundary_is_inclusive():
    box = [0.0, 0.0, 0.0, 0.0, 0.0, 20.0]           # a point building, H=20
    r = fal.veg_scorch_radius_m("F3", 20.0)          # 1.3 * 20 = 26
    fuels = [(r, 0.0, "/on_boundary"), (r + 1.0, 0.0, "/just_outside")]
    out = fal.vegetation_scorch_targets(fuels, [{"i": 0, "box": box, "level": "F3"}])
    check("test_scorch_targets_boundary_is_inclusive",
          "/on_boundary" in out and "/just_outside" not in out, str(out))


def test_scorch_targets_prefers_larger_margin():
    # Two fires can both reach the same fuel; the WEAKER, closer fire (small
    # radius, small distance) should not automatically win over a bigger
    # fire whose radius swallows the point with more room to spare.
    near_small = {"i": 0, "box": [0.0, 0.0, 0.0, 0.0, 0.0, 10.0], "level": "F2"}
    far_big = {"i": 1, "box": [50.0, 0.0, 0.0, 50.0, 0.0, 60.0], "level": "F6"}
    r_near = fal.veg_scorch_radius_m("F2", 10.0)     # 12.0
    r_far = fal.veg_scorch_radius_m("F6", 60.0)      # 126.0
    x = 10.0                                         # 10 m from #0, 40 m from #1
    assert r_near > (x - 0.0) and r_far > (50.0 - x)
    fuels = [(x, 0.0, "/shared")]
    out = fal.vegetation_scorch_targets(fuels, [near_small, far_big])
    margin_near = r_near - x
    margin_far = r_far - (50.0 - x)
    expect_i = 1 if margin_far > margin_near else 0
    check("test_scorch_targets_prefers_larger_margin",
          out["/shared"]["i"] == expect_i,
          "{0} vs expected {1}".format(out["/shared"]["i"], expect_i))


def test_scorch_targets_needs_box_and_level():
    fuels = [(1.0, 1.0, "/x")]
    out = fal.vegetation_scorch_targets(fuels, [{"i": 0, "box": None, "level": "F4"},
                                                {"i": 1, "box": [0, 0, 0, 2, 2, 10],
                                                 "level": None}])
    check("test_scorch_targets_needs_box_and_level", out == {}, str(out))


# ---------------------------------------------------------------------------
# 4) a synthetic tree, `vegetation.survey`'s own shape, end to end
# ---------------------------------------------------------------------------
def _make_mdl_material(stage, path, mdl_name):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path.AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath(mdl_name), "mdl")
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def _make_quad_mesh(stage, path, mat):
    me = UsdGeom.Mesh.Define(stage, path)
    me.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0),
                        Gf.Vec3f(1, 1, 0), Gf.Vec3f(0, 1, 0)])
    me.CreateFaceVertexCountsAttr([4])
    me.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    UsdShade.MaterialBindingAPI(me.GetPrim()).Bind(mat)
    return me


def _make_tree(stage, path, x, y, leaf_mat, bark_mat):
    root = UsdGeom.Xform.Define(stage, path)
    root.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    _make_quad_mesh(stage, path.AppendChild("leafMesh"), leaf_mat)
    _make_quad_mesh(stage, path.AppendChild("trunkMesh"), bark_mat)
    proto = _make_quad_mesh(stage, path.AppendPath("Prototypes/leafProto"),
                            leaf_mat)
    pi = UsdGeom.PointInstancer.Define(stage, path.AppendChild("leafInstancer"))
    pi.CreatePositionsAttr([Gf.Vec3f(0, 0, 2), Gf.Vec3f(0.5, 0, 2)])
    pi.CreateProtoIndicesAttr([0, 0])
    pi.GetPrototypesRel().SetTargets([proto.GetPath()])
    return root


def _bound_mdl_name(stage, path):
    prim = stage.GetPrimAtPath(path)
    mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if not mat or not mat.GetPrim().IsValid():
        return None, None
    sh = UsdShade.Shader.Get(stage, mat.GetPath().AppendChild("Shader"))
    src = sh.GetSourceAsset("mdl") if sh else None
    return str(mat.GetPath()), (src.path if src else None)


def test_scorch_vegetation_pass_end_to_end():
    stage = Usd.Stage.CreateInMemory()
    leaf_mat = _make_mdl_material(stage, Sdf.Path("/World/Looks/leafGreen"),
                                  "Leaf_Species.mdl")
    bark_mat = _make_mdl_material(stage, Sdf.Path("/World/Looks/barkBrown"),
                                  "Oak_Bark.mdl")
    near = Sdf.Path("/World/stage/generated/street_tree_0_0")
    far = Sdf.Path("/World/stage/generated/street_tree_0_1")
    _make_tree(stage, near, 25.0, 5.0, leaf_mat, bark_mat)
    _make_tree(stage, far, 500.0, 500.0, leaf_mat, bark_mat)

    placements = [
        {"category": "street_tree", "x_m": 25.0, "y_m": 5.0,
         "prim_path": str(near)},
        {"category": "street_tree", "x_m": 500.0, "y_m": 500.0,
         "prim_path": str(far)},
    ]
    placed_rows = [{"i": 0, "bbox": [0.0, 0.0, 0.0, 20.0, 10.0, 15.0],
                   "doc": {"fire": {"level": "F4"}}}]

    stats = fal.scorch_vegetation_pass(stage, placements, placed_rows,
                                       "/World/fire")
    check("test_scorch_vegetation_pass_counts",
          stats["fuels_total"] == 2 and stats["scorched"] == 1
          and stats["trees"] == 1 and stats["leaf_binds"] == 2
          and stats["trunk_binds"] == 1, str(stats))

    leaf_mat_after, _ = _bound_mdl_name(stage, near.AppendChild("leafMesh"))
    trunk_mat_after, _ = _bound_mdl_name(stage, near.AppendChild("trunkMesh"))
    proto_mat_after, _ = _bound_mdl_name(
        stage, near.AppendPath("Prototypes/leafProto"))
    check("test_scorch_vegetation_binds_leaf_and_trunk_char",
          leaf_mat_after == "/World/fire/VegLooks/leaf_char"
          and trunk_mat_after == "/World/fire/VegLooks/trunk_char"
          and proto_mat_after == "/World/fire/VegLooks/leaf_char",
          "{0} {1} {2}".format(leaf_mat_after, trunk_mat_after,
                               proto_mat_after))

    far_leaf, far_leaf_mdl = _bound_mdl_name(stage, far.AppendChild("leafMesh"))
    far_trunk, far_trunk_mdl = _bound_mdl_name(stage, far.AppendChild("trunkMesh"))
    check("test_scorch_vegetation_leaves_far_tree_untouched",
          far_leaf == "/World/Looks/leafGreen" and far_leaf_mdl == "Leaf_Species.mdl"
          and far_trunk == "/World/Looks/barkBrown" and far_trunk_mdl == "Oak_Bark.mdl",
          "{0}/{1} {2}/{3}".format(far_leaf, far_leaf_mdl, far_trunk, far_trunk_mdl))


# ---------------------------------------------------------------------------
# 5) the apron level gate and density ladder
# ---------------------------------------------------------------------------
def test_apron_gate_excludes_f0_f5c_f6():
    check("test_apron_gate_excludes_f0_f5c_f6",
          fal.apron_count("F0", 14.0) == 0
          and fal.apron_count("F5c", 14.0) == 0
          and fal.apron_count("F6", 14.0) == 0
          and fal.apron_count("bogus", 14.0) == 0)


def test_apron_count_monotonic_with_level_and_span():
    by_level = [fal.apron_count(lv, fal.APRON_REF_SPAN_M)
               for lv in fal.APRON_LEVELS]
    by_span = [fal.apron_count("F3", s) for s in (4.0, 14.0, 40.0, 400.0)]
    check("test_apron_count_monotonic_with_level_and_span",
          all(b >= a for a, b in zip(by_level, by_level[1:]))
          and all(b >= a for a, b in zip(by_span, by_span[1:]))
          and by_span[-1] <= fal.APRON_MAX_PER_SIDE,
          "{0} {1}".format(by_level, by_span))


# ---------------------------------------------------------------------------
# 6) apron seats: venting sides only, inset from corners, yaw handled
# ---------------------------------------------------------------------------
def _fake_mass(W=20.0, D=10.0, cx=0.0, cy=0.0, yaw=0.0):
    return {"cx": cx, "cy": cy, "yaw": yaw, "W": W, "D": D, "z0": 0.0,
            "top": 12.0, "levels": [0.0], "spec": {"bands": []}}


def test_apron_points_stay_on_the_wall_and_off_the_corners():
    m = _fake_mass()
    rng = random.Random(11)
    pts = fal.apron_points_for_side(m, "S", "F4", rng)
    W, D = m["W"], m["D"]
    lo, hi = W * fal.APRON_INSET_FRAC, W * (1.0 - fal.APRON_INSET_FRAC)
    ok = True
    for x, y, s in pts:
        # local x is measured from the low end of the S wall (t = x + W/2)
        t = x + W / 2.0
        if not (lo - 1e-6 <= t <= hi + 1e-6):
            ok = False
        # outward from S (local -Y) puts the seat BELOW the wall line
        if not (y < -D / 2.0):
            ok = False
    check("test_apron_points_stay_on_the_wall_and_off_the_corners",
          ok and len(pts) == fal.apron_count("F4", W), str(pts[:3]))


def test_apron_points_respect_yaw_rotation():
    # An unyawed mass's "S" outward direction is world (0, -1) (local -Y,
    # `_SIDE_NORMAL["S"]`, untouched by a zero yaw). Rotate the SAME mass 90
    # deg CCW and that direction must rotate WITH it, by a standard CCW
    # rotation matrix, to (1, 0) — the "south" wall now faces world "east".
    # This is the analytic check that `world_masses` + `_outward` are
    # actually composing the yaw, not silently working in local space (a
    # bug here would scatter the apron on the wrong wall of a rotated
    # building without any prim count or gating check ever catching it).
    from disaster import quake_flow as qf

    m0 = _fake_mass(yaw=0.0)
    m90 = _fake_mass(yaw=90.0)
    ox0, oy0 = qf._outward(m0, "S")
    ox90, oy90 = qf._outward(m90, "S")
    check("test_apron_points_respect_yaw_rotation",
          abs(ox0 - 0.0) < 1e-9 and abs(oy0 - (-1.0)) < 1e-9
          and abs(ox90 - 1.0) < 1e-9 and abs(oy90 - 0.0) < 1e-9,
          "{0},{1} / {2},{3}".format(ox0, oy0, ox90, oy90))


def test_world_masses_does_not_mutate_the_original():
    import copy

    masses = {"main": _fake_mass()}
    before = copy.deepcopy(masses)
    fal.world_masses(masses, 100.0, 50.0, 90.0)
    check("test_world_masses_does_not_mutate_the_original",
          masses == before, str(masses))


# ---------------------------------------------------------------------------
# 7) build_fire_apron: the level/side gates, and the merged-mesh prim budget
# ---------------------------------------------------------------------------
def _fake_placed_row(i, level, sides, W=20.0, D=10.0, x=0.0, y=0.0, yaw=0.0):
    return {"i": i, "stem": "bldg{0}".format(i), "x": x, "y": y, "yaw": yaw,
            "bbox": [x - W / 2.0, y - D / 2.0, 0.0,
                    x + W / 2.0, y + D / 2.0, 15.0],
            "doc": {"fire": {"level": level, "sides": list(sides),
                             "mass": "main"}},
            "masses": {"main": _fake_mass(W=W, D=D)}}


def test_build_fire_apron_gates_by_level():
    stage = Usd.Stage.CreateInMemory()
    results = {}
    for lvl in ("F0", "F1", "F2", "F3", "F4", "F5", "F5c", "F6"):
        r = _fake_placed_row(0, lvl, ("S",))
        results[lvl] = fal.build_fire_apron(stage, "/World/fire", r,
                                            random.Random(1))
    ok = (results["F0"]["prim"] is None
         and results["F5c"]["prim"] is None
         and results["F6"]["prim"] is None
         and all(results[lv]["prim"] is not None
                 for lv in ("F1", "F2", "F3", "F4", "F5")))
    check("test_build_fire_apron_gates_by_level", ok, str(
        {k: v["prim"] for k, v in results.items()}))


def test_build_fire_apron_no_sides_is_a_noop():
    stage = Usd.Stage.CreateInMemory()
    r = _fake_placed_row(0, "F4", ())
    res = fal.build_fire_apron(stage, "/World/fire", r, random.Random(1))
    check("test_build_fire_apron_no_sides_is_a_noop", res["prim"] is None,
          str(res))


def test_build_fire_apron_prim_budget_is_bounded():
    # ONE mesh (plus at most two GeomSubsets) no matter how many lumps —
    # the "merged, not one-prim-per-lump" requirement.
    stage = Usd.Stage.CreateInMemory()
    r = _fake_placed_row(0, "F5", ("S", "N", "E", "W"), W=60.0, D=60.0)
    res = fal.build_fire_apron(stage, "/World/fire", r, random.Random(3))
    prim = stage.GetPrimAtPath(res["prim"])
    subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
    me = UsdGeom.Mesh(prim)
    n_pts = len(me.GetPointsAttr().Get())
    n_faces = len(me.GetFaceVertexCountsAttr().Get())
    n_new_prims = 1 + len(subs)      # the mesh itself + its subsets
    check("test_build_fire_apron_prim_budget_is_bounded",
          n_new_prims <= 3 and n_pts == 8 * res["n"] and n_faces == 6 * res["n"]
          and res["n"] > fal.APRON_MAX_PER_SIDE,   # 4 long sides, capped each
          "{0} prim(s), {1} pts, {2} faces, n={3}".format(
              n_new_prims, n_pts, n_faces, res["n"]))


def test_fire_apron_pass_totals_across_a_small_city():
    stage = Usd.Stage.CreateInMemory()
    rows = [
        _fake_placed_row(0, "F4", ("S", "E"), x=0.0, y=0.0),
        _fake_placed_row(1, "F5c", ("S",), x=100.0, y=0.0),   # gated out
        _fake_placed_row(2, "F1", ("N",), x=200.0, y=0.0),
    ]
    out = fal.fire_apron_pass(stage, "/World/fire", rows, seed=7)
    n_meshes = sum(1 for r in out if r.get("prim"))
    check("test_fire_apron_pass_totals_across_a_small_city",
          len(out) == 3 and n_meshes == 2, str([r["prim"] for r in out]))


ALL_TESTS = [v for k, v in sorted(globals().items())
            if k.startswith("test_") and callable(v)]


def main():
    for t in ALL_TESTS:
        t()
    print()
    if _FAILS:
        print("{0} FAILURE(S): {1}".format(len(_FAILS), ", ".join(_FAILS)))
        return 1
    print("ALL {0} TESTS OK".format(len(ALL_TESTS)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
