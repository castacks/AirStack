#!/usr/bin/env python3
"""test_quake_gac_bake.py — offline verification for the earthquake bake of
a SLICED GAC/downtowncity building (`quake_gac_bake_launch_script.py`,
`scene_gen/tools/quake_gac_bake.sh`), before either ever touches Kit.

    python3 scene_gen/tests/test_quake_gac_bake.py           # host: planner only
    pytest -q scene_gen/tests/test_quake_gac_bake.py

    # in the container, with pxr — runs the stage half too:
    docker exec isaac-sim bash -c \
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
       /isaac-sim/AirStack/scene_gen/tests/test_quake_gac_bake.py"

    # and re-check bakes that already exist:
    docker exec isaac-sim bash -c \
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
       /isaac-sim/AirStack/scene_gen/tests/test_quake_gac_bake.py \
       --verify /isaac-sim/AirStack/scene_gen/assets/gac_quake"

WHAT THIS TESTS, AND WHY IT USES A SYNTHETIC ELEMENT TABLE
------------------------------------------------------------
There is no local mirror of a merged GreatAmericanCity asset in this repo
(`scene_gen/assets/` carries only two PRE-SLICED kits, `SM_Building_02.usd`
and `SM_Building_09.usd`, under `assets/kits/` — and their manifest
`kits.json` records placement coordinates at the asset's OWN pre-scale
units, ~100x too large to read as metres; the actual bake launcher never
goes through that cache for exactly this reason, see its own docstring) and
the merged sources themselves live only on the Nucleus mirror this host has
no access to. So, per the brief, this degrades to the SAME synthetic
element table `test_quake_sliced.py` already built and validated for
exercising `plan_damage`/`apply_plan` — `fake_sliced_building` mirrors
`gac_storey_slice.ring()`'s real bay/storey/corner grammar closely enough
that `quake_flow.describe` (the REAL function, not a stub) builds a REAL
mass box and element table off it. Reusing that fixture (rather than
re-deriving a second copy of the same grammar here) is a deliberate choice:
it is the one place in this repo that has already been checked against the
slicer's own conventions (`test_the_fixture_is_shaped_like_a_real_slice`).

A. `plan_damage` + `plan_to_json`/`plan_from_json`, host-side, no USD at
   all — DG3, DG4, DG5 and OV, across both ladders that reach a real
   collapse (`urm` -> `masonry_collapse`, `rc` -> `pancake`/`storey_
   collapse`). Every plan is round-tripped through `plan_to_json`/`json.
   dumps`/`json.loads`/`plan_from_json` and checked byte-identical to a
   second round-trip (idempotent) and against the pile specs' own numbers.
B. "nothing at negative z" — pure arithmetic (`quake_sliced.apply_rigid`,
   the exact matrix `apply_plan` hands to every displaced prim): every
   displaced piece's own origin, moved through its plan spec, must not land
   more than a small tolerance below the building's own foundation line
   (`info["masses"]["main"]["z0"]`).
C. `apply_plan` on a bare `Usd.Stage` (pxr only, no Kit): real `UsdGeom.
   Mesh` prims are authored at every placement's `prim_path`, `PLAN_PILE`/
   `AUTHOR` are stubbed (`quake_sliced`'s own documented seam — see
   `test_quake_sliced.test_the_rubble_modules_are_only_touched_through_
   the_two_hooks`) so this runs whether or not the real rubble modules are
   importable, and: every `plan["removed"]` prim is deactivated, every
   `plan["displaced"]` prim's ACTUAL new world position (read back off the
   stage) matches `apply_rigid`'s prediction, and one pile is authored per
   `plan["piles"]` entry with the stub's own numbers landing in
   `ctx["rubble"]` (the `quake_sliced._author_pile` fix this bake launcher
   depends on).
D. the export round-trip: root-layer export of the post-`apply_plan` stage,
   reopened cold, prim count preserved, `fire_bake.verify_export` passes —
   the same COLD-REOPEN check `fire_bake_launch_script.py` runs on every
   real bake, exercised here on synthetic geometry so it needs no Kit run
   to catch a wiring mistake (wrong `doomed` path, a stray root prim, ...).
"""

import json
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
for _p in (_HERE, _SG):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from disaster import fire_bake as fb                           # noqa: E402
from disaster import quake_flow as qf                          # noqa: E402
from disaster import quake_sliced as qs                        # noqa: E402

try:
    from pxr import Gf, Sdf, Usd, UsdGeom                      # noqa: E402
    HAVE_USD = True
except Exception:                                              # pragma: no cover
    HAVE_USD = False

GRADES_WITH_COLLAPSE = {
    "urm": ("DG3", "DG4", "DG5", "OV"),
    "rc": ("DG3", "DG4", "DG5", "OV"),
}


def _plan(grade, btype="urm", seed=5, **kw):
    """A fixture building + its plan, through the real `describe` — the same
    helper `test_quake_sliced.py` uses, kept here rather than imported so
    this file's own assertions are explicit about which `info`/`plan` they
    are reading (importing `_plan` too would hide that from a reader)."""
    # Keep the pytest-based synthetic fixture out of the documented
    # ``--verify`` path.  Isaac's bare-USD interpreter deliberately ships no
    # pytest, and a cold check of already-baked files does not need the
    # planner fixture at all.  Importing it at module scope made the verifier
    # fail before it opened its first USD.
    from test_quake_sliced import fake_sliced_building
    pls, style, grid = fake_sliced_building(seed=seed, **kw)
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = btype
    rng = random.Random(seed)
    plan = qs.plan_damage(info, info["elements"], grade, btype, rng)
    return info, plan


def _by_path(info):
    return {(e.get("p") or {}).get("prim_path"): e for e in info["elements"]}


# ---------------------------------------------------------------------------
# A. plan_damage + plan_to_json / plan_from_json, host-side
# ---------------------------------------------------------------------------
def test_plan_damage_runs_dg3_dg4_dg5_ov_on_both_collapsing_ladders():
    for btype, grades in GRADES_WITH_COLLAPSE.items():
        for grade in grades:
            info, plan = _plan(grade, btype, seed=11)
            assert plan["btype"] == btype
            assert plan["grade"] == grade
            known = set(_by_path(info))
            assert set(plan["removed"]) <= known
            assert set(plan["displaced"]) <= known
            # DG5 and OV are the two grades in `LADDER_S` that always author
            # at least one pile (`masonry_collapse`/`pancake` at DG5,
            # `overturn`'s landing windrow at OV); DG3 never does for either
            # ladder in this table.
            if grade in ("DG5", "OV"):
                assert plan["piles"], (btype, grade, "expected a pile")


def test_plan_to_json_round_trips_and_is_idempotent():
    for btype, grades in GRADES_WITH_COLLAPSE.items():
        for grade in grades:
            _info, plan = _plan(grade, btype, seed=12)
            data = qs.plan_to_json(plan)
            # a bare `json.dumps`/`json.loads` must also work on the result —
            # that is the whole point of calling this "json-safe"
            reparsed = json.loads(json.dumps(data))
            assert reparsed == data
            # idempotent: serialising the ALREADY-serialised plan changes
            # nothing further (every tuple is already a list, every number
            # already a plain float/int)
            assert qs.plan_to_json(data) == data
            restored = qs.plan_from_json(data)
            assert restored["btype"] == plan["btype"]
            assert restored["grade"] == plan["grade"]
            assert restored["removed"] == plan["removed"]
            assert sorted(restored["displaced"]) == sorted(plan["displaced"])
            assert restored["piles"] == data["piles"]
            assert "_removed_set" not in restored


def test_plan_to_json_turns_every_tuple_into_a_list():
    """The one real behavioural difference between `plan` and `plan_to_json
    (plan)`: a `_disp()` spec's `pivot`/`axis` are already lists (`_disp`
    builds them with `[float(q) for q in ...]`), but `s_glass_loss`'s own
    `frac` kwarg -- a bare Python tuple, recorded straight into
    `plan["recipes"]` -- is not, and this is where it gets fixed."""
    _info, plan = _plan("DG2", "urm", seed=4)
    frac_kw = next(kw for name, kw in plan["recipes"] if name == "glass_loss")
    assert isinstance(frac_kw.get("frac"), tuple), \
        "fixture assumption changed: glass_loss frac is no longer a tuple"
    data = qs.plan_to_json(plan)
    frac_after = next(kw for name, kw in data["recipes"] if name == "glass_loss")
    assert isinstance(frac_after["frac"], list)
    assert list(frac_kw["frac"]) == frac_after["frac"]


# ---------------------------------------------------------------------------
# B. nothing at negative z — pure arithmetic, no USD
# ---------------------------------------------------------------------------
def test_no_displaced_piece_lands_below_the_foundation_line():
    """Every `plan["displaced"]` spec, applied to its own piece's ORIGIN
    (`quake_sliced.apply_rigid` — the exact matrix `apply_plan` hands to
    `_gf`/`Gf.Matrix4d` for the real prim), must not land more than a small
    tolerance below the building's own z0. A generous tolerance (not a
    strict `>= z0`): a wall panel folded flat about its own base edge
    legitimately dips `~its own half-thickness` below the pivot line — the
    bug this guards is a sign/scale error that sends a piece metres
    underground, not that geometric nuance."""
    TOL_M = 1.5
    for btype, grades in GRADES_WITH_COLLAPSE.items():
        for grade in grades:
            info, plan = _plan(grade, btype, seed=9)
            by_path = _by_path(info)
            z0 = info["masses"]["main"]["z0"]
            for path, spec in plan["displaced"].items():
                e = by_path.get(path)
                if e is None:
                    continue
                x, y, z = float(e["x"]), float(e["y"]), float(e["z"])
                _nx, _ny, nz = qs.apply_rigid(spec, (x, y, z))
                assert nz >= z0 - TOL_M, (
                    btype, grade, path, "z0={0} new_z={1}".format(z0, nz))


# ---------------------------------------------------------------------------
# C. apply_plan on a bare Usd.Stage (pxr only, no Kit)
# ---------------------------------------------------------------------------
_UNIT_CUBE_PTS = None
_UNIT_CUBE_COUNTS = [4, 4, 4, 4, 4, 4]
_UNIT_CUBE_IDX = [0, 1, 2, 3, 4, 5, 6, 7, 0, 4, 7, 1,
                 1, 7, 6, 2, 2, 6, 5, 3, 3, 5, 4, 0]


def _author_piece_mesh(stage, p):
    """A trivial unit-box `UsdGeom.Mesh` at `p["prim_path"]`, translated and
    scaled to the placement's own `x_m`/`y_m`/`z_m`/`_size` — enough for
    `quake_flow._deactivate`/`_transform_prims` (any Xformable prim) and for
    `fire_bake.verify_export`'s mesh count, without needing the real
    slicer's VTK-clipped geometry (this test exercises the ELEMENT-TABLE
    half of the pipeline, not the mesh-clipping half `_gac_region_probe.py`
    already covers)."""
    from pxr import Gf, Sdf, UsdGeom

    global _UNIT_CUBE_PTS
    if _UNIT_CUBE_PTS is None:
        _UNIT_CUBE_PTS = [Gf.Vec3f(x, y, z) for x in (-0.5, 0.5)
                          for y in (-0.5, 0.5) for z in (-0.5, 0.5)]
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(p["prim_path"]))
    mesh.CreatePointsAttr(_UNIT_CUBE_PTS)
    mesh.CreateFaceVertexCountsAttr(_UNIT_CUBE_COUNTS)
    mesh.CreateFaceVertexIndicesAttr(_UNIT_CUBE_IDX)
    sx, sy, sz = (p.get("_size") or (1.0, 1.0, 3.0))
    xf = UsdGeom.Xformable(mesh)
    xf.AddTranslateOp().Set(Gf.Vec3d(float(p["x_m"]), float(p["y_m"]),
                                     float(p["z_m"])))
    xf.AddScaleOp().Set(Gf.Vec3f(max(sx, 0.1), max(sy, 0.1), max(sz, 0.1)))
    return mesh


def _new_bare_stage():
    from pxr import Sdf, Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/cell"))
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/cell/pieces"))
    return stage


def _fake_plan_pile(m, btype, rng, **kw):
    """Stands in for `quake_rubble.plan_pile` — same seam
    `test_quake_sliced.test_the_rubble_modules_are_only_touched_through_
    the_two_hooks` stubs, so this runs whether or not the real rubble
    modules (heavy, and dependent on `RUBBLE_ASSET_ROOT`) are reachable from
    this host. The returned `"stats"` shape is exactly `quake_rubble.
    plan_pile`'s own (`fall_sides`/`reach_m`/`extent_m`/`crown_m`) — what
    `quake_sliced._author_pile` forwards into `ctx["rubble"]`."""
    sides = list(kw.get("sides") or ["S"])
    return {"mound": None, "large": [], "instances": {},
            "stats": {"fall_sides": sides,
                      "reach_m": {s: 3.0 for s in sides},
                      "extent_m": {s: 2.5 for s in sides},
                      "crown_m": float(kw.get("crown_m") or 2.0)}}


def _fake_author(stage, parent, plan, **kw):
    from pxr import Sdf, UsdGeom

    tag = kw.get("tag", "rubble")
    path = "{0}/rubble_{1}".format(parent, _safe(tag))
    UsdGeom.Xform.Define(stage, Sdf.Path(path))
    return {"mound": path, "apron": None, "static": [path], "large": [],
            "instancers": [], "all": [path]}


def _safe(s):
    return "".join(c if (c.isalnum() or c in "._-") else "_" for c in str(s))


class _AutoMats(dict):
    """`ctx["mats"]` for an offline stage-level test — a plain `dict` that
    auto-vivifies a trivial LOCAL `UsdShade.Material` (no Nucleus reference)
    the first time any key is indexed (`mats["rebar"]`, the way `quake_flow`'s
    ground-response recipes — `_c_overturn_ground`/`_rebar_tuft`/`_cyl` —
    read theirs). `dict.get(key)` still returns `None` for an unset key
    (`__missing__` is only invoked by `__getitem__`), which is exactly the
    "no such material" fallback `_glass_void` already checks for — so this
    plays both roles the real `quake_flow.materials()` dict does, with no
    network access at all."""

    def __init__(self, stage, parent):
        super().__init__()
        self._stage = stage
        self._parent = parent

    def __missing__(self, key):
        from pxr import Sdf, UsdShade

        path = "{0}/QuakeLooksFake/{1}".format(self._parent, _safe(key))
        mat = (UsdShade.Material.Get(self._stage, path)
              or UsdShade.Material.Define(self._stage, Sdf.Path(path)))
        self[key] = mat
        return mat


def _apply_on_bare_stage(grade, btype, seed=13):
    """`(stage, ctx, plan, info, pls)` after a real `apply_plan` call on a
    freshly-authored bare stage — one building, ready for section C/D's
    assertions."""
    from pxr import Usd, UsdGeom

    pls, style, _grid = fake_sliced_building(seed=seed)
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = btype
    rng = random.Random(seed)
    plan = qs.plan_damage(info, info["elements"], grade, btype, rng)

    stage = _new_bare_stage()
    for p in pls:
        _author_piece_mesh(stage, p)

    ctx = {"stage": stage, "parent": "/World/cell", "info": info, "rng": rng,
          "nrng": None, "mats": _AutoMats(stage, "/World/cell"), "cache": {},
          "tag": "t0", "loose": [],
          "static_extra": [], "authored": [], "notes": [],
          "fit": {"slabs": {}, "columns": {}, "partitions": [], "props": {},
                  "all": []}}

    old_pp, old_au = qs.PLAN_PILE, qs.AUTHOR
    qs.PLAN_PILE, qs.AUTHOR = _fake_plan_pile, _fake_author
    try:
        qs.apply_plan(stage, ctx, plan, verbose=False)
    finally:
        qs.PLAN_PILE, qs.AUTHOR = old_pp, old_au
    return stage, ctx, plan, info, pls


def test_apply_plan_deactivates_removed_pieces():
    if not HAVE_USD:
        return
    from pxr import Sdf

    stage, _ctx, plan, _info, _pls = _apply_on_bare_stage("DG3", "urm")
    assert plan["removed"], "DG3 urm should remove some pieces (glass aside)"
    for path in plan["removed"]:
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        assert prim.IsValid(), path
        assert not prim.IsActive(), "{0} should be deactivated".format(path)


def test_apply_plan_moves_displaced_pieces_to_the_predicted_point():
    if not HAVE_USD:
        return
    from pxr import Sdf, UsdGeom

    stage, _ctx, plan, info, _pls = _apply_on_bare_stage("DG4", "urm")
    assert plan["displaced"], "DG4 urm should displace some pieces"
    by_path = _by_path(info)
    xc = UsdGeom.XformCache()
    for path, spec in list(plan["displaced"].items())[:8]:
        e = by_path[path]
        want = qs.apply_rigid(spec, (float(e["x"]), float(e["y"]), float(e["z"])))
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        assert prim.IsValid()
        got = xc.GetLocalToWorldTransform(prim).ExtractTranslation()
        for a, b in zip(want, (got[0], got[1], got[2])):
            assert abs(a - b) < 1e-4, (path, want, tuple(got))


def test_apply_plan_authors_one_pile_per_pile_spec_and_records_rubble():
    if not HAVE_USD:
        return
    stage, ctx, plan, _info, _pls = _apply_on_bare_stage("DG5", "urm")
    assert plan["piles"], "DG5 urm (masonry_collapse) must plan a pile"
    from pxr import Sdf

    assert len(ctx.get("rubble") or []) == len(plan["piles"])
    for spec, rec in zip(plan["piles"], ctx["rubble"]):
        # the STUB's own numbers, forwarded through `_author_pile` into
        # `ctx["rubble"]` — this is the fix `quake_gac_bake_launch_script.py`
        # depends on to fill its sidecar's `quake.fall_sides`/`extent_m`/
        # `crown_m` fields.
        assert rec["fall_sides"], spec
        assert rec["crown_m"] > 0.0
        assert rec["kind"] == spec["kind"]
    # every authored pile prim actually landed on the stage
    for path in ctx["static_extra"]:
        if "rubble_" in path:
            assert stage.GetPrimAtPath(Sdf.Path(path)).IsValid()


def test_apply_plan_on_overturn():
    """OV — the foundation family's rigid whole-body case, routed through
    this module's OWN `_ov_apply` (never `quake_flow.r_overturn`, which
    fractures the landing side)."""
    if not HAVE_USD:
        return
    stage, ctx, plan, _info, _pls = _apply_on_bare_stage("OV", "rc")
    assert plan["ground"] and plan["ground"]["recipe"] == "overturn"
    assert plan["piles"], "overturn lays a landing windrow"
    assert ctx.get("rubble")


# ---------------------------------------------------------------------------
# D. the export round-trip
# ---------------------------------------------------------------------------
def test_export_round_trips_after_apply_plan():
    if not HAVE_USD:
        return
    import tempfile

    from pxr import Sdf, Usd, UsdGeom

    stage, ctx, _plan, _info, _pls = _apply_on_bare_stage("DG5", "urm")

    fb.strip_physics(stage, root=None, remove_prims=fb.STRIP_PRIMS,
                     verbose=False)
    before = sum(1 for _ in Usd.PrimRange(stage.GetPseudoRoot(),
                                          Usd.PrimAllPrimsPredicate))

    tmp_dir = tempfile.mkdtemp(prefix="qgac_export_")
    try:
        out = os.path.join(tmp_dir, "gac_test_DG5_s1.usd")
        stage.GetRootLayer().Export(out)
        assert os.path.exists(out)

        reopened = Usd.Stage.Open(out)
        assert reopened is not None
        after = sum(1 for _ in Usd.PrimRange(reopened.GetPseudoRoot(),
                                             Usd.PrimAllPrimsPredicate))
        assert after == before, (before, after)
        assert str(reopened.GetDefaultPrim().GetPath()) == "/World"

        info = fb.verify_export(out, doomed=("/no/such/path",),
                                expect_root="/World/cell",
                                check_remote=False, verbose=False)
        assert info["meshes"] > 0
        assert info["n_physics_prims"] == 0
        assert info["n_flow_prims"] == 0
        assert info["n_doomed_prims"] == 0
        assert info["ok"], info
    finally:
        import shutil

        shutil.rmtree(tmp_dir, ignore_errors=True)


# ---------------------------------------------------------------------------
# `--verify <dir>` — re-check bakes that already exist (mirrors
# `test_fire_bake.py`'s own `--verify` mode, so `quake_gac_bake.sh
# --verify-only` has the same shape as `fire_bake.sh --verify-only`)
# ---------------------------------------------------------------------------
def verify_paths(spec):
    import glob as _g

    paths = []
    for item in [q.strip() for q in str(spec).split(",") if q.strip()]:
        if os.path.isdir(item):
            paths += sorted(_g.glob(os.path.join(item, "gac_*.usd")))
        else:
            paths.append(item)
    if not paths:
        print("[test_quake_gac_bake] nothing matched {0!r}".format(spec))
        return 1
    bad = 0
    for p in paths:
        js = os.path.splitext(p)[0] + ".json"
        doomed = ("/src",)
        if os.path.exists(js):
            try:
                with open(js) as fh:
                    doc = json.load(fh)
                doomed = (doc.get("cell", fb.BAKE_ROOT) + "/src",)
                q = doc.get("quake") or {}
                print("[sidecar] {0}: grade {1}, btype {2}, {3} fall side(s), "
                      "crown {4}, src_kept={5}".format(
                          os.path.basename(js), q.get("grade"), q.get("btype"),
                          len(q.get("fall_sides") or []), q.get("crown_m"),
                          doc.get("src_kept")))
            except Exception as exc:
                print("[sidecar] {0} unreadable: {1}".format(js, exc))
                bad += 1
        else:
            print("[sidecar] MISSING for {0}".format(os.path.basename(p)))
            bad += 1
        info = fb.verify_export(p, doomed=doomed, expect_root=fb.BAKE_ROOT,
                                check_remote=True, verbose=True)
        bad += 0 if info.get("ok") else 1
    print("\n{0}/{1} bake(s) clean".format(len(paths) - bad, len(paths)))
    return 0 if bad == 0 else 1


TESTS = [test_plan_damage_runs_dg3_dg4_dg5_ov_on_both_collapsing_ladders,
         test_plan_to_json_round_trips_and_is_idempotent,
         test_plan_to_json_turns_every_tuple_into_a_list,
         test_no_displaced_piece_lands_below_the_foundation_line,
         test_apply_plan_deactivates_removed_pieces,
         test_apply_plan_moves_displaced_pieces_to_the_predicted_point,
         test_apply_plan_authors_one_pile_per_pile_spec_and_records_rubble,
         test_apply_plan_on_overturn,
         test_export_round_trips_after_apply_plan]


def main(argv):
    if "--verify" in argv:
        i = argv.index("--verify")
        return verify_paths(argv[i + 1] if len(argv) > i + 1
                            else "/isaac-sim/AirStack/scene_gen/assets/gac_quake")
    print("test_quake_gac_bake  (pxr {0})".format(
        "present" if HAVE_USD else "ABSENT — USD tests skipped"))
    failed = 0
    for t in TESTS:
        try:
            t()
        except AssertionError as exc:
            failed += 1
            print("  {0:<19} FAIL  {1}".format(t.__name__, exc))
        except Exception as exc:
            failed += 1
            import traceback
            traceback.print_exc()
            print("  {0:<19} ERROR {1}".format(t.__name__, exc))
    print("\n{0}/{1} passed".format(len(TESTS) - failed, len(TESTS)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


def test_author_pile_runs_the_real_emitter():
    """Regression for the pilot bake's TypeError: `_author_pile` must hand
    `quake_rubble_usd.author` a CALLABLE uid (the emitter calls it once per
    large element). Runs the REAL emitter on an in-memory stage."""
    import random
    import numpy as np
    from pxr import Usd, UsdGeom
    from disaster import quake_sliced as qs
    from disaster import quake_rubble as qr
    from disaster import quake_rubble_usd as qru

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.DefinePrim("/World", "Xform")
    ctx = {"stage": stage, "parent": "/World", "tag": "t", "rng": random.Random(4),
           "mats": None, "static_extra": [], "authored": [], "plate_ok": None,
           "info": {"masses": {"main": {"W": 12.0, "D": 10.0, "H": 9.0, "top": 9.0,
                                        "cx": 0.0, "cy": 0.0, "yaw": 0.0, "z0": 0.0}}},
           "_uid": 0}
    plan = {"btype": "urm"}
    spec = {"kind": "windrow", "sides": ("S",), "depth_m": 0.5, "tag": "w"}
    n = qs._author_pile(stage, ctx, plan, spec,
                        0, qr.plan_pile, qru.author)
    assert spec.get("authored", {}).get("n_all", 0) > 0
    assert ctx["authored"], "the emitter authored nothing"
    assert ctx.get("rubble"), "pile stats were not recorded"
