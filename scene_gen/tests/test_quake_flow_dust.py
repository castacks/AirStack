#!/usr/bin/env python3
"""test_quake_flow_dust.py — does `quake_flow._dust_loose` actually dust a
collapse's loose fragments, and leave everything else alone?

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        --with pytest python -m pytest tests/test_quake_flow_dust.py -q

WHY THIS EXISTS
---------------
Isaac RTX renders of the earthquake DG5 piles (`~/docker/isaac-sim/logs/
eq500_gui/b2_office_DG5_obl.png`, `~/docker/isaac-sim/logs/r4_commercial/
0_commercial_DG5_close.png`) showed the LOOSE FRAGMENTS a collapse recipe
fractures off a kit wall/slab/column keeping the standing building's own
PRISTINE bound material — clean saturated brick, clean pale concrete — so a
DG5 pile reads as a heap of new toy bricks rather than dusty rubble (the
user, round 5: "too animated and not real"). `_dust_loose` (in
`disaster/quake_flow.py`, next to `_rubble`/`_RUBBLE_MODE`) fixes this by
binding a DARKER, DESATURATED COPY of each fragment's own bound material
onto it, `strongerThanDescendants`, `EQ_RUBBLE=v2` only.

This is host-side (`usd-core`, no Kit/Isaac): a real in-memory `Usd.Stage`
(no Nucleus, no referenced kit assets) with a hand-built source material
shaped like the ones `_chunk_material`/`_clad_material`/`materials()`
actually bind to a fragment — an OmniPBR MDL `Shader` (`info:mdl:
sourceAsset` = `OmniPBR.mdl`, a bound `diffuse_texture`) plus a
`UsdPreviewSurface` fallback, the same two-shader shape `_add_preview_
fallback` builds for the round-4 rubble-pile materials. `ctx` is the same
kind of stub dict `test_quake_flow_rubble_routing.py`'s `_fake_ctx()` uses,
with a REAL stage instead of `None` (`_dust_loose` genuinely touches USD).

WHAT THIS CANNOT SEE: whether the tint reads right under RTX, whether a
REAL Nucleus-referenced kit material copies as cleanly as this hand-built
one does — that needs a render.
"""
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import quake_flow as qf                     # noqa: E402

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt        # noqa: E402


PARENT = "/World/Bldg"
SCOPE = PARENT + "/QuakeLooks"


def _new_stage():
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, PARENT)
    UsdGeom.Scope.Define(st, SCOPE)
    return st


def _make_source_material(st, name="brick", tex="./brick.jpg"):
    """A material shaped like `_clad_material`'s / `materials()`'s own
    fragment materials: an OmniPBR MDL `Shader` with a bound
    `diffuse_texture`, plus a `UsdPreviewSurface` fallback reading a
    constant colour — the same two-shader shape `quake_rubble_usd.
    _add_preview_fallback` builds for every textured rubble-v2 look."""
    mat_path = "{0}/{1}".format(SCOPE, name)
    mat = UsdShade.Material.Define(st, mat_path)
    sh = UsdShade.Shader.Define(st, mat_path + "/Shader")
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(tex))
    sh.CreateInput("diffuse_color_constant",
                   Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 1.0))
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")

    prev = UsdShade.Shader.Define(st, mat_path + "/PreviewSurface")
    prev.CreateIdAttr("UsdPreviewSurface")
    prev.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.8, 0.5, 0.4))
    mat.CreateSurfaceOutput().ConnectToSource(prev.ConnectableAPI(), "surface")
    return mat


def _make_fragments(st, mat, n=3, prefix="frag"):
    paths = []
    for i in range(n):
        p = "{0}/{1}_{2}".format(PARENT, prefix, i)
        me = UsdGeom.Mesh.Define(st, p)
        me.CreatePointsAttr(Vt.Vec3fArray(
            [Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 1, 0)]))
        me.CreateFaceVertexCountsAttr(Vt.IntArray([3]))
        me.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2]))
        UsdShade.MaterialBindingAPI(me.GetPrim()).Bind(mat)
        paths.append(p)
    return paths


def _fake_ctx(stage, seed=1, btype="urm"):
    return {"stage": stage, "parent": PARENT, "tag": "t0",
            "rng": random.Random(seed), "nrng": None, "mats": {}, "cache": {},
            "info": {"type": btype, "masses": {}}, "authored": [],
            "static_extra": [], "loose": [], "velocity": {}, "notes": [],
            "n_uid": 0}


def _bound_path(st, prim_path):
    pr = st.GetPrimAtPath(prim_path)
    bm = UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0]
    return str(bm.GetPath()) if bm and bm.GetPrim().IsValid() else None


# ---------------------------------------------------------------------------
# EQ_RUBBLE=v2, skip_p forced to 0: every fragment gets dusted, one copy.
# ---------------------------------------------------------------------------
def test_dust_loose_binds_one_dusted_copy_stronger_than_descendants():
    st = _new_stage()
    src = _make_source_material(st, "brick")
    frags = _make_fragments(st, src, n=3)
    ctx = _fake_ctx(st)

    prev_mode = qf._RUBBLE_MODE
    qf._RUBBLE_MODE = "v2"
    try:
        qf._dust_loose(ctx, frags, tint=(0.5, 0.5, 0.5), desat=0.2, skip_p=0.0)
    finally:
        qf._RUBBLE_MODE = prev_mode

    expected = "{0}/dust_brick".format(SCOPE)
    bound = {_bound_path(st, p) for p in frags}
    # every fragment moved off the source and onto the SAME dusted copy
    assert bound == {expected}, bound

    dst = st.GetPrimAtPath(expected)
    assert dst.IsValid()
    dsh = UsdShade.Shader.Get(st, expected + "/Shader")
    assert dsh and dsh.GetPrim().IsValid()
    tint_in = dsh.GetInput("diffuse_tint")
    assert tint_in is not None
    got_tint = tuple(tint_in.Get())
    assert all(abs(a - b) < 1e-6 for a, b in zip(got_tint, (0.5, 0.5, 0.5)))
    desat_in = dsh.GetInput("albedo_desaturation")
    assert desat_in is not None and abs(float(desat_in.Get()) - 0.2) < 1e-6
    # the texture itself rode along on the copy, untouched
    tex_in = dsh.GetInput("diffuse_texture")
    assert tex_in is not None
    assert str(tex_in.Get().path).endswith("brick.jpg")

    # cached in ctx["mats"], the same discipline _a_mat/_clad_material use
    assert ctx["mats"].get("dust_brick") is not None
    assert str(ctx["mats"]["dust_brick"].GetPath()) == expected


def test_dust_loose_does_not_touch_the_source_material():
    st = _new_stage()
    src = _make_source_material(st, "brick")
    frags = _make_fragments(st, src, n=3)
    ctx = _fake_ctx(st)

    prev_mode = qf._RUBBLE_MODE
    qf._RUBBLE_MODE = "v2"
    try:
        qf._dust_loose(ctx, frags, tint=(0.5, 0.5, 0.5), desat=0.2, skip_p=0.0)
    finally:
        qf._RUBBLE_MODE = prev_mode

    src_sh = UsdShade.Shader.Get(st, "{0}/brick/Shader".format(SCOPE))
    assert src_sh and src_sh.GetPrim().IsValid()
    # the source shader never gained a diffuse_tint/albedo_desaturation VALUE
    # (`GetInput` on a not-yet-authored name still hands back a wrapper
    # object in this USD version, so the check is on the authored value)
    tint_attr = src_sh.GetInput("diffuse_tint")
    assert tint_attr is None or tint_attr.Get() is None
    desat_attr = src_sh.GetInput("albedo_desaturation")
    assert desat_attr is None or desat_attr.Get() is None
    # and its own diffuse colour/texture are exactly as authored
    assert tuple(src_sh.GetInput("diffuse_color_constant").Get()) == (1.0, 1.0, 1.0)
    assert str(src_sh.GetInput("diffuse_texture").Get().path).endswith("brick.jpg")


def test_dust_loose_only_authors_one_copy_for_many_fragments():
    st = _new_stage()
    src = _make_source_material(st, "brick")
    frags = _make_fragments(st, src, n=25, prefix="many")
    ctx = _fake_ctx(st)

    prev_mode = qf._RUBBLE_MODE
    qf._RUBBLE_MODE = "v2"
    try:
        qf._dust_loose(ctx, frags, tint=(0.5, 0.5, 0.5), desat=0.2, skip_p=0.0)
    finally:
        qf._RUBBLE_MODE = prev_mode

    bound = {_bound_path(st, p) for p in frags}
    assert bound == {"{0}/dust_brick".format(SCOPE)}
    # nothing named dust_brick_1, dust_brick_2, ... was ever authored
    scope_prim = st.GetPrimAtPath(SCOPE)
    dust_children = [c.GetName() for c in scope_prim.GetChildren()
                     if c.GetName().startswith("dust_")]
    assert dust_children == ["dust_brick"]


# ---------------------------------------------------------------------------
# the 15%-style skip share, forced to the two extremes
# ---------------------------------------------------------------------------
def test_dust_loose_skip_p_zero_dusts_everything():
    st = _new_stage()
    src = _make_source_material(st, "brick")
    frags = _make_fragments(st, src, n=6)
    ctx = _fake_ctx(st)

    prev_mode = qf._RUBBLE_MODE
    qf._RUBBLE_MODE = "v2"
    try:
        qf._dust_loose(ctx, frags, tint=(0.5, 0.5, 0.5), desat=0.2, skip_p=0.0)
    finally:
        qf._RUBBLE_MODE = prev_mode

    dusted = "{0}/dust_brick".format(SCOPE)
    assert all(_bound_path(st, p) == dusted for p in frags)


def test_dust_loose_skip_p_one_dusts_nothing():
    st = _new_stage()
    src = _make_source_material(st, "brick")
    frags = _make_fragments(st, src, n=6)
    ctx = _fake_ctx(st)
    src_path = str(src.GetPath())

    prev_mode = qf._RUBBLE_MODE
    qf._RUBBLE_MODE = "v2"
    try:
        qf._dust_loose(ctx, frags, tint=(0.5, 0.5, 0.5), desat=0.2, skip_p=1.0)
    finally:
        qf._RUBBLE_MODE = prev_mode

    assert all(_bound_path(st, p) == src_path for p in frags)
    # and no dust material was even authored
    assert not st.GetPrimAtPath("{0}/dust_brick".format(SCOPE)).IsValid()
    assert "dust_brick" not in ctx["mats"]


# ---------------------------------------------------------------------------
# EQ_RUBBLE=v1 must be byte-identical to round 3 — _dust_loose is a no-op.
# ---------------------------------------------------------------------------
def test_dust_loose_is_a_noop_under_rubble_v1():
    st = _new_stage()
    src = _make_source_material(st, "brick")
    frags = _make_fragments(st, src, n=4)
    ctx = _fake_ctx(st)
    src_path = str(src.GetPath())

    prev_mode = qf._RUBBLE_MODE
    qf._RUBBLE_MODE = "v1"
    try:
        qf._dust_loose(ctx, frags, tint=(0.5, 0.5, 0.5), desat=0.2, skip_p=0.0)
    finally:
        qf._RUBBLE_MODE = prev_mode

    assert all(_bound_path(st, p) == src_path for p in frags)
    assert ctx["mats"] == {}
    assert not st.GetPrimAtPath("{0}/dust_brick".format(SCOPE)).IsValid()


# ---------------------------------------------------------------------------
# tint by construction type, when tint/desat are left to auto-derive
# ---------------------------------------------------------------------------
def test_dust_tint_for_urm_and_rc():
    ctx_urm = _fake_ctx(None, btype="urm")
    ctx_rc = _fake_ctx(None, btype="rc")
    tint_urm, desat_urm = qf._dust_tint_for(ctx_urm)
    tint_rc, desat_rc = qf._dust_tint_for(ctx_rc)
    assert tint_urm == (0.62, 0.55, 0.50)
    assert abs(desat_urm - 0.35) < 1e-9
    assert tint_rc == (0.55, 0.54, 0.52)
    assert abs(desat_rc - 0.0) < 1e-9
    # rc_glass (a concrete-framed tower) falls onto the concrete tint
    ctx_glass = _fake_ctx(None, btype="rc_glass")
    assert qf._dust_tint_for(ctx_glass) == qf._dust_tint_for(ctx_rc)


def test_dust_loose_auto_derives_tint_from_construction_type():
    st = _new_stage()
    src = _make_source_material(st, "brick")
    frags = _make_fragments(st, src, n=2)
    ctx = _fake_ctx(st, btype="urm")

    prev_mode = qf._RUBBLE_MODE
    qf._RUBBLE_MODE = "v2"
    try:
        qf._dust_loose(ctx, frags, skip_p=0.0)     # no explicit tint/desat
    finally:
        qf._RUBBLE_MODE = prev_mode

    dsh = UsdShade.Shader.Get(st, "{0}/dust_brick/Shader".format(SCOPE))
    got_tint = tuple(dsh.GetInput("diffuse_tint").Get())
    assert all(abs(a - b) < 1e-6 for a, b in zip(got_tint, (0.62, 0.55, 0.50)))
    assert abs(float(dsh.GetInput("albedo_desaturation").Get()) - 0.35) < 1e-6


# ---------------------------------------------------------------------------
# unresolvable / missing material — skip silently, never raise
# ---------------------------------------------------------------------------
def test_dust_loose_skips_a_fragment_with_no_bound_material():
    st = _new_stage()
    p = "{0}/naked_0".format(PARENT)
    me = UsdGeom.Mesh.Define(st, p)
    me.CreatePointsAttr(Vt.Vec3fArray(
        [Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 1, 0)]))
    me.CreateFaceVertexCountsAttr(Vt.IntArray([3]))
    me.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2]))
    ctx = _fake_ctx(st)

    prev_mode = qf._RUBBLE_MODE
    qf._RUBBLE_MODE = "v2"
    try:
        qf._dust_loose(ctx, [p, "/World/Bldg/does_not_exist"],
                       tint=(0.5, 0.5, 0.5), desat=0.2, skip_p=0.0)
    finally:
        qf._RUBBLE_MODE = prev_mode
    # no exception, no material authored, the prim keeps having no binding
    bm = UsdShade.MaterialBindingAPI(
        st.GetPrimAtPath(p)).ComputeBoundMaterial()[0]
    assert not bm or not bm.GetPrim().IsValid()
    assert ctx["mats"] == {}


def test_dust_loose_empty_paths_and_none_paths_are_harmless():
    st = _new_stage()
    ctx = _fake_ctx(st)
    prev_mode = qf._RUBBLE_MODE
    qf._RUBBLE_MODE = "v2"
    try:
        qf._dust_loose(ctx, [], tint=(0.5, 0.5, 0.5), desat=0.2)
        qf._dust_loose(ctx, [None, ""], tint=(0.5, 0.5, 0.5), desat=0.2,
                       skip_p=0.0)
    finally:
        qf._RUBBLE_MODE = prev_mode
    assert ctx["mats"] == {}


# ---------------------------------------------------------------------------
# a second, DIFFERENT source material dusts to its OWN copy, not the first's
# ---------------------------------------------------------------------------
def test_dust_loose_different_sources_get_different_copies():
    st = _new_stage()
    brick = _make_source_material(st, "brick", tex="./brick.jpg")
    concrete = _make_source_material(st, "concrete", tex="./concrete.jpg")
    brick_frags = _make_fragments(st, brick, n=2, prefix="b")
    conc_frags = _make_fragments(st, concrete, n=2, prefix="c")
    ctx = _fake_ctx(st)

    prev_mode = qf._RUBBLE_MODE
    qf._RUBBLE_MODE = "v2"
    try:
        qf._dust_loose(ctx, brick_frags, tint=(0.62, 0.55, 0.50), desat=0.35,
                       skip_p=0.0)
        qf._dust_loose(ctx, conc_frags, tint=(0.55, 0.54, 0.52), desat=0.0,
                       skip_p=0.0)
    finally:
        qf._RUBBLE_MODE = prev_mode

    assert {_bound_path(st, p) for p in brick_frags} == \
        {"{0}/dust_brick".format(SCOPE)}
    assert {_bound_path(st, p) for p in conc_frags} == \
        {"{0}/dust_concrete".format(SCOPE)}
    assert set(ctx["mats"].keys()) == {"dust_brick", "dust_concrete"}


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
