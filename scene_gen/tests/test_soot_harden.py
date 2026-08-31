#!/usr/bin/env python3
"""test_soot_harden.py — a sooted material COPY must read matte, not
mirror-glossy.

    python3 scene_gen/tests/test_soot_harden.py
    pytest -q scene_gen/tests/test_soot_harden.py

    # the USD-dependent tests need pxr — run under the container:
    docker exec isaac-sim bash -c \
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
       /isaac-sim/AirStack/scene_gen/tests/test_soot_harden.py"

WHY THIS EXISTS
---------------
`urban_fire._bind_soot` rebinds a sooted module to a COPY of its own
material (`soot_plume.piece_material_like`: an internal reference to the
source material with only the diffuse map swapped) — normal, roughness, AO
and metallic pass through untouched. That is right for a lightly-fringed
module next to an untouched neighbour, but wrong once the coverage is
heavy: a real curtain-wall tower (dtc Amar_Tower, fire_dtc3 bench,
2026-08-30) kept its burnt band mirror-reflective because the copy still
carried the source's own metalness/roughness texture straight through
(measured with `tools/piece_soot_probe.py`: `roughness=tex:...StoA_
Metalness.jpg  metallic=tex:...StoA_Metalness.jpg` on the sooted copy).

`disaster/soot_bake.py` adds `harden_sooted_shader` (matte a material COPY
in place, given a coverage number), `matte_values` (the pure metallic/
roughness curve it uses), and two integration points that need no change
to `_bind_soot`/`bake_atlases` themselves: `bake_module`'s own coverage
side-channel (`_record_coverage`/`_content_digest`) and `harden_baked_
materials`, a sweep that finds a bake's own `sootbake_<digest>.png` copies
on a stage after the fact and matte the significantly-sooted ones.

A. `matte_values` — pure numeric, host-side, no USD.
B. `harden_sooted_shader` — needs `pxr`: disconnects a TEXTURED metallic/
   roughness/ior/clearcoat on the COPY only, leaves the SOURCE (and a
   below-threshold copy) exactly as authored.
C. `bake_module` + `harden_baked_materials` — the same coverage number
   `bake_module` already computes for its composite, recovered afterwards
   from a stage purely by the `sootbake_<digest>.png` filename `_bind_soot`
   already gives its copies, with no line of `_bind_soot` touched.
"""

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)

from disaster import soot_bake as sb                          # noqa: E402

try:
    from pxr import Sdf, Usd, UsdGeom, UsdShade                # noqa: E402
    HAVE_USD = True
except Exception:                                              # pragma: no cover
    HAVE_USD = False


# ---------------------------------------------------------------------------
# A. matte_values — pure numeric
# ---------------------------------------------------------------------------
def test_matte_values_below_threshold_is_none():
    assert sb.matte_values(0.0) is None
    assert sb.matte_values(sb.SOOT_HARDEN_MIN - 0.01) is None


def test_matte_values_at_min_and_full():
    lo = sb.matte_values(sb.SOOT_HARDEN_MIN)
    hi = sb.matte_values(sb.SOOT_HARDEN_FULL)
    beyond = sb.matte_values(1.0)

    def _close(a, b, tol=1e-9):
        return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol

    assert _close(lo, (sb.METALLIC_AT_MIN, sb.ROUGHNESS_AT_MIN))
    assert _close(hi, (sb.METALLIC_AT_FULL, sb.ROUGHNESS_AT_FULL))
    # beyond SOOT_HARDEN_FULL clamps, does not keep extrapolating
    assert _close(beyond, hi)


def test_matte_values_interpolates_and_stays_in_task_spec_range():
    mid_cov = 0.5 * (sb.SOOT_HARDEN_MIN + sb.SOOT_HARDEN_FULL)
    m, r = sb.matte_values(mid_cov)
    # strictly between the two endpoints (monotone ladder, not a jump)
    assert sb.METALLIC_AT_FULL < m < sb.METALLIC_AT_MIN
    assert sb.ROUGHNESS_AT_MIN < r < sb.ROUGHNESS_AT_FULL
    for cov in np.linspace(0.0, 1.0, 41):
        vals = sb.matte_values(float(cov))
        if vals is None:
            continue
        metallic_v, roughness_v = vals
        # the task's own spec: metallic in 0..0.1, roughness >= 0.65
        assert 0.0 <= metallic_v <= 0.1, (cov, metallic_v)
        assert roughness_v >= 0.65, (cov, roughness_v)


# ---------------------------------------------------------------------------
# helpers for the USD-dependent tests: a tiny UsdPreviewSurface network with
# TEXTURED metallic/roughness/ior, matching the real kit materials' shape
# ---------------------------------------------------------------------------
def _textured_preview_surface_material(stage, path, diffuse_tex="clean.png"):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")

    dtex = UsdShade.Shader.Define(stage, path + "/DiffTex")
    dtex.CreateIdAttr("UsdUVTexture")
    dtex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(diffuse_tex))
    dtex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        dtex.ConnectableAPI(), "rgb")

    mtex = UsdShade.Shader.Define(stage, path + "/MetalRoughTex")
    mtex.CreateIdAttr("UsdUVTexture")
    mtex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath("StoA_Metalness.jpg"))
    mtex.CreateOutput("r", Sdf.ValueTypeNames.Float)
    mtex.CreateOutput("g", Sdf.ValueTypeNames.Float)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).ConnectToSource(
        mtex.ConnectableAPI(), "r")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).ConnectToSource(
        mtex.ConnectableAPI(), "g")

    itex = UsdShade.Shader.Define(stage, path + "/IorTex")
    itex.CreateIdAttr("UsdUVTexture")
    itex.CreateOutput("r", Sdf.ValueTypeNames.Float)
    sh.CreateInput("ior", Sdf.ValueTypeNames.Float).ConnectToSource(
        itex.ConnectableAPI(), "r")
    sh.CreateInput("clearcoat", Sdf.ValueTypeNames.Float).Set(1.0)
    sh.CreateInput("clearcoatRoughness", Sdf.ValueTypeNames.Float).Set(0.1)

    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat, sh


def _sooted_copy(stage, src_path, dst_path, tex_name):
    """The exact shape `soot_plume.piece_material_like` makes: an internal
    reference to the source, with only the diffuse texture's `file`
    overridden on the copy."""
    dst_mat = UsdShade.Material.Define(stage, dst_path)
    dst_mat.GetPrim().GetReferences().AddInternalReference(Sdf.Path(src_path))
    dst_tex = UsdShade.Shader(stage.GetPrimAtPath(dst_path + "/DiffTex"))
    dst_tex.GetInput("file").Set(Sdf.AssetPath(tex_name))
    dst_sh = UsdShade.Shader(stage.GetPrimAtPath(dst_path + "/Shader"))
    return dst_mat, dst_sh


def _new_stage():
    st = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    return st


# ---------------------------------------------------------------------------
# B. harden_sooted_shader — needs pxr
# ---------------------------------------------------------------------------
def test_harden_sooted_shader_mattes_copy_leaves_source_untouched():
    if not HAVE_USD:
        print("  harden_sooted_shader  SKIP (no pxr -- run under usd_python.sh)")
        return
    st = _new_stage()
    src_mat, src_sh = _textured_preview_surface_material(st, "/W/SrcMat")
    dst_mat, dst_sh = _sooted_copy(st, "/W/SrcMat", "/W/Copy",
                                   "sootbake_deadbeefcafef00d.png")

    assert dst_sh.GetInput("metallic").HasConnectedSource()
    assert dst_sh.GetInput("roughness").HasConnectedSource()

    n = sb.harden_sooted_shader(dst_mat, 0.9)   # heavy coverage
    assert n >= 2, n

    m_in = dst_sh.GetInput("metallic")
    r_in = dst_sh.GetInput("roughness")
    assert not m_in.HasConnectedSource()
    assert not r_in.HasConnectedSource()
    exp_m, exp_r = sb.matte_values(0.9)
    assert abs(float(m_in.Get()) - exp_m) < 1e-6
    assert abs(float(r_in.Get()) - exp_r) < 1e-6
    assert float(m_in.Get()) <= 0.1
    assert float(r_in.Get()) >= 0.65

    # -- (b): the SOURCE material is a separate prim spec; disconnecting on
    # the copy must not have touched it (this is the "only the copies
    # change" requirement, checked directly against the un-sooted original)
    assert src_sh.GetInput("metallic").HasConnectedSource()
    assert src_sh.GetInput("roughness").HasConnectedSource()

    # round-trips: an override authored on a referencing prim must survive
    # export/reopen, not just look right in the same in-memory session
    tmp = "/tmp/soot_harden_test_roundtrip.usda"
    st.GetRootLayer().Export(tmp)
    st2 = Usd.Stage.Open(tmp)
    dst_sh2 = UsdShade.Shader(st2.GetPrimAtPath("/W/Copy/Shader"))
    src_sh2 = UsdShade.Shader(st2.GetPrimAtPath("/W/SrcMat/Shader"))
    assert not dst_sh2.GetInput("metallic").HasConnectedSource()
    assert abs(float(dst_sh2.GetInput("metallic").Get()) - exp_m) < 1e-6
    assert src_sh2.GetInput("metallic").HasConnectedSource()


def test_harden_sooted_shader_below_threshold_is_a_no_op():
    if not HAVE_USD:
        print("  below-threshold no-op  SKIP (no pxr)")
        return
    st = _new_stage()
    _src, _ = _textured_preview_surface_material(st, "/W/SrcMat2")
    dst_mat, dst_sh = _sooted_copy(st, "/W/SrcMat2", "/W/Copy2",
                                   "sootbake_0000000000000001.png")

    n = sb.harden_sooted_shader(dst_mat, 0.1)   # light -- below SOOT_HARDEN_MIN
    assert n == 0
    # -- (b): a lightly-sooted copy is left completely alone, textures and all
    assert dst_sh.GetInput("metallic").HasConnectedSource()
    assert dst_sh.GetInput("roughness").HasConnectedSource()
    assert dst_sh.GetInput("clearcoat").Get() == 1.0


def test_harden_sooted_shader_neutralizes_ior_and_clearcoat():
    if not HAVE_USD:
        print("  ior/clearcoat neutralise  SKIP (no pxr)")
        return
    st = _new_stage()
    _src, src_sh = _textured_preview_surface_material(st, "/W/SrcMat3")
    dst_mat, dst_sh = _sooted_copy(st, "/W/SrcMat3", "/W/Copy3",
                                   "sootbake_1111111111111111.png")

    assert dst_sh.GetInput("ior").HasConnectedSource()
    assert dst_sh.GetInput("clearcoat").Get() == 1.0

    sb.harden_sooted_shader(dst_mat, 1.0)

    ior_in = dst_sh.GetInput("ior")
    cc_in = dst_sh.GetInput("clearcoat")
    ccr_in = dst_sh.GetInput("clearcoatRoughness")
    assert not ior_in.HasConnectedSource()
    assert abs(float(ior_in.Get()) - sb.IOR_NEUTRAL) < 1e-6
    assert not cc_in.HasConnectedSource()
    assert float(cc_in.Get()) == 0.0
    assert not ccr_in.HasConnectedSource()
    # the source's clearcoat is untouched
    assert src_sh.GetInput("clearcoat").Get() == 1.0


# ---------------------------------------------------------------------------
# C. bake_module's coverage side-channel + harden_baked_materials sweep
# ---------------------------------------------------------------------------
def _tiny_sk(alpha=1.0, h=16, w=64):
    rgba = np.zeros((h, w, 4), dtype=np.float32)
    rgba[..., 3] = alpha
    rgba[..., :3] = 0.05          # near-black soot colour
    return {"rgba": rgba, "ppm": 4.0, "per": 16.0, "H": 4.0, "z0": 0.0,
            "offsets": {"S": 0.0, "E": 4.0, "N": 8.0, "W": 12.0}}


def _tiny_mass():
    return dict(W=4.0, D=4.0, cx=2.0, cy=2.0, yaw=0.0)


def test_bake_module_records_coverage_by_content_digest():
    sk = _tiny_sk(alpha=1.0)          # fully sooted everywhere
    m = _tiny_mass()
    points = np.array([[0, 0, 0], [4, 0, 0], [4, 0, 4], [0, 0, 4]],
                      dtype=np.float64)
    uv = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float64)
    pos, mask = sb.uv_position_map(points, [4], [0, 1, 2, 3], uv, "vertex",
                                   px=32)
    base = np.full((8, 8, 3), 0.6, dtype=np.float32)
    out = sb.bake_module(sk, "S", m, np.eye(4), pos, mask, base, px=32)

    digest = sb._content_digest(out)
    assert digest in sb._COVERAGE_BY_DIGEST
    # alpha is 1.0 everywhere the skin is sampled, over the whole covered face
    assert abs(sb._COVERAGE_BY_DIGEST[digest] - 1.0) < 1e-6


def test_harden_baked_materials_sweeps_only_sooted_digests_above_threshold():
    if not HAVE_USD:
        print("  harden_baked_materials  SKIP (no pxr)")
        return
    sk_heavy = _tiny_sk(alpha=1.0)
    sk_light = _tiny_sk(alpha=0.05)   # well under SOOT_HARDEN_MIN
    m = _tiny_mass()
    points = np.array([[0, 0, 0], [4, 0, 0], [4, 0, 4], [0, 0, 4]],
                      dtype=np.float64)
    uv = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float64)
    pos, mask = sb.uv_position_map(points, [4], [0, 1, 2, 3], uv, "vertex",
                                   px=32)
    base = np.full((8, 8, 3), 0.6, dtype=np.float32)

    out_heavy = sb.bake_module(sk_heavy, "S", m, np.eye(4), pos, mask, base,
                               px=32)
    out_light = sb.bake_module(sk_light, "S", m, np.eye(4), pos, mask, base,
                               px=32)
    digest_heavy = sb._content_digest(out_heavy)
    digest_light = sb._content_digest(out_light)
    assert digest_heavy != digest_light

    st = _new_stage()
    src, _ = _textured_preview_surface_material(st, "/W/Src")
    heavy_mat, heavy_sh = _sooted_copy(
        st, "/W/Src", "/W/FireLooks/soot_0",
        "sootbake_{0}.png".format(digest_heavy))
    light_mat, light_sh = _sooted_copy(
        st, "/W/Src", "/W/FireLooks/soot_1",
        "sootbake_{0}.png".format(digest_light))
    # a material NOT baked through this module at all (e.g. an untouched
    # neighbour, or gac_fire's own merged-atlas "gacsoot_" naming) must be
    # left alone -- no digest, no registry entry, no touch
    other_mat, other_sh = _sooted_copy(
        st, "/W/Src", "/W/FireLooks/soot_2", "wall_diffuse.jpg")

    n = sb.harden_baked_materials(st)
    assert n == 1, n

    assert not heavy_sh.GetInput("metallic").HasConnectedSource()
    assert float(heavy_sh.GetInput("roughness").Get()) >= 0.65

    # -- (b): the lightly-sooted copy (below SOOT_HARDEN_MIN) is unchanged
    assert light_sh.GetInput("metallic").HasConnectedSource()
    assert light_sh.GetInput("roughness").HasConnectedSource()
    # -- and the unrelated/non-sootbake material is unchanged too
    assert other_sh.GetInput("metallic").HasConnectedSource()
    assert other_sh.GetInput("roughness").HasConnectedSource()
    assert src.GetPrim().IsValid()   # the source itself was never touched


if __name__ == "__main__":
    print("pxr: " + ("present" if HAVE_USD else "ABSENT -- USD tests skipped"))
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)
