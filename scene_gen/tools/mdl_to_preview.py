#!/usr/bin/env python
"""mdl_to_preview.py — rebuild a sliced bake's MDL materials as plain
`UsdPreviewSurface` networks driven by the MDL module's OWN textures.

    usd_python.sh scene_gen/tools/mdl_to_preview.py <in.usd> [out.usd]

WHY (2026-09-02). A sliced AEC brownstone renders WHITE while the raw asset
renders full brick — proven side by side in
`aec_material_probe_launch_script.py` (raw column brick, sliced column white).
Two things were tried and MEASURED not to fix it: binding the source Material
prim directly (it composes to a typeless stub in the exported bake), and
cloning the MDL shader with its module path (renders white even with
`MDL_USER_PATH`/`MDL_SYSTEM_PATH`/`--/renderer/mdl/searchPaths` pointed at the
module directory — MDL resolves by module name against a search path, and a
re-authored absolute path does not reliably rehydrate).

What DOES work is not using MDL at all. These modules are plain text wrapping
ordinary OmniPBR textures:

    Brick_Wall_Red.mdl -> ./Brick_Wall_Red/Brick_Wall_Red_BaseColor.png
                          ./Brick_Wall_Red/Brick_Wall_Red_N.png
                          ./Brick_Wall_Red/Brick_Wall_Red_ORM.png

so the textures are recoverable by READING THE MODULE, and a
`UsdPreviewSurface` fed from them is self-contained, renders in any consumer,
and — the reason this matters for fire — puts the base colour back in an
ordinary PNG that `soot_plume`/`soot_bake` can composite soot INTO, exactly
the path GAC and MCE already use. No decals.

ORM packing is the Omniverse convention: R = ambient occlusion,
G = roughness, B = metallic, so one texture feeds two inputs via two readers.
"""
import os
import re
import sys

from pxr import Sdf, Usd, UsdGeom, UsdShade  # noqa: E402

TEX_RE = re.compile(r'"([^"]+\.(?:png|jpg|jpeg|exr))"', re.I)



SOOT_DIR = os.environ.get("AEC_SOOT_TEX_DIR", "/isaac-sim/.cache/aec_soot_tex")
SOOT = float(os.environ.get("AEC_SOOT", "0"))          # 0..1, 0 = clean


def sooted_copy(src):
    """A soot-darkened copy of a base-colour map, cached on disk.

    Soot is composited INTO the texture — the same thing `soot_plume`/
    `soot_bake` do for GAC and MCE — so it is part of the material and needs
    no decal geometry. Darkening ramps with V (heaviest at the top of the
    map, where a compartment fire's plume actually deposits) and desaturates
    toward carbon, so brick reads as scorched brick rather than grey paint.
    """
    if SOOT <= 0.0:
        return src
    try:
        from PIL import Image
        import numpy as np
    except Exception:
        return src
    try:
        os.makedirs(SOOT_DIR, exist_ok=True)
        out = os.path.join(SOOT_DIR, "soot%02d_%s" % (
            int(round(SOOT * 99)), os.path.basename(src).rsplit(".", 1)[0] + ".png"))
        if os.path.isfile(out):
            return out
        im = Image.open(src).convert("RGB")
        a = np.asarray(im).astype("float32") / 255.0
        # UNIFORM WITHIN THE TEXTURE. An earlier version ramped soot over the
        # map's V axis to mimic a plume — but these maps TILE across a wall,
        # so the ramp repeated every tile and read as "a repeating pattern"
        # (user, 2026-09-02). Height variation belongs to the per-part BAND
        # (`aec_soot._soot_at`), never to the inside of a tiling texture.
        k = float(SOOT)
        # MULTIPLICATIVE DARKENING, HUE PRESERVED. Blending toward a greyscale
        # mean turned red brick into "black and white instead of base colour"
        # (same review). Soot darkens a surface; it does not desaturate it to
        # grey until it is very heavy, so chroma is only pulled at the top of
        # the range and even then only partly.
        a = a * (1.0 - 0.80 * k)
        if k > 0.60:
            t = 0.30 * (k - 0.60) / 0.40
            g = a.mean(axis=2, keepdims=True)
            a = a * (1.0 - t) + g * t
        Image.fromarray((np.clip(a, 0, 1) * 255.0 + 0.5).astype("uint8")).save(out)
        return out
    except Exception:
        return src


def module_textures(mdl_path):
    """`{"base":…, "normal":…, "orm":…}` absolute paths, from the module."""
    out = {}
    try:
        with open(mdl_path, "rb") as fh:
            txt = fh.read().decode("utf-8", "ignore")
    except Exception:
        return out
    base_dir = os.path.dirname(os.path.abspath(mdl_path))
    for raw in TEX_RE.findall(txt):
        if "/.thumbs/" in raw.replace("\\", "/"):
            continue        # module preview thumbnails, never surface maps
        p = os.path.normpath(os.path.join(base_dir, raw))
        if not os.path.isfile(p):
            continue
        low = os.path.basename(p).lower()
        # NAMING VARIES BY LIBRARY, and getting this wrong is silent: the
        # OmniPBR-style modules ship `*_BaseColor.png` while vMaterials ship
        # `*_diff.jpg` / `*_norm.jpg` / `*_multi_R_rough_G_ao_B_height.jpg`.
        # A classifier that only knew "basecolor" put every vMaterials BRICK
        # facade in the no-base-map bucket — 136 of 153 materials on a
        # brownstone — and the slices rendered white while the parser
        # reported success (measured 2026-09-02).
        if any(k in low for k in ("basecolor", "albedo", "_diff", "diffuse",
                                  "_col", "_d.")):
            out.setdefault("base", p)
        elif any(k in low for k in ("_norm", "normal", "_n.")):
            out.setdefault("normal", p)
        elif any(k in low for k in ("orm", "rough", "_multi", "metal")):
            out.setdefault("orm", p)
    return out


def rebuild(stage, mat_prim, tex):
    """Replace `mat_prim`'s shader network with a PreviewSurface using `tex`."""
    mat = UsdShade.Material(mat_prim)
    for child in list(mat_prim.GetChildren()):
        stage.RemovePrim(child.GetPath())
    p = mat_prim.GetPath()
    st_reader = UsdShade.Shader.Define(stage, p.AppendChild("stReader"))
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    sh = UsdShade.Shader.Define(stage, p.AppendChild("PreviewSurface"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    def _tex(name, path, out_name, ttype=Sdf.ValueTypeNames.Float3):
        t = UsdShade.Shader.Define(stage, p.AppendChild(name))
        t.CreateIdAttr("UsdUVTexture")
        t.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(path))
        t.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            st_reader.ConnectableAPI(), "result")
        t.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        t.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        if name != "diffuseTex":
            t.CreateInput("sourceColorSpace",
                          Sdf.ValueTypeNames.Token).Set("raw")
        t.CreateOutput(out_name, ttype)
        return t

    if tex.get("base"):
        t = _tex("diffuseTex", sooted_copy(tex["base"]), "rgb")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f) \
          .ConnectToSource(t.ConnectableAPI(), "rgb")
    if tex.get("normal"):
        t = _tex("normalTex", tex["normal"], "rgb")
        sh.CreateInput("normal", Sdf.ValueTypeNames.Normal3f) \
          .ConnectToSource(t.ConnectableAPI(), "rgb")
    if tex.get("orm"):
        tr = _tex("ormTexR", tex["orm"], "g", Sdf.ValueTypeNames.Float)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float) \
          .ConnectToSource(tr.ConnectableAPI(), "g")
        tm = _tex("ormTexM", tex["orm"], "b", Sdf.ValueTypeNames.Float)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float) \
          .ConnectToSource(tm.ConnectableAPI(), "b")
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return bool(tex.get("base"))


def main():
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else src.replace(".usd", ".pv.usd")
    stage = Usd.Stage.Open(src)
    done = skipped = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Material):
            continue
        shader = None
        for ctx in ("mdl", ""):
            try:
                s = (UsdShade.Material(prim).ComputeSurfaceSource(ctx)[0]
                     if ctx else
                     UsdShade.Material(prim).ComputeSurfaceSource()[0])
            except Exception:
                s = None
            if s:
                shader = s
                break
        if not shader:
            continue
        asset = shader.GetSourceAsset("mdl")
        if not asset:
            continue
        mdl = getattr(asset, "resolvedPath", "") or asset.path
        tex = module_textures(mdl) if mdl else {}
        if rebuild(stage, prim, tex):
            done += 1
        else:
            skipped += 1
    stage.GetRootLayer().Export(dst)
    print("[mdl2pv] rebuilt {0} material(s), {1} without a base map -> {2}"
          .format(done, skipped, dst))


if __name__ == "__main__":
    main()
