#!/usr/bin/env python
"""aec_facade_2d.py — assemble ONE elevation of an AEC building as a flat 2D
image, from each part's own texture, with no Kit and no render.

    usd_python.sh scene_gen/tools/aec_facade_2d.py [SIDE] [OUT.png]

WHY (user, 2026-09-02): *"first just assemble in 2d and see if you can get the
material correct. Can you export mdl as png? might make it easier."*

Yes — the `.mdl` modules are plain text wrapping ordinary PNGs, so
"export the MDL as PNG" is just reading the module's own BaseColor map
(`mdl_to_preview.module_textures`). That makes the whole material question
answerable as an IMAGE, offline, in seconds:

  facade_base.png   every facade part drawn at its real (u, z) rectangle on
                    the elevation, filled with ITS OWN base-colour texture.
                    If the brick shows here, the material chain is correct
                    and any white in a render is a binding/instancing
                    problem, not a texture one.
  facade_parts.png  the same rectangles as flat colour-coded blocks, labelled
                    by category — the layout sanity check ("do the pieces fit
                    the facade").

This is the 2D half of the MCE approach: get the facade right first, THEN cut
the soot canvas over it.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from pxr import Usd, UsdGeom, UsdShade          # noqa: E402
import mdl_to_preview as m2p                    # noqa: E402

ASSET = os.environ.get(
    "AEC_ASSET",
    "/isaac-sim/AirStack/scene_gen/assets/aec/brownstone/Assets/"
    "Create_Brownstone02/Reference_Brownstone5Row.usd")
SIDE = (sys.argv[1] if len(sys.argv) > 1 else "S").upper()
OUT = sys.argv[2] if len(sys.argv) > 2 else "/isaac-sim/.cache/facade_base.png"
PPM = float(os.environ.get("AEC_2D_PPM", "40"))     # pixels per metre

CATS = ("Structural_Framing", "Walls", "Windows", "Doors", "Roofs",
        "Stairs", "Railings", "Generic_Models")


def cat_of(p):
    parts = str(p).split("/")
    if "Geometry" in parts:
        i = parts.index("Geometry")
        if i + 1 < len(parts):
            return parts[i + 1]
    return "?"


def main():
    from PIL import Image, ImageDraw
    import numpy as np

    stage = Usd.Stage.Open(ASSET)
    root = stage.GetPseudoRoot()
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    whole = bc.ComputeWorldBound(root).ComputeAlignedRange()
    mn, mx = whole.GetMin(), whole.GetMax()
    # asset is centimetres; work in metres
    sc = float(UsdGeom.GetStageMetersPerUnit(stage)) or 1.0
    W = (mx[0] - mn[0]) * sc
    Dp = (mx[1] - mn[1]) * sc
    H = (mx[2] - mn[2]) * sc
    span = W if SIDE in ("S", "N") else Dp
    px_w, px_h = int(span * PPM), int(H * PPM)
    print("[2d] {0} elevation: {1:.1f} x {2:.1f} m -> {3} x {4} px"
          .format(SIDE, span, H, px_w, px_h))

    base = Image.new("RGB", (px_w, px_h), (26, 28, 32))
    blocks = Image.new("RGB", (px_w, px_h), (18, 18, 20))
    dr = ImageDraw.Draw(blocks)
    tex_cache = {}
    drawn = notex = 0
    hits = {}

    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        cat = cat_of(prim.GetPath())
        if cat not in CATS:
            continue
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        a, b = r.GetMin(), r.GetMax()
        c1 = 0.5 * (a[1] + b[1])
        c0 = 0.5 * (a[0] + b[0])
        d = {"S": abs(c1 - mn[1]), "N": abs(mx[1] - c1),
             "W": abs(c0 - mn[0]), "E": abs(mx[0] - c0)}
        if min(d, key=d.get) != SIDE:
            continue
        # depth cull: only the outer skin of this elevation
        depth = d[SIDE] * sc
        if depth > 1.2:
            continue
        if SIDE in ("S", "N"):
            u0, u1 = (a[0] - mn[0]) * sc, (b[0] - mn[0]) * sc
        else:
            u0, u1 = (a[1] - mn[1]) * sc, (b[1] - mn[1]) * sc
        z0, z1 = (a[2] - mn[2]) * sc, (b[2] - mn[2]) * sc
        x0, x1 = int(u0 * PPM), int(u1 * PPM)
        y0, y1 = int((H - z1) * PPM), int((H - z0) * PPM)
        if x1 - x0 < 2 or y1 - y0 < 2:
            continue
        hits[cat] = hits.get(cat, 0) + 1
        col = {"Windows": (40, 70, 110), "Doors": (120, 80, 40),
               "Roofs": (60, 60, 66), "Railings": (90, 90, 95),
               "Stairs": (100, 95, 85)}.get(cat, (150, 80, 60))
        dr.rectangle([x0, y0, x1, y1], fill=col, outline=(230, 230, 230))

        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        tpath = None
        if mat:
            key = str(mat.GetPath())
            if key not in tex_cache:
                t = {}
                try:
                    sh = None
                    for cx in ("mdl", ""):
                        s = (UsdShade.Material(mat.GetPrim())
                             .ComputeSurfaceSource(cx)[0] if cx else
                             UsdShade.Material(mat.GetPrim())
                             .ComputeSurfaceSource()[0])
                        if s:
                            sh = s
                            break
                    if sh is not None:
                        aa = sh.GetSourceAsset("mdl")
                        if aa:
                            mdl = getattr(aa, "resolvedPath", "") or aa.path
                            t = m2p.module_textures(mdl) if mdl else {}
                except Exception:
                    t = {}
                tex_cache[key] = t.get("base")
            tpath = tex_cache[key]
        if not tpath or not os.path.isfile(tpath):
            notex += 1
            continue
        try:
            im = Image.open(tpath).convert("RGB")
            # tile the map over the part's rect at ~1 m per texture repeat
            tw = max(8, int(PPM)), max(8, int(PPM))
            im = im.resize(tw)
            tile = Image.new("RGB", (x1 - x0, y1 - y0))
            for ty in range(0, y1 - y0, tw[1]):
                for tx in range(0, x1 - x0, tw[0]):
                    tile.paste(im, (tx, ty))
            base.paste(tile, (x0, y0))
            drawn += 1
        except Exception:
            notex += 1

    print("[2d] parts on {0}: {1}".format(SIDE, hits))
    print("[2d] textured {0}, no-texture {1}".format(drawn, notex))
    base.save(OUT)
    blocks.save(OUT.replace(".png", "_parts.png"))
    print("[2d] wrote {0} and {1}".format(OUT, OUT.replace(".png", "_parts.png")))


if __name__ == "__main__":
    main()
