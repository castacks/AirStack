#!/usr/bin/env python
"""aec_scorch_unfold.py -- the "unfolder" acceptance check for the AEC MDL
soot-overlay fix (see `gac_fire.PACKS["aec"]["soot"] == "overlay"` and
`gac_fire.overlay_soot`), per the user's own review method: "take photos of
the facades and match it with the 'unfolder' scorch pattern that's supposed
to wrap around the facades."

For one AEC brownstone + fire level, per elevation that vented, writes THREE
PNGs:

    base     the assembled elevation, geometry-accurate (real window/trim/
             corner silhouettes, rasterised from the merged mesh's own
             triangles -- `soot_elevation.render_gac_elevation`), no soot.
             THE BRICK TEXTURE ITSELF IS A FLAT PLACEHOLDER TONE, not a
             photograph: AEC's facade materials are compiled NVIDIA `.mdl`
             modules (measured, `tools/aec_material_probe.py`) whose base-
             colour texture is not visible to USD attribute introspection
             at all -- the same wall that made the per-piece soot BAKE
             impossible and motivated the overlay route in the first place.
             This is a real, disclosed limitation of an OFFLINE (no Kit, no
             GPU) check: it verifies the scorch pattern's ALIGNMENT and
             CONTINUITY against the real window/corner geometry, not the
             real brick colour -- the lead's GPU re-bake is what confirms
             brick-with-overlay actually renders.
    skin     `soot_plume.skin`'s own per-pixel RGBA composited over `base`
             -- the reference the fire model itself predicts, independent
             of how the overlay quad ends up authored.
    overlay  the ACTUAL diffuse+opacity PNGs `gac_fire.overlay_soot` writes
             for this elevation, composited over `base` the same way. This
             is not a simulation of the overlay -- it is the literal file
             pair the real scene will bind, so `skin` and `overlay` must
             match pixel-for-pixel (mod resampling): if they do not, the
             overlay's own crop math is wrong, not the fire model.

    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \
      ./scene_gen/tools/usd_python.sh scene_gen/tools/aec_scorch_unfold.py \
      Reference_Brownstone5Row F3 --sides S,E --seed 14 --origin 1"

Outputs to `/isaac-sim/.nvidia-omniverse/logs/aec_scorch_unfold/` (host,
via the existing bind mount: `~/docker/isaac-sim/logs/aec_scorch_unfold/`).
"""
import argparse
import os
import random
import sys
import time

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")

from pxr import Usd, UsdGeom                                  # noqa: E402
from disaster import gac_fire as gf, soot_plume as spl        # noqa: E402
import soot_elevation as se                                   # noqa: E402

OUT = "/isaac-sim/.nvidia-omniverse/logs/aec_scorch_unfold"
# a plausible warm-brick placeholder tone (sRGB-ish, linear-encoded like
# every other constant in this pipeline) -- NOT a measurement, just enough
# contrast that window/corner silhouettes and the soot wash both read.
BRICK_PLACEHOLDER_RGB = (0.42, 0.22, 0.16)


def _placeholder_images(mesh):
    """`{material index: (2,2,3) constant array}` for every material this
    mesh's triangles reference -- `soot_elevation.render_gac_elevation`
    happily bilinear-samples a tiny constant array the same way it would a
    real texture (`sample_tex`)."""
    fill = np.full((2, 2, 3), BRICK_PLACEHOLDER_RGB, dtype=np.float32)
    out = {}
    for k, mat in enumerate(mesh["mats"]):
        if mat is None:
            continue
        out[k] = fill
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("asset")
    ap.add_argument("level")
    ap.add_argument("--sides", default="", help="e.g. S,E -- default: the fire's own sides")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--origin", type=int, default=None)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    t0 = time.time()
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    cell = "/W/b0"
    UsdGeom.Xform.Define(st, cell)
    rng = random.Random(a.seed)
    sides_arg = tuple(v.strip() for v in a.sides.split(",") if v.strip()) or None

    pre = gf.prepare(st, cell, "aec:" + a.asset, a.level, rng,
                     "unfold{0}".format(a.seed), origin=a.origin,
                     sides=sides_arg, out_dir=a.out)
    mesh, m, sk = pre["mesh"], pre["mass"], pre["skin"]
    fire_sides = pre["fire"]["sides"]
    print("[aec_scorch_unfold] {0} {1}: origin st{2}, sides {3}, {4}"
          .format(a.asset, a.level, pre["fire"]["origin"],
                  "/".join(fire_sides), spl.summarise(pre["events"])))
    if pre.get("overlay_prebaked"):
        print("[aec_scorch_unfold] {0} source material(s) routed to the "
              "overlay (no readable base map)".format(
                  len(pre["overlay_prebaked"])))
    else:
        print("[aec_scorch_unfold] WARNING: nothing was routed to the "
              "overlay -- overlay_soot found a readable base map on every "
              "material, or the pack's soot mode is not 'overlay'")

    images = _placeholder_images(mesh)

    # every side that actually vented (fire_sides), plus anything --sides
    # asked for explicitly
    want_sides = sorted(set(fire_sides) | set(sides_arg or ()))
    for side in want_sides:
        rgb_base, depth, covered, n_tri = se.render_gac_elevation(mesh, images, m, side)
        if not covered.any():
            print("[aec_scorch_unfold] {0}: side {1} has no covered pixels "
                  "(no triangles faced outward here) -- skipped".format(a.asset, side))
            continue

        # --- base ---
        _save(rgb_base, os.path.join(
            a.out, "unfold_{0}_{1}_{2}_base.png".format(a.asset, a.level, side)))

        # --- skin: soot_plume.skin composited directly, pixel by pixel ---
        rgb_skin = rgb_base.copy()
        rows, cols = np.nonzero(covered)
        H = float(m["top"] - m["z0"])
        u = (cols + 0.5) / se.PPM
        z = m["z0"] + H - (rows + 0.5) / se.PPM
        off = sk["offsets"][side]
        sc_ = ((off + u) * sk["ppm"]) % sk["rgba"].shape[1]
        sr = np.clip((sk["H"] - (z - sk["z0"])) * sk["ppm"], 0, sk["rgba"].shape[0] - 1)
        s_ = sk["rgba"][sr.astype(int), sc_.astype(int)]
        al = s_[:, 3:4]
        b_ = rgb_skin[rows, cols]
        grey = b_.mean(axis=1, keepdims=True)
        desat = b_ * (1 - spl.DESAT * al) + grey * (spl.DESAT * al)
        rgb_skin[rows, cols] = desat * (1 - al) + s_[:, :3] * al
        _save(rgb_skin, os.path.join(
            a.out, "unfold_{0}_{1}_{2}_skin.png".format(a.asset, a.level, side)))

        # --- overlay: the ACTUAL sootovl_dif_/sootovl_opa_ PNGs, sampled
        # over the SAME u/z grid every covered pixel here already has ---
        dif_png, opa_png = _find_overlay_pair(a.out, "unfold{0}".format(a.seed), side)
        rgb_ovl = rgb_base.copy()
        if dif_png is None:
            print("[aec_scorch_unfold] {0}: side {1} -- no overlay PNG pair "
                  "found (this elevation never vented, or overlay_soot "
                  "skipped it)".format(a.asset, side))
        else:
            from PIL import Image
            dif = np.asarray(Image.open(dif_png).convert("RGB"), dtype=np.float32) / 255.0
            opa = np.asarray(Image.open(opa_png).convert("L"), dtype=np.float32) / 255.0
            oh, ow = opa.shape
            # same row-0-is-top / u,z addressing the overlay quad's own UV
            # mapping uses (`wall_overlay.author_quad`: u0->0, u1->1,
            # v0(=z0, bottom)->0, v1(=top,top)->1; the PNG's row 0 is the
            # WALL's top, matching soot_plume's convention) -- so u maps
            # linearly 0..L across columns and z maps 0..H across rows,
            # top row first.
            L = spl.side_length(m, side)
            uu = np.clip(u / max(L, 1e-6), 0.0, 1.0)
            zz = np.clip((z - m["z0"]) / max(H, 1e-6), 0.0, 1.0)
            col = np.clip((uu * (ow - 1)).astype(int), 0, ow - 1)
            row = np.clip(((1.0 - zz) * (oh - 1)).astype(int), 0, oh - 1)
            al2 = opa[row, col][:, None]
            c2 = dif[row, col]
            b2 = rgb_ovl[rows, cols]
            grey2 = b2.mean(axis=1, keepdims=True)
            desat2 = b2 * (1 - spl.DESAT * al2) + grey2 * (spl.DESAT * al2)
            rgb_ovl[rows, cols] = desat2 * (1 - al2) + c2 * al2
        _save(rgb_ovl, os.path.join(
            a.out, "unfold_{0}_{1}_{2}_overlay.png".format(a.asset, a.level, side)))

        diff = float(np.abs(rgb_skin[covered] - rgb_ovl[covered]).mean())
        print("[aec_scorch_unfold] {0} side {1}: {2}x{3}, {4} triangle(s), "
              "{5:.0%} covered, mean |skin-overlay| = {6:.4f} (0 = perfect "
              "match)".format(a.asset, side, rgb_base.shape[1],
                              rgb_base.shape[0], n_tri, float(covered.mean()),
                              diff))
    print("[aec_scorch_unfold] done in {0:.1f}s -> {1}".format(
        time.time() - t0, a.out))


def _save(rgb, path):
    from PIL import Image
    Image.fromarray((np.clip(rgb, 0, 1) * 255 + 0.5).astype(np.uint8)).save(path)
    print("  wrote", path)


def _find_overlay_pair(out_dir, tag, side):
    import glob
    key = "{0}_{1}".format(tag, side)
    dif = sorted(glob.glob(os.path.join(out_dir, "sootovl_dif_{0}_*.png".format(key))))
    opa = sorted(glob.glob(os.path.join(out_dir, "sootovl_opa_{0}_*.png".format(key))))
    if not dif or not opa:
        return None, None
    return dif[-1], opa[-1]


if __name__ == "__main__":
    main()
