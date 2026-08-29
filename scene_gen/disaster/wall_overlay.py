"""wall_overlay — a mask-driven translucent wash for a VERTICAL kit face.

THE FAÇADE EQUIVALENT OF `disaster.ground`'s BURN-SCAR OVERLAY. That module
bakes one low-frequency coverage mask across a whole ground plate and reveals
a burnt-floor material through it as OPACITY, so the scar's edge is the
mask's own ragged shape rather than the shape of whatever polygon it happens
to be drawn on (`ground.py`'s own docstring). This module does the same job
for a wall: ONE mask, sized to a whole RUN of wall pieces (not one placed
module), authored as a single translucent quad and revealed as opacity over
a flat soot material.

WHY THIS EXISTS SEPARATELY FROM `urban_fire.py`. `disaster.scorch` and
`disaster.ground` are the ground-plate infrastructure and neither can author
a mesh standing off a vertical `quake_flow._piece_frame` face; growing that
capability inside `urban_fire.py` would bury a reusable primitive (bake a
field-driven, level-set-perturbed mask; reveal it as opacity on an explicit
UV quad) inside a file that also carries this dataset's entire fire model.
Any future disaster that wants a mask-revealed wash on a wall — mud on a
flood-marked elevation, ash on a lahar path — can reuse this without taking
a dependency on fire's own severity model.

WHY OPACITY IS TEXTURE-DRIVEN HERE AND CONSTANT IN `ground.overlay_material`.
That function's own docstring explains its choice: OmniPBR carries ONE
`texture_scale` for every map it samples, so it "cannot tile a diffuse while
stretching a mask once across the plate" — its diffuse is a PHOTOGRAPHED
burnt-floor material that has to repeat every few metres, while its mask
spans the whole plate, two incompatible scales on one shader input. This
overlay has no such conflict: the quad is authored FRESH for exactly the
mask it carries (`author_quad`, below), with its own explicit `primvars:st`
sized 0..1 to that mask and nothing else sampling it, so there is no second
scale to fight over. `enable_opacity_texture` / `opacity_texture` /
`opacity_mode=2` (mono_luminance) is the genuine per-pixel opacity recipe;
`scene_gen/_plans/hurricane_water.md` records it "[V]"-verified against the
shipped `OmniPBR.mdl` (that plan's own L1 water surface, unbuilt elsewhere in
this repo) — this module is the first place in the codebase that actually
authors it.

WITHOUT THE RTX FLAG, THIS STILL FIXES THE EDGE. `ground.py`'s own opening
note: `opacity_constant`/`opacity_texture` is a FRACTIONAL CUTOUT and RTX
Real-Time discards the fractional blend unless
`--/rtx/raytracing/fractionalCutoutOpacity` is on the Kit command line — none
of the urban-fire launch scripts pass it (`urban_fire.materials`'s own note
on the glass-deposit gradient, checked 2026-08-29). Without that flag this
overlay renders as a binary cutout: opaque wherever the mask is above zero,
transparent where it is exactly zero. The GRADED translucency (soot fading
rather than cutting off) is gated on that flag; the SHAPE of the cutout is
not — it is the zero-contour of a spectral, level-set-perturbed field, which
is what makes the edge cross module boundaries today, before that flag ever
lands.
"""

import hashlib
import os

import numpy as np

from . import scorch

OUT_DIR = scorch.OUT_DIR


# ---------------------------------------------------------------------------
# the mask
# ---------------------------------------------------------------------------
def _level_set_mask(h, w, rng, field_rows, streak_stretch=8.0, edge=0.16,
                    wobble=0.55, mottle=0.22, u_span=(0.0, 1.0),
                    v_span=(0.0, 1.0), holes=None, hole_pad=0.15):
    """A ragged coverage mask driven by a REAL per-row field.

    `field_rows` (length `h`, 0..1) is the true, un-noised coverage at each
    row, broadcast across every column — for a fire wash this is
    `urban_fire._severity` sampled per storey, a peaked-and-decaying curve,
    not a generic monotonic wash: a compartment fire's vertical profile is
    not a straight ramp (see that function's own docstring). Nothing here
    varies coverage ALONG the wall; only the edge noise does, which is what
    turns a per-row step into one continuous, streaky boundary that runs
    past several modules' worth of width rather than stopping at any one of
    them.

    LEVEL-SET PERTURBATION — the same trick `scorch.burn_mask_map` uses for
    the ground scar: the noise wobbles a THRESHOLD, it never adds to the
    field. `thresh`'s floor (at the noise's most extreme draw) is
    `edge * (0.5 - 0.5*wobble)`, strictly positive as long as `wobble < 1`,
    so wherever `field_rows` is exactly 0 — below the fire's origin, the
    file's deliberately hard cut — `field - thresh` cannot cross zero for
    ANY draw of the noise. `burn_mask_map`'s own docstring records the
    alternative's failure mode exactly: adding noise straight to the field
    instead of perturbing the threshold "comes out speckled" over ground the
    fire never reached at all. Same lesson, same fix, here for a wall.

    `holes` (optional) is a list of `(u0, u1, v0, v1)` rectangles in the same
    `u_span`/`v_span` units — window and shopfront openings the overlay must
    NOT paint over, because their own frame/pane/crack detail is authored
    more proud than this overlay and would otherwise sit UNDER a flat sooty
    membrane with no hole in it. Suppression is a smooth falloff over
    `hole_pad` metres outward from each rectangle, not a hard cut — this
    file has no hard rectangular edges anywhere else and a punched-out hole
    would be exactly that.
    """
    field = np.clip(np.asarray(field_rows, dtype=np.float64), 0.0, 1.0)
    if field.shape[0] != h:
        raise ValueError(
            "field_rows must have length h ({0}), got {1}".format(
                h, field.shape[0]))
    field = np.repeat(field[:, None], w, axis=1)

    # Wavelength of the edge wobble, in cycles/px — building scale, so the
    # ragged line licks a couple of modules wide rather than fizzing at the
    # pixel grain (`lo`) or bowing in one smooth arc the width of the whole
    # run (`hi`).
    lo = 1.0 / max(8.0, h * 0.9)
    hi = 1.0 / max(2.0, h * 0.12)
    wob = scorch._noise(rng, h, w, beta=2.0, stretch_v=streak_stretch,
                        lo=lo, hi=hi)
    edge = max(0.04, float(edge))
    wobble = max(0.0, min(0.92, float(wobble)))
    thresh = edge * (0.5 + wobble * (wob - 0.5))
    m = np.clip((field - thresh) / edge, 0.0, 1.0)

    # Interior mottling — DAMPENS only (multiplies by a factor <= 1), so it
    # can never raise coverage where the field (and therefore `m`) is zero;
    # the same "islands only ever remove" discipline `burn_mask_map` uses.
    patch = scorch._noise(rng, h, w, beta=2.6)
    mottle = max(0.0, min(0.6, float(mottle)))
    m = m * (1.0 - mottle * (1.0 - patch))
    m = np.clip(m, 0.0, 1.0) ** 0.85

    if holes:
        m = _suppress_holes(m, u_span, v_span, holes, hole_pad)
    return m


def _suppress_holes(m, u_span, v_span, holes, pad):
    """Zero `m` (smoothly, over `pad` metres) inside every `(u0,u1,v0,v1)`
    rectangle in `holes`. Row 0 of `m` is the TOP of the run (`v_span[1]`),
    matching `_facade_field`'s own convention and `author_quad`'s UV
    mapping — see both."""
    h, w = m.shape
    u0s, u1s = float(u_span[0]), float(u_span[1])
    v0s, v1s = float(v_span[0]), float(v_span[1])
    us = u0s + (np.arange(w) + 0.5) / float(w) * (u1s - u0s)
    vs = v1s - (np.arange(h) + 0.5) / float(h) * (v1s - v0s)
    U = np.repeat(us[None, :], h, axis=0)
    V = np.repeat(vs[:, None], w, axis=1)
    keep = np.ones((h, w), dtype=np.float64)
    pad = max(1e-3, float(pad))
    for (hu0, hu1, hv0, hv1) in holes:
        du = np.maximum(0.0, np.maximum(hu0 - U, U - hu1))
        dv = np.maximum(0.0, np.maximum(hv0 - V, V - hv1))
        d = np.hypot(du, dv)
        keep = np.minimum(keep, np.clip(d / pad, 0.0, 1.0))
    return m * keep


def bake_mask(h, w, rng, field_rows, streak_stretch=8.0, edge=0.16,
             wobble=0.55, mottle=0.22, u_span=(0.0, 1.0), v_span=(0.0, 1.0),
             holes=None, hole_pad=0.15, out_dir=None, key=""):
    """Composite and cache one wall mask. Returns a local PNG path.

    Cached under `disaster.scorch.OUT_DIR`, same directory as every other
    baked scorch map, keyed by every input that changes the pixels — the
    field's own values included, not just its shape, so two different
    buildings (or two sides of one) that happen to share `h`/`w` and every
    tuning knob never collide on the same file (the trap `scorch.py`
    records against `SOOT_RGB` and `char_bite`: "change the recipe without
    changing the key and every caller silently gets back the map baked
    under the old rule").
    """
    from PIL import Image

    out_dir = out_dir or OUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    field_key = hashlib.md5(
        ",".join("{0:.4f}".format(v) for v in field_rows).encode("utf-8")
    ).hexdigest()[:12]
    holes_key = hashlib.md5(
        "|".join("{0:.3f},{1:.3f},{2:.3f},{3:.3f}".format(*hh)
                for hh in (holes or ())).encode("utf-8")
    ).hexdigest()[:12]
    cache_key = hashlib.md5(
        "wallmask|v1|{0}|{1}|{2:.4f}|{3:.4f}|{4:.4f}|{5:.2f}|{6:.3f}|{7}|"
        "{8}|{9}".format(
            h, w, edge, wobble, mottle, streak_stretch, hole_pad, field_key,
            holes_key, key
        ).encode("utf-8")
    ).hexdigest()[:16]
    path = os.path.join(out_dir, "wallmask_{0}.png".format(cache_key))
    if os.path.exists(path):
        return path
    m = _level_set_mask(h, w, rng, field_rows, streak_stretch=streak_stretch,
                        edge=edge, wobble=wobble, mottle=mottle,
                        u_span=u_span, v_span=v_span, holes=holes,
                        hole_pad=hole_pad)
    Image.fromarray((np.clip(m, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8),
                    "L").save(path)
    return path


# ---------------------------------------------------------------------------
# material + geometry
# ---------------------------------------------------------------------------
def overlay_material(stage, path, mask_path, rgb, roughness):
    """A flat soot OmniPBR, opacity textured by `mask_path`.

    `diffuse_color_constant` carries the whole colour (no diffuse texture —
    unlike `ground.overlay_material`, which stretches a photographed burnt-
    floor map at its own scale, there is nothing here that needs a second
    physical scale; the soot's own mottling comes from the SAME mask, via
    opacity, once translucency is real). See the module docstring for why
    `opacity_mode=2` (mono_luminance) + `enable_opacity_texture` is the
    right recipe rather than `ground.py`'s constant-opacity bands.
    """
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*[float(c) for c in rgb]))
    sh.CreateInput("reflection_roughness_constant",
                  Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("enable_opacity_texture", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("opacity_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(mask_path))
    sh.CreateInput("opacity_mode", Sdf.ValueTypeNames.Int).Set(2)
    # 0 blends once the RTX flag is on; above 0 stipples a soft edge into a
    # dither cutout (`ground.overlay_material`'s own note) — the same reason
    # that module holds this at 0.
    sh.CreateInput("opacity_threshold", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def author_quad(ctx, fr, u0, u1, v0, v1, out, mat, kind="wallovl"):
    """A single quad on a wall run, `u0..u1` along it (`quake_flow.
    _piece_frame` units) and `v0..v1` in world height, `out` metres proud.

    EXPLICIT `primvars:st`, NOT WORLD-PROJECTED. `ground.overlay_material`
    needs `project_uvw`/world-space UVs because its mask spans a plate many
    separate band meshes share; this quad is authored fresh for exactly the
    one mask it carries, so a plain 0..1 parameterisation over its own
    extent is both simpler and exact — no world-scale tiling problem to
    solve, and nothing else samples this material's UVs to disagree with.
    `(0,0)` is `(u0, v0)` — the run's own bottom, matching `bake_mask`'s
    row-0-is-top convention (v=1 at the top row of the image, v=0 at the
    bottom, the ordinary raster convention `scorch.soot_mask` already uses:
    "1 at the top of the image").
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    from . import quake_flow as qf

    corners = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
    P = [qf._b_face_pt(fr, u, v, out) for (u, v) in corners]
    uvs = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    path = "{0}/{1}_{2}_{3}".format(ctx["parent"], kind, ctx["tag"],
                                    qf._uid(ctx))
    mesh = UsdGeom.Mesh.Define(ctx["stage"], Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in P]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.vertex).Set(Vt.Vec2fArray([Gf.Vec2f(*uv) for uv in uvs]))
    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(mat)
    ctx["authored"].append(path)
    return path
