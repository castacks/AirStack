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
# tunable constants — see the code below each one for what it governs
# ---------------------------------------------------------------------------
# DEFECT 2 — "In the parts that it is correct it looks better but it's still
# kinda rectangular instead of the scorch streaks like in suburban" (user
# review, 2026-08-29). MEASURED CAUSE: `field = np.repeat(field_rows[:,
# None], w, axis=1)` makes every column read the identical severity curve,
# so however ragged the per-pixel threshold noise (`wob`, below) makes any
# ONE row's crossing, that crossing still lands at very nearly the same row
# for every column whenever the field's own row-to-row step is bigger than
# `edge` (the ordinary case — `_severity` is piecewise-constant per storey,
# so most of its transitions ARE bigger than `edge`) — a band with a fuzzy
# top/bottom edge is still a band. `scorch.soot_mask`'s streaks are not edge
# noise: they are a genuine per-column REACH, which is what turns a wash
# into licks. `_STREAK_DEFAULT` is a MULTIPLICATIVE gain in
# `[1 - streak, 1]`, built from noise stretched `streak_stretch`x vertically
# (so it reads as nearly constant down one column — a lick runs the height
# of the wash, not a speckle) but band-limited across COLUMNS at building
# scale, so it varies finger to finger rather than pixel to pixel or as one
# arc the width of the run. Multiplicative and never above 1, so it can only
# ever REMOVE coverage the level set already allowed — 0 times any gain is
# still 0, so the "never add where the field is zero" guarantee (below)
# holds automatically, without this term needing to know where zero is.
#
# Calibrated against the one statistic that actually separates a streaked
# wash from a band with a wobbly edge: the mask's own variance ALONG a row,
# `np.mean(np.var(m, axis=1))` — a constant-per-row band scores exactly 0 on
# this no matter how ragged its edge is. Target (measured against
# `scorch.soot_mask` at coverage 0.35-0.60): 0.004-0.012. Calibrated two
# ways: a controlled sweep of synthetic fields at fixed size (matching the
# suburb measurement's own methodology), AND the REAL `_facade_field` output
# for every non-curtain-wall style in `detail.urban_building.STYLES`,
# restricted to the rows the fire actually reaches (a tall building with a
# short burnt band legitimately averages near 0 over its many untouched
# storeys — that dilution is correct, not a miscalibration, so it is
# excluded here). 0.50 lands in range for 3/3 synthetic coverages and 88% of
# real (style, side) fields tested (the rest run a little hot, never a
# little flat) — see this file's offline verification.
_STREAK_DEFAULT = 0.50

# DEFECT 3 — "For buildings whose fire isn't starting from the ground floor
# have the streaks both up and downward" (user review, 2026-08-29). The
# up-dominance is PHYSICAL and stays: a compartment fire's plume is
# buoyant, so soot lands overwhelmingly ABOVE the origin (`_severity`'s own
# docstring, `urban_fire.py`). But a window venting mid-building also drips
# burning material, sheds smoke that curls down past the sill, and runs
# hose water down the face below it — smaller, but real, and the user has
# now asked for it explicitly. `_apply_downward_tail` (below) adds it as a
# SHORT, WEAK, deterministic extension of `field_rows` itself, anchored to
# the field's own value at the origin row — not a random draw, so it stays
# inside the same "moves the boundary / extends the field on purpose, never
# speckles it" discipline the rest of this file uses (see that function's
# own docstring).
_DOWN_TAIL_REACH_FRAC = 0.22    # tail length, as a fraction of the plume's
                                 # OWN upward extent (rows from the origin to
                                 # its own topmost nonzero row) — SHORT
_DOWN_TAIL_STRENGTH = 0.30      # tail's value at the origin row, as a
                                 # fraction of the field's own value there —
                                 # WEAK, and it fades out fast under that


def _apply_downward_tail(field_rows, reach_frac=_DOWN_TAIL_REACH_FRAC,
                         strength=_DOWN_TAIL_STRENGTH):
    """Extend `field_rows` a short, weak, deterministic step below the
    origin. See `_DOWN_TAIL_REACH_FRAC`/`_DOWN_TAIL_STRENGTH` above for why
    the tail is sized the way it is.

    `field_rows` is length-`h`, row 0 the TOP of the run — this file's
    convention throughout (`_facade_field`'s own docstring, `bake_mask`'s
    row-0-is-top note, `author_quad`'s UV mapping). So "below the origin" is
    the LARGER row indices, and "the origin" is inferred as the deepest
    (largest-index) row that is still nonzero: `_facade_field` hard-zeroes
    every row below it (bar its own one-storey spill, `_severity`'s
    `storey == origin - 1` case), so that row IS the transition this softens.

    A building whose fire starts at the ground floor has no rows below the
    origin to extend into — `origin_row` is already the array's last row —
    and this is a no-op. REQUIRED, not incidental: see the offline
    verification this change left behind.

    Deterministic, not noise-driven, and this is the point: this changes
    what counts as the FIELD (the ceiling every later step in
    `_level_set_mask` only ever thresholds, dampens or reveals — never
    raises), the same role `_facade_field`'s own severity curve already
    plays for every other row. It is therefore not the mistake
    `burn_mask_map`'s docstring warns against (noise added straight to a
    field that is supposed to be a hard, spatially exact cutoff): it never
    runs on a random draw, and it is bounded to `reach_frac` of the plume's
    own height, so it can never reach a row far below the origin regardless
    of how that constant is tuned.
    """
    field = np.clip(np.asarray(field_rows, dtype=np.float64), 0.0, 1.0)
    h = field.shape[0]
    nz = np.nonzero(field > 1e-9)[0]
    if nz.size == 0:
        return field
    origin_row = int(nz.max())
    if origin_row >= h - 1:
        return field   # ground-floor fire: nothing below it to extend into
    up_extent = max(1, origin_row - int(nz.min()))
    reach = max(1, int(round(up_extent * max(0.0, float(reach_frac)))))
    reach = min(reach, h - 1 - origin_row)
    base = field[origin_row]
    strength = max(0.0, min(1.0, float(strength)))
    for k in range(1, reach + 1):
        r = origin_row + k
        decay = 1.0 - k / float(reach + 1)
        val = base * strength * decay
        if val > field[r]:
            field[r] = val
    return field


# ---------------------------------------------------------------------------
# the mask
# ---------------------------------------------------------------------------
def _level_set_mask(h, w, rng, field_rows, streak_stretch=8.0, edge=0.16,
                    wobble=0.55, mottle=0.22, streak=_STREAK_DEFAULT,
                    down_tail_reach=_DOWN_TAIL_REACH_FRAC,
                    down_tail_strength=_DOWN_TAIL_STRENGTH,
                    u_span=(0.0, 1.0), v_span=(0.0, 1.0), holes=None,
                    hole_pad=0.15):
    """A ragged coverage mask driven by a REAL per-row field.

    `field_rows` (length `h`, 0..1) is the true, un-noised coverage at each
    row — for a fire wash this is `urban_fire._severity` sampled per storey,
    a peaked-and-decaying curve, not a generic monotonic wash: a compartment
    fire's vertical profile is not a straight ramp (see that function's own
    docstring). It is broadcast across every column, THEN extended by
    `_apply_downward_tail` (a short, weak, deterministic step below the
    origin — defect 3) and by `streak` (a per-column multiplicative gain —
    defect 2, see `_STREAK_DEFAULT` above): between them, coverage now DOES
    vary along the wall, which is what turns a per-row step into fingers and
    licks rather than one uniform band with a merely ragged edge.

    LEVEL-SET PERTURBATION — the same trick `scorch.burn_mask_map` uses for
    the ground scar: the noise wobbles a THRESHOLD, it never adds to the
    field. `thresh`'s floor (at the noise's most extreme draw) is
    `edge * (0.5 - 0.5*wobble)`, strictly positive as long as `wobble < 1`,
    so wherever `field` is exactly 0 `field - thresh` cannot cross zero for
    ANY draw of the noise. `burn_mask_map`'s own docstring records the
    alternative's failure mode exactly: adding noise straight to the field
    instead of perturbing the threshold "comes out speckled" over ground the
    fire never reached at all. Same lesson, same fix, here for a wall — and
    the same reason `streak` (below) is a MULTIPLIER on `m`, never a term
    added to `field`: a row far below the origin (well outside
    `_apply_downward_tail`'s own bounded reach) is exactly 0 in `field` for
    every column, so `m` is exactly 0 there for every column too, and no
    finite `streak` can raise a zero.

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
    field = _apply_downward_tail(field, reach_frac=down_tail_reach,
                                 strength=down_tail_strength)
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

    # DEFECT 2 — along-wall variation, multiplicative-only (see
    # `_STREAK_DEFAULT` above for the full reasoning). `lo_s`/`hi_s` are
    # sized off `w` the way `lo`/`hi` above are sized off `h`, so this reads
    # as a handful of licks spanning the run rather than pixel noise or one
    # arc the width of the whole wall.
    lo_s = 1.0 / max(8.0, w * 0.9)
    hi_s = 1.0 / max(2.0, w * 0.12)
    streaks = scorch._noise(rng, h, w, beta=2.0, stretch_v=streak_stretch,
                            lo=lo_s, hi=hi_s)
    streak = max(0.0, min(0.95, float(streak)))
    m = m * ((1.0 - streak) + streak * streaks)

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
             wobble=0.55, mottle=0.22, streak=_STREAK_DEFAULT,
             down_tail_reach=_DOWN_TAIL_REACH_FRAC,
             down_tail_strength=_DOWN_TAIL_STRENGTH,
             u_span=(0.0, 1.0), v_span=(0.0, 1.0),
             holes=None, hole_pad=0.15, out_dir=None, key=""):
    """Composite and cache one wall mask. Returns a local PNG path.

    Cached under `disaster.scorch.OUT_DIR`, same directory as every other
    baked scorch map, keyed by every input that changes the pixels — the
    field's own values included, not just its shape, so two different
    buildings (or two sides of one) that happen to share `h`/`w` and every
    tuning knob never collide on the same file (the trap `scorch.py`
    records against `SOOT_RGB` and `char_bite`: "change the recipe without
    changing the key and every caller silently gets back the map baked
    under the old rule"). `streak`/`down_tail_reach`/`down_tail_strength`
    (defects 2 and 3) are in the key for exactly that reason — the cache
    key version bumped to `v2` alongside them so no stale `v1` file (baked
    under the pre-streak, pre-tail recipe) is ever silently reused.
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
        "wallmask|v2|{0}|{1}|{2:.4f}|{3:.4f}|{4:.4f}|{5:.2f}|{6:.3f}|"
        "{7:.3f}|{8:.3f}|{9:.3f}|{10}|{11}|{12}".format(
            h, w, edge, wobble, mottle, streak_stretch, hole_pad, streak,
            down_tail_reach, down_tail_strength, field_key, holes_key, key
        ).encode("utf-8")
    ).hexdigest()[:16]
    path = os.path.join(out_dir, "wallmask_{0}.png".format(cache_key))
    if os.path.exists(path):
        return path
    m = _level_set_mask(h, w, rng, field_rows, streak_stretch=streak_stretch,
                        edge=edge, wobble=wobble, mottle=mottle,
                        streak=streak, down_tail_reach=down_tail_reach,
                        down_tail_strength=down_tail_strength,
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
