"""facade_bake — ONE composited texture per (building, material), not a
translucent overlay quad standing off the wall.

"You can consider buildings a unit of burn and in the overall material that
will go on the entire building only have scorches on some parts. It's a png
that's mapped onto the texture." (user, 2026-08-29) — this module is that:
bake ONE image per (building elevation, base texture), soot painted straight
INTO it where the fire actually reached, then map that single image across
the whole run with `project_uvw` so it never repeats within one elevation.

WHICH OF `scorch.py`'s TWO GROUND RECIPES THIS IS, AND WHY THAT CHOICE
FLIPS FOR A FAÇADE. `scorch.ground_burn_map` bakes source detail AND the
burn together into one image for the whole surface; `scorch.burn_mask_map`
bakes only the mask and lets a burnt material tile at its own scale under
it, expressly BECAUSE (that module's own docstring) a 320 m ground plate
baked whole resamples its source down to ~31 cm/px — "mush at any sane
camera height" — and throws away every map but diffuse.

A building elevation is one to two orders of magnitude smaller than that
plate. `_PX_PER_M` here (50, i.e. 2 cm/px) on a 40 m elevation is a 2048 px
image — the exact arithmetic the brief posed, and "plenty" per that same
docstring's own yardstick. So THIS module takes `ground_burn_map`'s
recipe — composite everything into one baked diffuse map — not
`burn_mask_map`'s. The trade that recipe pays ("resampled down… diffuse
only") is real but affordable here for a second reason too: a wall's own
normal/ORM maps are not being touched by this bake AT ALL — `facade_material`
(below) still binds ONLY a `diffuse_texture`, exactly like every other
material in this codebase (`damage._pbr`, `damage.scorched_material`); this
dataset has never bound a full material stack for burnt cladding, so there
is no second map this bake newly loses that the rest of the fire pipeline
was already keeping.

WHY BAKED-IN, NOT AN OPACITY OVERLAY (`wall_overlay.py`'s own design, which
this replaces for the broad wall/corner wash). That module traded off
`ground.overlay_material`'s constant-opacity-band compromise for a mask
TEXTURED as opacity, because OmniPBR "cannot tile a diffuse while
stretching a mask once across the plate — its diffuse is a PHOTOGRAPHED
burnt-floor material that has to repeat every few metres, while its mask
spans the whole plate" (`wall_overlay.py`'s own docstring, restating
`ground.overlay_material`'s). THIS module removes that conflict a different
way: instead of keeping two textures at two scales alive on one shader and
reconciling them with opacity, it resamples the base texture ONCE into the
same canvas as the mask and composites them together in numpy, so the
result is a SINGLE diffuse texture at a SINGLE scale with no second shader
input fighting it. That also drops the two things `wall_overlay.py`'s own
docstring flags as its real cost: `opacity_mode=2` needs
`--/rtx/raytracing/fractionalCutoutOpacity` for a graded blend (none of the
urban-fire launch scripts pass it today, so that overlay renders as a
binary cutout without it), and every overlay needed its own proud quad
mesh, one more prim and one more depth-fight per run. A baked diffuse has
neither problem: it is bound straight onto the wall's OWN geometry, so
there is no separate mesh, no proud offset, and no RTX flag it is waiting
on to look graded rather than cut out.

THE MASK ITSELF IS REUSED, NOT REWRITTEN. `wall_overlay._level_set_mask` is
already the calibrated, field-driven, level-set-perturbed soot mask this
dataset tuned against the suburb's own along-row-variance statistic (see
that module's docstring for the full history — defects 2 and 3, the
streak/down-tail knobs, the "noise moves the boundary, never adds coverage"
discipline). This module calls it directly rather than authoring a third
mask generator, and inherits every one of those guarantees for free: soot
never appears where `field_rows` is exactly 0, the vertical profile comes
straight from `urban_fire._severity` (heaviest at the top of the module
containing the fire, hard-clean below the origin bar one short weak tail),
and the along-row variance is the same statistic already calibrated to
0.004-0.012.

WHAT IS NEW HERE, on top of that mask, is only:
  1. tiling the surface's OWN base texture at its natural metric scale
     underneath the mask (`_tile_base`), so brick stays brick-sized instead
     of being stretched into one poster-sized image or, the opposite
     failure, resampled down into mush;
  2. compositing the two with `scorch.composite` into one PNG, cached by
     every input that changes its pixels (`bake_facade_texture`);
  3. a material that maps that ONE image across a WHOLE run with
     `project_uvw` world-triplanar at building scale, so it does not repeat
     within one elevation (`facade_material`) — the direct fix for
     `damage.scorched_material(triplanar=True)`'s own `scale_uv` default,
     which repeats every ~3.6 m and reads as a visible tile on anything
     bigger than a storey.

A CURTAIN WALL IS REFUSED, NOT SILENTLY MISHANDLED. `urban_fire.py` (~978)
already recorded why compositing soot into a tower family's pale glazing
atlas is wrong — "white panes with black ink-runs down them" — for the
identical reason `damage.scorched_material`'s own per-module path avoids it
today. `facade_material` raises if told `is_curtain_wall=True` rather than
producing a material that would reproduce that defect; the caller keeps
binding a flat soot tone for glass, same as `r_smoke_stain` already does.
"""

import hashlib
import math
import os

import numpy as np

from . import scorch
from . import wall_overlay as wo

OUT_DIR = scorch.OUT_DIR

# Bumped whenever the RECIPE changes (which base/mask knobs feed the pixels,
# not just their values) — see `scorch.py`'s own note on `SOOT_RGB`/
# `char_bite` and `wall_overlay.bake_mask`'s `v2` bump for the failure mode
# this guards: change the recipe without changing the key and every caller
# silently gets back the image baked under the OLD rule.
_RECIPE_VERSION = "facadebake_v1"

# 2 cm/px — 2048 px across a 40 m elevation, the exact arithmetic in the
# brief and "plenty" per `scorch.burn_mask_map`'s own 31 cm/px "mush"
# yardstick for a ground plate 8x that size. `_SIZE_MAX` is a safety cap for
# an unusually long run (a full block face), where density degrades
# gracefully the same way the ground plate's fixed 2048 px canvas already
# does for a large region — expected to be rare at building scale, unlike at
# ground-plate scale where it is the ordinary case.
_PX_PER_M = 50.0
_SIZE_MIN, _SIZE_MAX = 256, 2048

# Metres of wall ONE copy of the base texture is assumed to represent before
# it repeats. NOT a measured per-texture property — nothing in this kit
# records a real-world scale for an arbitrary wall base-colour map, so this
# is a documented ASSUMPTION, tuned to keep brick/panel cladding reading as
# roughly its own size rather than as a stretched poster or a fine static
# hiss. A caller that knows a better value for a specific texture should
# pass it through `tile_m`.
_TILE_M_DEFAULT = 3.0

# `soot_mask` saturates into a solid slab well before 1.0; the suburb's
# own good-looking references measure 0.25-0.45 and the licks that make
# it read are gone by 0.7. The fire's peak is mapped into that band.
# Coverage band. Raised after review: the first pass capped at 0.52 and
# came back "too light". `soot_mask` still keeps its licks well past
# that; what it loses at the very top is the CLEAN GAPS, which is
# correct for a burnt-out shell and wrong for a smoke-stained one — so
# the cap is high and the SEVERITY decides where in the band a building
# lands.
# MEASURED, and the reason the first curve read as "too light": coverage
# is NOT linear in visual density. `soot_mask` at 0.25 puts soot on ~2%
# of pixels — a merged F2 elevation came back at 0.204 brightness
# against a bare base of 0.204, i.e. no change at all. The suburb's own
# references bear this out: 0.25 is a thin fringe, 0.45 is the one that
# reads. So the floor sits where soot is actually visible and the gamma
# still reserves the top of the range for a burnt-out shell.
#     0.35 -> 0.43   0.62 -> 0.63   0.88 -> 0.84   1.00 -> 0.95
_COV_LO, _COV_SPAN, _COV_MAX, _COV_GAMMA = 0.28, 0.67, 0.95, 1.4

# The downward dribble below the fire floor, as a fraction of the plume's
# own height and strength. Short and faint on purpose: a compartment fire
# CLIMBS, and a stain as heavy below the opening as above it reads as a
# flood mark rather than a fire.
_DOWN_REACH, _DOWN_STRENGTH = 0.55, 0.70

# How far the plume reaches ABOVE the storeys that are actually alight,
# as a fraction of the lit band's own height.
_UP_REACH = 0.85

# THE V-PATTERN, from NFPA 921 Ch.6 via the fire-investigation literature.
# A V on a wall is a PLUME-GENERATED pattern: the three-dimensional fire plume
# cut by a two-dimensional surface, so the stain widens as it rises because
# the plume does. The geometry is diagnostic — "a narrow, sharply defined V
# with a low apex suggests a high-intensity fire of short duration, while a
# wide, broadly defined V with a high apex suggests lower intensity or longer
# duration, or both". So severity does not just darken the stain, it changes
# the ANGLE: an intense short burn throws a narrow V, a long smouldering one a
# wide one.
#   half-angle, radians, at severity 0 and 1
_V_ANGLE_LO, _V_ANGLE_HI = 0.62, 0.30      # ~35 deg wide -> ~17 deg narrow
# how wide the stain is AT the source, as a fraction of the source spacing
_V_ROOT_FRAC = 1.05
# Plumes per elevation when the caller does not know the real bay pitch.
_V_BAYS = 6
# How far a plume rises before fading, as a fraction of the elevation.
_V_RISE_FRAC = 0.34
# The downward continuation: narrower splay, shorter reach. It is a
# continuation of the same V, not a second pattern with its own end.
_V_DOWN_SPLAY, _V_DOWN_REACH = 0.45, 0.40
# PER-PLUME SIZE. Identical Vs read as a printed pattern no matter how good
# the individual shape is, and real ones differ wildly with how much fuel was
# behind each opening and how long it vented. Each source draws its own
# multiplier in this range, applied to BOTH the root width and the rise, so a
# big V is genuinely bigger rather than just wider. The count is unchanged —
# the sources still sit at the bay pitch — so bigger plumes overlap more,
# which is the point.
_V_SCALE_LO, _V_SCALE_HI = 2.0, 3.5
# A band around the seat held at full strength, as a fraction of the
# elevation — the fire's own compartment, the largest and darkest part
# of the plume.
_V_SEAT_HOLD = 0.06
# Sigmoid width for blending the upward and downward behaviour through
# the seat, as a fraction of the elevation. Bigger = softer handover.
_V_SOFT = 0.05
# Sources per bay. >1 scatters more plumes than there are bays, which is
# what stops a regular wall reading as a regular pattern.
_V_SOURCE_DENSITY = 1.6
# Exponent on the hard core under each plume. Lower = the solid black
# region around the apex spreads further out along the X.
_APEX_CORE = 2.2

# Merged pieces are worked at up to this size rather than at the base
# map's own resolution; see `merge_piece`.
_PIECE_MAX_PX = 512

# How hard local severity thins the mask away from the fire.
# 0 = the old uniform fade; 1 = only the strongest licks survive at
# the far edge. See the note in `building_skin`.
# Lowered after review: at 0.75 the soot died off too quickly as it
# wrapped round the building — "it fades into light a little too early
# for the fires around the wall" (user, 2026-08-29). At 0.45 the far
# elevations keep a real, if sparse, deposit instead of going clean.
_DENSITY_FLOOR = 0.30

# Vertical stretch of the streak noise. Higher = longer licks that run
# across several storeys instead of stopping inside one module.
_STREAK_STRETCH = 18.0


def _x_plume(h, w, r_src, c_src, sev, root_px, scale=1.0, seat_px=0.0):
    """ONE X-shaped plume rooted at (`r_src`, `c_src`). No top half, no bottom
    half, no join.

    "Have 1 X pattern. no top, no bottom. The X is like the V pattern on top
    and bottom is a smaller V but same pattern and continuous" (user,
    2026-08-29).

    The silhouette is a bowtie: half-width grows with distance from the seat
    in BOTH directions, faster upward (`tan_up`, the NFPA plume half-angle)
    than downward (`tan_dn`), so the upper V is large and the lower one is a
    smaller mirror of the same shape. Because the width is a function of
    `|dz|` alone, and the intensity is a single decay in `|dz|`, there is
    nothing piecewise anywhere in it — every earlier version split on the sign
    of `dz` and left a kink across the seat that read as a hard line however
    it was faded.

    `seat_px` is the height of the source itself. A fire venting from one
    window has a small seat; one that has taken several floors has a tall one
    and the X stands on a correspondingly long root.
    """
    sev = max(0.0, min(1.0, float(sev)))
    scale = max(0.1, float(scale))
    tan_up = math.tan(_V_ANGLE_LO + (_V_ANGLE_HI - _V_ANGLE_LO) * sev)
    tan_dn = tan_up * _V_DOWN_SPLAY
    root = root_px * scale

    rows = np.arange(h, dtype=float)[:, None]
    dz = float(r_src) - rows                     # >0 above the seat
    # smooth blend of the two splays through the apex: no kink at dz = 0
    soft = max(2.0, h * _V_SOFT * scale)
    up = 1.0 / (1.0 + np.exp(-dz / soft))
    tan_mix = tan_dn + (tan_up - tan_dn) * up
    lam = (h * _V_RISE_FRAC * scale) * (_V_DOWN_REACH
                                        + (1.0 - _V_DOWN_REACH) * up)
    lam = np.maximum(3.0, lam)

    hold = max(1.0, seat_px * 0.5 + h * _V_SEAT_HOLD * scale)
    az = np.abs(dz)
    half = root + az * tan_mix
    fall = np.exp(-np.maximum(0.0, az - hold) / lam)

    d = np.abs(np.arange(w, dtype=float)[None, :] - float(c_src))
    env = np.clip(1.0 - (d / np.maximum(1e-6, half)) ** 2, 0.0, 1.0)
    return np.clip(env * fall, 0.0, 1.0)


def _coverage_for(sev):
    """Map a 0..1 fire severity onto `soot_mask` coverage, on a CURVE.

    A straight line does not work here. `soot_mask` keeps its licks and its
    clean gaps at low coverage and fills into a solid slab near the top, so a
    linear map either leaves a burnt-out shell too light or makes a
    smoke-stained one solid black. Both have now been seen: capping at 0.52
    came back "too light", and a linear 0.30-0.92 came back as a uniformly
    black wall with no streaks left in it.

    The gamma keeps the bottom of the ladder in the range where the licks are
    the whole look, and lets only the top of it saturate — which is correct,
    because a burnt-out shell IS almost uniformly black ("For the building
    that's completely burnt out the texture has to be almost completely black
    anyway", user 2026-08-29) and a smoke-stained one is not.

        sev 0.30 -> 0.24     stain, licks and clean wall between them
        sev 0.50 -> 0.42     well alight
        sev 0.75 -> 0.66     heavy
        sev 1.00 -> 0.95     burnt out, near solid
    """
    sev = max(0.0, min(1.0, float(sev)))
    return max(0.05, min(_COV_MAX, _COV_LO + _COV_SPAN * (sev ** _COV_GAMMA)))


def _px_dims(w_m, h_m, px_per_m=_PX_PER_M, size_min=_SIZE_MIN,
             size_max=_SIZE_MAX):
    """(h, w) pixel canvas size for a `w_m` x `h_m` elevation."""
    w = max(size_min, min(size_max, int(round(float(w_m) * px_per_m))))
    h = max(size_min, min(size_max, int(round(float(h_m) * px_per_m))))
    return h, w


def _resample_rows(field_rows, h):
    """Nearest-neighbour resample of a length-N row field to length `h`.

    The same indexing `urban_fire._r_soot_overlay` already runs before
    handing a field to `wall_overlay.bake_mask` when the field's own row
    count does not match the target canvas — copied rather than imported
    because it is three lines of arithmetic with no state to share.
    """
    field_rows = list(field_rows)
    src = len(field_rows)
    if src == h:
        return field_rows
    return [field_rows[min(src - 1, int(round(
                i * (src - 1) / float(max(1, h - 1)))))]
            for i in range(h)]


def _tile_base(base, h, w, h_m, w_m, tile_m):
    """Tile `base` (Hb x Wb x 3, 0..1) across an `h` x `w` canvas at
    `tile_m` metres per copy.

    SAME TECHNIQUE `scorch.ground_burn_map`'s local `tiled()` closure uses
    (resize the source down to one tile's own pixel budget, `np.tile` it out
    to cover the canvas, crop to size) — generalised here to a non-square
    canvas and a non-square tile, since a building elevation is rarely
    either. Reusing the pattern rather than the closure itself because that
    one is private to `ground_burn_map` and keyed to a single `size x size`
    square canvas.
    """
    from PIL import Image

    px_per_m_h = h / float(h_m)
    px_per_m_w = w / float(w_m)
    k_h = max(2, int(round(float(tile_m) * px_per_m_h)))
    k_w = max(2, int(round(float(tile_m) * px_per_m_w)))
    small = np.asarray(
        Image.fromarray((base * 255.0).astype(np.uint8)).resize((k_w, k_h)),
        dtype=np.float64) / 255.0
    # Tile count from the ROUNDED tile size, not from `tile_m` directly —
    # same reasoning `ground_burn_map` records for its own `nt`: int() on a
    # metres/tile_m ratio loses up to a whole tile and leaves the canvas
    # short.
    nt_h = int(np.ceil(h / float(k_h))) + 1
    nt_w = int(np.ceil(w / float(k_w))) + 1
    return np.tile(small, (nt_h, nt_w, 1))[:h, :w, :]


def bake_facade_texture(base_url, field_rows, w_m, h_m, rng, out_dir=None,
                        px_per_m=_PX_PER_M, tile_m=_TILE_M_DEFAULT,
                        size_min=_SIZE_MIN, size_max=_SIZE_MAX,
                        streak_stretch=8.0, edge=0.16, wobble=0.55,
                        mottle=0.22, streak=None, down_tail_reach=None,
                        down_tail_strength=None, soot_rgb=None,
                        char_bite=0.72, verbose=False, key=""):
    """Bake ONE image for a whole elevation: `base_url`'s own texture,
    tiled at `tile_m` metres/copy, with the fire's mask composited over it.
    Returns a local PNG path, or `None` if the base texture could not be
    read.

    `field_rows` is `urban_fire._facade_field`'s own per-row (0..1) output —
    row 0 the TOP of the elevation, the same convention `_facade_field`,
    `wall_overlay.bake_mask` and `wall_overlay.author_quad` all already
    share — resampled here to the baked canvas height via
    `_resample_rows`, exactly as `_r_soot_overlay` already does before
    calling `wall_overlay.bake_mask`.

    `w_m`/`h_m` are the elevation's own metric size — `w_m` the run's own
    measured length (`urban_fire._wall_run_frame`'s `L`), `h_m` the mass's
    own height (`m["top"] - m["z0"]`) — NOT a generic "big enough" guess:
    the whole point of mapping this image with `facade_material` below is
    that ONE copy spans exactly this rectangle, so getting it right here is
    what keeps the bake and the material in agreement.

    CACHED BY CONTENT — `base_url`, the resampled field's own VALUES (not
    just its length), every mask and tiling knob, and `_RECIPE_VERSION` —
    so a bench of clone buildings that share a texture and a severity curve
    bake this once and every other instance reuses the file. Pass `key` to
    force distinct files for two calls that would otherwise collide on
    identical content but should not share a cache entry (or leave it
    default to get the reuse). The random DRAWS `rng` makes are NOT part of
    the key — same convention `wall_overlay.bake_mask` already uses for its
    own `rng` parameter — so two calls with identical stated inputs but a
    freshly-seeded `rng` silently share the first call's file rather than
    re-rolling; this is deliberate (it is what makes the cache work at all
    for a bench of near-identical buildings) and matches existing practice
    in this file's siblings, not a new trade-off invented here.
    """
    from PIL import Image

    streak = wo._STREAK_DEFAULT if streak is None else streak
    down_tail_reach = (wo._DOWN_TAIL_REACH_FRAC if down_tail_reach is None
                       else down_tail_reach)
    down_tail_strength = (wo._DOWN_TAIL_STRENGTH if down_tail_strength is None
                          else down_tail_strength)
    soot_rgb = scorch.SOOT_RGB if soot_rgb is None else soot_rgb

    out_dir = out_dir or OUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    h, w = _px_dims(w_m, h_m, px_per_m, size_min, size_max)
    field_h = _resample_rows(field_rows, h)

    field_key = hashlib.md5(
        ",".join("{0:.4f}".format(v) for v in field_h).encode("utf-8")
    ).hexdigest()[:12]
    recipe_key = "{0}-{1:.3f}".format(
        tuple(round(float(c), 4) for c in soot_rgb), float(char_bite))
    key_str = (
        "{0}|{1}|{2}|{3}|{4:.4f}|{5:.4f}|{6:.3f}|{7:.3f}|{8:.4f}|{9:.4f}|"
        "{10:.4f}|{11:.3f}|{12:.3f}|{13:.3f}|{14}|{15}|{16}".format(
            _RECIPE_VERSION, base_url, h, w, float(w_m), float(h_m),
            float(tile_m), float(streak_stretch), float(edge),
            float(wobble), float(mottle), float(streak),
            float(down_tail_reach), float(down_tail_strength), recipe_key,
            field_key, key))
    cache_key = hashlib.md5(key_str.encode("utf-8")).hexdigest()[:16]
    path = os.path.join(out_dir, "facadebake_{0}.png".format(cache_key))
    if os.path.exists(path):
        return path

    base = scorch.read_texture(base_url, max_px=2048)
    if base is None:
        if verbose:
            print("[facade_bake] could not read {0}".format(base_url))
        return None

    tiled = _tile_base(base, h, w, float(h_m), float(w_m), float(tile_m))

    mask = wo._level_set_mask(
        h, w, rng, field_h, streak_stretch=streak_stretch, edge=edge,
        wobble=wobble, mottle=mottle, streak=streak,
        down_tail_reach=down_tail_reach,
        down_tail_strength=down_tail_strength)

    out = scorch.composite(tiled, mask, soot_rgb=soot_rgb,
                           char_bite=char_bite)
    Image.fromarray((np.clip(out, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8),
                    "RGB").save(path)
    if verbose:
        print("[facade_bake] {0}x{1} <- {2} -> {3} ({4:.0%} mean soot)"
             .format(w, h, base_url.rsplit("/", 1)[-1],
                     os.path.basename(path), float(mask.mean())))
    return path


def facade_material(stage, path, tex_path, w_m, h_m, origin, yaw, *,
                    is_curtain_wall=False, z0=0.0, roughness=0.85,
                    brightness=1.0):
    """One OmniPBR, `tex_path` as its diffuse, `project_uvw` world-triplanar
    scaled so exactly ONE copy spans a `w_m` x `h_m` run anchored at
    `origin` — bind this directly onto every wall/corner element of that
    run (`urban_fire._bind_subsets`, the same mechanism every other pass in
    this file already uses) and the baked image is continuous across every
    module in the run, because the UV comes from world position rather than
    from any one module's own UV space.

    `origin` is `(x, y)`, `yaw` radians — `urban_fire._wall_run_frame`'s own
    `fr[0], fr[1], fr[2]`, the frame the run's bake (`bake_facade_texture`)
    was already sized against via that same function's `L`. `z0` is the
    mass's own base height (`m["z0"]`) — world Z is used directly as the
    vertical axis (`quake_flow._b_face_pt` does the same: its `v` parameter
    IS world height, no transform), so this only needs the one scalar to
    anchor the image's bottom edge.

    WHICH WORLD AXIS BECOMES "U" DEPENDS ON WHICH WAY THE WALL FACES. Every
    wall run in this kit is CARDINAL — `urban_fire._wall_run_frame`'s own
    docstring: "S faces +X, N faces -X, E faces +Y, W faces -Y" — so
    `_piece_frame`'s `u`-along-the-wall term (`ox + cos(yaw)*u`,
    `oy + sin(yaw)*u`) moves EXACTLY ONE of world X or world Y as `u`
    varies and leaves the other constant, never a blend of both. Triplanar
    projection is selected by the same logic in reverse — the plane whose
    NORMAL is closest to a world axis is excluded, so a wall running along
    world X (normal along Y) samples its in-plane U from world X, and one
    running along world Y samples U from world Y. `abs(cos(yaw))` vs
    `abs(sin(yaw))` distinguishes exactly those two cases.

    NOT VERIFIED IN KIT. This IS the established `project_uvw` /
    `world_or_object` recipe already shipped in this file's siblings
    (`damage._pbr`, `damage.scorched_material(triplanar=True)`,
    `ground.overlay_material`) — same shader inputs, same "u = x *
    texture_scale + texture_translate" formula `ground.overlay_material`
    documents — but which world axis OmniPBR's own triplanar branch treats
    as "U" for a given face orientation, and its sign, is Kit/MDL internal
    behaviour this module cannot see without a running Kit process (this
    dataset's rule throughout is verify offline; nothing here launches
    one). Getting the WORLD AXIS choice wrong would show as the baked
    image's along-wall streak pattern mirrored or start from the wrong end
    of the run on some sides — cosmetic, since the streak's own left/right
    position carries no meaning the way its VERTICAL position does — not as
    a violation of the hard placement constraints (soot above the origin,
    clean below it), which come from `field_rows`/`z0` alone and do not
    depend on this choice at all. Confirm the U-axis pick against a real
    building before trusting it for anything where left/right matters.

    A CURTAIN WALL IS REFUSED. Compositing soot into a tower family's pale
    glazing atlas is not a smaller version of this problem, it is a
    different failure the rest of this file already avoids per-module
    (`urban_fire.py`, ~978: "white panes with black ink-runs down them") —
    so this raises rather than producing a material that would reproduce
    it. The caller keeps binding a flat soot tone for glass, unchanged.
    """
    if is_curtain_wall:
        raise ValueError(
            "facade_material refuses a curtain wall: compositing soot into "
            "a pale glazing atlas gives \"white panes with black ink-runs "
            "down them\" (urban_fire.py, uf5 skyscraper_a, 2026-08-28) — "
            "bind a flat soot tone instead, the way r_smoke_stain already "
            "does for glassy buildings.")

    from pxr import Gf, Sdf, UsdShade

    ox, oy = float(origin[0]), float(origin[1])
    ax, ay = math.cos(float(yaw)), math.sin(float(yaw))
    u_world = ox if abs(ax) >= abs(ay) else oy

    su = 1.0 / max(1e-6, float(w_m))
    sv = 1.0 / max(1e-6, float(h_m))
    tu = -u_world * su
    tv = -float(z0) * sv

    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(tex_path))
    sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(1.0, 1.0, 1.0))
    sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(
        Gf.Vec2f(su, sv))
    sh.CreateInput("texture_translate", Sdf.ValueTypeNames.Float2).Set(
        Gf.Vec2f(tu, tv))
    if abs(float(brightness) - 1.0) > 1e-3:
        sh.CreateInput("albedo_brightness", Sdf.ValueTypeNames.Float).Set(
            float(brightness))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(),
                                                        "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


# ---------------------------------------------------------------------------
# PER-PRIM CROP — the user's own design, and the one that finally works
# ---------------------------------------------------------------------------
#
# "You create a scorch pattern for the whole building on a transparent
# background and then break it up into the same number of parts as the face of
# the building and merge it with each texture on each facade prim since we
# know the location of the physical prim and therefore the equivalent location
# on the new scorched png." (user, 2026-08-29)
#
# WHY THIS BEATS THE WORLD-TRIPLANAR MAPPING ABOVE. `facade_material` maps one
# baked image across a run with `project_uvw`, which works but replaces the
# module's own UVs wholesale — so a module's modelled relief no longer samples
# its texture the way the kit intended, and the projection has to be kept in
# agreement with the run's metric extent by hand. Cropping instead leaves
# every module's own UVs and its own material stack completely alone: the only
# thing that changes is that its base-colour PNG now has this building's soot
# painted into the part of it that faces the fire.
#
# The soot stays continuous across module boundaries because every crop comes
# out of ONE building-wide image, and adjacent modules take adjacent crops —
# the same reason a jigsaw's picture survives being cut up.


def scorch_layer(field_rows, w_m, h_m, rng, px_per_m=_PX_PER_M,
                 size_min=_SIZE_MIN, size_max=_SIZE_MAX, streak_stretch=8.0,
                 soot_rgb=None, char_bite=0.72, **mask_kw):
    """The building-wide soot, as RGBA on a TRANSPARENT background.

    Alpha is the mask itself, so "unburnt" is genuinely absent rather than a
    white or black rectangle that would wash the base texture when composited.
    Returns `(rgba, h, w)` with row 0 at the TOP of the elevation — the
    convention `urban_fire._facade_field` already uses.
    """
    h, w = _px_dims(w_m, h_m, px_per_m, size_min, size_max)
    field = _resample_rows(field_rows, h)

    # THE SUBURB'S OWN MASK, SCALED BY THE FIRE'S VERTICAL PROFILE.
    #
    # NOT `wall_overlay._level_set_mask`. That builds a THRESHOLDED REGION —
    # `(field - thresh) / edge` — which saturates to fully opaque wherever the
    # field comfortably clears the threshold, so all of its structure lives at
    # the boundary and its interior is a flat slab. Rendered, that is a solid
    # rectangle with a wavy bottom edge, which is exactly what came back:
    # "it's very rectangular, what was the scortch pattern" (user,
    # 2026-08-29), on a layer measuring alpha mean 0.489 with the variation
    # confined to broad soft banding.
    #
    # `scorch.soot_mask` is the generator the SUBURB uses and it is not a
    # region at all: a wash plus streaks (noise stretched ~8x vertically, so
    # it smears into licks) plus patches, continuous everywhere, so it is
    # mottled and streaky across its whole area rather than only at its rim.
    # Multiplying it by the per-row profile gives the fire's vertical falloff
    # while keeping that structure — and multiplication preserves the one
    # invariant that matters, because `field == 0` rows stay exactly 0 no
    # matter what the noise draws.
    field = np.asarray(field, dtype=float)   # `_resample_rows` yields a list
    peak = float(np.max(field)) if len(field) else 0.0
    base_mask = np.asarray(
        scorch.soot_mask(h, w, rng, coverage=max(0.05, min(1.0, peak)),
                         from_below=True, streak_stretch=streak_stretch),
        dtype=float)
    prof = field / max(1e-9, peak)
    m = np.clip(base_mask * prof[:, None], 0.0, 1.0)
    rgb = np.asarray(soot_rgb if soot_rgb is not None else scorch.SOOT_RGB,
                     dtype=float)
    rgba = np.zeros((h, w, 4), dtype=float)
    rgba[..., :3] = rgb[None, None, :]
    # ALPHA TO THE FULL RANGE. It used to cap at ~0.82 (`0.35 + 0.65 *
    # char_bite`), so even a fully charred seat stayed translucent and the
    # wall showed through it — the apex could never be pitch black. Opacity
    # still drives the desaturation in `merge_piece`, matching
    # `scorch.composite`'s own behaviour; it simply reaches 1.0 now.
    rgba[..., 3] = np.clip(m, 0.0, 1.0)
    return rgba, h, w


def crop_for_piece(rgba, u0, u1, v0, v1):
    """The part of the building-wide layer that covers one prim's face.

    `u0/u1` are the piece's own span ALONG the run and `v0/v1` its span UP
    the elevation, both normalised 0..1 against the run's extent. Row 0 of
    `rgba` is the TOP, so `v` is flipped on the way in — getting that wrong
    puts the soot on the wrong storey and is exactly the class of error the
    placement test exists to catch.
    """
    h, w = rgba.shape[0], rgba.shape[1]
    x0 = int(round(max(0.0, min(1.0, min(u0, u1))) * (w - 1)))
    x1 = int(round(max(0.0, min(1.0, max(u0, u1))) * (w - 1)))
    y0 = int(round((1.0 - max(0.0, min(1.0, max(v0, v1)))) * (h - 1)))
    y1 = int(round((1.0 - max(0.0, min(1.0, min(v0, v1)))) * (h - 1)))
    x1 = max(x1, x0 + 1)
    y1 = max(y1, y0 + 1)
    return rgba[y0:y1, x0:x1]


def merge_piece(base_url, crop, out_dir=None, verbose=False, key=""):
    """This prim's OWN texture with its crop of the building's soot merged in.

    Returns a local PNG path, or `None` if the base cannot be read. Cached on
    the base URL and the crop's own pixels, so the many modules of a run that
    share a texture but take different crops each bake once and clones of a
    building reuse the lot.
    """
    from PIL import Image

    a = np.asarray(crop, dtype=float)
    ck = hashlib.md5(np.round(a, 3).tobytes()).hexdigest()[:16]
    kk = hashlib.md5("|".join((_RECIPE_VERSION, "piece", str(base_url), ck,
                               str(key))).encode("utf-8")).hexdigest()[:16]
    d = out_dir or OUT_DIR
    try:
        os.makedirs(d, exist_ok=True)
    except OSError:
        pass
    path = os.path.join(d, "facadepiece_{0}.png".format(kk))
    if os.path.exists(path):
        return path

    base = scorch.read_texture(base_url) if base_url else None
    if base is None:
        return None
    base = np.asarray(base, dtype=float)
    if base.ndim == 2:
        base = np.repeat(base[..., None], 3, axis=2)
    base = base[..., :3]
    bh, bw = base.shape[0], base.shape[1]

    # stretch the crop onto the base's own pixel grid — the crop IS this
    # piece's face, so it maps corner to corner
    # DO NOT RESAMPLE THE CROP DOWN ONTO A SMALL BASE MAP.
    # These kit base textures are 128x128. Sampling a crop taken from a
    # 2048-wide skin onto that grid destroys the licks — measured: every one
    # of 842 baked pieces came out 128x128 and the streak structure was gone,
    # which is why it read as "too light and the streaks aren't long enough"
    # (user, 2026-08-29) no matter how good the skin itself was. Work at
    # whichever is LARGER, upsampling the base if need be: the base is a
    # tiling cladding map with no fine detail to lose, while the soot is the
    # thing being looked at.
    oh = max(bh, min(_PIECE_MAX_PX, a.shape[0]))
    ow = max(bw, min(_PIECE_MAX_PX, a.shape[1]))
    byi = np.linspace(0, bh - 1, oh).astype(int)
    bxi = np.linspace(0, bw - 1, ow).astype(int)
    base = base[byi][:, bxi]
    yi = np.linspace(0, a.shape[0] - 1, oh).astype(int)
    xi = np.linspace(0, a.shape[1] - 1, ow).astype(int)
    c = a[yi][:, xi]
    al = c[..., 3:4]
    grey = base.mean(axis=2, keepdims=True)
    desat = base * (1.0 - 0.72 * al) + grey * (0.72 * al)
    out = np.clip(desat * (1.0 - al) + c[..., :3] * al, 0.0, 1.0)
    Image.fromarray((out * 255.0 + 0.5).astype("uint8")).save(path)
    if verbose:
        print("[facade_bake] piece {0} <- crop {1}x{2}, alpha mean {3:.3f}"
              .format(os.path.basename(path), a.shape[1], a.shape[0],
                      float(al.mean())))
    return path


def piece_material(stage, path, tex_path, roughness=0.85):
    """The merged per-prim texture on the module's OWN UVs.

    NOT `facade_material`: that one turns on `project_uvw` so a single image
    can span a run, which is right for the whole-run mapping and WRONG here.
    A crop is already cut to this module's own face, so the module's existing
    UVs address it correctly — and leaving them alone is the entire point of
    the crop approach, because it keeps the kit's modelled relief sampling its
    texture the way the kit intended and needs no guess about which world axis
    becomes U on which elevation.
    """
    from pxr import Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(str(tex_path)))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(float(roughness))
    for nm in ("mdl",):
        mat.CreateSurfaceOutput(nm).ConnectToSource(sh.ConnectableAPI(), "out")
        mat.CreateDisplacementOutput(nm).ConnectToSource(sh.ConnectableAPI(),
                                                         "out")
        mat.CreateVolumeOutput(nm).ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


# ---------------------------------------------------------------------------
# THE BUILDING SKIN — one unwrapped image around the whole perimeter
# ---------------------------------------------------------------------------
#
# "you can generate it like people do textures that wrap around things. Think
# minecraft skin. That way you can make sure it's continuous across sides and
# stuff." (user, 2026-08-29)
#
# Baking a separate mask per elevation cannot be continuous at a corner: two
# independent noise draws meet there and the soot stops dead. Unwrapping the
# building into ONE image whose width is the PERIMETER makes every corner an
# interior seam of a single pattern, so a plume that reaches a corner carries
# round it exactly as it would on a real building.
#
# The last corner (W back to S) closes for free: `scorch._noise` builds its
# field in the frequency domain, and its own docstring records that this makes
# it "spectral and its DFT is periodic, so streaks and patches tile exactly" —
# so column 0 and column w-1 already agree.


def _ragged_profile(prof, h, w, rng, amp_frac=0.06):
    """The vertical profile, with its boundary DISPLACED per column.

    A profile applied straight across every column puts the bottom of the
    burn on one dead-straight row — the hard horizontal line that made the
    first skin read as a band. Displacing the profile vertically by a
    low-frequency noise per column makes that boundary ragged WITHOUT adding
    coverage anywhere: the values are the same, sampled at a shifted row, so
    a row whose profile is 0 stays 0 wherever it lands. Same level-set
    discipline `burn_mask_map` states — noise moves the boundary, it never
    adds.
    """
    amp = max(1.0, amp_frac * h)
    d = scorch._noise(rng, 8, w, beta=2.0, stretch_v=1.0)[0]
    d = (d - d.mean())
    d = d / max(1e-9, np.abs(d).max()) * amp
    rows = np.arange(h)[:, None] + d[None, :]
    lo = np.clip(np.floor(rows).astype(int), 0, h - 1)
    hi = np.clip(lo + 1, 0, h - 1)
    t = np.clip(rows - lo, 0.0, 1.0)
    return prof[lo] * (1.0 - t) + prof[hi] * t


def building_skin(side_fields, side_lengths, h_m, rng, storey_px=None,
                  px_per_m=_PX_PER_M,
                  size_min=_SIZE_MIN, size_max=_SIZE_MAX,
                  streak_stretch=_STREAK_STRETCH,
                  soot_rgb=None, char_bite=0.72, amp_frac=0.06):
    """ONE soot layer unwrapped around the whole building.

    `side_fields` maps side -> per-row coverage (row 0 = top), `side_lengths`
    maps side -> that elevation's length in metres. Returns
    `(rgba, spans)` where `spans[side] = (u0, u1)` in 0..1 of the skin, so a
    prim's crop is taken from its own side's span and the pattern is shared
    across every corner.
    """
    order = [s for s in ("S", "E", "N", "W") if s in side_lengths]
    total_m = sum(float(side_lengths[s]) for s in order) or 1.0
    h, w = _px_dims(total_m, h_m, px_per_m, size_min, size_max)

    # one mask for the whole perimeter, at the strongest side's coverage
    peak = 0.0
    for s in order:
        f = np.asarray(side_fields.get(s) or [0.0], dtype=float)
        peak = max(peak, float(f.max()) if f.size else 0.0)

    # ANCHOR THE SUBURB'S MASK AT THE FIRE, DO NOT MULTIPLY BY A PROFILE.
    #
    # `scorch.soot_mask(from_below=True)` ALREADY carries the vertical
    # structure that makes the suburb read: solid at its base, distinct
    # licks rising off it to different heights, clean gaps between them, a
    # jagged fingered top edge. Multiplying that by a smooth decaying
    # profile — the first attempt here — destroys precisely that structure
    # and leaves a smooth gradient with fine hair texture and no gaps, which
    # is what came back as "it's very rectangular" and then "the scorch
    # streaks in the suburb is what made it look really good" (user,
    # 2026-08-29).
    #
    # So the mask is generated at the height of the BURN BAND and pasted so
    # its base sits on the fire's own origin row. Its own falloff then IS the
    # vertical profile, licking upward from the seat of the fire the way a
    # plume actually stains a wall — and everything below the origin stays
    # untouched because nothing is written there.
    # ONE MASK AROUND THE WHOLE PERIMETER, amplitude varied per side.
    #
    # Drawing a separate `soot_mask` per elevation makes every corner a hard
    # seam — two independent noise draws meeting, so the licks stop dead at
    # the red line. Generating ONE mask the full width of the perimeter and
    # scaling it by a per-column amplitude keeps the pattern continuous round
    # every corner (and round the LAST one for free: `scorch._noise` builds
    # its field in the frequency domain and is therefore periodic, so column
    # 0 and column w-1 already agree) while each elevation still shows only
    # as much soot as its own distance from the fire earns.
    #
    # COVERAGE IS CAPPED. `soot_mask` fills solid well before 1.0 — measured,
    # a 0.95 draw is a near-uniform black slab with no licks left in it,
    # while the suburb's own good-looking references sit at 0.25-0.45. The
    # licks ARE the look, so the field's peak is mapped into that band rather
    # than passed through raw.
    spans, acc = {}, 0.0
    amp = np.zeros(w)
    prof2d = np.zeros((h, w))
    top_r, base_r = h, 0
    for s_ in order:
        L = float(side_lengths[s_])
        u0, u1 = acc / total_m, (acc + L) / total_m
        acc += L
        spans[s_] = (u0, u1)
        x0 = int(round(u0 * (w - 1)))
        x1 = max(x0 + 1, int(round(u1 * (w - 1))))
        f = np.asarray(_resample_rows(list(side_fields.get(s_) or [0.0]), h),
                       dtype=float)
        lit = np.where(f > 0.01)[0]
        if len(lit):
            top_r = min(top_r, int(lit.min()))
            base_r = max(base_r, int(lit.max()))
        amp[x0:x1] = float(f.max()) if f.size else 0.0
        pk = float(f.max()) if f.size else 0.0
        if pk > 0:
            prof2d[:, x0:x1] = (f / pk)[:, None]

    # EVERY ALIGHT POINT IS ITS OWN SOURCE.
    #
    # "if there is a point that's alight or was alight based on the state of
    # the house, that should be a starting point for the spread of scorch up
    # and down. So there can be multiple starts for scorching up and down
    # based on where the fire is" (user, 2026-08-29).
    #
    # One merged band over all the lit storeys was wrong twice over: it put a
    # single plume base at the lowest lit row, so a fire on storeys 1 AND 4
    # stained as one tall smear instead of two; and it left one shared row
    # where the upward mask met the downward one, visible as a ruled line
    # across the whole elevation no matter how the two were crossfaded.
    #
    # Each contiguous lit block now emits its OWN plume — up strongly, down
    # weakly — and they are combined with a maximum. Overlapping plumes build
    # up naturally, the handover rows differ per source so no single line
    # survives, and a building alight at two separate levels reads as exactly
    # that.
    # smooth the per-side amplitude around the perimeter FIRST, so a corner
    # is a gradient rather than a step; wraps, because the skin wraps
    k = max(3, int(round(w * 0.11)) | 1)
    pad = np.concatenate([amp[-k:], amp, amp[:k]])
    amp_s = np.convolve(pad, np.ones(k) / float(k), mode="same")[k:k + w]

    # ONE noise field for the whole skin, so every plume shares the same
    # streak texture and the wall reads as one surface. The wash stays ON:
    # removing it flattened the plumes into a uniform field with no shape
    # left in them (measured, and visibly worse), because the wash is what
    # gives `soot_mask` its licks their length.
    # ONE noise field, and it is TEXTURE ONLY.
    #
    # `scorch.soot_mask` builds its mask from a WASH (a smooth vertical ramp),
    # STREAKS (noise stretched vertically into licks) and PATCHES (mottling).
    # The wash is right for a ground scar, where it IS the profile, and wrong
    # here: the X envelopes already own every bit of vertical structure, so a
    # second unrelated ramp underneath them darkens the bottom of the canvas,
    # fades its top, and shows its own edge wherever a plume is thin. That
    # edge is the horizontal line that survived every attempt to fix the
    # plume split, because it was never the plume.
    #
    # `wash_weight=0` leaves the licks and the mottling, which is all this
    # layer is for. There is one field for the whole skin — not one per
    # direction and not one per side — so the whole building reads as one
    # surface.
    noise = np.asarray(
        scorch.soot_mask(h, w, rng, coverage=_coverage_for(float(np.max(amp))),
                         from_below=True, wash_weight=0.0,
                         streak_stretch=streak_stretch),
        dtype=float)
    # renormalise: with the wash gone the mask's own peak drops, and the
    # texture should still reach full strength where the fire is
    nmax = float(noise.max())
    if nmax > 1e-6:
        noise = noise / nmax

    # SOURCES ARE SCATTERED, NOT GRIDDED.
    # "Not all the fire sources will be on the same story. Some will have
    # source across stories. They also all be different sizes" (user,
    # 2026-08-29). A regular lattice of identical plumes is what made every
    # earlier version read as a printed pattern, however good one plume
    # looked. So each source draws its own row, column, size and seat height,
    # with the row weighted by the field so sources land where the fire
    # actually is.
    m = np.zeros((h, w))
    st_px = float(storey_px or max(6.0, h / 6.0))
    for s_ in order:
        u0, u1 = spans.get(s_, (0.0, 0.0))
        x0 = int(round(u0 * (w - 1)))
        x1 = max(x0 + 1, int(round(u1 * (w - 1))))
        f = np.asarray(_resample_rows(list(side_fields.get(s_) or [0.0]), h),
                       dtype=float)
        if f.max() <= 0.01:
            continue
        span_px = x1 - x0
        n_src = max(2, int(round(span_px / max(8.0, span_px / float(_V_BAYS)))))
        n_src = int(round(n_src * _V_SOURCE_DENSITY))
        wgt = f / f.sum()
        rows_pool = np.arange(h)
        for _k in range(n_src):
            r_seat = int(rng.choice(rows_pool, p=wgt))
            loc = float(f[r_seat])
            if loc <= 0.01:
                continue
            cx = x0 + float(rng.uniform(0.0, 1.0)) * span_px
            sc = float(rng.uniform(_V_SCALE_LO, _V_SCALE_HI))
            # some sources are a single vented window, some have taken
            # several floors and stand on a much taller seat
            seat = st_px * float(rng.choice([0.5, 1.0, 1.0, 2.0, 3.0],
                                            p=[0.30, 0.30, 0.20, 0.15, 0.05]))
            env = _x_plume(h, w, r_seat, cx, loc, _V_ROOT_FRAC * st_px,
                           scale=sc, seat_px=seat)
            m = np.maximum(m, env * loc)

    peak_env = float(m.max())
    if peak_env > 1e-6:
        m = m / peak_env

    # PITCH BLACK AT THE APEX. The apex of an X is where the fire actually is,
    # so it is not stained — it is charred through, and no texture should show
    # in it: "Think that a fire is at the apex of the X. That means it has to
    # be pitch black at that apex" (user, 2026-08-29).
    #
    # Multiplying the envelope by the noise alone can never do that, because
    # the noise is below 1 almost everywhere. Taking the MAXIMUM of the
    # textured mask and a hard core keeps the licks out at the edges — where
    # the deposit really is patchy — while the seat itself goes solid.
    m = np.maximum(m * noise, np.clip(m, 0.0, 1.0) ** _APEX_CORE)

    # DENSITY FOLLOWS THE ENVELOPE, NOT THE RAW FIELD.
    #
    # This used to derive local severity from `prof2d`, i.e. straight from
    # `_facade_field`'s per-row values — and that field is a STEP: full inside
    # the burning storeys, zero outside. Subtracting a floor computed from it
    # re-imposed that step on top of the plumes, which is what produced the
    # hard horizontal cut-off the plumes themselves never had. MEASURED: the
    # raw skin jumped 0.2226 in a single row, exactly at the bottom edge of
    # the lit band, while every plume crossing it was smooth.
    #
    # The accumulated envelope already IS proximity to the fire — it is built
    # from the sources' own positions and falls off smoothly in every
    # direction — so thinning against `m` itself keeps the coupling ("how much
    # it fills and gets dark should be proportional to ... proximity of that
    # part to the fire") with nothing piecewise left in it.
    m = np.clip((m - _DENSITY_FLOOR * (1.0 - m))
                / max(1e-6, 1.0 - _DENSITY_FLOOR * 0.5), 0.0, 1.0)

    m = np.clip(m, 0.0, 1.0)
    rgb = np.asarray(soot_rgb if soot_rgb is not None else scorch.SOOT_RGB,
                     dtype=float)
    rgba = np.zeros((h, w, 4), dtype=float)
    rgba[..., :3] = rgb[None, None, :]
    # ALPHA TO THE FULL RANGE, so the apex can actually reach solid black.
    rgba[..., 3] = np.clip(m, 0.0, 1.0)
    return rgba, spans
