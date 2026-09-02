"""gac_fire — a MERGED whole-asset building (GreatAmericanCity) through the
fire ladder, with its soot planned and baked BEFORE the slice.

WHY BEFORE THE SLICE. `urban_fire.burn_building` on a kit building bakes the
soot skin into every module's own base map through that module's UVs
(`_bind_soot` / `soot_bake`) — one baked map per sooted module, which is right
for a kit whose modules each carry their own small atlas. A GAC building is
the opposite: ONE mesh, ~14 material subsets, a few dozen textures shared by
the whole façade, then sliced into hundreds of pieces that all sample the
same atlases. Baking per piece would write hundreds of copies of the same
2K maps. So (user, 2026-08-30: "we have the 1 png that's on the building and
we can set up the soot pattern before splitting it up"):

    measure the windows and the storey grid on the MERGED asset
      -> a mass box + a fire plan (`urban_fire.plan_fire`) + fire EVENTS from
         the asset's own window islands (`soot_plume.plan_events`)
      -> the soot skin (`soot_plume.skin`) round that box
      -> baked ONCE into each material's atlas through the merged mesh's UVs
         (`soot_bake.uv_position_map` on the de-indexed mesh `read_mesh`
         already produces for the slicer; the side of every texel is the
         nearest wall line, so one bake covers all four elevations)
      -> a copy of each material with only its diffuse map swapped
         (`soot_plume.piece_material_like`)
      -> THEN `gac_storey_slice.slice_to_kit` (or a baked kit via
         `kit_bake.load_kit`), every piece subset rebound to the sooted copy
         of whatever it was bound to
      -> `burn_building(..., fire=, events=, openings_fn=, soot_prebaked=True)`
         runs the rest of the ladder — windows, gutting, roof, collapse,
         flames from the SAME events — and skips its own per-piece soot.

The same list of events drives the stain and the flames, exactly as on a kit
building; the only difference is WHERE the bake happens.

WHAT THE WINDOWS ARE. `gac_slice.window_centres` returns one point per glass
FACE; a window is an island of those. `window_rects` groups the glass faces
of each elevation into islands by grid-hashed union-find on their bboxes and
returns (u0, u1, z0, z1) per island in the asset's frame — these are the
openings the events vent through, in the record shape `soot_plume` and
`urban_fire._flame_sources` expect (a synthetic wall frame per side, so
`quake_flow._b_face_pt` places the Flow emitters on the real façade plane).

GLASS. The slicer cannot address a window, so `r_window_burnout` does
nothing on a GAC piece (it says so and returns, `soot_prebaked` being a
set/frozenset). `damage_windows` (`darken_glass` is now an alias) treats the
merged asset's own measured window ISLANDS instead, per window rather than
per whole material subset: a hot-side window in the fire's own band is
burnt out to a real see-through hole; one storey above the band, and any
band-or-above storey on a side that neighbours a hot one, is crazed —
sooted, dark, semi-transparent, still glazed; everything else keeps its
glass. `burn_gac` tops up its own `fit_storeys` so there is a gutted floor
and its fallen debris behind a hole that is now something you can see
through, not just an opaque void tone.
"""

import hashlib
import math
import os

import numpy as np

GAC_DIR = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
           "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
GAC_SCALE = 0.01                 # the pack is authored in centimetres
ISLAND_CELL_M = 0.30             # grid hash cell for window-island grouping
# WINDOW-SHAPED, OR NOT (fire_dtc2 review, 2026-08-30: "B2 [dtc Building_12
# F3] seems to be cut up into triangles"). `_islands` merges glazing faces by
# proximity; a material that matches `gac_slice.is_glazing` on NAME ALONE
# (not a real texture) can still bridge into that merge. MEASURED
# (`tools/dtc_island_probe.py`): downtowncity's `ICity_Window_AC_00N` air-
# conditioner units match on "window" in the name and merged their own
# triangulated grille/fin geometry into an 18 m tall island alongside the
# real punched windows beside them (a 146/79/50-face blob on Building_12, a
# 218-face one on Building_11); `damage_windows` then burnt/crazed those
# small triangular facets individually -- the "triangles". Real punched
# windows on this stock measure aspect (width/height) 0.6-2.6, height
# 1.0-2.8 m, width 0.6-4.1 m, essentially fully glazed (fill >= 0.85); the
# ranges below are looser on purpose -- wide enough for a taller storefront
# pane and for every GAC window (MEASURED zero islands rejected on
# `SM_Building_02`, whose glass is aspect 0.61, height 1.8 m, width 1.1 m),
# tight enough to reject an equipment-contaminated blob or a whole
# continuous curtain-wall skin (dtc Amar_Tower: one glazing island the full
# 192 m of the tower's height).
WINDOW_ASPECT_RANGE = (0.2, 5.0)        # island width / height
WINDOW_HEIGHT_RANGE_M = (0.8, 4.0)
WINDOW_WIDTH_RANGE_M = (0.5, 8.0)
WINDOW_FILL_MIN = 0.4                   # glazing face area / bbox area
# A MERGED ISLAND'S BBOX IS NOT EXACTLY THE AUTHORED DIMENSION. Union-find
# aggregates thousands of individual face bboxes (`_islands`); their extreme
# corners carry the source mesh's own floating-point noise, so a window
# authored at a round 4.0 x 4.0 m can merge to 4.000003 m and miss the `<=`
# bound by a few microns (MEASURED, `tools/_gac_glass_probe_TMP.py`,
# fire_dtc4 2026-08-31: SM_Building_26's E elevation lost 5 of its 85 real
# islands this way — h=4.000001..4.000005 m against a hard 4.0 m ceiling,
# every one aspect ~1.0-1.25 and fill 0.88-1.45, textbook windows in every
# way except a sub-millimetre overshoot). `WINDOW_DIM_EPS_M` slack on both
# height and width bounds absorbs that noise without opening the door to a
# real curtain-wall/strip island (those overshoot by METRES, not
# millimetres, and still fall to the STRIP_* split below).
WINDOW_DIM_EPS_M = 0.08
# CONTIGUOUS GLAZING IS A GRID OF BAYS, NOT "NO WINDOWS AT ALL". An island
# the shape filter rejects for SIZE (a storey-strip window running the
# height of the façade — dtc Building_12, z 9-27 m; a curtain wall whose
# panes all touch — dtc Amar_Tower, one 38 x 192 m island) is real glazing:
# dropping it left those façades with ZERO openings in the burning band, so
# `plan_events` drew no events, and an F3/F5c baked with no flames, no smoke
# and no soot (fire_dtc3 review, 2026-08-30). Such an island is split into a
# synthetic bay grid at the pitches below and each cell re-tested; only
# islands that are mostly glass are split (`STRIP_FILL_MIN` — a sparse blob
# chained across a wall must not paint windows onto masonry).
STRIP_ROW_M = 3.2                       # bay-grid row pitch (storey-ish)
STRIP_COL_M = 4.0                       # bay-grid column pitch
STRIP_FILL_MIN = 0.4                    # min glazing fill to split at all
# NEITHER SHAPE FIX ABOVE CATCHES A WALL WITH NO GLAZING FACES AT ALL.
# `window_rects` only ever islands GEOMETRY that already matched
# `gac_slice.is_glazing` on its texture/material name; a side (or a whole
# building) that carries none gives `_islands` nothing to merge in the first
# place, however loose the shape test gets. MEASURED
# (`tools/_gac_glass_probe_TMP.py`, fire_dtc4 2026-08-31): `SM_Building_11`
# and `SM_Building_27` have ZERO vertical faces on ANY glazing-tagged
# material anywhere on the whole mesh — every texture on both ("...Metall...",
# "...Marble...", "...Images_Fake_03...", "...Images_Office_Home...") misses
# every token in `gac_slice.GLASS_TEX`, which is why `Building_27` never
# joined the pack's other 9 known painted-glazing towers even though it is
# named right beside them in that token list's own comment — a real gap in
# the token list, but chasing one texture name per building is the same
# whack-a-mole that list's history already is. And a wall can be a genuine,
# intentional blank party wall: `SM_Building_26`'s glazing is real and only
# on E (31488 candidate faces there, ZERO nearest-face candidates on S/N/W);
# `SM_Building_02`'s real E-side glazing stops climbing a couple of storeys
# short of its own roofline, so a fire planned at the top storey (a
# mechanical/parapet floor with no windows at all) finds nothing there
# either, even though E is a real elevation elsewhere on the same building.
# A CALLER CAN ALSO JUST ASK FOR THE WRONG SIDE. The city bake never lets
# `prepare` pick its own `sides` — `urban_fire_spread.entry_for_plan_fire`
# hands them in, chosen by which NEIGHBOUR lit this building (contagion
# geometry), never by which wall has glass — so a building whose only real
# glazing is on E can still be told to vent on N and W.
#
# All three read the same downstream: `openings_provider` asked for a
# (side, storey) inside the fire's own band that the real islands never
# reached returns nothing, `soot_plume.plan_events` draws no events, and the
# bake finishes as a building with a fire plan and no fire — the
# starved-events trap, one level up. The fix here is NOT a shape test: it is
# a synthetic bay-window grid over the MEASURED wall plane, authored only
# for a (side, storey) the real islands leave completely empty (see
# `_synthetic_side_rects` / `openings_provider`'s `band=` argument below), so
# a building keeps its real windows wherever it has any and only ever
# invents openings where there truly are none to find.
SYN_BAY_PITCH_M = 4.0                   # centre-to-centre bay spacing
SYN_WIN_W_M = 1.6                       # synthetic window width
SYN_WIN_H_FRAC = 0.5                    # fraction of the storey band used
SYN_SILL_FRAC = 0.35                    # sill height as a fraction of the band
SYN_EDGE_MARGIN_M = 1.2                 # blank corner/pier margin per end
#: "avoid fires at the extreme top of buildings cause a lot of them don't
#: have windows there and it just looks weird... unless we're 100% sure
#: about windows on the top floor" (user, 2026-08-31). A SYNTHETIC opening is
#: never "sure" by definition — it is a plausible bay-window grid invented
#: where the real islands found NOTHING — so the top `SYN_TOP_EXCLUDE_
#: STOREYS` storeys of a mass (mechanical penthouse, parapet/roof-access
#: floor, the top-floor "no real windows up there" pattern this whole
#: mechanism exists to paper over) never get a synthetic grid at all. A
#: building measured with REAL glazing on its top floor is untouched — this
#: only ever gates `_synthetic_side_rects`.
SYN_TOP_EXCLUDE_STOREYS = 2
BAKE_PX_MIN, BAKE_PX_MAX = 1024, 2048
# A texel is SHARED when faces more than this far apart in height both
# sample it — the atlas tiles up the building, and a pre-slice bake would put
# one storey's soot on every storey that reuses the texel. Such atlases are
# baked per PIECE after the slice instead (the kit path).
SHARED_TEXEL_M = 2.0
SHARED_FRAC_MAX = 0.08

DTC_DIR = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
           "scene_gen/assets/downtowncity/")
DTC_SCALE = 1.0                  # `mpu = 1`: downtowncity is authored in metres

# AEC BROWNSTONES. `airstack://`, not `omniverse://` — this pack ships as a
# LOCAL mirror under the repo (`scene_gen/assets/aec/`, `.gitignore`d but
# present on every built container/pod), the same tree every
# `config/asset_sets/*.yaml` "rowhouse" pool already references
# (`urban.yaml`: "airstack://scene_gen/assets/aec/brownstone/Assets/
# Create_Brownstone02/Reference_Brownstone{N}Row.usd"). The `omniverse://
# .../NVIDIA/Demos/AEC/Residential/Brownstones/...` path an earlier pass of
# this investigation assumed is the RAW, un-mirrored NVIDIA content pack —
# MEASURED unreachable from this container (`Usd.Stage.Open` on it raises
# "Failed to open layer"), and it is not what any real placement's `usd`
# field carries anyway. `AEC_DIR` matches production exactly so `bake_kind`'s
# `usd.startswith(spec["dir"])` gate (a plain string prefix test against a
# placement's OWN un-resolved `usd` field) actually fires for a real
# building; `asset_url()` below resolves the `airstack://` scheme to an
# openable path before anything calls `place_source` with it, the same way
# every OTHER `airstack://` caller in this repo resolves before referencing
# (`scene_generator._join_asset_root` — see `fire_pack_rows_launch_script.
# place_asset`, `urban_tornado_bench_launch_script`'s `shed_url`). `gac`/`dtc`
# are unaffected: an `omniverse://` URL passes `_join_asset_root` through
# unchanged.
AEC_DIR = "airstack://scene_gen/assets/aec/brownstone/Assets/Create_Brownstone02/"
AEC_SCALE = 0.01                 # every AEC pool entry in the asset sets is
                                  # authored `scale: 0.01` (centimetres)

# Materials that are NOT the building. Amar_Tower carries a ROOF GARDEN baked
# into the same merged mesh — MEASURED (`tools/_dtc_reg_probe.py`,
# 2026-08-30): ten tree/grass subsets between z 217 m and z 231 m that inflate
# the asset bbox from the building's own 42.3 x 42.3 x 221.1 m to
# 42.3 x 48.8 x 231.4 m — 6.5 m of depth and 10.3 m of height. Left in,
# `measure_grid` lattices three phantom storeys of canopy above the real roof,
# `mass_from_grid`'s deck search starts in the leaves, and every opening frame
# is 1.0 m off in y because the plan centre moved. They are excluded from the
# BBOX AND THE GRID MEASUREMENT ONLY; the slicer still cuts them with the rest
# of the mesh, so the trees stay in the scene (they are just no longer part of
# the building's measured box).
GREENERY_TOKENS = ("grass", "tree", "leaves", "leaf", "trunk", "bark",
                   "branch", "platanus", "robinia", "tilia", "shrub",
                   "hedge", "foliage", "ivy")
#: below this the trim is noise and the original bbox is kept verbatim
BBOX_TRIM_MIN_M = 1.0

#: WHICH PACK AN ASSET COMES FROM, AND WHAT THAT IMPLIES.
#: `kind` is the same token `fire_bake`'s manifest uses (`gac:NAME:LEVEL` /
#: `dtc:NAME:LEVEL`), so a bake entry, a probe argument and this table all
#: name the pack the same way. Everything pack-specific in `prepare` reads
#: from here and nothing else, which is what keeps the `gac` path
#: byte-identical: its row is the constants `prepare` used to inline.
PACKS = {
    "gac": {                     # GreatAmericanCity — one merged .usd per
                                 # asset, materials in Materials/*_Inst.usd
        "dir": GAC_DIR, "ext": ".usd", "scale": GAC_SCALE,
        "style_prefix": "gac_",
        "bbox_exclude": (),      # nothing baked in that is not the building
        "construction_table": False,   # height rule (see `prepare`)
        "force_regular_grid": set(),   # no asset in this pack needs it
        "glazing_material_deny": set(),  # every GAC material prim is named
                                         # "UnrealMaterial" -- nothing to deny
        "soot": "bake",           # bake into the merged atlases pre-slice
                                  # (`bake_atlases`) -- every GAC material
                                  # resolves a real UsdPreviewSurface
                                  # diffuseColor map, so this has always
                                  # worked. See `PACKS["aec"]`'s own note.
    },
    "dtc": {                     # downtowncity — one merged .usdc per asset,
                                 # materials INLINE in the same file
        "dir": DTC_DIR, "ext": ".usdc", "scale": DTC_SCALE,
        "style_prefix": "dtc_",
        "bbox_exclude": GREENERY_TOKENS,
        "construction_table": True,    # quake_sliced.CONSTRUCTION + height,
        # USER BLACKLIST (2026-08-30): the Carved_* blocks read as the same
        # building ("B1, B3-B5 kinda all look the same ... blacklist it from
        # the pack") — never pick them for a fire row or a city pool.
        # Building_12 added 2026-08-31 (live-city review: "house_49_275/
        # Building_12 let's blacklist this building").
        "blacklist": ("Carved_", "Building_11", "Building_12"),
        # "cut up into triangles" (fire_dtc2 review, 2026-08-30): Building_12
        # rejects the measured grid anyway (confidence 0.46 < MIN_CONFIDENCE
        # 0.55) and falls back to `regular_grid` -- listed here too so that
        # stays true no matter how the confidence scoring changes later, per
        # the user's own ask ("You can just split it by fixed grid").
        "force_regular_grid": {"Building_12"},
        # `ICity_Window_AC_00N` -- an air-conditioner unit, matched by
        # `is_glazing` only because "window" is inside its name (see
        # `WINDOW_ASPECT_RANGE` above). MEASURED on every downtowncity block
        # probed, not just Building_12 (`tools/dtc_island_probe.py`:
        # Building_11 carries the same contamination). Denied by name, on
        # top of the shape filter, because its own facets can locally look
        # window-shaped even though the material is never a window.
        "glazing_material_deny": {"window_ac"},
        "soot": "bake",
    },
    "aec": {                      # AEC "small brownstone" rowhouses —
                                  # `Reference_Brownstone{2,5,6,8,10,11,12}
                                  # Row.usd`, already a bag of ~300-1,500
                                  # meshes each (internally INSTANCED, but
                                  # `gac_storey_slice.read_mesh` traverses
                                  # instance proxies read-only and writes
                                  # brand-new pieces, so nothing needs
                                  # de-instancing first — see the module's
                                  # own MEASURED note by `read_mesh`)
        "dir": AEC_DIR, "ext": ".usd", "scale": AEC_SCALE,
        "style_prefix": "aec_",
        "bbox_exclude": (),       # no baked-in landscaping in the *Row.usd
                                  # files themselves (measured: `/Environment`
                                  # is an empty sibling Xform, the row's own
                                  # geometry lives entirely under
                                  # `/World/Reference_Brownstone{N}Row`)
        # ALWAYS masonry, always short. `Reference_Brownstone*Row` measures
        # 14.6-15.3 m tall regardless of row length (`config/asset_sets/
        # urban.yaml`'s own per-entry comments) -- nowhere near the 25 m
        # height-rule cutoff to `rc`, so the height rule alone is correct
        # and there is no per-name construction table to maintain here
        # (unlike `dtc`, whose Amar_Tower needs a NAMED rc_glass override).
        "construction_table": False,
        # NO GLAZING SUBSET ANYWHERE ON THIS ASSET (MEASURED,
        # `tools/openings_probe.py`: "NO glass subset -- windows are painted
        # into the texture"). `gsl.window_centres` therefore returns nothing
        # to measure a grid from, and `gac_storey_slice.grid_for` already
        # falls back to `regular_grid` whenever its `wins` argument is empty
        # -- forcing every name here would be redundant with that fallback,
        # not a stronger guarantee, so this stays empty and the fallback is
        # left to do its job (verified per-asset by the offline slice probe,
        # not assumed).
        "force_regular_grid": set(),
        "glazing_material_deny": set(),
        # MDL SOOT OVERLAY, NOT A BAKE. Every AEC facade material is a
        # compiled NVIDIA vMaterials/Base `.mdl` module reference
        # (`Brick_Wall_Red.mdl`, `Facade_Brick_*.mdl`, ...) with `info:id`
        # UNSET and, at most, a handful of SCALAR parameter overrides
        # (`diffuse_brightness`, `grime_weight`, `leak_color`, ...) — never
        # an `Sdf.AssetPath`-valued texture input. MEASURED directly against
        # the raw source stage (`tools/aec_material_probe.py`, every one of
        # the first 7 materials on `Reference_Brownstone5Row`): the base-
        # colour texture is baked INSIDE the compiled `.mdl` module and is
        # simply not visible to USD attribute introspection —
        # `soot_plume.find_basecolor`/`_diffuse_of` both correctly return
        # `(None, None, None)` on every one. `bake_atlases` (the `gac`/`dtc`
        # route) therefore cannot compose a sooted copy at all: it logs
        # every AEC material as "without a diffuse map" and moves on
        # (harmless), but the KIT-STYLE per-piece fallback that then runs on
        # every un-prebaked piece (`urban_fire._bind_soot`) hits the exact
        # same wall and falls back to `_flat_diffuse`'s flat (0.6, 0.6, 0.6)
        # grey — soot baked over a pale, textureless swatch instead of real
        # brick ("looks white with weird ash instead of brick with overlay",
        # user). `"overlay"` routes these materials to `gac_fire.
        # overlay_soot` instead: the brick MDL material is left bound and
        # UNTOUCHED (real brick renders exactly as shipped) and the soot
        # rides as a translucent decal quad standing proud of each
        # elevation, textured directly from `soot_plume.skin`'s own
        # unwrapped canvas — `disaster.wall_overlay`'s architecture (built
        # for this exact "can't rewrite the base material" problem before
        # GAC's per-piece UV bake superseded it there) fits AEC's MDL walls
        # precisely. See `overlay_soot`'s own docstring for the mechanism
        # and `PACKS["gac"]`'s note for why that pack stays on the old path.
        "soot": "overlay",
    },
}
DEFAULT_KIND = "gac"

#: measured `metersPerUnit` per asset URL — see `asset_scale`
_MPU_CACHE = {}


def split_kind(name, kind=None):
    """`("dtc", "Amar_Tower")` from `"dtc:Amar_Tower"`, `("gac", "SM_Building_02")`
    from a bare name. An explicit `kind=` wins over a bare name and must agree
    with a prefixed one."""
    text = str(name or "")
    k, sep, bare = text.partition(":")
    if sep and k.strip().lower() in PACKS:
        k = k.strip().lower()
        if kind and str(kind).lower() != k:
            raise ValueError("{0!r} says kind {1!r} but kind={2!r} was passed"
                             .format(text, k, kind))
        return k, bare.strip()
    k = str(kind or DEFAULT_KIND).lower()
    if k not in PACKS:
        raise ValueError("unknown asset kind {0!r} (expected one of {1})"
                         .format(kind, "/".join(sorted(PACKS))))
    return k, text.strip()


def asset_url(name, kind=None):
    """The OPENABLE URL of merged asset `name` in pack `kind`.

    `gac`/`dtc` are already-absolute `omniverse://` URLs and this is a no-op
    for them (`scene_generator._join_asset_root` passes any string containing
    `"://"` through unchanged once its own local-root scheme check misses).
    `aec`'s `PACKS["aec"]["dir"]` is `airstack://...` — a repo-relative
    pseudo-scheme every OTHER caller in this codebase resolves before handing
    a URL to `place_source`/`AddReference` (`fire_pack_rows_launch_script.
    place_asset`, `urban_tornado_bench_launch_script`'s `shed_url`) — so this
    resolves it here too, the one place every `PACKS` consumer routes
    through, rather than leaving each new caller to remember. Lazy import:
    `scene_generator` lives one directory up from `disaster/`, and importing
    it at module scope would need the same `sys.path` shim every other
    `disaster.*` module already does lazily for a sibling import.
    """
    k, bare = split_kind(name, kind)
    p = PACKS[k]
    url = p["dir"] + bare + p["ext"]
    scheme = url.split("://", 1)[0] if "://" in url else ""
    if scheme and scheme != "omniverse":
        import sys
        _scene_gen = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if _scene_gen not in sys.path:
            sys.path.insert(0, _scene_gen)
        import scene_generator as _sg
        if scheme in _sg.LOCAL_ASSET_ROOTS:
            url = _sg._join_asset_root(url, "")
    return url


def asset_scale(url, default, verbose=True):
    """The scale factor that puts `url` on a metres stage — its own
    `metersPerUnit`, MEASURED, not the asset set's `scale:` key.

    "UNITS ARE PER PACK AND MUST BE MEASURED, NOT ASSUMED"
    (`config/asset_sets/urban_gac.yaml`): assuming GAC's 0.01 for
    downtowncity shrank every block to ~0.4 m and the packer fitted 5,628 of
    them into 26 blocks. A wrong scale does not error, it changes the layout.

    `Usd.Stage.Open(url, LoadNone)` is the cheap read — payloads stay
    unloaded, so this costs 0.00-0.05 s even on the 500k-triangle tower
    (measured) — and `Sdf.Layer.GetField` is NOT available in this pxr
    build. Falls back to `default` (the pack's declared scale) if the open
    fails, so an unreachable Nucleus degrades to today's behaviour rather
    than to a scale of 1.
    """
    from pxr import Usd, UsdGeom

    if url in _MPU_CACHE:
        return _MPU_CACHE[url]
    val = float(default)
    try:
        st = Usd.Stage.Open(url, Usd.Stage.LoadNone)
        mpu = UsdGeom.GetStageMetersPerUnit(st) if st is not None else None
        if mpu and float(mpu) > 0.0:
            val = float(mpu)
    except Exception as exc:                                # pragma: no cover
        if verbose:
            print("[gac_fire] could not read metersPerUnit of {0} ({1}); "
                  "using the pack default {2}".format(url, exc, default))
    if verbose and abs(val - float(default)) > 1e-9:
        print("[gac_fire] {0}: measured metersPerUnit {1:g}, pack default "
              "{2:g} — using the MEASURED value".format(
                  url.rsplit("/", 1)[-1], val, default))
    _MPU_CACHE[url] = val
    return val


def _material_name_of(prim):
    from pxr import UsdShade

    m = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    return m.GetPrim().GetName() if (m and m.GetPrim().IsValid()) else ""


def _is_prop_material(name, tokens):
    low = str(name or "").lower()
    return any(t in low for t in (tokens or ()))


def trim_bbox(stage, src, bbox, tokens, min_delta=BBOX_TRIM_MIN_M, verbose=True):
    """`(bbox, note)` with the faces of `tokens`-named materials excluded.

    Returns the ORIGINAL `bbox` object and `None` unless the trim moves some
    dimension by more than `min_delta` — so a pack with no such materials
    (`gac`, whose `bbox_exclude` is empty) short-circuits before it reads a
    single point and every existing caller is bit-identical.
    """
    from pxr import Usd, UsdGeom

    if not tokens or bbox is None:
        return bbox, None
    root = stage.GetPrimAtPath(src)
    if not root or not root.IsValid():
        return bbox, None
    xc = UsdGeom.XformCache()
    root_inv = xc.GetLocalToWorldTransform(root).GetInverse()
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    hits = []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        if pts is None or not len(pts):
            continue
        M = np.asarray(xc.GetLocalToWorldTransform(prim) * root_inv, dtype=float)
        P = np.asarray(pts, dtype=float)
        P = (np.c_[P, np.ones(len(P))] @ M)[:, :3]
        counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [],
                            dtype=np.int64)
        fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [],
                         dtype=np.int64)
        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
        if not len(counts) or len(fvi) != int(counts.sum()) or not subs:
            if not _is_prop_material(_material_name_of(prim), tokens):
                lo = np.minimum(lo, P.min(0))
                hi = np.maximum(hi, P.max(0))
            continue
        start = np.concatenate([[0], np.cumsum(counts)[:-1]])
        for sub in subs:
            nm = _material_name_of(sub.GetPrim())
            idx = np.asarray(sub.GetIndicesAttr().Get() or [], dtype=np.int64)
            idx = idx[(idx >= 0) & (idx < len(counts))]
            if not len(idx):
                continue
            if _is_prop_material(nm, tokens):
                hits.append((nm, int(len(idx))))
                continue
            vids = np.concatenate([fvi[start[f]:start[f] + counts[f]]
                                   for f in idx])
            V = P[vids]
            lo = np.minimum(lo, V.min(0))
            hi = np.maximum(hi, V.max(0))
    if not hits or not np.all(np.isfinite(lo)):
        return bbox, None
    (x0, y0, z0), (x1, y1, z1) = bbox
    d = (abs((x1 - x0) - (hi[0] - lo[0])), abs((y1 - y0) - (hi[1] - lo[1])),
         abs((z1 - z0) - (hi[2] - lo[2])))
    if max(d) <= float(min_delta):
        return bbox, None
    new = ((float(lo[0]), float(lo[1]), float(lo[2])),
           (float(hi[0]), float(hi[1]), float(hi[2])))
    note = ("bbox trimmed of {0} baked-in prop subset(s) ({1}): "
            "{2:.1f} x {3:.1f} x {4:.1f} m -> {5:.1f} x {6:.1f} x {7:.1f} m"
            .format(len(hits), ", ".join(sorted(set(n for n, _c in hits)))[:90],
                    x1 - x0, y1 - y0, z1 - z0,
                    hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]))
    if verbose:
        print("[gac_fire] " + note)
    return new, note


def mesh_without_props(mesh, tokens):
    """`mesh` with the triangles of `tokens`-named materials dropped.

    Only `mass_from_grid`'s deck search is fed this — it walks upward faces
    in the top 15 % of the height, and on Amar_Tower that band is FULL of
    tree canopy. The bake still runs on the untrimmed mesh (the trees keep
    their own atlases), and the slicer cuts the untrimmed asset.
    """
    if mesh is None or not tokens:
        return mesh
    mats = mesh.get("mats") or []
    bad = set(i for i, m in enumerate(mats)
              if m is not None and m.GetPrim().IsValid()
              and _is_prop_material(m.GetPrim().GetName(), tokens))
    if not bad:
        return mesh
    keep = ~np.isin(mesh["MID"], np.fromiter(bad, dtype=np.int64, count=len(bad)))
    if not keep.any():
        return mesh
    out = dict(mesh)
    out["tris"] = mesh["tris"][keep]
    return out


# ---------------------------------------------------------------------------
# Placing the merged asset
# ---------------------------------------------------------------------------
def place_source(stage, cell, usd, scale=GAC_SCALE):
    """Reference the merged asset under `cell/src`, centred in plan on the
    cell with its base at the cell's z. Returns the holder path or None.
    (The same seat `gac_kit_launch_script.place_source` uses.)

    THE CENTRING IS DONE IN THE CELL'S OWN FRAME (2026-09-02).  It used to
    subtract a WORLD-space bbox centre from the cell's world translation and
    write that delta as the asset's LOCAL translate.  Under a cell with no
    rotation (every bake launcher, every probe, the bench's yaw-0 holders)
    the two frames coincide and it was right.  Under a YAWED holder — the
    tornado city launcher yaws every holder by the placement's yaw, and the
    bench's A-row is yawed 180 — the delta was applied along the rotated
    axes: measured with usd-core on a corner-pivot GAC-sized box (60 x 142
    m, pivot at the corner like SM_Building_30's, `_plans/gac_buildings.
    json` cx -28 / cy +70), the source landed 109 m from its holder at yaw
    90, 154 m at yaw 180, 109 m at yaw 270, dead on at yaw 0.  The intact
    cell was already hidden, so the city showed an empty lot AND a building
    on a road — the "frame changes when we reassemble it" the user named.
    `ComputeRelativeBound(holder, cell)` measures the asset in the cell's
    frame, so the correction is exact for any yaw (and identical to before
    for yaw 0).  Test: `tests/test_place_source_frame.py`."""
    from pxr import Gf, Sdf, Usd, UsdGeom

    holder = cell + "/src"
    UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    kid = stage.DefinePrim(Sdf.Path(holder + "/asset"))
    kid.GetReferences().AddReference(usd)
    stage.Load(Sdf.Path(holder))
    xf = UsdGeom.Xformable(kid)
    xf.ClearXformOpOrder()
    tr = xf.AddTranslateOp()
    if abs(scale - 1.0) > 1e-9:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    cell_prim = stage.GetPrimAtPath(cell)
    r = cache.ComputeRelativeBound(stage.GetPrimAtPath(holder),
                                   cell_prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    # `holder` is the cell's direct child with no ops of its own, so the
    # cell-relative bound is expressed in exactly the frame `tr` acts in.
    tr.Set(Gf.Vec3d(-0.5 * (mn[0] + mx[0]), -0.5 * (mn[1] + mx[1]), -mn[2]))
    return holder


# ---------------------------------------------------------------------------
# Windows as islands
# ---------------------------------------------------------------------------
def window_rects(stage, src, glass_tex=None, planes=None, deny_mat=None):
    """`{side: [(u0, u1, z0, z1), ...]}` of the glass ISLANDS on each
    elevation, in the asset holder's frame (`u` = x on S/N, y on E/W — the
    same `u` `gac_slice.window_centres` reports). Islands are glass faces
    whose bboxes share a 0.3 m grid cell, joined by union-find, KEPT ONLY IF
    THE MERGED RESULT LOOKS LIKE A WINDOW (`_is_window_shaped`, `_islands`)
    — see the `WINDOW_*` constants above.

    `planes`, if given a dict, is filled in place with `{side: plane_coord}`
    — the real façade plane per elevation (x for E/W, y for S/N), measured as
    the MEDIAN outward coordinate of the glass faces on that side plus 0.15 m
    outward (the glass sits recessed into the wall opening by roughly that
    much). The asset's overall bbox face is the wrong plane to hang a flame
    or a frame origin off: it is the outer extent of whatever sticks out
    furthest (cornices, canopies, signage), which can be 1-3 m proud of the
    real wall the windows sit in.

    `deny_mat`, if given (`PACKS[kind]["glazing_material_deny"]`), is a set
    of lower-cased substrings; a candidate face whose bound MATERIAL name
    contains one is never treated as glazing at all, however it scores on
    `gac_slice.is_glazing` — belt-and-suspenders on top of the shape filter
    for a known offender (downtowncity's `ICity_Window_AC_00N` air
    conditioners, matched by `is_glazing` only because "window" is in the
    name)."""
    from detail import gac_slice as gsl
    from pxr import Usd, UsdGeom, UsdShade

    glass_tex = glass_tex or gsl.GLASS_TEX
    deny_mat = tuple(str(d).lower() for d in (deny_mat or ()))
    root = stage.GetPrimAtPath(src)
    if not root or not root.IsValid():
        return {}
    xc = UsdGeom.XformCache()
    root_inv = xc.GetLocalToWorldTransform(root).GetInverse()
    # outward-normal sign per side, matching `quake_flow._b_face_pt`'s
    # convention (E/N outward is +x/+y, S/W outward is -y/-x): the plane
    # measured from the glass is pushed 0.15 m further OUT, away from the
    # building centre, to land on the wall face rather than the pane.
    out_sign = {"E": 1.0, "N": 1.0, "S": -1.0, "W": -1.0}
    glass_recess_m = 0.15

    def _tex(p):
        """(diffuse basename, MATERIAL PRIM NAME) — both, because a
        downtowncity window material carries no diffuse map and its name is
        the only evidence there is (`gac_slice.is_glazing`). A no-op on
        GreatAmericanCity, where every material prim is `UnrealMaterial`."""
        mat = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            return "", ""
        sp, inp, url = _diffuse_of(mat.GetPrim())
        return (url or "").rsplit("/", 1)[-1], mat.GetPrim().GetName()

    # A PANE IS ON THE ELEVATION IT STANDS IN, NOT THE ONE ITS NORMAL POINTS
    # TO. Glass is double-sided in these assets: every pane has a back face
    # with the opposite normal, and classifying by normal (`gsl._side_of`)
    # put that back face on the OPPOSITE elevation as a phantom window — a
    # mirror image of the real E windows filed under W (SM_Building_02's
    # "W" plane measured 11.27 m, its real E plane 11.63 m; insets of up to
    # 57 m on the phantom sides, `tools/gac_plane_probe.py`, 2026-08-30).
    # `prepare` then picked E/W as the burning sides by island count and
    # planned events, soot and flames on a blank W wall. Two passes: gather
    # every vertical glass face with the asset's extent, file each by the
    # bbox face its centroid is NEAREST, take the median plane per side,
    # and keep only the faces within `plane_tol_m` of that plane (interior
    # partitions and atrium glass are not façade).
    plane_tol_m = 1.5
    PLANE_MAX_INSET_M = 3.0
    faces = []        # (V, side_by_normal_ok)
    areas = []        # parallel to `faces` -- polygon area, for the fill test
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        if pts is None or not len(pts):
            continue
        M = np.asarray(xc.GetLocalToWorldTransform(prim) * root_inv, dtype=float)
        P = np.asarray(pts, dtype=float)
        P = (np.c_[P, np.ones(len(P))] @ M)[:, :3]
        lo = np.minimum(lo, P.min(0))
        hi = np.maximum(hi, P.max(0))
        counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
        fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
        if not len(counts) or len(fvi) != int(counts.sum()):
            continue
        start = np.concatenate([[0], np.cumsum(counts)[:-1]])
        for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            _t, _mn = _tex(sub.GetPrim())
            if not gsl.is_glazing(_t, glass_tex, mat_name=_mn):
                continue
            if deny_mat and any(d in (_mn or "").lower() for d in deny_mat):
                continue
            for f in (sub.GetIndicesAttr().Get() or []):
                f = int(f)
                if f >= len(counts):
                    continue
                V = P[fvi[start[f]:start[f] + counts[f]]]
                n = np.cross(V[1] - V[0], V[2] - V[0])
                ln = float(np.linalg.norm(n))
                if ln < 1e-12:
                    continue
                n = n / ln
                if abs(n[2]) >= max(abs(n[0]), abs(n[1])):
                    continue          # roof light / floor glass
                faces.append(V)
                areas.append(_poly_area(V))
    if not faces or not np.all(np.isfinite(lo)):
        return {}
    cen = np.array([V.mean(0) for V in faces])
    # distance of each face centroid to the four bbox faces: S, E, N, W
    dist = np.stack([cen[:, 1] - lo[1], hi[0] - cen[:, 0],
                     hi[1] - cen[:, 1], cen[:, 0] - lo[0]], axis=1)
    side_ix = np.argmin(dist, axis=1)
    ring = ("S", "E", "N", "W")
    boxes = {}   # side -> list of (u0, u1, z0, z1, area)
    plane_vals = {}
    for k, side in enumerate(ring):
        sel = np.nonzero(side_ix == k)[0]
        if not len(sel):
            continue
        axis = 0 if side in ("E", "W") else 1
        oc = cen[sel, axis]
        plane = float(np.median(oc))
        keep = sel[np.abs(oc - plane) <= plane_tol_m]
        if not len(keep):
            continue
        plane_vals[side] = [float(v) for v in cen[keep, axis]]
        for i in keep:
            V = faces[i]
            uu = V[:, 1] if side in ("E", "W") else V[:, 0]
            boxes.setdefault(side, []).append(
                (float(uu.min()), float(uu.max()),
                 float(V[:, 2].min()), float(V[:, 2].max()), areas[i]))
    out = {}
    for side, bl in boxes.items():
        out[side] = _islands(bl)
    if planes is not None:
        face = {"S": lo[1], "E": hi[0], "N": hi[1], "W": lo[0]}
        for side, vals in plane_vals.items():
            plane = float(np.median(vals))
            # A FAÇADE IS NEAR ITS BBOX FACE. Interior or atrium glass can win
            # the median for a side with few real windows: dtc Carved_13's
            # "N" plane measured 12 m inside its N face and every stamp on
            # that side snapped into the building (2026-08-30). More than
            # `PLANE_MAX_INSET_M` from the bbox face is not a façade — that
            # side keeps the bbox face (no entry) and its islands are dropped.
            if abs(plane - face[side]) > PLANE_MAX_INSET_M:
                out.pop(side, None)
                continue
            planes[side] = plane + out_sign[side] * glass_recess_m
    return out


def _poly_area(V):
    """Area of a planar polygon (fan triangulation from `V[0]`) -- exact for
    a triangle, a fair approximation for the quads/ngons a glazing subset
    may carry. Used only by the window-shape fill test below."""
    if len(V) < 3:
        return 0.0
    c = np.zeros(3)
    for i in range(1, len(V) - 1):
        c += np.cross(V[i] - V[0], V[i + 1] - V[0])
    return 0.5 * float(np.linalg.norm(c))


def _is_window_shaped(w, h, area):
    """Does a merged island of width `w`, height `h` and glazing face `area`
    (m2) look like an actual window opening? See `WINDOW_*` above.

    The height/width bounds carry `WINDOW_DIM_EPS_M` of slack for the same
    reason a merged island's dimensions are never bit-exact to the authored
    size — see that constant's own comment."""
    if w <= 0 or h <= 0:
        return False
    aspect = w / h
    fill = area / (w * h) if w * h > 0 else 0.0
    eps = WINDOW_DIM_EPS_M
    return (WINDOW_ASPECT_RANGE[0] <= aspect <= WINDOW_ASPECT_RANGE[1]
            and WINDOW_HEIGHT_RANGE_M[0] - eps <= h <= WINDOW_HEIGHT_RANGE_M[1] + eps
            and WINDOW_WIDTH_RANGE_M[0] - eps <= w <= WINDOW_WIDTH_RANGE_M[1] + eps
            and fill >= WINDOW_FILL_MIN)


def _islands(boxes, cell=ISLAND_CELL_M):
    """Union-find over boxes that share a grid cell -> merged bboxes kept
    only if they pass `_is_window_shaped`.

    `boxes` items are `(u0, u1, z0, z1, area)` — `area` is each face's own
    polygon area, summed per merged island purely for the fill test; the
    returned rects are the old plain `(u0, u1, z0, z1)` 4-tuples, unchanged
    shape for every caller."""
    n = len(boxes)
    if n == 0:
        return []
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    owner = {}
    for i, (u0, u1, z0, z1, _a) in enumerate(boxes):
        c0, c1 = int(math.floor(u0 / cell)), int(math.floor(u1 / cell))
        r0, r1 = int(math.floor(z0 / cell)), int(math.floor(z1 / cell))
        for c in range(c0, c1 + 1):
            for r in range(r0, r1 + 1):
                k = (c, r)
                j = owner.get(k)
                if j is None:
                    owner[k] = i
                else:
                    union(i, j)
    agg = {}
    for i, (u0, u1, z0, z1, area) in enumerate(boxes):
        r = find(i)
        a = agg.get(r)
        if a is None:
            agg[r] = [u0, u1, z0, z1, area]
        else:
            a[0], a[1] = min(a[0], u0), max(a[1], u1)
            a[2], a[3] = min(a[2], z0), max(a[3], z1)
            a[4] += area
    rects = []
    for u0, u1, z0, z1, area in agg.values():
        w, h = u1 - u0, z1 - z0
        if w < 0.25 or h < 0.25:
            continue
        if _is_window_shaped(w, h, area):
            rects.append((u0, u1, z0, z1))
            continue
        # the oversize-island split — see the STRIP_* constants' comment.
        # `fill` can exceed 1.0 because double-sided glass contributes both
        # faces to `area`; clamp before handing a per-cell area estimate to
        # the shape test.
        fill = area / max(w * h, 1e-6)
        tall = h > WINDOW_HEIGHT_RANGE_M[1] + 0.5
        wide = w > WINDOW_WIDTH_RANGE_M[1] + 0.5
        if not (tall or wide) or fill < STRIP_FILL_MIN:
            continue
        rows = max(1, int(round(h / STRIP_ROW_M))) if tall else 1
        cols = max(1, int(round(w / STRIP_COL_M))) if wide else 1
        ch, cw = h / rows, w / cols
        mh = min(0.30, 0.12 * ch)
        mw = min(0.30, 0.10 * cw)
        cf = min(1.0, fill)
        for r in range(rows):
            for c in range(cols):
                a0, a1 = u0 + c * cw + mw, u0 + (c + 1) * cw - mw
                b0, b1 = z0 + r * ch + mh, z0 + (r + 1) * ch - mh
                if _is_window_shaped(a1 - a0, b1 - b0,
                                     (a1 - a0) * (b1 - b0) * cf):
                    rects.append((a0, a1, b0, b1))
    rects.sort(key=lambda r: (r[2], r[0]))
    return rects


# ---------------------------------------------------------------------------
# The mass box and the openings provider
# ---------------------------------------------------------------------------
def mass_from_grid(g, bbox, mesh=None):
    """A `quake_flow`-shaped mass dict for the merged asset, in the CELL
    frame (the asset is centred on the cell by `place_source`).

    `top` is the bbox z1 -- the highest point ANYTHING on the roof reaches,
    which is the parapet coping, not the deck. `deck_z` is the real roof
    surface everything should sit ON (row-2 review, 2026-08-30: "floating
    roof props and also floating debris"); every roof consumer downstream
    reads `m.get("deck_z", m["top"])` so the kit path (whose masses carry no
    `deck_z`) is untouched.

    MEASUREMENT, when `mesh` (`gac_storey_slice.read_mesh`'s de-indexed dict:
    `P` per face-corner, `tris` = `arange(len(P)).reshape(-1, 3)`) is given:
    among its upward-facing triangles (normal z > 0.8) whose centroid falls
    in the top 15% of the building height, bin z by triangle AREA in 0.2 m
    steps.

    THE GLOBAL BIGGEST BIN IS THE WRONG BIN. A GAC mesh's "fake interior"
    carries a visible floor plate at EVERY storey, so the top-15% window on a
    tall building holds several near-identical big-area bins spaced one
    storey apart, not just one (measured, `tools/gac_deck_probe.py`
    diagnostics, 2026-08-30: SM_Building_12 repeats a ~1724 m2 slab bin every
    ~4.6 m for four storeys inside the window). Taking the single largest bin
    in the whole window is a coin flip across those repeats -- it landed on
    one two storeys down and reported a 14.9 m parapet, obvious nonsense for
    a coping. What is reliable: walking DOWN from the bbox top, the first bin
    that is actually roof-sized -- area >= 20% of the plan footprint (`W *
    D`) -- IS the deck, because a thin coping/parapet ring (perimeter x wall
    thickness) never gets close to that fraction of the full plan. Bins
    thinner than that, from the top down to the deck bin, are the parapet.
    Falls back to the single largest bin in the window if nothing clears the
    threshold (an irregular roof with no flat majority), and to `top` itself
    when `mesh` is None or nothing in it qualifies at all; `deck_note`
    records which case fired, for `tools/gac_deck_probe.py`.
    """
    (x0, y0, z0), (x1, y1, z1) = bbox
    floors = list(g.get("storeys") or [])
    levels = sorted(z for z in floors if z0 - 1e-6 <= z < z1 - 0.5)
    if not levels or levels[0] > z0 + 0.5:
        levels = [z0] + levels
    deck_z, deck_note = float(z1), "no mesh given, falling back to top"
    P = mesh.get("P") if mesh is not None else None
    tris = mesh.get("tris") if mesh is not None else None
    if P is not None and tris is not None and len(tris):
        V0, V1, V2 = P[tris[:, 0]], P[tris[:, 1]], P[tris[:, 2]]
        cross = np.cross(V1 - V0, V2 - V0)
        ln = np.linalg.norm(cross, axis=1)
        ok = ln > 1e-9
        nz = np.zeros(len(tris))
        nz[ok] = cross[ok, 2] / ln[ok]
        area = 0.5 * ln
        zc = (V0[:, 2] + V1[:, 2] + V2[:, 2]) / 3.0
        H = float(z1 - z0)
        band = float(z1) - 0.15 * H
        up = ok & (nz > 0.8) & (zc >= band)
        if up.any():
            zc_u, area_u = zc[up], area[up]
            bw = 0.2
            b0 = float(zc_u.min())
            idx = np.floor((zc_u - b0) / bw).astype(int)
            nbin = int(idx.max()) + 1
            bin_area = np.zeros(nbin)
            bin_zw = np.zeros(nbin)
            np.add.at(bin_area, idx, area_u)
            np.add.at(bin_zw, idx, area_u * zc_u)
            nz_bins = np.nonzero(bin_area > 1e-6)[0]
            if len(nz_bins):
                thresh = 0.2 * float(x1 - x0) * float(y1 - y0)
                order = nz_bins[::-1]          # highest z first
                deck_bin = None
                for b in order:
                    if bin_area[b] >= thresh:
                        deck_bin = int(b)
                        break
                top_bin = int(order[0])
                if deck_bin is None:
                    # NOTHING IN THE WINDOW READS AS A FULL PLATEAU -- an
                    # irregular roof (a lantern, a sawtooth). Fall back to
                    # the old rule (biggest bin in the window) rather than
                    # leave `deck_z` at the bbox top.
                    deck_bin = int(nz_bins[np.argmax(bin_area[nz_bins])])
                    deck_z = float(bin_zw[deck_bin] / bin_area[deck_bin])
                    deck_note = ("no bin reached the {0:.0f} m2 plateau "
                                "threshold (20% of the {1:.0f} x {2:.0f} m "
                                "footprint); used the largest available bin "
                                "instead".format(thresh, x1 - x0, y1 - y0))
                elif deck_bin == top_bin:
                    deck_z = float(bin_zw[deck_bin] / bin_area[deck_bin])
                    deck_note = "no separate parapet -- top bin is already a full plateau"
                else:
                    deck_z = float(bin_zw[deck_bin] / bin_area[deck_bin])
                    par_z = float(bin_zw[top_bin] / bin_area[top_bin])
                    deck_note = ("parapet coping excluded (its bin at "
                                "z={0:.2f}, {1:.1f} m2 < {2:.0f} m2 "
                                "threshold)".format(par_z, bin_area[top_bin],
                                                    thresh))
            else:
                deck_note = "no area in top-15% band after binning (shouldn't happen)"
        else:
            deck_note = "no upward face (normal z > 0.8) in top 15% of height"
    elif mesh is not None:
        deck_note = "mesh had no usable P/tris"
    return {"tag": "main", "cx": 0.5 * (x0 + x1), "cy": 0.5 * (y0 + y1),
            "yaw": 0.0, "W": float(x1 - x0), "D": float(y1 - y0),
            "z0": float(z0), "top": float(z1), "deck_z": deck_z,
            "deck_note": deck_note, "levels": levels,
            "module": float(((g.get("bays") or {}).get("E") or {}).get("pitch")
                            or 4.0),
            "spec": {"bands": []}}


def side_frame(m, side, planes=None):
    """A `quake_flow._piece_frame`-shaped wall frame spanning a whole
    elevation, so `_b_face_pt(fr, u, v, out)` lands on that façade plane.
    `u` runs the way `soot_plume.side_u` counts it.

    `planes`, if given, is the `{side: plane_coord}` dict `window_rects`
    fills — the measured façade plane, in place of the mass BBOX face (which
    is the outer extent of whatever protrudes furthest and can be 1-3 m
    proud of the real wall). Only the frame origin's OUTWARD coordinate
    (x for E/W, y for S/N) moves; the `u`-origin coordinate and the yaw are
    unchanged, so `soot_plume.side_u`'s convention still holds."""
    W, D, cx, cy = m["W"], m["D"], m["cx"], m["cy"]
    H = float(m["top"] - m["z0"])
    pl = planes or {}
    if side == "S":
        return (cx - W / 2.0, pl.get("S", cy - D / 2.0), 0.0, W, H, -0.02, False)
    if side == "E":
        return (pl.get("E", cx + W / 2.0), cy - D / 2.0, math.pi / 2.0, D, H, -0.02, False)
    if side == "N":
        return (cx + W / 2.0, pl.get("N", cy + D / 2.0), math.pi, W, H, -0.02, False)
    return (pl.get("W", cx - W / 2.0), cy + D / 2.0, 1.5 * math.pi, D, H, -0.02, False)


def _real_storeys_by_side(rects, m):
    """`{side: sorted [storey indices]}` that carry at least one REAL window
    island -- the same z -> storey classification `openings_provider`'s own
    main loop uses to bucket a rect, factored out here so a caller can ask
    "does this side have ANY real coverage in this storey range" (or "at
    this one storey") WITHOUT building the whole `by` table first.

    Used by `prepare`'s sides-reconciliation step (below) to (a) tell a
    side with real glazing somewhere on the building from one with none at
    all, and (b) find the nearest real coverage to slide the fire's origin
    onto when the requested band misses it (fire_dtc5 review, 2026-08-31 --
    see the block comment above `_reconcile_sides`)."""
    levels = list(m["levels"])
    out = {}
    for side, rl in rects.items():
        sts = set()
        for (u0, u1, z0, z1) in rl:
            zc = 0.5 * (z0 + z1)
            st = 0
            for i, lv in enumerate(levels):
                if zc >= lv - 0.05:
                    st = i
            sts.add(st)
        if sts:
            out[side] = sorted(sts)
    return out


def _reconcile_sides(rects, sides, name=""):
    """A compartment fire vents through openings that EXIST -- a blank wall
    does not vent, however contagion got there. (bench review, 2026-08-31,
    quoting the user on `gac_SM_Building_26_F5_o3_NW_s684`: "also has smoke
    coming out of its sides that are blank rather than the windows. The
    windows are on the long side.") `sides`, as handed to `prepare`, is
    either explicit (the city bake: `urban_fire_spread.entry_for_plan_fire`
    picks it by which NEIGHBOUR lit the building -- contagion geometry,
    completely blind to which wall has glass) or this function's own
    caller's island-count ranking (the `sides is None` bench path, which
    already prefers real glazing but can still include a blank side to top
    up the count). Either way `sides` can name a wall with zero real
    islands on a building that has real glazing SOMEWHERE ELSE, and the
    synthetic-openings fallback used to fill that blank wall in rather than
    questioning the request -- flames and soot on a party wall while the
    real windowed elevation stayed dark.

    Returns `(venting_sides, note)`. `note` is a log line when the returned
    sides differ from the request, `None` when they don't (an ALREADY-real
    request, e.g. every healthy multi-sided building, is returned
    byte-identical with no note -- this function only ever narrows or
    substitutes, never widens with a side that was not either requested or
    real).

      * If the building has NO real glazing anywhere (`SM_Building_11`/
        `SM_Building_27` -- painted windows this pack's texture-token list
        does not catch), `sides` stands untouched: venting through a
        painted-window wall is the least-wrong option, and the synthetic
        fallback still applies to every requested side exactly as before
        this reconciliation existed.
      * Otherwise, any requested side that DOES carry real islands is kept,
        in the order requested (`SM_Building_26`'s N/W request keeps
        neither -- see below; a healthy building's request keeps every
        side, which is what leaves it untouched).
      * If NONE of the requested sides carry real islands (contagion's
        whole pick is physically impossible on this building --
        `SM_Building_26`: requested N/W, real glazing is E only), the
        request is replaced outright with the real-glazed elevations,
        ranked by island count ("sides are chosen by window-island
        count"), taking as many as were originally requested.
    """
    ranked_real = [sd for sd in sorted(rects.keys(), key=lambda sd: -len(rects[sd]))
                  if rects[sd]]
    if not ranked_real:
        return tuple(sides), None
    kept = tuple(sd for sd in sides if sd in ranked_real)
    if kept:
        venting = kept
    else:
        venting = tuple(ranked_real[:max(1, len(sides))])
    if venting == tuple(sides):
        return venting, None
    note = ("{0}: sides {1} requested by contagion -> venting {2} "
           "(real glazing)".format(name, "/".join(sides), "/".join(venting)))
    return venting, note


def _nudge_origin_to_real(origin, venting_sides, real_by_side):
    """If the ORIGIN STOREY ITSELF has no real opening on any venting side,
    but real coverage exists somewhere at or below it, lower `origin` to
    the nearest such storey. Returns `(origin, note)`.

    WHY THE ORIGIN STOREY SPECIFICALLY. `soot_plume.plan_events`'s own
    invariant is "the first event drawn is ALWAYS in the compartment of
    origin, so the ladder's clean-below-origin signature is anchored to a
    real vent" -- every level's band (F1's 1-2 storeys through F4+'s
    origin-to-roof) starts AT the origin, so guaranteeing real coverage
    there is enough to pull the WHOLE band onto real glazing without
    touching `plan_fire`'s own band-width roll (which is an `rng` draw for
    F3 and rerunning it here would desync every recipe that draws from the
    same `rng` afterward). This only ever moves `origin` DOWN -- never past
    0, never above what was asked -- so `plan_fire`'s own upper clamp
    (`lo_min`, ensuring the band's MINIMUM size still fits under the roof)
    is never at risk of being violated by a change this function makes.

    A side with real glazing but no coverage ANYWHERE at or below the
    requested origin (measured, `SM_Building_02`: E's real glazing stops a
    couple of storeys short of the roofline, so a fire whose origin was
    already pinned to the topmost storey has nothing below it to slide
    onto either) is left for the synthetic-openings fallback, unchanged --
    "prefer widening/lowering... if the level's band rules allow; else
    fall back per today" (2026-08-31 policy)."""
    if any(origin in real_by_side.get(sd, ()) for sd in venting_sides):
        return origin, None
    candidates = [st for sd in venting_sides for st in real_by_side.get(sd, ())
                 if st <= origin]
    if not candidates:
        return origin, None
    new_origin = max(candidates)
    if new_origin == origin:
        return origin, None
    return new_origin, ("origin {0} -> {1} (nearest real glazing on {2})"
                        .format(origin, new_origin, "/".join(venting_sides)))


def max_synthetic_storey(n_levels, top_exclude=SYN_TOP_EXCLUDE_STOREYS):
    """The highest 0-indexed storey `_synthetic_side_rects` is willing to
    invent a window on, given `n_levels` (`len(m["levels"])`, i.e. the
    building's own storey count) -- floored at 0 so a 1- or 2-storey
    building (nothing left once the top `top_exclude` are cut) still gets
    its ground floor rather than losing every synthetic opening outright."""
    return max(0, int(n_levels) - 1 - int(top_exclude))


def _synthetic_side_rects(m, side, storeys, top_exclude=SYN_TOP_EXCLUDE_STOREYS):
    """A plausible bay-window grid on `side`, one row per storey in
    `storeys` -- the fallback authored for a (side, storey)
    `window_rects`/`_islands` found NOTHING at all for. See the block
    comment above `SYN_BAY_PITCH_M` for why this exists instead of another
    shape-filter tweak.

    THE TOP `top_exclude` STOREYS NEVER GET A GRID (2026-08-31, "avoid fires
    at the extreme top of buildings... unless we're 100% sure about windows
    on the top floor") -- a synthetic opening is never "sure". `storeys`
    entries above `max_synthetic_storey(len(m["levels"]), top_exclude)` are
    dropped. If that empties the request entirely (a short building whose
    whole requested band sits in the excluded top, e.g. a fire burning only
    its top two storeys) the HIGHEST ALLOWED storey gets a grid instead of
    the building showing nothing at all -- `note` (the second return value)
    records that this fallback fired, `None` otherwise.

    Returns `({storey: [(a0, a1, z0, z1), ...]}, note)` -- the rects already
    in the ALONG-WALL `u` coordinate `soot_plume.side_u` counts (0 at the
    side's own start corner, running to `side_length(m, side)`) -- the SAME
    frame `openings_provider` converts its real, asset-frame `rects` into
    below, so these splice straight into its `by` table with no further
    conversion.

    A window sits centred in ITS OWN storey's band (`levels[st]` to
    `levels[st + 1]`, or `m["top"]` on the roof storey), not some
    building-wide average floor height, so a short ground floor and a tall
    penthouse both get a window that actually fits inside its own band."""
    W, D = float(m["W"]), float(m["D"])
    span = W if side in ("S", "N") else D
    levels = list(m["levels"])
    top = float(m["top"])
    margin = SYN_EDGE_MARGIN_M if span > 2.0 * SYN_EDGE_MARGIN_M + 1.0 else 0.0
    usable = max(1.0, span - 2.0 * margin)
    n_bay = max(1, int(round(usable / SYN_BAY_PITCH_M)))
    pitch = usable / n_bay
    win_w = min(SYN_WIN_W_M, 0.7 * pitch)

    max_st = max_synthetic_storey(len(levels), top_exclude)
    requested = sorted({int(s) for s in storeys if 0 <= int(s) < len(levels)})
    allowed = [st for st in requested if st <= max_st]
    note = None
    if requested and not allowed:
        allowed = [max_st]
        note = ("every requested storey ({0}) is in the excluded top "
                "{1} of {2} -- synthesised on storey {3} instead".format(
                    requested, top_exclude, len(levels), max_st))

    out = {}
    for st in allowed:
        z0 = levels[st]
        z1 = levels[st + 1] if st + 1 < len(levels) else top
        band_h = max(0.1, z1 - z0)
        h = min(WINDOW_HEIGHT_RANGE_M[1],
               max(WINDOW_HEIGHT_RANGE_M[0], SYN_WIN_H_FRAC * band_h))
        h = min(h, 0.85 * band_h)
        zc = z0 + SYN_SILL_FRAC * band_h + 0.5 * h
        zc = min(zc, z1 - 0.05 * band_h - 0.5 * h)
        zc = max(zc, z0 + 0.05 * band_h + 0.5 * h)
        rects = []
        for i in range(n_bay):
            uc = margin + (i + 0.5) * pitch
            rects.append((uc - 0.5 * win_w, uc + 0.5 * win_w,
                         zc - 0.5 * h, zc + 0.5 * h))
        out[st] = rects
    return out, note


def openings_provider(rects, m, planes=None, band=None):
    """`(ctx, mass, side, storey) -> [opening records]` over the measured
    window islands: each record carries `span` (u0, u1, z_sill, z_head) in
    `soot_plume.side_u`'s convention plus the fields `_flame_sources` reads.

    `planes`, if given, is `window_rects`' measured `{side: plane_coord}`
    dict, forwarded to `side_frame` so every opening's `fr` sits on the real
    façade plane instead of the mass bbox face.

    `band`, if given, is `(sides, storeys)` -- the fire plan's own venting
    elevations and involved storey range (`fire["sides"]`, `fire["storeys"]`
    in `prepare`). For any side in it whose REAL islands contribute nothing
    across EVERY storey in that band, a synthetic bay-window grid
    (`_synthetic_side_rects`) is authored for exactly that side's band
    storeys and folded into the same `by` table the real islands populate --
    see the block comment above `SYN_BAY_PITCH_M`. A side that already has
    even one real opening somewhere in the band is left alone -- real
    islands are never overridden, only topped up where they are completely
    absent -- and a side outside `band` is never touched at all (`plan_fire`
    only ever queries `fire["sides"]`, so nothing downstream can reach it).
    `provider.synthetic_sides` records which sides needed it, for
    `tools/_gac_starved_probe.py` and offline verification. EVERY synthetic
    opening's own `e["synthetic"] = True` (2026-08-31, "avoid fires at the
    extreme top of buildings... unless we're 100% sure about windows on the
    top floor" -- `fire_bake._E_KEYS` now carries `"synthetic"`, additively,
    so it DOES round-trip to the sidecar this time; a real opening never
    sets the key at all, so "absent" still means "real" for every existing
    bake). `fire_assembly_lib`'s own top-storey filter falls back to the
    `"gac_window_synth"` vs `"gac_window"` name convention for a bake baked
    before this field existed -- see that module's `is_synthetic_op`.
    `provider.synthetic_notes` carries one string per side where
    `_synthetic_side_rects`' own top-storey fallback fired (a short building
    burning only in its excluded top storeys)."""
    W, D, cx, cy = m["W"], m["D"], m["cx"], m["cy"]
    levels = list(m["levels"])
    frames = {s: side_frame(m, s, planes) for s in ("S", "E", "N", "W")}
    by = {}
    for side, rl in rects.items():
        for (u0, u1, z0, z1) in rl:
            if side == "S":
                a, b = u0 - (cx - W / 2.0), u1 - (cx - W / 2.0)
            elif side == "E":
                a, b = u0 - (cy - D / 2.0), u1 - (cy - D / 2.0)
            elif side == "N":
                a, b = (cx + W / 2.0) - u1, (cx + W / 2.0) - u0
            else:
                a, b = (cy + D / 2.0) - u1, (cy + D / 2.0) - u0
            zc = 0.5 * (z0 + z1)
            st = 0
            for i, lv in enumerate(levels):
                if zc >= lv - 0.05:
                    st = i
            e = {"mass": "main", "x": 0.5 * (u0 + u1), "y": zc, "z": z0,
                 "storey": st, "side": side, "role": "wall",
                 "name": "gac_window", "p": {}, "dead": False}
            by.setdefault((side, st), []).append({
                "fr": frames[side], "ua": a, "ub": b, "va": z0, "vb": z1,
                "hua": a, "hub": b, "hva": z0, "hvb": z1, "out": -0.05,
                "e": e, "m": m, "side": side, "storey": st, "mass": "main",
                "span": (min(a, b), max(a, b), float(z0), float(z1))})

    synthetic_sides = []
    synthetic_notes = []
    if band:
        band_sides, band_storeys = band
        band_storeys = [int(s) for s in band_storeys]
        for side in band_sides:
            if side not in ("S", "E", "N", "W"):
                continue
            if any(by.get((side, st)) for st in band_storeys):
                continue          # at least one REAL opening in this band
            synth, note = _synthetic_side_rects(m, side, band_storeys)
            if not synth:
                continue
            synthetic_sides.append(side)
            if note:
                synthetic_notes.append("{0}: {1}".format(side, note))
            for st, rl in synth.items():
                for (a0, a1, z0, z1) in rl:
                    e = {"mass": "main", "x": 0.5 * (a0 + a1),
                         "y": 0.5 * (z0 + z1), "z": z0, "storey": st,
                         "side": side, "role": "wall",
                         "name": "gac_window_synth", "p": {}, "dead": False,
                         "synthetic": True}
                    by.setdefault((side, st), []).append({
                        "fr": frames[side], "ua": a0, "ub": a1,
                        "va": z0, "vb": z1, "hua": a0, "hub": a1,
                        "hva": z0, "hvb": z1, "out": -0.05,
                        "e": e, "m": m, "side": side, "storey": st,
                        "mass": "main",
                        "span": (min(a0, a1), max(a0, a1),
                                float(z0), float(z1))})

    def provider(ctx, mtag, side, storey):
        return list(by.get((side, storey), []))

    provider.count = sum(len(v) for v in by.values())
    provider.synthetic_sides = tuple(synthetic_sides)
    provider.synthetic_notes = tuple(synthetic_notes)
    return provider


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
def _diffuse_of(mat_prim):
    """(shader path, input name, texture url) of the map feeding a
    UsdPreviewSurface's diffuseColor — the GAC materials' layout — falling
    back to `soot_plume.find_basecolor`'s name heuristics."""
    from pxr import Sdf, Usd, UsdShade
    from . import soot_plume as spl

    if not mat_prim or not mat_prim.IsValid():
        return None, None, None
    for c in Usd.PrimRange(mat_prim):
        sh = UsdShade.Shader(c)
        if not sh or sh.GetIdAttr().Get() != "UsdPreviewSurface":
            continue
        d = sh.GetInput("diffuseColor")
        if d is not None and d.HasConnectedSource():
            ts = UsdShade.Shader(d.GetConnectedSource()[0].GetPrim())
            f = ts.GetInput("file") if ts else None
            v = f.Get() if f else None
            if isinstance(v, Sdf.AssetPath) and (v.resolvedPath or v.path):
                return ts.GetPrim().GetPath(), "file", (v.resolvedPath or v.path)
        break
    return spl.find_basecolor(mat_prim)


def _sample_skin_any_side(sk, m, pts):
    """Skin RGBA at cell-frame points on ANY elevation: each point's side is
    the wall line it is nearest, then `soot_plume.side_u` on that side."""
    from . import soot_bake as sb

    W, D = float(m["W"]), float(m["D"])
    lx = pts[:, 0] - float(m["cx"])
    ly = pts[:, 1] - float(m["cy"])
    dist = np.stack([np.abs(ly + D / 2.0), np.abs(lx - W / 2.0),
                     np.abs(ly - D / 2.0), np.abs(lx + W / 2.0)], axis=1)
    side_ix = np.argmin(dist, axis=1)            # 0 S, 1 E, 2 N, 3 W
    out = np.zeros((len(pts), 4), dtype=np.float32)
    for k, side in enumerate(("S", "E", "N", "W")):
        sel = side_ix == k
        if not sel.any():
            continue
        out[sel] = sb.sample_skin(sk, side, m, pts[sel])
    return out


def bake_atlases(stage, cell, mesh, sk, m, out_dir, verbose=True):
    """Bake the skin into every textured material of the merged mesh, once
    per texture, through the mesh's own UVs. Returns
    `{key: UsdShade.Material}` where `key` is both the source material's
    prim path AND its texture url (so pieces can be rebound by either)."""
    from . import soot_bake as sb, soot_plume as spl
    from . import tex_compress as tc

    P, UV, MID, mats = mesh["P"], mesh["UV"], mesh["MID"], mesh["mats"]
    n_tri = len(MID)
    counts = np.full(n_tri, 3, dtype=np.int64)
    indices = np.arange(3 * n_tri, dtype=np.int64)
    os.makedirs(out_dir, exist_ok=True)
    sooted, by_tex, stats = {}, {}, {"baked": 0, "clean": 0, "notex": 0}
    for k, mat in enumerate(mats):
        if mat is None:
            continue
        mp = mat.GetPrim()
        sh_path, inp, tex = _diffuse_of(mp)
        if not tex:
            stats["notex"] += 1
            continue
        if tex in by_tex:
            sooted[str(mp.GetPath())] = by_tex[tex]
            sooted["_png"][str(mp.GetPath())] = sooted["_png"][tex]
            continue
        face_ids = np.nonzero(MID == k)[0]
        if len(face_ids) == 0:
            continue
        base = spl._read_rgb(tex, max_px=BAKE_PX_MAX)
        if base is None:
            stats["notex"] += 1
            continue
        px = int(max(BAKE_PX_MIN, min(BAKE_PX_MAX, max(base.shape[0], base.shape[1]))))
        # IS THIS ATLAS TILED UP THE BUILDING? Rasterise the same faces in
        # the opposite order, at a quarter of the working resolution: a
        # texel that two faces at different heights both cover comes back
        # with two different positions. Cheap and decisive -- run BEFORE
        # the full-resolution pass below (same test, same numbers, just
        # moved ahead of the work it used to gate only after the fact) so a
        # TILED atlas -- thrown away to be baked per piece after the slice
        # -- never pays for the expensive rasterisation (including the
        # seam-straddler path) at full resolution at all.
        pq = max(256, px // 4)
        pa, ma = sb.uv_position_map(P, counts, indices, UV, "vertex", None,
                                    face_ids=face_ids.tolist(), px=pq)
        pb, mb = sb.uv_position_map(P, counts, indices, UV, "vertex", None,
                                    face_ids=face_ids[::-1].tolist(), px=pq)
        both = ma & mb
        shared = 0.0
        if both.any():
            # THE FULL 3D DISTANCE, NOT THE HEIGHT DIFFERENCE. The test used
            # to compare z only, so an atlas MIRRORED left-to-right (the SW
            # corner's faces on the same texels as the SE corner's, at the
            # same height) passed as unique and was baked once by position:
            # the soot the plume laid at the burning SE/NE corners landed on
            # the clean SW/NW corners too -- "GAC seems to wrap around the
            # wall in the wrong direction ... only 1 piece is correct" (user,
            # 2026-09-02, gac_SM_Building_06_Small_F5_s38). Measured with
            # the skin rebuilt from that bake's sidecar: alpha 0.92/0.95 at
            # SE/NE, 0.000 at SW/NW -- the plan was right, the atlas wrong.
            # `M_Building_05_WallBack` reads 0 % shared by height and 100 %
            # in 3D; SM_Building_02's Concrete 4 % vs 98 %, its awnings and
            # trim 76-100 %. A shared atlas takes the per-piece bake after
            # the slice, which samples every piece by its own world position.
            d3 = np.linalg.norm(pa[both] - pb[both], axis=1)
            shared = float((d3 > SHARED_TEXEL_M).mean())
        if shared > SHARED_FRAC_MAX:
            stats["tiled"] = stats.get("tiled", 0) + 1
            sooted.setdefault("_tiled", set()).add(tex)
            sooted["_tiled"].add(str(mp.GetPath()))
            if verbose:
                print("[gac_fire]   atlas {0}: TILED ({1:.0%} of its texels serve "
                      "faces >{2:.0f} m apart) -> baked per piece after the "
                      "slice".format(tex.rsplit("/", 1)[-1], shared,
                                     SHARED_TEXEL_M))
            continue
        pos, mask = sb.uv_position_map(P, counts, indices, UV, "vertex", None,
                                       face_ids=face_ids.tolist(), px=px)
        if not mask.any():
            continue
        pts = pos[mask].astype(np.float64)
        rgba = _sample_skin_any_side(sk, m, pts)
        if float(rgba[:, 3].max()) < 0.03:
            stats["clean"] += 1
            continue
        base = np.asarray(base, dtype=np.float32)
        if base.ndim == 2:
            base = np.repeat(base[..., None], 3, axis=2)
        base = base[..., :3]
        byi = np.linspace(0, base.shape[0] - 1, px).astype(int)
        bxi = np.linspace(0, base.shape[1] - 1, px).astype(int)
        out = base[byi][:, bxi].copy()
        a = rgba[:, 3:4]
        b = out[mask]
        grey = b.mean(axis=1, keepdims=True)
        desat = b * (1.0 - spl.DESAT * a) + grey * (spl.DESAT * a)
        out[mask] = np.clip(desat * (1.0 - a) + rgba[:, :3] * a, 0.0, 1.0)
        digest = hashlib.md5(np.round(out * 255.0).astype(np.uint8).tobytes()
                             ).hexdigest()[:16]
        # `.dds` by default (`SOOT_TEX_COMPRESS`, see `tex_compress.py`),
        # `.png` only with the gate off — `png` keeps its name for every
        # downstream use below.
        png = tc.save_soot_texture(
            out, os.path.join(out_dir, "gacsoot_{0}".format(digest)))
        mpath = "{0}/SootLooks/m{1}".format(cell, len(by_tex))
        new = spl.piece_material_like(stage, mpath, mp, sh_path, inp, png)
        if new is None:
            new = spl.piece_material(stage, mpath, png)
        by_tex[tex] = new
        sooted[str(mp.GetPath())] = new
        sooted[tex] = new
        sooted.setdefault("_png", {})[tex] = png
        sooted["_png"][str(mp.GetPath())] = png
        stats["baked"] += 1
        if verbose:
            print("[gac_fire]   atlas {0}: {1} tri(s), {2} px, soot on "
                  "{3:.0%} of its texels -> {4}".format(
                      tex.rsplit("/", 1)[-1], len(face_ids), px,
                      float((rgba[:, 3] > 0.1).mean()), os.path.basename(png)))
    if verbose:
        print("[gac_fire] {0} atlas(es) baked, {1} untouched by the fire, "
              "{2} tiled (left to the per-piece bake), {3} material(s) "
              "without a diffuse map".format(
                  stats["baked"], stats["clean"], stats.get("tiled", 0),
                  stats["notex"]))
    return sooted


# ---------------------------------------------------------------------------
# MDL SOOT OVERLAY — the route for a pack whose materials cannot be read at
# all (`PACKS[kind]["soot"] == "overlay"`, today only "aec"). See that
# table's own long comment for the measured reason `bake_atlases` cannot work
# here (a compiled `.mdl` module's base-colour texture is not visible to USD
# attribute introspection).
# ---------------------------------------------------------------------------
#: how far the decal stands proud of the real wall plane, in metres — enough
#: to clear z-fighting against the brick mesh, small enough to still read as
#: sitting ON the wall rather than floating off it. The same order of
#: magnitude `urban_fire._stamp_pt`'s own wall stamps (spall/char patches)
#: use, and `side_frame`'s own `-0.02` baseline depth already clears the
#: mesh itself before this is added.
OVERLAY_STANDOFF_M = 0.03


def _write_overlay_textures(crop, out_dir, key):
    """Two small, cached PNGs for one elevation's overlay decal —
    `(diffuse_path, opacity_path)`. DELIBERATELY NEVER `tex_compress.
    save_soot_texture`: that writer's default output (BC1/DXT1) is OPAQUE
    RGB with no alpha channel at all — fine for the bake-into-map route's
    fully pre-composited output, wrong here where the whole point is a
    translucent decal whose coverage rides on a real alpha/opacity map. A
    per-building overlay PNG pair is a handful of images at the skin
    canvas's own resolution (`soot_plume.canvas_dims`, capped at
    4096 x 2048), negligible next to the per-piece soot atlases this
    replaces.

    `crop` is a `soot_plume.piece_crop`-shaped (h, w, 4) RGBA array, row 0
    the top of the wall (`soot_plume.skin`'s own convention) — the same row
    order `wall_overlay.author_quad`'s UV mapping expects (`v=0` at the
    quad's own bottom corner, `v=1` at its top), so no flip is needed here.
    """
    import hashlib as _hl

    from PIL import Image

    os.makedirs(out_dir, exist_ok=True)
    a = np.asarray(crop, dtype=np.float32)
    digest = _hl.md5(np.round(a * 255.0).astype(np.uint8).tobytes()).hexdigest()[:16]
    dpath = os.path.join(out_dir, "sootovl_dif_{0}_{1}.png".format(key, digest))
    opath = os.path.join(out_dir, "sootovl_opa_{0}_{1}.png".format(key, digest))
    if not (os.path.exists(dpath) and os.path.exists(opath)):
        rgb = (np.clip(a[..., :3], 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)
        alpha = (np.clip(a[..., 3], 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)
        Image.fromarray(rgb, "RGB").save(dpath)
        Image.fromarray(alpha, "L").save(opath)
    return dpath, opath


def overlay_soot(stage, cell, mesh, sk, m, out_dir, tag="ovl", verbose=True):
    """The MDL route: soot as a TRANSLUCENT DECAL standing proud of each
    elevation instead of baked into a per-material atlas.

    OFF BY DEFAULT since 2026-09-02 (`GF_MDL_OVERLAY=1` re-enables).
    Rendered, the decals read exactly as the user described: "it looks like
    a rectangle that's been placed on the side of the house ... also the
    wrong size", with `mdlsootovl_a0_2` visibly standing off the wall, and
    they darken the whole building. A soot pass that has to float geometry
    in front of a facade is the wrong shape for this pipeline — GAC and MCE
    composite soot INTO the base-colour map and bind a material copy, and
    the brownstone can do the same: its `.mdl` modules are plain text
    wrapping ordinary OmniPBR BaseColor/Normal/ORM PNGs (measured:
    `Brick_Wall_Red.mdl` -> `./Brick_Wall_Red/Brick_Wall_Red_{BaseColor,N,
    ORM}.png`, all present in the local mirror), so the textures ARE
    recoverable by parsing the module and the standard bake path applies.
    Until that lands, no decals.

    For every source material `bake_atlases` could not find a diffuse map
    for (the whole AEC brick library — see `PACKS["aec"]`'s comment), the
    brick MDL material is left bound and untouched on the sliced pieces
    (real brick renders exactly as shipped); the soot itself is laid on top
    as one `wall_overlay`-authored quad per elevation that ever vented,
    textured directly from `soot_plume.skin`'s own unwrapped canvas for that
    side (`soot_plume.piece_crop`, the FULL span — u 0..side length, z from
    the mass base to the parapet top) — this is the same "one continuous
    surface wrapped around the building" canvas `tools/soot_elevation.py
    --mode skin` already previews, so a render that matches that preview's
    plume shape is the acceptance test (see that tool and the
    `place-people-in-scenes`-style offline proof this change ships with).

    The soot PATTERN is untouched: this only changes how it lands on the
    piece, never `soot_plume.plan_events`/`skin`'s own physics.

    Returns `(sooted, prebaked_paths)`. `sooted` is always `{}` — this route
    rebinds nothing (`rebind_sooted` is therefore a no-op on it by design,
    called anyway so the mixed-material case below still works).
    `prebaked_paths` is the set of ORIGINAL material prim paths (still bound
    on the sliced pieces after `slice_to_kit`, which binds the SAME
    `UsdShade.Material` prims `read_mesh` harvested off the source mesh —
    see that function's own "MESH-LEVEL BINDING" note) that had no readable
    diffuse map. `gac_fire.burn_gac` folds this into `ctx["soot_prebaked"]`
    so `urban_fire._bind_soot`'s per-piece kit-style bake — which hits the
    exact same `find_basecolor` wall on these materials and falls back to
    `_flat_diffuse`'s flat (0.6, 0.6, 0.6) grey, the "white with weird ash"
    defect this whole route exists to fix — skips them outright rather than
    stamping a second, redundant (and wrong-looking) soot pass on top of the
    overlay decal.

    A material this pack's own bake_atlases COULD read (measured: a handful
    of non-brick materials on the AEC asset, e.g. trim/railings) is left
    alone here entirely — `bake_atlases` still runs first and handles it the
    normal way, unaffected by this function.
    """
    from . import soot_plume as spl
    from . import wall_overlay as wov

    prebaked = set()
    for mat in (mesh or {}).get("mats") or []:
        if mat is None:
            continue
        mp = mat.GetPrim()
        _sh, _inp, tex = _diffuse_of(mp)
        if not tex:
            prebaked.add(str(mp.GetPath()))
    if not prebaked:
        # every material on this asset already has a readable base map --
        # nothing for the overlay route to do (never hit on the aec pack
        # today, kept as a safety net if a future asset in this pack ships
        # ordinary UsdPreviewSurface materials throughout).
        if verbose:
            print("[gac_fire]   MDL soot overlay: every material had a "
                  "readable base map -- nothing routed here")
        return {}, set()

    parapet = spl.parapet_height(m)
    ctx = {"stage": stage, "parent": cell, "tag": tag, "authored": []}
    n_sides = 0
    for side in ("S", "E", "N", "W"):
        L = spl.side_length(m, side)
        crop = spl.piece_crop(sk, side, 0.0, L, m["z0"], m["top"] + parapet)
        if float(crop[..., 3].max()) < 0.03:
            continue          # this elevation never vented -- nothing to lay
        fr = side_frame(m, side, planes=None)
        dif_path, opa_path = _write_overlay_textures(
            crop, out_dir, key="{0}_{1}".format(tag, side))
        mat = wov.overlay_material_textured(
            stage, "{0}/SootOverlayLooks/{1}".format(cell, side),
            opa_path, dif_path, roughness=0.9)
        wov.author_quad(ctx, fr, 0.0, L, m["z0"], m["top"] + parapet,
                        OVERLAY_STANDOFF_M, mat, kind="mdlsootovl")
        n_sides += 1
    if verbose:
        print("[gac_fire]   MDL soot overlay: {0} elevation(s) decaled, "
              "{1} source material(s) routed away from the per-piece "
              "kit-style bake".format(n_sides, len(prebaked)))
    return {}, prebaked


def rebind_sooted(stage, pls, sooted):
    """Every piece subset bound to a material that has a sooted copy is
    rebound to the copy (matched by material path, then by texture url —
    a baked kit's rehomed materials share the texture, not the path)."""
    from pxr import Usd, UsdGeom, UsdShade

    n = 0
    for p in pls:
        prim = stage.GetPrimAtPath(p["prim_path"]) if p.get("prim_path") else None
        if not prim or not prim.IsValid():
            continue
        for mesh in Usd.PrimRange(prim):
            if not mesh.IsA(UsdGeom.Mesh):
                continue
            subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh)))
            for t in ([s.GetPrim() for s in subs] or [mesh]):
                cur = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
                if not cur or not cur.GetPrim().IsValid():
                    continue
                new = sooted.get(str(cur.GetPrim().GetPath()))
                if new is None:
                    _sp, _inp, tex = _diffuse_of(cur.GetPrim())
                    new = sooted.get(tex) if tex else None
                if new is None or isinstance(new, dict):
                    continue
                UsdShade.MaterialBindingAPI(t).Bind(new)
                n += 1
    return n


def uf_side_neighbours(side):
    ring = ("S", "E", "N", "W")
    i = ring.index(side) if side in ring else 0
    return (ring[i - 1], ring[(i + 1) % 4])


_SIDES4 = ("S", "E", "N", "W")


def _piece_sides(side_tag):
    """Every cardinal side of `_SIDES4` a piece's `_side` tag touches.

    MEASURED (`tools/gac_burn_probe.py`'s window census, 2026-08-30), not
    assumed: `gac_storey_slice.as_placements` writes THREE different shapes
    into `p["_side"]`, and the first pass here only handled one of them —
    every corner and every merged-band piece on SM_Building_03 F1 kept its
    plain glazing untouched, all the way through the fire's own origin
    storey, because none of them ever passed the old `side.split("x")` test:

      * a plain run: one of `_SIDES4` already ("S", "E", "N", "W");
      * a RING corner (`_cut`'s own `("SW", ...), ("SE", ...), ("NW", ...),
        ("NE", ...)`): two compass letters, no separator at all — the
        `_darken_glass_legacy` comment's "'SxE'" describes a DIFFERENT
        corner convention (`urban_building`'s kit pieces) that never
        applied to a GAC region cut;
      * a MERGED band (`slice_to_kit`'s `merged_lower`/`merged_upper`,
        everything below the fire's origin or above its `region["top"]`
        collapsed to one piece): the literal string `"x"` — the WHOLE
        perimeter in one mesh, touching every side at once.

    A tag this function does not recognise is treated the same as the
    merged-band case (every side) rather than excluded — the OUTER filter
    this feeds is only ever a cheap skip ahead of the real, per-FACE side
    test (nearest wall line, inside `damage_windows`'s own loop), so the
    failure mode of guessing too WIDE here is a few extra faces checked and
    discarded; guessing too NARROW is a window silently left untreated,
    which is the bug this function exists to stop happening again."""
    t = str(side_tag or "")
    if t in _SIDES4:
        return {t}
    parts = [c for c in t.replace("-", "x").split("x") if c]
    hit = set(c for c in parts if c in _SIDES4) if len(parts) > 1 else set()
    if not hit:
        hit = set(c for c in t if c in _SIDES4)   # compass corner, e.g. "SW"
    return hit or set(_SIDES4)                     # unrecognised -> every side


def _storey_of(z, levels):
    """Storey index of world height `z` on a `levels` list (storey i's floor
    at `levels[i]`) — the convention `openings_provider` counts by."""
    st = 0
    for i, lv in enumerate(levels):
        if z >= lv - 0.05:
            st = i
    return st


def _match_island(side, u, z, rects, pad=0.15):
    """The window ISLAND (u0, u1, z0, z1) on `side` that (u, z) falls in —
    `pad` metres of slack for a face centroid that sits a little inside the
    frame rather than dead centre on the glass — or None."""
    for box in (rects or {}).get(side, ()):
        u0, u1, z0, z1 = box
        if u0 - pad <= u <= u1 + pad and z0 - pad <= z <= z1 + pad:
            return box
    return None


def _window_materials(stage, cell):
    """The two GAC-only window looks `damage_windows` binds.

    Both use the `enable_opacity` / `opacity_threshold=0` / `opacity_mode=0`
    OmniPBR recipe `urban_fire.materials`'s glass-deposit bands already use:
    a genuine per-pixel cutout on a bench that passes `--/rtx/raytracing/
    fractionalCutoutOpacity` (`gac_fire_bench_launch_script.KIT_ARGS` does),
    a binary cutout — opaque at 1, a real hole at exactly 0, no fractional
    blend — everywhere else (`wall_overlay`'s own note on the same flag).
    `burnt` is authored at exactly 0 for that reason: a fully burnt-out pane
    is meant to read as a hole on EITHER renderer state, not only the one
    with the flag on."""
    from pxr import Sdf, UsdShade
    from . import damage as dmg

    scope = cell + "/GacWinLooks"

    def _mk(name, rgb, rough, opacity):
        path = scope + "/" + name
        m = UsdShade.Material.Get(stage, path)
        if m and m.GetPrim().IsValid():
            return m
        m = dmg._pbr(stage, path, rgb, rough)
        sh = UsdShade.Shader.Get(stage, m.GetPath().AppendChild("Shader"))
        if sh:
            sh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
            sh.CreateInput("opacity_constant",
                          Sdf.ValueTypeNames.Float).Set(float(opacity))
            sh.CreateInput("opacity_threshold",
                          Sdf.ValueTypeNames.Float).Set(0.0)
            sh.CreateInput("opacity_mode", Sdf.ValueTypeNames.Int).Set(0)
        return m

    return {
        # fully cut: a burnt-out pane is a hole, not a dark pane
        "burnt": _mk("burnt", (0.008, 0.007, 0.006), 0.90, 0.0),
        # sooted, cracked, still glazed: dark and half gone, not a hole
        "crazed": _mk("crazed", (0.026, 0.029, 0.029), 0.78, 0.6),
    }


def _darken_glass_legacy(stage, ctx, pls, sooted=None, glass_tex=None):
    """`darken_glass`'s ORIGINAL behaviour, kept verbatim for any call site
    that has not been updated to pass `damage_windows` the measured window
    islands (`rects`/`mass`): the whole glass subset of every piece on the
    burning storeys/sides -> the void tone, with no per-window split."""
    from detail import gac_slice as gsl
    from pxr import Usd, UsdGeom, UsdShade

    glass_tex = glass_tex or gsl.GLASS_TEX
    band = set(int(s) for s in ctx["fire"]["storeys"])
    hot = set(ctx["fire"]["sides"])
    for sd in list(hot):
        hot.update(uf_side_neighbours(sd))
    mat = ctx["mats"]["void"]
    back = {}
    for k, v in ((sooted or {}).get("_png") or {}).items():
        if "/" in k and not k.startswith("/World") and "://" in k:
            back[v] = k          # baked png -> original texture url
    # ...and the per-piece copies `urban_fire._bind_soot` made for the tiled
    # atlases: `ctx["soot_mats"]` maps (original material path, png) -> copy
    orig_of = {}
    for key, cm in (ctx.get("soot_mats") or {}).items():
        try:
            orig_of[str(cm.GetPrim().GetPath())] = key[0]
        except Exception:
            continue
    n = 0
    for p in pls:
        if p.get("_storey") not in band or p.get("dead"):
            continue
        side = str(p.get("_side", ""))
        # a corner piece is named for both of its sides ("SxE"); a run for one
        if not any(sd in ctx["fire"]["sides"] for sd in side.split("x")):
            continue
        prim = stage.GetPrimAtPath(p["prim_path"]) if p.get("prim_path") else None
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        for mesh in Usd.PrimRange(prim):
            if not mesh.IsA(UsdGeom.Mesh):
                continue
            for s in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh)):
                cur = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
                if not cur or not cur.GetPrim().IsValid():
                    continue
                cpath = str(cur.GetPrim().GetPath())
                if cpath in orig_of:
                    op = stage.GetPrimAtPath(orig_of[cpath])
                    _sp, _inp, tex = _diffuse_of(op) if op and op.IsValid() else (None, None, None)
                else:
                    _sp, _inp, tex = _diffuse_of(cur.GetPrim())
                tex = back.get(tex, tex)
                if tex and gsl.is_glazing(tex, glass_tex):
                    UsdShade.MaterialBindingAPI(s.GetPrim()).Bind(mat)
                    n += 1
    return n


def damage_windows(stage, ctx, pls, rects=None, mass=None, sooted=None,
                   glass_tex=None, deny_mat=None):
    """Per-WINDOW glazing treatment on the burning elevations of a sliced
    GAC building — a burnt-out pane is now a real hole, not a blacked-out
    subset (`darken_glass`'s old, whole-subset behaviour: see
    `_darken_glass_legacy`, still reachable with the old call signature).

    Three states, decided per WINDOW ISLAND (`rects`, `window_rects`'
    measured (u0, u1, z0, z1) boxes, matched to each face by its own
    centroid) rather than per material subset — a subset is typically ONE
    material for a whole elevation, and a corner piece's single "glass"
    subset covers windows on BOTH of its faces, so binding the whole subset
    one way put a hot-side window's treatment on its cold-side neighbour too
    (the exact "rectangles" / "random black windows on a cold elevation"
    failure `darken_glass` already had to stop once, 2026-08-30):

      * BURNT OUT (`mats["burnt"]`, opacity 0, a real hole) — a hot-side
        window whose storey is in the fire's own band.
      * CRAZED / SMOKED (`mats["crazed"]`, opacity ~0.6, dark, still glazed)
        — a hot-side window one storey above the band, or any window in the
        band or one above it on a side that NEIGHBOURS a hot one (the same
        corner bleed `uf_side_neighbours` already modelled for the old
        whole-subset bind).
      * left alone everywhere else: whatever the sooted-atlas bake already
        bound it to, or the asset's own glass.

    A subset that ends up split has its face list SHRUNK to the faces that
    keep their existing binding, and one new partition-family GeomSubset
    per treatment is added alongside it with the reassigned faces — faces
    are exclusive to one subset, so the shrink is what keeps the partition
    honest.

    `rects`/`mass` are `prepare()`'s own `pre["rects"]`/`pre["mass"]` — the
    caller passes them explicitly because `ctx` does not carry them yet
    (`ctx["gac"]` is only assembled by `burn_gac` AFTER this returns).
    Either omitted (any call site still on `darken_glass`'s original
    signature) falls back to `_darken_glass_legacy`, unchanged.

    `deny_mat` (`PACKS[kind]["glazing_material_deny"]`) is the same
    lower-cased-substring deny list `window_rects` takes — a face bound to
    one of these materials is left alone here too, on top of never having
    formed an island in the first place."""
    if rects is None or mass is None:
        return _darken_glass_legacy(stage, ctx, pls, sooted=sooted,
                                    glass_tex=glass_tex)

    from detail import gac_slice as gsl
    from pxr import Usd, UsdGeom, UsdShade, Vt

    glass_tex = glass_tex or gsl.GLASS_TEX
    deny_mat = tuple(str(d).lower() for d in (deny_mat or ()))
    fire = ctx["fire"]
    W, D = float(mass["W"]), float(mass["D"])
    cx, cy = float(mass["cx"]), float(mass["cy"])
    levels = list(mass["levels"])
    n_st = len(levels)
    band = set(int(s) for s in fire["storeys"])
    top_band = int(fire["top"])
    above_st = top_band + 1 if top_band + 1 < n_st else None
    hot_sides = set(fire["sides"])
    bleed_sides = set()
    for sd in hot_sides:
        bleed_sides.update(uf_side_neighbours(sd))
    bleed_sides -= hot_sides
    relevant_sides = hot_sides | bleed_sides

    mats = _window_materials(stage, ctx["parent"])
    back = {}
    for k, v in ((sooted or {}).get("_png") or {}).items():
        if "/" in k and not k.startswith("/World") and "://" in k:
            back[v] = k
    orig_of = {}
    for key, cm in (ctx.get("soot_mats") or {}).items():
        try:
            orig_of[str(cm.GetPrim().GetPath())] = key[0]
        except Exception:
            continue
    # ...and the PRE-SLICE atlases (`bake_atlases`): `sooted` maps the
    # ORIGINAL material's prim path to its sooted copy, so inverting it
    # recovers the original material's NAME behind a `<cell>/SootLooks/mN`
    # binding. That name is the only evidence a downtowncity window has —
    # `Glass_window` / `glass` / `Window_003` carry no diffuse map at all,
    # so the texture test below cannot see them (`gac_slice.is_glazing`).
    name_of = {}
    for k, v in (sooted or {}).items():
        if k in ("_png", "_tiled") or not hasattr(v, "GetPrim"):
            continue
        if isinstance(k, str) and k.startswith("/"):
            name_of[str(v.GetPrim().GetPath())] = k.rsplit("/", 1)[-1]

    n_burnt = n_crazed = 0
    for p in pls:
        if p.get("dead"):
            continue
        sides_here = _piece_sides(p.get("_side", ""))
        if not (sides_here & relevant_sides):
            continue
        prim = stage.GetPrimAtPath(p["prim_path"]) if p.get("prim_path") else None
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        for meshp in Usd.PrimRange(prim):
            if not meshp.IsA(UsdGeom.Mesh):
                continue
            me = UsdGeom.Mesh(meshp)
            pts = me.GetPointsAttr().Get()
            if not pts:
                continue
            P = np.asarray(pts, dtype=np.float64)
            counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [],
                                dtype=np.int64)
            fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [],
                             dtype=np.int64)
            if not len(counts) or len(fvi) != int(counts.sum()):
                continue
            start = np.concatenate([[0], np.cumsum(counts)[:-1]])
            for sub in list(UsdGeom.Subset.GetAllGeomSubsets(
                    UsdGeom.Imageable(meshp))):
                cur = UsdShade.MaterialBindingAPI(
                    sub.GetPrim()).ComputeBoundMaterial()[0]
                if not cur or not cur.GetPrim().IsValid():
                    continue
                cpath = str(cur.GetPrim().GetPath())
                if cpath in orig_of:
                    op = stage.GetPrimAtPath(orig_of[cpath])
                    _live = bool(op and op.IsValid())
                    _sp, _inp, tex = (_diffuse_of(op) if _live
                                     else (None, None, None))
                    mname = op.GetName() if _live else ""
                else:
                    _sp, _inp, tex = _diffuse_of(cur.GetPrim())
                    mname = name_of.get(cpath) or cur.GetPrim().GetName()
                tex = back.get(tex, tex)
                # `is_glazing` with an empty `tex` and a non-glazing material
                # name is False, which is exactly what the old `not tex or
                # not is_glazing(tex)` said — GAC is unaffected.
                if not gsl.is_glazing(tex, glass_tex, mat_name=mname):
                    continue
                if deny_mat and any(d in (mname or "").lower() for d in deny_mat):
                    continue
                idx = [int(f) for f in (sub.GetIndicesAttr().Get() or [])]
                if not idx:
                    continue
                burnt_f, crazed_f, keep_f = [], [], []
                for f in idx:
                    if f >= len(counts):
                        keep_f.append(f)
                        continue
                    vids = fvi[start[f]:start[f] + counts[f]]
                    cen = P[vids].mean(axis=0)
                    dist = (abs(cen[1] - cy + D / 2.0),
                           abs(cen[0] - cx - W / 2.0),
                           abs(cen[1] - cy - D / 2.0),
                           abs(cen[0] - cx + W / 2.0))
                    side = _SIDES4[int(np.argmin(dist))]
                    if side not in relevant_sides:
                        keep_f.append(f)
                        continue
                    u = cen[0] if side in ("S", "N") else cen[1]
                    z = float(cen[2])
                    island = _match_island(side, u, z, rects)
                    if island is None:
                        # NOT A REAL WINDOW -- either off the measured
                        # lattice or (fire_dtc2 review, 2026-08-30: dtc
                        # Building_12 "cut up into triangles") a face whose
                        # SUBSET matched `is_glazing` by name/texture but
                        # whose merged island failed the shape test in
                        # `window_rects` (an air-conditioner unit's own
                        # triangulated grille/fin geometry). Leave it alone
                        # rather than guessing a storey from its raw z --
                        # burning/crazing it individually is exactly what
                        # produced the triangles.
                        keep_f.append(f)
                        continue
                    st = _storey_of(0.5 * (island[2] + island[3]), levels)
                    hot = side in hot_sides
                    bleed = side in bleed_sides
                    if hot and st in band:
                        burnt_f.append(f)
                    elif ((hot and above_st is not None and st == above_st)
                         or (bleed and (st in band or st == above_st))):
                        crazed_f.append(f)
                    else:
                        keep_f.append(f)
                if not burnt_f and not crazed_f:
                    continue
                if keep_f:
                    sub.GetIndicesAttr().Set(Vt.IntArray(keep_f))
                else:
                    sub.GetPrim().SetActive(False)
                base_name = sub.GetPrim().GetName()
                if burnt_f:
                    bs = UsdGeom.Subset.CreateGeomSubset(
                        me, base_name + "_burnt", UsdGeom.Tokens.face,
                        Vt.IntArray(burnt_f), UsdShade.Tokens.materialBind,
                        UsdGeom.Tokens.partition)
                    UsdShade.MaterialBindingAPI.Apply(
                        bs.GetPrim()).Bind(mats["burnt"])
                    n_burnt += len(burnt_f)
                if crazed_f:
                    cs = UsdGeom.Subset.CreateGeomSubset(
                        me, base_name + "_crazed", UsdGeom.Tokens.face,
                        Vt.IntArray(crazed_f), UsdShade.Tokens.materialBind,
                        UsdGeom.Tokens.partition)
                    UsdShade.MaterialBindingAPI.Apply(
                        cs.GetPrim()).Bind(mats["crazed"])
                    n_crazed += len(crazed_f)
    ctx["notes"].append(
        "windows: {0} burnt out (see-through), {1} crazed/smoked -- "
        "per-window, not per-subset".format(n_burnt, n_crazed))
    return n_burnt + n_crazed


darken_glass = damage_windows


# ---------------------------------------------------------------------------
# The whole thing
# ---------------------------------------------------------------------------
def prepare(stage, cell, name, level, rng, tag, origin=None, sides=None,
            out_dir=None, verbose=True, kind=None):
    """Everything up to (not including) the slice: place the asset, measure
    it, plan the fire and its events from the window islands, rasterise the
    skin, bake it into the atlases. Returns a dict the slice/burn tail and
    the offline tools (`tools/gac_fire_probe.py`, `tools/soot_elevation.py
    --gac`) both consume: src, grid, measured, rects, mass, info, btype,
    fire, provider, events, skin, mesh (None after the bake), sooted, planes
    (the measured per-side façade plane `window_rects` filled — see
    `side_frame`), kind and asset (the bare name).

    `name` may be `"dtc:Amar_Tower"` (or `kind="dtc"` with a bare name) to
    reach a downtowncity block instead of a GreatAmericanCity one — see
    `PACKS`. Everything that differs between the two packs (directory, file
    extension, unit scale, whether a material's own NAME may declare it
    glazing, whether the asset has baked-in landscaping to keep out of the
    measured box, and where the construction type comes from) is a row in
    that table; the `gac` row is the constants this function used to inline,
    so the GAC path is unchanged.
    """
    from detail import gac_slice as gsl, gac_storey_slice as gss
    from . import soot_plume as spl, urban_fire as uf

    def _hull2d(pts):
        """CCW convex hull of 2-D points (Andrew's monotone chain; a
        cross-product sign is the only geometry primitive it needs, so no
        scipy). Rounds first to kill duplicate mesh-seam vertices."""
        uniq = sorted(set((round(float(x), 4), round(float(y), 4))
                          for x, y in pts))
        if len(uniq) < 3:
            return uniq

        def cross(o, a, b):
            return ((a[0] - o[0]) * (b[1] - o[1])
                    - (a[1] - o[1]) * (b[0] - o[0]))

        def half(seq):
            out = []
            for p in seq:
                while len(out) >= 2 and cross(out[-2], out[-1], p) <= 0.0:
                    out.pop()
                out.append(p)
            return out

        lower, upper = half(uniq), half(list(reversed(uniq)))
        hull = lower[:-1] + upper[:-1]
        # FORCE CCW so `_inset_edges`'s inward-normal convention holds — the
        # monotone chain above already comes out CCW, but the shoelace check
        # is cheap and removes the assumption.
        area2 = sum(hull[i][0] * hull[(i + 1) % len(hull)][1]
                   - hull[(i + 1) % len(hull)][0] * hull[i][1]
                   for i in range(len(hull)))
        return hull if area2 >= 0 else hull[::-1]

    def _inset_edges(poly, t):
        """Inward per-edge offset of a CCW convex polygon by `t` metres,
        re-intersecting consecutive offset edge lines (a Minkowski erosion) —
        NOT a centroid scale, which over-cuts a long wing and under-cuts a
        squat one on an elongated plan (`min_area_rect`-style shapes are
        common on a setback tower). A near-parallel pair of offset edges (a
        rounding artefact rather than a real corner) falls back to moving
        that vertex along its own edge's inward normal.

        RE-HULLED BEFORE RETURNING. A SHORT RAW EDGE SANDWICHED BETWEEN TWO
        LONG ONES IS AN EFFECTIVELY ACUTE CORNER, and the per-edge-offset-
        then-intersect construction overshoots there (the correct inward
        shift at a convex vertex is `t / sin(half-angle)`, which grows
        without bound as the corner sharpens) -- measured on SM_Building_09
        storey 12 (`tools/catch_clip_probe.py`): a real ~0.15 m corner
        chamfer produced ONE locally reflex vertex in the raw inset output
        (a cross-product sign check on the returned polygon flipped at
        exactly that vertex and its neighbour), which is not a convex
        polygon any more even though the polygon's NET winding still read
        CCW. A reflex point from overshoot always lies inside the convex
        hull of its neighbours, so hulling the raw inset points drops
        exactly that vertex and nothing else -- the result can only be
        slightly SMALLER at that one corner, never larger, which is the
        safe direction for a plate that must never reach past the wall."""
        n = len(poly)
        if n < 3:
            return poly
        P = np.asarray(poly, dtype=float)
        d = np.roll(P, -1, axis=0) - P
        L = np.hypot(d[:, 0], d[:, 1])
        L[L < 1e-9] = 1.0
        d = d / L[:, None]
        nrm = np.stack([-d[:, 1], d[:, 0]], axis=1)      # inward, CCW
        A = P + nrm * t
        out = []
        for i in range(n):
            a0, d0 = A[i - 1], d[i - 1]
            a1, d1 = A[i], d[i]
            den = d0[0] * d1[1] - d0[1] * d1[0]
            if abs(den) < 1e-9:
                out.append(tuple(P[i] + nrm[i] * t))
                continue
            diff = a1 - a0
            s = (diff[0] * d1[1] - diff[1] * d1[0]) / den
            out.append(tuple(a0 + s * d0))
        return _hull2d(out)

    def _poly_area(poly):
        n = len(poly)
        if n < 3:
            return 0.0
        return 0.5 * abs(sum(poly[i][0] * poly[(i + 1) % n][1]
                            - poly[(i + 1) % n][0] * poly[i][1]
                            for i in range(n)))

    def _storey_footprints(mesh, levels, inset_m=0.35, lo_m=0.3, hi_m=2.5):
        """{storey index: [(x, y), ...]} in the CELL frame (`mass_from_grid`'s
        own frame — NOT centred on the mass), one convex-hull-inset polygon
        per storey, off the merged mesh's own vertices in a slab clear of the
        floor slab and any window head/sill (z in [level + lo_m, level +
        hi_m]) — this is where the WALLS of that storey band are, including
        a setback that has stepped the plan in from the storeys below.

        A FULL-PLAN W x D BOX IS WRONG ON A SETBACK PLAN. `r_expose_interior`
        and `r_fire_collapse` used to author their catch/heap-floor plates as
        one box spanning the mass's own bounding box at every storey, which
        pokes a slab out past the façade wherever the real plan is narrower
        than that box (user review, fire_dtc2, 2026-08-30, dtc Building_11
        F1: "the catch ... looks like it's coming outside the side walls ...
        you can't treat it like a cuboid"). `urban_fire._plate` builds an
        extruded polygon from what this measures instead, in place of the
        box, wherever a storey's polygon was measured here.

        A CONVEX HULL IS AN APPROXIMATION OF A CONCAVE PLAN — a real notch
        can be straight-lined across by the hull, which is why the inset
        exists (it can leave a small gap at a notch, it cannot grow the
        plate past the walls) and why `tools/catch_clip_probe.py` measures
        the worst-case excursion instead of assuming this is exact.
        """
        out = {}
        if mesh is None:
            return out
        P, tris = mesh.get("P"), mesh.get("tris")
        if P is None or tris is None or not len(tris):
            return out
        idx = np.unique(tris)
        Pb = P[idx]
        for st, z in enumerate(levels):
            band = Pb[(Pb[:, 2] >= z + lo_m) & (Pb[:, 2] <= z + hi_m)]
            if len(band) < 3:
                continue
            hull = _hull2d([(float(x), float(y)) for x, y in band[:, :2]])
            if len(hull) < 3:
                continue
            poly = _inset_edges(hull, inset_m)
            if len(poly) < 3 or _poly_area(poly) < 1.0:
                continue
            out[st] = poly
        return out

    kind, asset = split_kind(name, kind)
    pack = PACKS[kind]
    style = pack["style_prefix"] + asset
    url = asset_url(asset, kind)
    scale = asset_scale(url, pack["scale"], verbose=verbose)
    src = place_source(stage, cell, url, scale)
    if not src:
        raise RuntimeError("{0}: nothing composed".format(name))
    wins, bbox = gsl.window_centres(stage, src)
    # BAKED-IN LANDSCAPING IS NOT THE BUILDING. A no-op for a pack whose
    # `bbox_exclude` is empty (`gac`), and the ONE thing that has to happen
    # before `grid_for`: `measure_grid` lattices its storey lines between
    # `bbox` z0 and z1, so a roof garden that raises z1 by 10 m invents
    # storeys of canopy the slicer would then ring.
    bbox, trim_note = trim_bbox(stage, src, bbox, pack["bbox_exclude"],
                                verbose=verbose)
    g, measured = gss.grid_for(stage, src, bbox, wins, name=asset, verbose=verbose)
    planes = {}
    rects = window_rects(stage, src, planes=planes,
                         deny_mat=pack.get("glazing_material_deny"))
    # THE MESH IS READ HERE, BEFORE THE MASS BOX, NOT AFTER. `mass_from_grid`
    # measures the real roof-deck height (`deck_z`) off this same array's
    # upward faces -- everything that sits "on the roof" downstream
    # (`dress_roof_urban`, `_deck_slab`, `_rafter_teeth`, `r_roof_scorch`)
    # reads `m.get("deck_z", m["top"])` in place of the bbox top, which is
    # the parapet coping and can be well over a metre proud of the real deck
    # (row-2 review, 2026-08-30: "floating roof props and also floating
    # debris"). The bake below still runs on this same `mesh`; nothing else
    # about it changes.
    mesh = gss.read_mesh(stage, src, verbose=False)
    # ...and the deck search must not land in the roof garden either: the
    # top-15% band it walks is exactly where Amar_Tower's canopy is. Only
    # the DECK MEASUREMENT sees the trimmed mesh; the atlas bake below still
    # gets the whole thing. The FOOTPRINT scan below reads the same trimmed
    # mesh, for the same reason (a roof-garden storey should not hull in the
    # canopy either).
    mesh_bldg = mesh_without_props(mesh, pack["bbox_exclude"])
    m = mass_from_grid(g, bbox, mesh=mesh_bldg)
    n_st = len(m["levels"])
    # THE PLATE THE FIRE LADDER STANDS A CATCH FLOOR / HEAP ON IS NOT A BOX.
    # See `_storey_footprints`'s own docstring above. Measured once here,
    # off the same greenery-trimmed mesh the deck search just read, and
    # handed through `fire["footprints"]` / this function's own return dict
    # so `urban_fire._plate` can build a plan-accurate plate wherever one was
    # measured.
    footprints = _storey_footprints(mesh_bldg, m["levels"])
    # CONSTRUCTION TYPE: THE MEASURED TABLE FIRST, HEIGHT ONLY AS A FALLBACK.
    # `quake_sliced.CONSTRUCTION` is the one place in this repo where a
    # per-asset construction judgement is allowed to live, and it already
    # carries the three named downtowncity blocks (`Amar_Tower` -> rc_glass —
    # a 231 m all-glass tower the height rule alone would call plain `rc`,
    # `Building_11`/`Building_12` -> rc). Everything it does not name falls
    # through to `construction_type`'s own height cut, which is the SAME
    # 25 m rule this line used to be. GreatAmericanCity deliberately does NOT
    # consult the table (`construction_table: False`): the GAC path is frozen
    # against the kit look and the table would move most of the stock
    # (`SM_Building_02` is `urm` there and `rc` by height).
    H = m["top"] - m["z0"]
    if pack["construction_table"]:
        from . import quake_sliced as qs
        btype = qs.construction_type(url, H=H)
    else:
        btype = "urm" if H <= 25.0 else "rc"
    info = {"style": style, "family": "01", "type": btype, "x": 0.0, "y": 0.0,
            "yaw": 0.0, "masses": {"main": m}, "elements": [],
            "H": m["top"] - m["z0"]}
    if origin is None:
        origin = max(0, min(n_st - 1, int(round(0.25 * (n_st - 1)))))
    if sides is None:
        # THE FIRE VENTS WHERE THE WINDOWS ARE. A GAC asset carries its
        # glazing on one or two elevations and blank party walls elsewhere
        # (`SM_Building_02`: real glazing on E, none on S/N/W), so a side
        # drawn at random is a blank wall half the time and the building
        # gets no events at all. Rank the elevations by REAL island count
        # first and take as many as the level's plan wants off the top.
        #
        # A BLANK SIDE IS STILL A CANDIDATE, JUST LAST. Real islands used to
        # be the only candidates at all, with an empty `rects` collapsing to
        # a single hardcoded `["S"]` regardless of how many sides the level
        # wanted -- so a building with real glazing on only one elevation
        # (or none, `SM_Building_11`/`SM_Building_27`, MEASURED: zero
        # glazing-tagged faces anywhere on either mesh) could never get the
        # 2-3 venting sides F3+ wants. The blank sides fill out the ring
        # behind the real ones instead of being dropped, so `n_side` is
        # never capped short of what the level asks for; every side past
        # the real ones gets its openings from `openings_provider`'s
        # synthetic fallback (`band=` below), never from here.
        ranked_real = [sd for sd in sorted(rects.keys(), key=lambda sd: -len(rects[sd]))
                      if rects[sd]]
        ranked_blank = [sd for sd in ("S", "E", "N", "W") if sd not in ranked_real]
        ranked = ranked_real + ranked_blank
        n_side = 1 if level in ("F1", "F2") else (2 if level == "F3" else
                                                  min(len(ranked), rng.randint(2, 4)))
        sides = tuple(ranked[:max(1, n_side)])
    # RECONCILE THE REQUESTED SIDES AGAINST MEASURED REAL GLAZING. See
    # `_reconcile_sides`'s own docstring. `sides` itself (the CALLER's
    # value -- the city manifest's own `entry_side`/contagion fact) is
    # never rewritten; only the venting sides `plan_fire` actually receives
    # change, so `fire["sides"]` (what the sidecar and every downstream
    # reader -- soot skin, events, people, `fire_assembly_lib`'s street
    # bias -- consult) carries the physically-corrected answer while the
    # manifest keeps recording what really lit the building.
    venting_sides, note = _reconcile_sides(rects, sides, name=name)
    if note:
        print("[gac_fire] " + note)
    # ...AND SLIDE A COVERED-ELSEWHERE ORIGIN ONTO ITS OWN REAL GLAZING.
    # See `_nudge_origin_to_real`'s own docstring -- only ever lowers
    # `origin`, only when the origin storey itself has no real opening on
    # any venting side but one exists below it (`SM_Building_02`-style: the
    # requested band's own storey is above where the real windows stop).
    real_by_side = _real_storeys_by_side(rects, m)
    origin, origin_note = _nudge_origin_to_real(origin, venting_sides, real_by_side)
    if origin_note:
        print("[gac_fire] {0}: {1}".format(name, origin_note))
    fire = uf.plan_fire(info, level, rng, origin=origin, sides=venting_sides)
    # `deck_z` HAS TO RIDE ON `fire`, NOT ON `m`. `burn_building` never sees
    # this `info`/`mass` dict again -- it rebuilds its own `ctx["info"]` from
    # `quake_flow.describe(style, placements, ...)`, which measures a fresh
    # mass box off the SLICED PIECES through `_mass_specs`/`register_style`
    # and knows nothing about `deck_z` (measured, `_roof_seat_probe.py`:
    # SM_Building_03 F1's `dress_roof_urban` read `m.get("deck_z", ...)` off
    # that rebuilt mass and silently fell back to `top` every time). `fire`,
    # by contrast, IS the same dict object `burn_building` copies verbatim
    # into `ctx["fire"]` (`fire=fire` all the way through `burn_gac`), so
    # stashing it here is the one channel that actually reaches the roof
    # recipes. The kit path never sets this key (`plan_fire`'s own return
    # never does), so `ctx["fire"].get("deck_z", m["top"])` there is
    # unchanged.
    fire["deck_z"] = m.get("deck_z")
    fire["deck_note"] = m.get("deck_note")
    # the measured façade planes travel with the fire plan too: wall stamps
    # (spall, halos, bars) are placed on THEM on the sliced path, not on the
    # piece frame's bbox face (`urban_fire._stamp_pt`)
    fire["planes"] = dict(planes or {})
    # AND THE FOOTPRINTS RIDE ON `fire` TOO — same channel, same reason
    # (`burn_gac` hands this exact dict object to `burn_building(fire=...)`,
    # and a plain kit `plan_fire` return never carries this key, which is
    # what keeps `urban_fire._plate`'s fallback box the kit path's only path).
    fire["footprints"] = footprints
    # THE SYNTHETIC-OPENINGS FALLBACK NEEDS THE FIRE'S OWN BAND. See the
    # block comment above `SYN_BAY_PITCH_M` and `openings_provider`'s own
    # docstring: a side in `fire["sides"]` that the real `rects` never put a
    # single opening into, anywhere across `fire["storeys"]`, gets a
    # synthetic bay-window grid instead of silently starving `plan_events`.
    provider = openings_provider(rects, m, planes=planes,
                                 band=(fire["sides"], fire["storeys"]))
    # Recorded for offline verification only (`tools/_gac_starved_probe.py`)
    # -- never reaches the sidecar; see `openings_provider`'s own docstring
    # for why not. Every synthetic OPENING's own `e["synthetic"]` DOES reach
    # the sidecar (`fire_bake._E_KEYS`) -- this is only the side-level
    # summary, plus the top-storey-fallback notes, if any fired.
    fire["synthetic_sides"] = list(provider.synthetic_sides)
    if provider.synthetic_notes:
        print("[gac_fire] {0}: synthetic-opening top-storey fallback: {1}"
              .format(name, "; ".join(provider.synthetic_notes)))
    ctx0 = {"info": info, "fire": fire, "rng": rng, "tag": tag,
            "soot_openings": provider}
    events = spl.plan_events(ctx0, uf._severity)
    heavy = 1.0
    for rname, kw in uf.LADDER.get(btype, {}).get(level, []):
        if rname == "smoke_stain":
            heavy = float((kw or {}).get("heavy", 1.0))
    # `glass=True` HARDENS THE ALPHA INTO A FILM, which is what a curtain
    # wall's soot is. `urban_fire._skin_of` already keys it off
    # `ctx["info"]["type"] == "rc_glass"` for a kit building; this is the
    # pre-slice skin's own copy of that decision, on the type this function
    # just derived. A no-op on the `gac` path, whose height rule never
    # returns `rc_glass`.
    # THE BURN ZONE REACHES THE PRE-SLICE ATLASES TOO. `r_partial_collapse`
    # writes `fire["burn_zone"]` at run time, which is after this skin has
    # been baked into the unique atlases — so on a GAC building the scorch
    # round the hole landed only on the per-piece (tiled) bakes and the
    # brick beside the hole stayed pristine (third review, 2026-08-30).
    # `plan_partial_collapse` is pure arithmetic on `info`/`fire`, so the
    # plan is made HERE, its zone handed to the skin, and the plan itself
    # handed on (`collapse_plan`) so `burn_gac` fits out the same storeys.
    burn_zone, collapse_plan, kw_pc = None, None, None
    for _rn, _kw in uf.LADDER.get(btype, {}).get(level, []):
        if _rn == "partial_collapse":
            kw_pc = dict(_kw or {})
    if kw_pc is not None:
        from . import fire_collapse as fcol
        try:
            collapse_plan = fcol.plan_partial_collapse(
                {"info": info, "fire": fire, "tag": tag}, **kw_pc)
            burn_zone = fcol.burn_zone_rects(collapse_plan, m)
        except Exception as exc:                          # pragma: no cover
            print("[gac_fire] burn-zone plan failed ({0}); the atlases get "
                  "the plume only".format(exc))
    sk = spl.skin(ctx0, events, np.random.default_rng(spl.event_seed(ctx0) ^ 0x5EED),
                  finish=fire.get("finish") or "char",
                  glass=(btype == "rc_glass"), duration_scale=heavy,
                  burn_zone=burn_zone)
    if verbose:
        print("[gac_fire] {0} {1}: {2:.0f} x {3:.0f} x {4:.0f} m, {5} storey(s), "
              "{6} window island(s), origin st{7} band {8}-{9} on {10}, {11}"
              .format(name, level, m["W"], m["D"], m["top"] - m["z0"], n_st,
                      provider.count, fire["origin"], fire["storeys"][0],
                      fire["top"], "/".join(fire["sides"]),
                      spl.summarise(events)))
    sooted = {}
    overlay_prebaked = set()
    if mesh is not None and events:
        sooted = bake_atlases(stage, cell, mesh, sk, m,
                              out_dir or spl.OUT_DIR, verbose=verbose)
        # MDL PACKS ALSO GET THE OVERLAY ROUTE (`PACKS[kind]["soot"] ==
        # "overlay"`, today only "aec"). `bake_atlases` above still runs
        # first and unconditionally — it correctly handles any material on
        # this pack that DOES resolve a readable base map (measured: a
        # handful of non-brick AEC materials do) and is a byte-identical
        # no-op for every other pack, so this never changes what `gac`/`dtc`
        # ship. See `overlay_soot`'s own docstring for the mechanism.
        if (pack.get("soot") == "overlay"
                and os.environ.get("GF_MDL_OVERLAY", "0").strip() == "1"):
            _ovl_sooted, overlay_prebaked = overlay_soot(
                stage, cell, mesh, sk, m, out_dir or spl.OUT_DIR, tag=tag,
                verbose=verbose)
    return {"name": name, "style": style, "src": src, "grid": g,
            "measured": measured, "rects": rects, "mass": m, "info": info,
            "btype": btype, "fire": fire, "provider": provider,
            "collapse_plan": collapse_plan,
            "events": events, "skin": sk, "mesh": mesh, "sooted": sooted,
            "overlay_prebaked": overlay_prebaked,
            "heavy": heavy, "planes": planes, "footprints": footprints,
            "kind": kind, "asset": asset, "url": url, "scale": scale,
            "trim_note": trim_note}


def burn_gac(stage, cell, name, level, rng, nrng, mats, tag, flow_root=None,
             mat_cache=None, ssf=1.0, origin=None, sides=None,
             use_baked_kit=True, out_dir=None, verbose=True):
    """Place GAC asset `name` under `cell`, plan its fire, bake its soot into
    its atlases, slice it, and run the ladder. Returns `burn_building`'s ctx
    with `ctx["gac"]` = {grid, events, n_pieces, n_atlases, n_rebound, ...}.

    `cell` must already exist (an Xform the caller positioned); everything
    is authored in the cell's frame, exactly as the kit benches do.
    """
    from detail import gac_slice as gsl, gac_storey_slice as gss, kit_bake as kb
    from detail import urban_building as ub
    from . import urban_fire as uf

    pre = prepare(stage, cell, name, level, rng, tag, origin=origin,
                  sides=sides, out_dir=out_dir, verbose=verbose)
    pre["mesh"] = None                      # the largest array; not needed now
    src, style, fire, events, provider, sooted = (
        pre["src"], pre["style"], pre["fire"], pre["events"],
        pre["provider"], pre["sooted"])
    # NAMES/N_ST, MOVED UP FROM THE FIT-OUT SECTION BELOW (which still uses
    # both). Needed here first: whether the recipe list can ever touch the
    # roof decides how far UP the region cut has to ring — see `region=`
    # below and `gac_storey_slice.plan_slice_budget`'s own `region["top"]`
    # docstring.
    names = set(n for n, _kw in uf.LADDER.get(pre["btype"], {}).get(level, []))
    n_st = len(pre["mass"]["levels"])
    # the kit
    # THE SYNTHETIC STYLE'S FAMILY FOLLOWS THE CONSTRUCTION TYPE.
    # `register_style` hardcoded family "01" (urm), so `quake_flow.describe`
    # typed every sliced building `urm` whatever `btype` said: rc ladders
    # ran with masonry piers, timber slabs and residential furniture, and
    # never the steel frame (Downtown pipeline review, 2026-08-30).
    _family = {"urm": "01", "rc": "02", "rc_glass": "05"}.get(pre["btype"], "01")
    # RING ONLY AS FAR UP AS SOMETHING CAN REACH. `fire["top"]` is the
    # highest storey the fire itself involves; a recipe that needs the
    # real roof regardless (a burn-through hole, either collapse) still
    # gets it, so `_deck_slab`/`r_roof_burnthrough`/`r_fire_collapse`
    # never find a merged wall piece where they expect a `role="roof"`
    # one. Nothing else in the ladder looks past `fire["top"]`, so
    # everything above it — the parapet/roof band included — collapses
    # to one piece instead of being ringed for no reader.
    roof_needed = (bool(fire.get("roof"))
                  or bool(names & {"roof_burnthrough", "fire_collapse"}))
    top = (n_st - 1) if roof_needed else int(fire["top"])
    # FORCE_REGULAR_GRID: an explicit, named override (`PACKS[kind]`),
    # not a side effect of `MIN_CONFIDENCE`. Building_12's measured grid
    # already fails confidence and falls back to `regular_grid` today,
    # but that is luck, not intent -- the user asked for a fixed grid on
    # this asset ("You can just split it by fixed grid"), and this keeps
    # that true even if the confidence scoring changes later.
    force_regular = pre["asset"] in (
        PACKS[pre["kind"]].get("force_regular_grid") or ())
    # THIS FIRE PLAN'S OWN CUT, computed BEFORE the cache check (not just
    # before the live slice) — `region`/`force_regular`/`_family`/`style`
    # are exactly the fields `kit_bake.slice_signature` hashes, because
    # `region=` cuts a DIFFERENT physical shape for a different fire plan on
    # the SAME building (a different `origin`/`top` merges a different band
    # into one piece; different `sides` ring different elevations) — a kit
    # cached per building NAME ALONE would silently serve the wrong ring to
    # whichever fire plan asked second. See `kit_bake`'s module docstring,
    # KEYING.
    region = {"origin": fire["origin"], "top": top, "sides": fire["sides"]}
    kit_sig = kb.slice_signature(region=region, family=_family,
                                 force_regular=force_regular, style=style)
    if use_baked_kit and kb.have_kit(name, kit_sig):
        pls, g2, meas2 = kb.load_kit(stage, cell, name, ssf, kit_sig)
        if style not in ub.STYLES:
            gsl.register_style(g2, style, pieces_of=pls, family=_family)
        from pxr import UsdGeom
        UsdGeom.Imageable(stage.GetPrimAtPath(src)).MakeInvisible()
        if verbose:
            # THE LINE A CACHE-HIT PROOF GREPS FOR (`tests/test_kit_bake.py`
            # `_live_two_run_proof_full`) — the only externally visible sign
            # that this bake skipped `slice_to_kit` entirely.
            print("[gac_fire] {0}: loaded from kit cache (signature={1})"
                  .format(name, kit_sig))
    else:
        pls, g2, meas2 = gss.slice_to_kit(
            stage, src, cell, style, verbose=verbose, region=region,
            family=_family, force_regular=force_regular)
        if use_baked_kit:
            # SAVE-ON-SLICE: freeze this EXACT (name, kit_sig) so the next
            # bake of the same building under the same fire plan and the
            # same slicer vintage hits `load_kit` instead of re-cutting —
            # "every time we slice a new building it is stored so that we
            # don't have to do it again" (user, 2026-08-31). Best-effort:
            # `save_kit` never raises, so a save failure (a bad Nucleus path,
            # a disk error) cannot take this bake down with it — the live
            # `pls` above are already good regardless of whether this
            # succeeds. Skipped entirely when `use_baked_kit` is False: a
            # caller that opted OUT of reading the cache should not silently
            # populate it either.
            kb.save_kit(name, kit_sig, pre["url"], pre["scale"], style,
                       region=region, family=_family,
                       force_regular=force_regular, verbose=verbose)
    n_rebound = rebind_sooted(stage, pls, sooted) if sooted else 0
    if verbose:
        print("[gac_fire] {0}: {1} piece(s), {2} subset(s) rebound to sooted "
              "atlases".format(name, len(pls), n_rebound))
    # every subset already on a sooted copy is done; the subsets of TILED
    # atlases (and any material the pre-bake could not read) still go through
    # the kit's per-piece bake, with this building's own skin
    prebaked = set(str(v.GetPrim().GetPath()) for k, v in sooted.items()
                   if k not in ("_png", "_tiled") and hasattr(v, "GetPrim"))
    # MDL MATERIALS ARE "PREBAKED" TOO, EVEN THOUGH NOTHING WAS ACTUALLY
    # BAKED INTO THEM. `overlay_soot` (in `prepare`, above) already laid the
    # soot on these elevations as a translucent decal and left the source
    # MDL material bound and untouched — `pre["overlay_prebaked"]` is the
    # set of THOSE materials' own paths (still bound on the sliced pieces
    # unchanged, since nothing rebinds them), and folding it in here stops
    # `urban_fire._bind_soot`'s per-piece kit-style fallback from also
    # trying (and failing, into `_flat_diffuse`'s flat grey) to soot-bake
    # them a second time.
    prebaked |= set(pre.get("overlay_prebaked") or ())
    # ONLY THE STOREYS SOMEBODY CAN SEE INTO GET A FULL FIT-OUT. `damage_
    # windows` (below) burns every hot-side window in the fire's own storey
    # band out to a real, see-through hole on EVERY level -- unconditional
    # on `level`, unlike this block -- so there is always SOMETHING to back
    # a burning building's own windows with; the recipes block further down
    # keeps `expose_interior`'s own small, footprint-aware catch floor
    # backing those regardless. What THIS `fit_storeys` set gates is
    # `quake_flow.fit_interior`'s much heavier per-storey slab/column/
    # partition/furniture GRID, laid on the mass's `W x D` bounding box --
    # right on a kit's plain rectangle, wrong on an L-shaped or multi-tier
    # whole-asset footprint (user, 2026-08-31, reviewing `gac_
    # SM_Building_28_F4_o22_SEW_s219`: "this building is L shaped however,
    # it's interior is rectangular and so it looks weird ... For building's
    # who's insides are not gonna be shown (intact but burnt on the
    # outside) don't have any interior") -- and costs real build time and
    # prims for nothing when nobody can see it ("on the next launch I don't
    # want props that nobody can see", user 2026-08-30). The grid earns its
    # keep only where the shell is actually opened STRUCTURALLY: through a
    # burnt-through roof (the top three storeys, `burn_building`'s own
    # rule), where a collapse takes the shell away (`fire_collapse`: the
    # top two; `partial_collapse`: the band on the lost elevation), or —
    # below — where the ladder run for this exact (construction type,
    # level) includes one of those recipes at all (`urban_fire.
    # shows_interior`, the single computed source of truth `LADDER` itself
    # drives; see that function's own docstring). `names`/`n_st` were
    # computed above, before the slice, because the region cut needed them
    # first.
    fit_storeys = set()
    if fire.get("roof") and names & {"roof_burnthrough", "fire_collapse"}:
        fit_storeys |= set(range(max(0, n_st - 3), n_st))
    if "fire_collapse" in names:
        fit_storeys |= set(range(max(0, n_st - 2), n_st))
    if "partial_collapse" in names:
        # NOT `fire["storeys"]` — THE STOREYS THE COLLAPSE ACTUALLY OPENS.
        # The burning band runs from the fire's origin to the top of the
        # block (SM_Building_05 F5c: 15 storeys, band 4-18, 2,500+ static
        # fit-out prims), but `partial_collapse` only takes the shell away
        # from its OWN failure line up, and everything under that line is
        # behind an intact elevation exactly like the rest of the building —
        # the "props nobody can see" this whole block exists to avoid.
        # `plan_partial_collapse` is pure arithmetic on the same `fire` plan
        # and the same measured mass box, so the failure line can be worked
        # out HERE, before the slice, without authoring anything: `_kill`
        # walks `info["elements"]`, which `prepare` leaves EMPTY, so the
        # module-budget trim (the only step that can RAISE `s0`) cannot fire
        # and the line comes back at or below the one the recipe will use.
        # That makes this a superset of the storeys the recipe opens, never a
        # subset. Two storeys below the lowest lost one are kept as well: the
        # floor the wreckage lands on and the one under it are both in view
        # through the hole.
        from . import fire_collapse as fcol
        kw_pc = {}
        for _rn, _kw in uf.LADDER.get(pre["btype"], {}).get(level, []):
            if _rn == "partial_collapse":
                kw_pc = dict(_kw or {})
        try:
            _pc = pre.get("collapse_plan") or fcol.plan_partial_collapse(
                {"info": pre["info"], "fire": fire, "tag": tag}, **kw_pc)
            _lo = max(0, int(_pc["s0"]) - 2)
        except Exception as exc:                          # pragma: no cover
            print("[gac_fire] partial-collapse fit-out plan failed ({0}); "
                  "falling back to the whole burning band".format(exc))
            _lo = min(int(st) for st in fire["storeys"])
        fit_storeys |= set(range(_lo, n_st))
    # THE FULL GRID, ONLY WHEN THIS (CONSTRUCTION TYPE, LEVEL)'S OWN LADDER
    # ACTUALLY PUTS THE INTERIOR ON SHOW. Before 2026-08-31 this branch fired
    # for every non-collapse level up to F4 unconditionally — the fix for
    # "an F1-F4 building that never reaches the roof and never collapses
    # used to show [damage_windows'] hole onto NOTHING" (user review,
    # 2026-08-30) — but a full `fit_interior` grid was overkill for that:
    # its job was only ever to give `expose_interior`'s catch floor
    # something, and that recipe authors its own fallback plate (`_plate`,
    # footprint-aware) with `ctx["fit"]` completely empty. `uf.shows_
    # interior` restricts the actual GRID to levels whose ladder run
    # reaches a burnthrough or a collapse — see the comment above
    # `fit_storeys = set()`. The LOWEST four of the band, nearest `origin`,
    # unchanged: a band tall enough to reach the roof is already covered up
    # there by the roof/collapse rules above, so this rule's job is the
    # part of the chimney those rules miss.
    if level in ("F1", "F2", "F3", "F4") and uf.shows_interior(pre["btype"], level):
        fit_storeys |= set(sorted(int(st) for st in fire["storeys"])[:4])
    # EXPOSE_INTERIOR'S OWN CATCH FLOOR BACKS `damage_windows`' REAL HOLES ON
    # EVERY LEVEL, GRID OR NO GRID. `damage_windows` (below) burns a
    # hot-side band window out to a real, see-through hole regardless of
    # `level` — there is always something behind a burning building's own
    # windows to back, even on a level whose ladder never reaches a
    # burnthrough or a collapse (`fit_storeys` above may be empty for
    # exactly that reason now). `r_gut_interior`/`r_expose_interior` both
    # degrade cleanly to `ctx["fit"]` being empty — the former reports "0
    # consumed" and does nothing, the latter still authors its one
    # footprint-aware catch floor + rubble scatter at the top of the band
    # (`_plate`, never `fit_interior`'s bounding-box grid) — so keeping both
    # in the recipe list unconditionally costs nothing extra on a shell
    # that otherwise stays closed (a recipe LIST is what `burn_building`
    # takes in place of a level name; the fire plan itself is handed in as
    # `fire=`).
    recipes = list(uf.LADDER.get(pre["btype"], {}).get(level, []))
    have = set(n for n, _kw in recipes)
    if "gut_interior" not in have:
        recipes = recipes + [("gut_interior", {"frac": 0.4})]
    if "expose_interior" not in have:
        recipes = recipes + [("expose_interior", {})]
    ctx = uf.burn_building(stage, cell, style, pls, 0.0, 0.0, 0.0, recipes, rng,
                           nrng, mats, tag, flow_root=flow_root,
                           origin=fire["origin"], sides=fire["sides"],
                           mat_cache=mat_cache, events=events,
                           openings_fn=provider, soot_prebaked=prebaked,
                           fire=fire, skin=pre["skin"], fit_storeys=fit_storeys)
    n_glass = damage_windows(stage, ctx, pls, rects=pre["rects"],
                             mass=pre["mass"], sooted=sooted,
                             deny_mat=PACKS[pre["kind"]].get("glazing_material_deny"))
    n_atlas = len(set(id(v) for k, v in sooted.items() if k != "_png"))
    ctx["gac"] = {"grid": pre["grid"], "events": events, "n_pieces": len(pls),
                  "n_atlases": n_atlas, "n_rebound": n_rebound,
                  "n_glass": n_glass, "mass": pre["mass"], "rects": pre["rects"],
                  "skin": pre["skin"]}
    ctx["notes"].append("gac: {0} piece(s), {1} sooted atlas(es), {2} subset(s) "
                        "rebound, {3} window(s) burnt out / crazed on the band, "
                        "full fit-out grid on {4} storey(s) ({5})"
                        .format(len(pls), n_atlas, n_rebound, n_glass,
                                len(fit_storeys),
                                "visible through the roof / collapse" if fit_storeys
                                else "shell stays closed — {0} {1} never "
                                     "reaches a burnthrough/collapse; "
                                     "expose_interior's own catch floor "
                                     "still backs the band's real-hole "
                                     "windows".format(pre["btype"], level)))
    return ctx
