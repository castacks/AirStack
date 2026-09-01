"""tornado_city — the CITY-SCALE damageable-set predicate, height class,
T-level draw and manifest-record helpers for the urban tornado (stream C of
`scene_gen/_plans/urban_tornado_plan.md`, §2.3 "which buildings are
candidates", §2.6 "the urban ladder" and §4 "the city dry run").

Modelled directly on `disaster.urban_fire_city` — same "pure python, no pxr
at import time, every sibling `disaster.*` module imported LAZILY" shape,
same "six/four gates, each with its own reason, applied in order" discipline,
same "a refusal here is a FIREBREAK-equivalent, never a silent drop"
doctrine. Read that module's docstring first if this one is confusing; this
file assumes it.

WHAT THIS DOES NOT DO. It does not sample the intensity field (`disaster.
tornado.intensity_field`, plate-agnostic already) and it does not touch a
stage. It answers exactly three questions, per plan §2.3/§2.6:

  1. `damageable` — is this placement even a candidate the tornado ladder is
     allowed to reach? (the route/pack/height gates, §2.3 gates 1-4).
  2. `height_class_for` — which of the FOUR §2.6 caps classes (lowrise/
     midrise/highrise/tower) does it belong to, for the caps table stream L
     applies downstream (this module never applies the caps itself — it only
     names the class).
  3. `btype_for` / `level_for_intensity` — the construction type and the T0-T4
     level a given (jittered) intensity draws.

and one assembly question, `record` / `entry_string` — once a caller (the
city dry run, later a real launcher) has `i`, `kind`/`name`, geometry, wind
and a level, turn that into the plan §4 manifest record and a
`tornado_bake.sh`-shaped entry string.

WHY `height_class_for` DOES NOT REUSE `urban_fire_spread.height_class`'S
RETURN VALUE (only its PRECEDENT) -- `disaster.urban_fire_spread.
TYPOLOGY_HEIGHT_CLASS` / `height_class()` is a THREE-way scheme (`"low"`,
`"mid_high"`, `"skyscraper"` -- tower and highrise are DELIBERATELY merged
there, because their measured height pools overlap 44.7-131 m vs.
103.7-312 m and the fire ladder's own caps treat both identically: "fire
only, never any collapse"). Plan §2.6's caps table is a FOUR-way scheme
(lowrise / midrise / highrise / tower each with their OWN removed-fraction
and chunk-storey caps) -- a genuinely finer classification the fire module
cannot supply, not a naming difference. So `height_class_for` below reuses
`urban_fire_spread.height_class`'s PRECEDENT (typology name first, a
measured-H fallback second) rather than its return value: when the block's
own district TYPOLOGY name is literally one of this module's four class
names (true for 4 of the dump's 6 typologies -- `lowrise`, `midrise`,
`highrise`, `tower` name themselves), that name IS the class; a typology
this module does not carry a direct answer for (`rowhouse`, `brick_midrise`,
`park`, or off-block/`None`) falls back to the measured-H bands §2.3 states
verbatim: lowrise < 18 m, midrise 18-45, highrise 45-100, tower >= 100.
`urban_fire_spread` is still imported, LAZILY, to confirm it is importable
in this environment (the "via urban_fire_spread when importable" instruction
in the work order) even though its return value is never read -- see
`height_class_for`'s own docstring.

`TORNADO_MAX_H_M` IS ITS OWN KNOB, NOT `urban_fire_city.FIRE_MAX_H_M` REUSED
-- plan §0 item 2 is explicit that this is deliberate ("carried as its own
knob ... so the user can lift it later without touching fire"), and
`urban_fire_city._height_cap_reason` cannot be reused directly for a
DIFFERENT cap regardless: it reads the module-level `FIRE_MAX_H_M` constant
with no parameter to override, and multiple concurrent sessions/tests
monkeypatching a shared module global is exactly the kind of cross-stream
interference plan §5's ownership rules exist to prevent. `_height_cap_reason`
below is a local twin, same wording convention, parameterised on `max_h`.
"""

import os
import random
import sys

# ---------------------------------------------------------------------------
# scene_gen on sys.path — see `urban_fire_city.py`'s identical helper and its
# docstring for why: a lazy `from disaster import X` needs `scene_gen` (this
# file's grandparent directory) on `sys.path`, which holds under Kit/a launch
# script but not when this file is run directly (`python3
# disaster/tornado_city.py`, the way `check()` below is verified).
# ---------------------------------------------------------------------------
_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _ensure_scene_gen_on_path():
    if _SCENE_GEN not in sys.path:
        sys.path.insert(0, _SCENE_GEN)


# ---------------------------------------------------------------------------
# Gate 4: the max-tornado-height cap (plan §0 item 2 / §2.3 gate 4)
# ---------------------------------------------------------------------------
#: default 232.0 m — the SAME number `urban_fire_city.FIRE_MAX_H_M` uses (just
#: above Amar_Tower's own measured 231.4 m), but its OWN module constant, env-
#: overridable (`TORNADO_MAX_H_M`) independently of fire's cap. Read once at
#: import time; a caller that wants a different cap without an env var can
#: still override by assignment (the same convention `FIRE_MAX_H_M` uses).
TORNADO_MAX_H_M = float(os.environ.get("TORNADO_MAX_H_M", 232.0))


def _height_cap_reason(H, max_h):
    """`reason` (a non-empty string) if `H` is a known height that exceeds
    `max_h`, else `None`. `H is None` (unmeasured) is NEVER a reason to
    refuse — the same "never invent a reason" discipline `urban_fire_city.
    _height_cap_reason` documents; this is that function's twin, local
    because that one hardcodes `FIRE_MAX_H_M` with no parameter (see module
    docstring)."""
    if H is None:
        return None
    h = float(H)
    if h <= max_h:
        return None
    return ("{0:.1f} m tall -- taller than the tornado-height cap "
            "(TORNADO_MAX_H_M={1:.1f} m) -- refused as a firebreak-"
            "equivalent, the same discipline the pack blacklist already "
            "uses: this building never becomes a T1+ damage candidate, it "
            "stays intact in the scene".format(h, max_h))


# ---------------------------------------------------------------------------
# btype (construction type) -- quake_sliced.CONSTRUCTION for merged assets,
# quake_flow.FAMILY_TYPE (via kit_substitute.styles()) for bld_*_DG0 kit
# archetypes.
# ---------------------------------------------------------------------------
def btype_for(usd, H=None):
    """`"urm"` | `"rc"` | `"rc_glass"` | `None` for `usd`.

    Two cases, in order:

      1. `usd` is a literal on-disk kit archetype build (`kit_substitute.
         pack_of(usd) == "kit"`, a `bld_<style>_DG0.usd`-shaped path) — the
         style name is the `bld_<style>_DG0` stem (`kit_substitute.
         _kit_style_of`, the SAME parser `route()` itself uses), and its
         `type` is looked up straight off the LIVE style table
         (`kit_substitute.styles()`, itself populated from `quake_flow.
         FAMILY_TYPE` — see that function's own docstring for why the live
         table and not a frozen bake).
      2. everything else (GAC, downtowncity, AEC brownstones, standalone
         packs, and a `same_art` MCE merged asset BEFORE any kit
         substitution — this function is never told the substituted style,
         only the ORIGINAL `usd`) — `quake_sliced.construction_type(usd, H)`,
         the measured per-asset table with its own material/height fallback.

    A `same_art` asset that `damageable()` routes to a kit style is reported
    here via case 2 (its OWN merged-mesh construction type, not the matched
    style's), a known approximation: this function's job is "what is this
    building physically made of", and `construction_type`'s height/material
    fallback answers that for an MCE merged mesh reasonably even though it
    was never in `quake_sliced.CONSTRUCTION`'s table (built for GAC/DTC) —
    refine with a `same_art`-specific row if this ever needs to be exact.
    """
    _ensure_scene_gen_on_path()
    from disaster import kit_substitute as ks
    from disaster import quake_sliced as qs

    if ks.pack_of(usd) == "kit":
        style = ks._kit_style_of(usd)
        if style:
            entry = ks.styles().get(style)
            if entry is not None:
                return entry.get("type")
    return qs.construction_type(usd, H=H)


# ---------------------------------------------------------------------------
# height class -- FOUR classes, see the module docstring for why this is not
# urban_fire_spread.height_class()'s three-way return value.
# ---------------------------------------------------------------------------
#: a district typology name IS its own class when it is literally one of
#: these four (4 of the dump's 6 typologies: `lowrise`, `midrise`,
#: `highrise`, `tower` — see the module docstring). `rowhouse`,
#: `brick_midrise`, `park` and off-block/`None` are NOT in this dict and fall
#: through to the measured-H bands in `height_class_for`.
_DIRECT_TYPOLOGY_CLASS = {"lowrise": "lowrise", "midrise": "midrise",
                          "highrise": "highrise", "tower": "tower"}

#: §2.3's measured-H fallback bands, verbatim.
_H_BAND_MIDRISE_MIN = 18.0
_H_BAND_HIGHRISE_MIN = 45.0
_H_BAND_TOWER_MIN = 100.0


def height_class_for(H, typology=None):
    """One of `"lowrise"` / `"midrise"` / `"highrise"` / `"tower"` — the
    class that drives plan §2.6's caps table.

    `disaster.tornado_urban.height_class_for` (stream L, landed after this
    function was first written) when it exists — the SAME "switch to the
    landed sibling, keep the fallback guarded" rule `level_for_intensity`
    follows, extended here on the same reasoning even though the work order
    only named it for the level draw: `tornado_urban` is the ladder that
    actually APPLIES this class's caps downstream, so this dry run should
    report the same class it will use, not a locally-reasoned approximation
    of it. `tornado_urban.height_class_for`'s own docstring records why it is
    MORE careful than this function's fallback: it trusts a typology name
    only for the unambiguous LOW end (`urban_fire_spread.height_class`'s
    3-way "low" bucket, which sits entirely under this ladder's 18 m cut) and
    falls through to measured `H` everywhere else, because the fire module's
    "mid_high"/"skyscraper" buckets each straddle two of THIS table's four
    bands (mid_high covers `midrise` AND `brick_midrise`; skyscraper covers
    `tower` AND `highrise`) and there is no lossless map from three classes
    onto four.

    THE FALLBACK BELOW IS THE CRUDER READING THIS FUNCTION SHIPPED WITH
    BEFORE `tornado_urban` LANDED: a district typology name IS its own class
    when it is literally one of `_DIRECT_TYPOLOGY_CLASS` (true for 4 of the
    dump's 6 typologies), H bands otherwise. Left in, guarded, exactly as
    instructed ("switch to it and delete nothing") — it only runs when
    `tornado_urban` is absent or raises.
    """
    _ensure_scene_gen_on_path()
    tu = None
    try:
        from disaster import tornado_urban as tu
    except ImportError:
        tu = None
    if tu is not None and hasattr(tu, "height_class_for"):
        try:
            return tu.height_class_for(H, typology=typology)
        except Exception:
            pass

    if typology in _DIRECT_TYPOLOGY_CLASS:
        return _DIRECT_TYPOLOGY_CLASS[typology]
    h = float(H) if H is not None else 0.0
    if h < _H_BAND_MIDRISE_MIN:
        return "lowrise"
    if h < _H_BAND_HIGHRISE_MIN:
        return "midrise"
    if h < _H_BAND_TOWER_MIN:
        return "highrise"
    return "tower"


# ---------------------------------------------------------------------------
# kit damage-capability -- ROUND 2 gate (plan §7's "GATES UPDATE"). A lazy
# import probe, exactly like `height_class_for`/`level_for_intensity` above
# probe `tornado_urban` -- "switch to the landed sibling, keep the fallback
# guarded" applied to a sibling that is STILL BEING WRITTEN as this function
# runs (stream K, parallel to this file). `disaster.tornado_kit` does not
# exist on disk as of this function's own first draft; every kit record is
# `bakeable: false` until it lands, and picks up `bakeable: true` on the next
# run with NO restart and NO edit here, the moment the module is importable
# -- the same "no restart" property `level_for_intensity` documents.
# ---------------------------------------------------------------------------
def _kit_damage_capable():
    """`True` iff `disaster.tornado_kit` is importable right now.

    Deliberately a plain import probe, NOT a call into whatever the module
    exposes: stream K's own API (which recipes it runs, which styles it
    covers) is exactly the thing this file must not need to know about --
    `tornado_city`'s whole job (module docstring) is "is this building even
    a candidate", not "can the planner actually damage this specific
    style". A finer per-style check belongs in `tornado_kit`/`tornado_urban`
    themselves, not duplicated here against a moving target.
    """
    _ensure_scene_gen_on_path()
    try:
        import disaster.tornado_kit  # noqa: F401
    except ImportError:
        return False
    return True


# ---------------------------------------------------------------------------
# industrial (tilt-up / light-roof) gate — R11, `_plans/urban_tornado_plan.
# md` §8c: "we have industrial buildings and brownstones. Those could
# collapse right ... So show those." `disaster.tornado_collapse` is the
# SECOND real urban collapse class (the first is `tornado_urban.
# t_facade_collapse`, gated through the ordinary `kind in ("gac", "dtc",
# "kit", "slice")` machinery above); a `kind == "industrial"` record routes
# to THAT module instead, never the sliced piece-grid ladder — a Dmytro
# FactoryDistrict shed is one merged mesh with no element table to speak
# of, so it would otherwise fall through to the generic `route() ==
# "slice"` -> `kind="slice"`, `bakeable=False` dead end every AEC
# brownstone/`standalone/buildings/...` asset already lands in (the same
# branch `bake_kind`'s own docstring calls "a real pack with real, unnamed
# parts but no fire_bake.KINDS entry"). This gate intercepts it BEFORE that
# branch, the same way `urban_fire_city.bake_kind` intercepts a GAC/dtc
# path before ever calling `kit_substitute.route` at all.
# ---------------------------------------------------------------------------
#: 4 of the 11 Dmytro FactoryDistrict sheds (`config/asset_sets/urban_gac.
#: yaml`'s own `lowrise` pool comments; measured — `config/harvested/
#: standalone_buildings.json` — 25.1x25.1x11.1, 42.0x31.5x11.7,
#: 67.1x45.1x12.0, 41.1x41.1x16.0 m respectively) chosen for the bench's
#: industrial pocket (`_plans/urban_tornado_C3_notes.md`): a small square
#: one, a mid one, the largest footprint in the pack, and the tallest —
#: real, RESOLVING assets (the harvested manifest measured them live via
#: Kit, not a guess off the yaml comment alone).
#:
#: Matched by SUFFIX, not by the exact preset root, because the asset-set
#: yaml's own relative form (`"Dmytro/Assets/Game/FactoryDistrict/Meshes/
#: Building_TypeC_A.usd"`) and the harvested manifest's fully-resolved form
#: (`".../Library/Stages/Dmytro/Assets/Game/FactoryDistrict/Meshes/
#: Building_TypeC_A.usd"`) differ only in their ROOT prefix — a suffix
#: match is robust to whichever one a given caller's placement dict
#: happens to carry.
INDUSTRIAL_SHED_SUFFIXES = (
    "FactoryDistrict/Meshes/Building_TypeC_A.usd",
    "FactoryDistrict/Meshes/Building_TypeD_A.usd",
    "FactoryDistrict/Meshes/Building_TypeB_C.usd",
    "FactoryDistrict/Meshes/Building_TypeC_D.usd",
)


def is_industrial_shed(usd):
    """`True` iff `usd` is one of `INDUSTRIAL_SHED_SUFFIXES`'s four chosen
    sheds — a plain suffix match, no Nucleus resolve, no `pxr` import."""
    u = str(usd or "")
    return any(u.endswith(sfx) for sfx in INDUSTRIAL_SHED_SUFFIXES)


def _industrial_damage_capable():
    """`True` iff `disaster.tornado_collapse` is importable right now — the
    SAME plain import probe `_kit_damage_capable` uses for `tornado_kit`,
    same reasoning (this file's own job is "is this building even a
    candidate", never "can the planner actually damage this specific
    shed")."""
    _ensure_scene_gen_on_path()
    try:
        import disaster.tornado_collapse  # noqa: F401
    except ImportError:
        return False
    return True


# ---------------------------------------------------------------------------
# damageable() -- §2.3 gates 1-4, in order.
# ---------------------------------------------------------------------------
def damageable(placement, size_of, max_h=None):
    """`(ok, reason, route, kind, name, bakeable)` — plan §2.3 gates 1-4, in
    order, applied to one placement. Gate 5 (the intensity threshold, "i >=
    T1's lower cut after jitter") is NOT here — this function does not know
    about intensity at all, the same separation `urban_fire_city.burnable()`
    keeps from the spread solve: a caller samples `disaster.tornado.
    intensity_field` itself and only asks this function "is this building
    even a candidate", independent of where the track happens to be.

    `size_of(usd) -> (W, D, H) | None`, or a plain `{usd: (W, D, H)}` dict —
    identical contract to `urban_fire_city._lookup_wdh` (reused directly
    here rather than re-derived).

    In order:

      1. `placement["category"] == "house"` — the dump is already
         houses-only, but a synthetic test or a padded non-house placeholder
         (`fire_city_dry_run.load_placements_dump`'s own padding) must still
         refuse cleanly here, not raise.
      2. `btype_for(usd, H)` is computed FIRST (it needs only `usd`/`H`, not
         this gate's outcome) and handed to `kit_substitute.route(usd, W, D,
         H, btype)` — `('skip', reason)` refuses here with `route`'s own
         reason (Muyang DownTown, an unmatchable `same_art` MCE merge).
      2.5. R11 (§8c): if `route`'s own action was `'slice'` AND `usd` is
         one of `INDUSTRIAL_SHED_SUFFIXES`'s four chosen Dmytro
         FactoryDistrict sheds, this is `kind="industrial"` — a SEPARATE
         collapse class (`disaster.tornado_collapse`, not the sliced
         piece-grid ladder), `bakeable` iff that module imports
         (`_industrial_damage_capable`, the same plain-probe convention
         `_kit_damage_capable` uses for `kind == "kit"`). Intercepted
         BEFORE gate 3 below runs at all, so it never falls into that
         gate's generic "no fire_bake.KINDS entry" dead end the way an AEC
         brownstone does.
      3. `urban_fire_city.bake_kind(usd, W, D, H, btype)` resolves `(kind,
         name)` for gate 4's blacklist lookup. When `bake_kind` itself
         refuses (`kind is None`): if `route`'s own action was `'slice'`
         (AEC brownstones, `standalone/buildings/...` — packs with real,
         unnamed parts but no `fire_bake.KINDS` entry, since GAC/downtowncity
         are always resolved by `bake_kind`'s own PACKS-prefix check before
         it ever reaches this branch — see that function's docstring), this
         is NOT a tornado refusal: plan §2.3's own text is explicit that
         "the tornado ladder is removal on an element table and does not
         need the fire's soot bake" — the building is ACCEPTED with
         `kind="slice"`, `name` = its own asset basename
         (`urban_fire_city._asset_basename`), `bakeable=False`. When
         `route`'s action was `'kit'` instead (an already-kit-shaped asset
         whose filename does not match the bake's naming convention, so no
         style name is recoverable at all) there is no name to record a
         damage plan against, and this DOES refuse, with `bake_kind`'s own
         reason.
      4. pack blacklist: `urban_fire_city._pack_blacklist_reason(kind, name)`
         against the LIVE `gac_fire.PACKS` table (reused verbatim — the same
         `dtc: Carved_*, Building_11, Building_12` prefixes the fire ladder
         refuses). A no-op for `kind` values with no `"blacklist"` entry
         (`gac`, `kit`, and this module's own `"slice"` sentinel alike).
      5. the height cap: `_height_cap_reason(H, max_h or TORNADO_MAX_H_M)`.

    On success, `reason` is `None`. `route` is `route()`'s own action
    (`"kit"` or `"slice"`) — never the `("slice", None)`-vs-bakeable-false
    distinction gate 3 introduces, which is what `bakeable` is for.
    """
    if placement.get("category") != "house":
        return (False, "not a building placement (category={0!r})"
                .format(placement.get("category")), None, None, None, False)

    _ensure_scene_gen_on_path()
    from disaster import kit_substitute as ks
    from disaster import urban_fire_city as ufc

    usd = placement.get("usd")
    W, D, H = ufc._lookup_wdh(size_of, usd)
    btype = btype_for(usd, H)

    action, val = ks.route(usd, W, D, H, btype=btype)
    if action not in ("kit", "slice"):
        return (False, "kit_substitute.route refused: {0}".format(val),
                action, None, None, False)

    # R11 (§8c): an industrial shed is a single merged mesh routed to
    # `action == "slice"` by `kit_substitute.route` (it is neither `kit`
    # nor `same_art`) — intercepted HERE, before `bake_kind`, so it never
    # falls into that function's generic "no fire_bake.KINDS entry ->
    # kind='slice', bakeable=False" dead end. `name` is still the asset's
    # own basename (`urban_fire_city._asset_basename`, the same helper
    # every other kind in this function uses), so a record/refusal is
    # reportable by name like any other.
    if action == "slice" and is_industrial_shed(usd):
        kind = "industrial"
        name = ufc._asset_basename(usd)
        bakeable = _industrial_damage_capable()
        blacklist_reason = ufc._pack_blacklist_reason(kind, None)
        if blacklist_reason is not None:
            return False, blacklist_reason, action, kind, name, bakeable
        height_reason = _height_cap_reason(
            H, float(max_h) if max_h is not None else TORNADO_MAX_H_M)
        if height_reason is not None:
            return False, height_reason, action, kind, name, bakeable
        return True, None, action, kind, name, bakeable

    kind, name_or_reason = ufc.bake_kind(usd, W, D, H, btype)
    if kind is None:
        if action == "slice":
            # AEC brownstone / standalone/buildings/... -- a real pack with
            # real, unnamed parts and no fire_bake.KINDS entry, which the
            # tornado ladder does not need (plan §2.3). Accepted, not
            # bakeable (this round produces the PLAN for it too).
            kind = "slice"
            name = ufc._asset_basename(usd)
            bakeable = False
        else:
            # action == "kit" but bake_kind still refused: an already-kit-
            # shaped asset whose filename carries no recoverable style name
            # -- nothing to key a damage plan against, so this DOES refuse.
            return (False, "bake_kind refused: {0}".format(name_or_reason),
                    action, None, None, False)
    else:
        name = name_or_reason
        bakeable = True
        if kind == "kit":
            # ROUND 2 (`_plans/urban_tornado_plan.md` §7, stream P2's "GATES
            # UPDATE"): a `kit`-routed record is damage-capable for THIS
            # ladder only once `disaster.tornado_kit` (stream K, built in
            # parallel to this file) is importable -- see
            # `_kit_damage_capable`'s own docstring for why the generic
            # `kind is not None -> bakeable=True` rule above is WRONG for
            # `kit` specifically: it was inherited from `urban_fire_city.
            # bake_kind`'s FIRE sense of "bakeable" (a `fire_bake.sh`-shaped
            # manifest entry exists), which says nothing about whether the
            # TORNADO ladder has anywhere to apply a plan. Before this round
            # every kit record read `bakeable: true` regardless -- an
            # overclaim this dry run's own "damage-capable coverage" check
            # would have silently trusted.
            bakeable = _kit_damage_capable()

    blacklist_reason = ufc._pack_blacklist_reason(
        kind, name if kind in ("gac", "dtc") else None)
    if blacklist_reason is not None:
        return False, blacklist_reason, action, kind, name, bakeable

    height_reason = _height_cap_reason(
        H, float(max_h) if max_h is not None else TORNADO_MAX_H_M)
    if height_reason is not None:
        return False, height_reason, action, kind, name, bakeable

    return True, None, action, kind, name, bakeable


# ---------------------------------------------------------------------------
# level_for_intensity -- §2.6's T0-T4 cuts, jitter 0.06. `disaster.
# tornado_urban.level_for_intensity` is imported LAZILY, per call, so a
# module that lands mid-session is picked up with no restart -- see the
# module docstring and plan §5's ownership row for stream C: "imports
# tornado_urban.level_for_intensity lazily with a local fallback ... until L
# lands". THE FALLBACK STAYS EVEN AFTER L LANDS (guarded, never deleted) --
# plan's own rule for this stream.
# ---------------------------------------------------------------------------
#: §2.6's T0-T4 cuts, in `tornado._ladder`'s `((upper_limit, name), ...)`
#: shape: T0 < 0.10, T1 0.10-0.36, T2 0.36-0.56, T3 0.56-0.74, T4 >= 0.74.
#: UPDATED 2026-09-01 to match `tornado_urban._URBAN_CUTS` exactly, which
#: landed with this revised table (the plan's own §2.6 table carries the
#: same numbers) -- this fallback is only ever consulted when `tornado_
#: urban.level_for_intensity` is absent or raises (see `level_for_intensity`
#: below), so it stays a byte-for-byte mirror of the live table rather than
#: the value this module first shipped with.
_URBAN_CUTS = ((0.10, "T0"), (0.36, "T1"), (0.56, "T2"), (0.74, "T3"),
              (1.01, "T4"))

#: §2.6's own jitter, one draw per building applied to `i` before the cut
#: lookup — the same "jitter stops the levels from being contour lines"
#: mechanism `tornado._ladder` already documents for the suburb ladders.
URBAN_JITTER = 0.06

LEVELS = ("T0", "T1", "T2", "T3", "T4")


def _fallback_level_for_intensity(i, rng, jitter=URBAN_JITTER):
    """§2.6's cut table THROUGH `tornado._ladder` — reused, not
    reimplemented, so a jitter-formula change in the suburb ladder cannot
    silently drift away from this one (plan §5's "reuse `tornado._ladder`"
    instruction)."""
    _ensure_scene_gen_on_path()
    from disaster import tornado as tn

    return tn._ladder(_URBAN_CUTS, i, rng, jitter)


def level_for_intensity(i, rng, jitter=URBAN_JITTER):
    """`"T0"`..`"T4"` for intensity `i` — `disaster.tornado_urban.
    level_for_intensity` when it exists (stream L, landing separately; see
    module docstring), else the local fallback above. Any exception raised
    BY a landed `tornado_urban.level_for_intensity` (a signature mismatch
    while that module is still in flux) also falls back rather than
    propagating, so this dry-run tool never breaks on a half-landed sibling
    module — a caller that wants to know which path actually ran should
    diff this module's `LEVELS`/cuts against `tornado_urban`'s own directly.
    """
    _ensure_scene_gen_on_path()
    tu = None
    try:
        from disaster import tornado_urban as tu
    except ImportError:
        tu = None
    if tu is not None and hasattr(tu, "level_for_intensity"):
        try:
            return tu.level_for_intensity(i, rng, jitter=jitter)
        except Exception:
            pass
    return _fallback_level_for_intensity(i, rng, jitter)


# ---------------------------------------------------------------------------
# the manifest record + entry string (plan §4)
# ---------------------------------------------------------------------------
def record(i, cell, usd, kind, name, x, y, yaw, W, D, H, btype, height_class,
          intensity, level, wind, route, bakeable, seed):
    """One plan §4 manifest record — plain, JSON-serialisable fields, in the
    order that section lists them."""
    return {
        "i": int(i), "cell": cell, "usd": usd, "kind": kind, "name": name,
        "x": float(x), "y": float(y), "yaw": float(yaw),
        "W": None if W is None else float(W),
        "D": None if D is None else float(D),
        "H": None if H is None else float(H),
        "btype": btype, "height_class": height_class,
        "intensity": float(intensity), "level": level,
        "wind": dict(wind) if wind is not None else None,
        "route": route, "bakeable": bool(bakeable), "seed": int(seed),
    }


def entry_string(rec):
    """`tornado:<kind>:<name>:<level>:<bearing_deg rounded>:<seed>` — a NEW
    prefix (`"tornado"`), never `fire_bake.parse_entry`-compatible (that
    parser expects a `kind` in `fire_bake.KINDS` and a `gac:name:level:
    origin:sides:seed` shape with an origin/sides pair this record does not
    carry). Round-trips through `parse_entry` below instead."""
    kind = rec["kind"]
    name = rec.get("name")
    level = rec.get("level")
    wind = rec.get("wind") or {}
    bearing = wind.get("bearing_deg")
    seed = rec.get("seed")
    return "tornado:{0}:{1}:{2}:{3}:{4}".format(
        kind, name, level,
        "" if bearing is None else int(round(float(bearing))),
        int(seed))


def parse_entry(s):
    """The inverse of `entry_string` — `{"kind", "name", "level",
    "bearing_deg", "seed"}`. Raises `ValueError`, with the reason, on
    anything that is not exactly a `tornado:...` five-field entry — the same
    "never silently misparse" discipline `fire_bake.parse_entry` follows."""
    text = str(s)
    parts = text.split(":")
    if len(parts) != 6 or parts[0] != "tornado":
        raise ValueError(
            "not a tornado_city entry_string (expected 'tornado:kind:name:"
            "level:bearing_deg:seed', got {0!r})".format(text))
    _tag, kind, name, level, bearing, seed = parts
    return {
        "kind": kind, "name": name, "level": level,
        "bearing_deg": None if bearing == "" else int(bearing),
        "seed": int(seed),
    }


# ---------------------------------------------------------------------------
# skyscraper_exposure -- plan §7 R1's hard check: "every building with
# height class highrise or tower must sample i <= 0.55 (at most T2 envelope
# damage)". This is the ONE function the R1 hard check and `--tune-track`'s
# candidate scorer both call — a track search that scored candidates on a
# DIFFERENT exposure computation than the report's own gate could not be
# trusted (a candidate could look like it passes the search and then fail
# the report, or the reverse).
#
# LEAD REVIEW, 2026-09-01, TWO FIXES to the first version of this function:
#
# 1. THE PROTECTED SET WAS WRONG. `height_class in ("highrise", "tower")`
#    conflates two different things: `height_class_for`'s OWN four-way
#    scheme trusts a district TYPOLOGY name first (§2.3's "unambiguous low
#    end" rule) and only falls back to a measured-H band, `highrise` 45-100 m,
#    for anything else -- including a `brick_midrise`-TYPOLOGY building,
#    since `"brick_midrise"` is not one of the four direct-class names. The
#    GAC `brick_midrise` pool measures 38.6-87.9 m
#    (`_plans/gac_buildings.json`), so a 47-72 m building zoned into the
#    `brick_midrise` district -- correctly counted as `low_mid` composition
#    (R2's own bucket) -- was ALSO hatched/protected as a "skyscraper" by
#    the old rule, purely because the coarse H-band fallback's `highrise`
#    floor (45 m) has nothing to do with the EF scale's own high-rise
#    boundary. `_plans/urban_tornado_research.md` §1 (the WISE 2006 DOD
#    tables) is explicit: **DI 18 "Mid-Rise Building" is 5-20 storeys, DI 19
#    "High-Rise Building" is ">20 storeys"** -- the EF scale's OWN
#    engineering line between "mid-rise" (no collapse DOD, same ladder this
#    disaster already uses for BOTH classes) and "high-rise" sits at 20
#    storeys, not at some fraction of a coarse 4-band H fallback. 20
#    storeys at a standard ~3.7-4 m commercial floor-to-floor height is
#    ~75-80 m -- SKYSCRAPER_PROTECTED_MIN_H_M below is that conversion
#    (a reviewer's own round number off the doc's storey figure, not a
#    value WISE itself states in metres). The PROTECTED set is now: class
#    `"tower"` (a district's own zoning intent, kept regardless of a short
#    kit tower archetype's measured H -- `tower` typology buildings are
#    already capped harder than `highrise` ones in `_plans/
#    urban_tornado_plan.md` §2.6's table) OR measured `H >=
#    SKYSCRAPER_PROTECTED_MIN_H_M`. A 45-75 m building (`highrise` by the
#    OLD rule, "mid-rise" by the EF scale) is NOT protected -- it stays a
#    damageable mid-rise, which is also what keeps R2's bench from losing
#    most of its T3/T4 stock (protecting the whole 45-100 m band would gut
#    it, per the same review).
#
# 2. THE SAMPLE POINT WAS WRONG. Sampling `field` at the building's own
#    (x, y) CENTRE missed `SM_Building_16` standing in the core band: 84.5 x
#    56.9 m in plan, its centre read i=0.32 while its footprint's near
#    corner (measured directly against the manifest) read i=0.787 -- the
#    corridor's edge cut straight through a HALF of the building. A large
#    footprint's own centre is not a valid stand-in for "is this building in
#    the corridor"; the corner nearest the track centreline is, because it
#    is the closest any part of the structure gets. `to_track` (optional --
#    `tn.frame(tcfg)`'s own first return value, `(x, y) -> (along, cross)`)
#    turns this on: given a footprint (`W`/`D`/`yaw`), the four OBB corners
#    are checked and the field is sampled at whichever has the SMALLEST
#    `|cross|` (nearest the centreline) rather than at the centre point. A
#    caller with no track frame (`to_track=None`, the historical default,
#    kept for any caller that only ever had a bare `field` closure) falls
#    back to the centre sample -- WORSE than the corner sample (never more
#    conservative), so every real caller in this codebase now passes
#    `to_track`; see `tornado_city_dry_run.check_r1_skyscraper_exposure` and
#    `_evaluate_track`.
# ---------------------------------------------------------------------------
#: DI 18 (MRB, "mid-rise", 5-20 storeys) vs DI 19 (HRB, "high-rise", >20
#: storeys) -- `_plans/urban_tornado_research.md` §1, the WISE (2006) DOD
#: tables. 20 storeys at a standard ~3.7-4 m commercial floor-to-floor
#: height is ~75-80 m; 75 is the round number used here. NOT a value WISE
#: itself states in metres -- the storey count is the cited fact, the
#: metres conversion is this codebase's own estimate from it.
SKYSCRAPER_PROTECTED_MIN_H_M = 75.0


def _obb_corners(x, y, W, D, yaw_deg):
    """The four world-space corners of a `W` x `D` footprint centred at
    `(x, y)` and yawed `yaw_deg` about it -- the SAME formula `tools/
    tornado_city_dry_run._footprint_corners` uses (kept as a small local
    duplicate rather than an upward import: `disaster/*` does not import
    from `tools/*` anywhere else in this codebase, and this is four lines
    of trigonometry, not a lookup worth a cross-layer dependency for)."""
    import math
    a = math.radians(float(yaw_deg))
    ca, sa = math.cos(a), math.sin(a)
    hw, hd = float(W) / 2.0, float(D) / 2.0
    return [(x + ca * dx - sa * dy, y + sa * dx + ca * dy)
           for dx, dy in ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd))]


def is_protected_skyscraper(height_class, H):
    """`True` iff `height_class == "tower"` or `H >=
    SKYSCRAPER_PROTECTED_MIN_H_M` -- the ONE predicate `skyscraper_
    exposure`, the dry run's composition count, and the plan PNG's hatching
    must all share (lead review: "the protected set and the hatching must
    agree with each other"). `H is None` (unmeasured) never protects on its
    own -- only a `tower`-typology item can be protected with no H."""
    if height_class == "tower":
        return True
    return H is not None and float(H) >= SKYSCRAPER_PROTECTED_MIN_H_M


def skyscraper_exposure(records_or_placements, field, to_track=None):
    """`[{"x", "y", "sample_x", "sample_y", "height_class", "H", "i_raw",
    "name", "usd"}, ...]` — the RAW (un-jittered) exposure of every
    PROTECTED item (`is_protected_skyscraper`) in `records_or_placements`,
    sampling `field` at the footprint corner nearest the track centreline
    when `to_track` and a footprint (`W`/`D`) are available, else at the
    item's own `(x, y)` — see the section docstring above for both fixes.

    `field` is any `f(x, y) -> i in 0..1` callable — a `disaster.tornado.
    intensity_field(tcfg, plate, rng)` partial application (a closure that
    already carries the noise draw) in the dry run, or a plain test double
    here. THE RAW FIELD, NEVER A JITTERED PER-BUILDING DRAW: plan §7 R1 is
    explicit ("after zero jitter — use the raw field, not the per-building
    draw") because the jitter (`URBAN_JITTER`, +-0.06) exists to stop the
    T0-T4 LEVEL boundary from reading as a contour line on a merely-COSMETIC
    axis; it has no business laundering a structural-safety exposure check,
    where a jitter-dodged 0.56 reading `"safe"` at 0.50 is exactly the kind
    of near-miss the hard gate exists to catch.

    `to_track` is `tn.frame(tcfg)`'s own first return value,
    `(x, y) -> (along, cross)`. When given AND the item carries `W`/`D`
    (`yaw`/`yaw_deg` optional, default 0), the four OBB corners are probed
    and the one with the smallest `|cross|` is sampled instead of the
    centre — never a WEAKER test than the centre sample, so every real
    caller should pass this.

    `records_or_placements` accepts EITHER shape, so the tune-track search
    (which has not built a manifest yet — only the layout/placements list)
    and the report-time check (which has a manifest of §4 records) can share
    one function without either one reshaping its data first:

      * a plan §4 manifest record: `x`, `y`, `height_class`, `H`, `W`, `D`,
        `yaw` already present.
      * a raw placement/candidate dict: `x_m`/`y_m` (falling back to `x`/`y`
        if already renamed) for position, and EITHER `height_class` directly
        or `H` (+ optional `typology`) from which `height_class_for` derives
        it the same way the dry run's own `solve()` does.

    An item that is NOT protected (`is_protected_skyscraper` false) is
    dropped silently (this is an exposure list for the tall-building gate,
    not a general-purpose annotator). `name` prefers a resolved asset name
    (`record`'s own `"name"` field) and falls back to the raw `"usd"` path,
    so a placement that has not yet been through `damageable()` still
    reports something identifiable.
    """
    out = []
    for item in records_or_placements:
        x = item.get("x", item.get("x_m"))
        y = item.get("y", item.get("y_m"))
        if x is None or y is None:
            continue
        H = item.get("H")
        height_class = item.get("height_class")
        if height_class is None:
            height_class = height_class_for(H, typology=item.get("typology"))
        if not is_protected_skyscraper(height_class, H):
            continue

        sample_x, sample_y = float(x), float(y)
        W, D = item.get("W"), item.get("D")
        if to_track is not None and W is not None and D is not None:
            yaw = float(item.get("yaw", item.get("yaw_deg", 0.0)) or 0.0)
            best_abs_c = None
            for (cx, cy) in _obb_corners(float(x), float(y), float(W),
                                         float(D), yaw):
                _a, c = to_track(cx, cy)
                if best_abs_c is None or abs(c) < best_abs_c:
                    best_abs_c = abs(c)
                    sample_x, sample_y = cx, cy

        out.append({
            "x": float(x), "y": float(y),
            "sample_x": float(sample_x), "sample_y": float(sample_y),
            "height_class": height_class, "H": (None if H is None else float(H)),
            "i_raw": float(field(sample_x, sample_y)),
            "name": item.get("name") or item.get("usd"),
            "usd": item.get("usd"),
        })
    return out


#: plan §7 R1's own threshold: "every PROTECTED building must sample
#: i <= 0.55".
SKYSCRAPER_MAX_I = 0.55


# ---------------------------------------------------------------------------
# Host-side self-test
# ---------------------------------------------------------------------------
def check(verbose=True):
    """Host-side: no Kit, no Nucleus. `python3 disaster/tornado_city.py`."""
    bad = []
    _ensure_scene_gen_on_path()
    from disaster import gac_fire as gf

    def _p(usd, x=50.0, y=50.0, W=None, D=None, H=None, category="house"):
        return {"category": category, "usd": usd, "x_m": x, "y_m": y,
               "z_m": 0.0, "yaw_deg": 0.0, "prim_path": "/World/x"}

    def _sz(placement, W, D, H):
        """`size_of` for `damageable()` — that function reads geometry from
        `size_of`, never from the placement dict itself (same contract as
        `urban_fire_city.burnable`); this test helper builds the one-entry
        mapping from whatever `usd` the placement carries."""
        return {placement["usd"]: (W, D, H)}

    # --- height_class_for -- via tornado_urban.height_class_for when it is
    # importable (it is, as of this module's own second draft: stream L
    # landed while this file was being written), else the local fallback.
    # Either way: a "low" typology (rowhouse/lowrise) beats a contradicting
    # H, everything else resolves by H. See `height_class_for`'s own
    # docstring for exactly why the two implementations agree here.
    if height_class_for(90.0, "lowrise") != "lowrise":
        bad.append("height_class_for should trust a low typology over a "
                  "contradicting H (90 m)")
    if height_class_for(90.0, "rowhouse") != "lowrise":
        bad.append("height_class_for should trust rowhouse (also a fire "
                  "'low' typology) over a contradicting H (90 m)")
    if height_class_for(9.0, None) != "lowrise":
        bad.append("height_class_for H fallback: 9 m should be lowrise")
    if height_class_for(30.0, None) != "midrise":
        bad.append("height_class_for H fallback: 30 m should be midrise")
    if height_class_for(70.0, None) != "highrise":
        bad.append("height_class_for H fallback: 70 m should be highrise")
    if height_class_for(150.0, None) != "tower":
        bad.append("height_class_for H fallback: 150 m should be tower")

    # --- level_for_intensity: fallback used while tornado_urban absent ------
    rng = random.Random(1)
    lv = level_for_intensity(0.0, rng)
    if lv != "T0":
        bad.append("level_for_intensity(0.0) should be T0, got {0}".format(lv))
    lv = level_for_intensity(0.90, random.Random(1))
    if lv != "T4":
        bad.append("level_for_intensity(0.90) should be T4, got {0}".format(lv))

    # --- damageable(): gate 1, category -------------------------------------
    p = _p(gf.GAC_DIR + "SM_Building_04.usd", category="tree")
    ok, reason, route, kind, name, bakeable = damageable(
        p, _sz(p, 31.5, 28.0, 42.3))
    if ok or "not a building placement" not in reason:
        bad.append("damageable() did not refuse a non-house category")

    # --- gate 2: route (Muyang unburnable) ----------------------------------
    muyang = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Muyang/DownTown/Assets/BG_Building_A.usd"
    p = _p(muyang)
    ok, reason, route, kind, name, bakeable = damageable(
        p, _sz(p, 45.0, 45.0, 90.0))
    if ok or "route refused" not in reason or "unburnable" not in reason:
        bad.append("damageable() did not refuse Muyang DownTown: {0}".format(
            (ok, reason)))

    # --- gate 3/bakeable: an AEC brownstone (route='slice', no fire_bake.
    # KINDS entry) is ACCEPTED with bakeable=False, not refused -----------
    aec = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/NVIDIA/Demos/"
          "AEC/Buildings/Brownstone/Brownstone_01.usd")
    p = _p(aec)
    ok, reason, route, kind, name, bakeable = damageable(
        p, _sz(p, 20.0, 15.0, 18.0))
    if not ok or route != "slice" or kind != "slice" or bakeable:
        bad.append("damageable() should accept an AEC brownstone with "
                  "bakeable=False, got {0}".format(
                      (ok, reason, route, kind, name, bakeable)))

    # --- gate 4: the pack blacklist (a dtc:Building_11, a Carved_03) -------
    b11 = gf.DTC_DIR + "Building_11.usdc"
    p = _p(b11)
    ok, reason, route, kind, name, bakeable = damageable(
        p, _sz(p, 30.9, 35.4, 32.6))
    if ok or "blacklisted" not in reason or "Building_11" not in reason:
        bad.append("damageable() did not refuse blacklisted Building_11: "
                  "{0}".format((ok, reason)))

    carved03 = gf.DTC_DIR + "Carved_03.usdc"
    p = _p(carved03)
    ok, reason, route, kind, name, bakeable = damageable(
        p, _sz(p, 42.6, 44.4, 27.6))
    if ok or "blacklisted" not in reason or "Carved_" not in reason:
        bad.append("damageable() did not refuse blacklisted Carved_03: "
                  "{0}".format((ok, reason)))

    # --- gate 5/height cap: a 302 m SM_Building_31 refused, a 231.4 m Amar
    # accepted -----------------------------------------------------------
    sm31 = gf.GAC_DIR + "SM_Building_31.usd"
    p = _p(sm31)
    ok, reason, route, kind, name, bakeable = damageable(
        p, _sz(p, 60.3, 142.2, 302.2))
    if ok or "taller than the tornado-height cap" not in reason:
        bad.append("damageable() did not refuse a 302 m SM_Building_31: "
                  "{0}".format((ok, reason)))

    amar = gf.DTC_DIR + "Amar_Tower.usdc"
    p = _p(amar)
    ok, reason, route, kind, name, bakeable = damageable(
        p, _sz(p, 42.3, 48.8, 231.4))
    if not ok or kind != "dtc" or name != "Amar_Tower":
        bad.append("damageable() should accept Amar_Tower at 231.4 m: "
                  "{0}".format((ok, reason, kind, name)))

    # --- an accepted kit archetype (bld_office_wide_DG0) --------------------
    # `bakeable` here is `_kit_damage_capable()`'s OWN live answer, not a
    # fixed True/False -- round 2's gate update (module docstring): kit
    # records only become damage-capable once `disaster.tornado_kit` is
    # importable. Whichever way that probe answers on THIS machine right
    # now, `damageable()` must AGREE with it -- that agreement, not a
    # hardcoded expectation, is what this assertion checks (a hardcoded
    # `bakeable is False` would start failing the moment stream K's module
    # lands, for a reason that has nothing to do with a real regression).
    kit_usd = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/"
              "SEI-COA/scene_gen/assets/archetype/bld_office_wide_DG0.usd")
    p = _p(kit_usd)
    ok, reason, route, kind, name, bakeable = damageable(
        p, _sz(p, 40.0, 30.0, 35.0))
    if not ok or route != "kit" or kind != "kit":
        bad.append("damageable() should accept a kit archetype with "
                  "route/kind=='kit': {0}".format(
                      (ok, reason, route, kind, name, bakeable)))
    if bakeable is not _kit_damage_capable():
        bad.append("damageable() kit bakeable={0} disagrees with "
                  "_kit_damage_capable()={1}".format(
                      bakeable, _kit_damage_capable()))

    # --- skyscraper_exposure(): highrise/tower only, raw field, no jitter ---
    recs = [
        record(0, "/World/a", gf.GAC_DIR + "SM_Building_31.usd", "gac",
              "SM_Building_31", 100.0, 0.0, 0.0, 60.3, 142.2, 302.2, "rc",
              "tower", 0.61, "T2", None, "slice", True, 1),
        record(1, "/World/b", gf.GAC_DIR + "SM_Building_02.usd", "gac",
              "SM_Building_02", -50.0, 20.0, 0.0, 28.0, 14.4, 38.6, "urm",
              "midrise", 0.61, "T2", None, "slice", True, 1),
    ]
    field_calls = []

    def _field(x, y):
        field_calls.append((x, y))
        return 0.42

    exp = skyscraper_exposure(recs, _field)
    if len(exp) != 1 or exp[0]["height_class"] != "tower":
        bad.append("skyscraper_exposure() should keep only the tower "
                  "record, got {0}".format(exp))
    if field_calls != [(100.0, 0.0)]:
        bad.append("skyscraper_exposure() should sample the field only at "
                  "the highrise/tower item's own (x, y), got {0}".format(
                      field_calls))
    if exp and abs(exp[0]["i_raw"] - 0.42) > 1e-9:
        bad.append("skyscraper_exposure() i_raw should be the field's raw "
                  "value verbatim (no jitter applied), got {0}".format(exp))

    # a raw placement/candidate shape (H + typology, no height_class yet)
    # resolves the SAME way `height_class_for` does.
    placements = [{"x_m": 10.0, "y_m": 10.0, "H": 302.2, "usd": "u1"},
                 {"x_m": 20.0, "y_m": 20.0, "H": 20.0, "usd": "u2"}]
    exp2 = skyscraper_exposure(placements, lambda x, y: 0.9)
    if len(exp2) != 1 or exp2[0]["usd"] != "u1":
        bad.append("skyscraper_exposure() should resolve height_class from "
                  "H when not given, and drop the non-skyscraper item, got "
                  "{0}".format(exp2))

    # --- LEAD REVIEW FIX 1: a 45-75 m `highrise`-by-H-band building (the
    # GAC brick_midrise pool's own range, 38.6-87.9 m) is NOT protected --
    # `height_class == "highrise"` alone must no longer be enough. ---------
    if is_protected_skyscraper("highrise", 60.0):
        bad.append("is_protected_skyscraper(highrise, 60.0) should be "
                  "False -- a 60 m building is DI 18 'mid-rise' territory "
                  "(_plans/urban_tornado_research.md §1), not protected")
    if not is_protected_skyscraper("highrise", 80.0):
        bad.append("is_protected_skyscraper(highrise, 80.0) should be True "
                  "-- above SKYSCRAPER_PROTECTED_MIN_H_M")
    if not is_protected_skyscraper("tower", 33.0):
        bad.append("is_protected_skyscraper(tower, 33.0) should be True -- "
                  "class tower protects regardless of measured H (a kit "
                  "tower archetype)")
    if is_protected_skyscraper("highrise", None):
        bad.append("is_protected_skyscraper(highrise, None) should be "
                  "False -- unmeasured H never protects on its own")

    rec_mid_by_h = record(2, "/World/c", gf.GAC_DIR + "SM_Building_26.usd",
                          "gac", "SM_Building_26", 0.0, 0.0, 0.0, 14.3, 28.4,
                          60.0, "urm", "highrise", 0.61, "T2", None, "slice",
                          True, 1)
    exp3 = skyscraper_exposure([rec_mid_by_h], lambda x, y: 0.9)
    if exp3:
        bad.append("skyscraper_exposure() should drop a 60 m "
                  "'highrise'-by-H-band record (not protected under the "
                  "corrected rule), got {0}".format(exp3))

    # --- LEAD REVIEW FIX 2: corner sampling. A wide record straddling the
    # centreline (x=0) reads a HIGH i at its near corner even though its
    # own centre sits well clear of it. ------------------------------------
    def _to_track(x, y):
        # a trivial along/cross frame: cross is just x itself.
        return x, x

    def _field_by_cross(x, y):
        # a step field: 0.9 inside |cross|<=40, else 0.1 -- a wide
        # building's centre can sit outside this while a corner sits in.
        return 0.9 if abs(x) <= 40.0 else 0.1

    # x=50 (centre) sits just OUTSIDE the |cross|<=40 hot zone (field 0.1);
    # the near corner at x=50-42.25=7.75 sits well INSIDE it (field 0.9) --
    # exactly `SM_Building_16`'s own real fault (84.5 x 56.9 m, centre read
    # i=0.32 while a corner read i=0.787).
    wide_tower = record(3, "/World/d", gf.GAC_DIR + "SM_Building_16.usd",
                        "gac", "SM_Building_16", 50.0, 0.0, 0.0, 84.5, 56.9,
                        312.0, "rc", "tower", 0.2, "T1", None, "slice", True, 1)
    exp_no_track = skyscraper_exposure([wide_tower], _field_by_cross)
    exp_with_track = skyscraper_exposure([wide_tower], _field_by_cross,
                                         to_track=_to_track)
    if not (exp_no_track and exp_no_track[0]["i_raw"] < 0.55):
        bad.append("skyscraper_exposure() centre-only fallback should read "
                  "the LOW value at this building's own (x=50) centre, got "
                  "{0}".format(exp_no_track))
    if not (exp_with_track and exp_with_track[0]["i_raw"] >= 0.9):
        bad.append("skyscraper_exposure() with to_track should sample the "
                  "near corner (x=50-42.25=7.75, inside the |cross|<=40 hot "
                  "zone) and read the HIGH value the centre sample misses, "
                  "got {0}".format(exp_with_track))

    # --- entry_string round-trips through parse_entry ------------------------
    rec = record(3, "/World/x", gf.GAC_DIR + "SM_Building_04.usd", "gac",
                "SM_Building_04", 10.0, -5.0, 90.0, 28.4, 42.4, 48.0, "urm",
                "midrise", 0.61, "T3",
                {"bearing_deg": 224.7, "speed_frac": 0.6, "cross_frac": -0.2,
                 "over": False}, "slice", True, 4123)
    s = entry_string(rec)
    parsed = parse_entry(s)
    if (parsed["kind"], parsed["name"], parsed["level"], parsed["bearing_deg"],
            parsed["seed"]) != ("gac", "SM_Building_04", "T3", 225, 4123):
        bad.append("entry_string/parse_entry round-trip mismatch: {0} -> "
                  "{1}".format(s, parsed))

    rec_no_bearing = dict(rec, wind=None)
    parsed2 = parse_entry(entry_string(rec_no_bearing))
    if parsed2["bearing_deg"] is not None:
        bad.append("entry_string should write an empty bearing field when "
                  "wind is None, got {0}".format(parsed2))

    if verbose:
        print("[tornado_city] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
    return bad


if __name__ == "__main__":
    raise SystemExit(1 if check() else 0)
