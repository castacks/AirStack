"""urban_fire_city — the CITY-SCALE burnable-set predicate and bake-kind
mapper for the 500 m urban fire (`scene_gen/_plans/urban_fire_city_plan.md`,
work item #3 of its `## 6. Work breakdown`).

Pure python, no `pxr` at import time (checked: this module's own top-level
imports are `os`, `random`, `sys` only) — every sibling `disaster.*` module it needs
(`gac_fire`, `kit_substitute`, `fire_bake`) is imported LAZILY, inside the
function that needs it, for the same reason `kit_substitute.py` does its own
lazy imports: `gac_fire` pulls in `numpy` at module scope, and importing this
file should stay cheap and Kit-free even though its callers eventually run
inside Kit.

WHAT THIS DOES NOT DO. It does not run the spread solve (`urban_fire_spread`,
work item #1) and it does not touch a stage. It answers exactly two
questions, per plan section 1:

  1. `typology_at` / `burnable` — is this placement even a candidate the fire
     is allowed to reach? (the district rule + the pack/route gates)
  2. `bake_kind` — if it IS a candidate, does `fire_bake` actually have a
     bake path for it?

and one assembly question, `damaged_manifest` / `entry_string` — once the
spread solve (elsewhere) has decided WHICH candidates actually ignite and
WHEN, turn that into the plan's JSON manifest schema (plan section 2) and
into `fire_bake.sh` entry strings.

THE SIX GATES, IN ORDER (plan section 1; gate 2 is NARROWER than it used to
be, gates 5 and 6 are NEW — see below). `burnable()` applies exactly these,
and stops at the first one that fails:

  1. `placement["category"] == "house"` — only building placements are
     candidates at all (street furniture, trees, vehicles, debris are not).
  2. `typology_at(layout, x, y)` is not `None` — off the block map entirely
     (a street, a park, an unzoned cell) refuses here.
  3. `kit_substitute.route(usd, W, D, H, btype)[0] in ("kit", "slice")` — a
     `('skip', reason)` from `route()` refuses here with route's own reason:
     Muyang DownTown (`unburnable()`) and a `same_art` MCE merged asset with
     no kit style inside `MAX_H_RATIO`/`MAX_AREA_RATIO` both land here.
  4. `bake_kind(usd, W, D, H, btype)` is not `(None, reason)` — the gate
     `route()` does NOT provide: `route()` says `'slice'` for every pack with
     real, unnamed parts alike (GAC, downtowncity, the AEC brownstones,
     `standalone/buildings/...`), but only GAC and downtowncity have a
     `fire_bake.KINDS` entry (`gac_fire.gac_fire`'s per-building bake). The
     rest are refused here, WITH A REASON — never silently dropped, the same
     discipline `kit_substitute.route`'s own docstring insists on.
  5. `_pack_blacklist_reason(kind, name)` is `None` (2026-08-31) — the
     resolved `gac`/`dtc` asset NAME is not one of `gac_fire.PACKS[kind]
     ["blacklist"]`'s prefixes (`dtc`: `"Carved_"`, `"Building_11"` — a
     user dislike, not a routing/bake defect, so it belongs after gates 3/4
     have already proven the asset CAN be baked). BLACKLISTED BUILDINGS ARE
     FIREBREAKS, NOT MISSING CANDIDATES: refusing them here means they never
     enter `urban_fire_spread.solve()`'s graph at all, so a fire routes
     AROUND one instead of being asked to light it — the same "not on the
     graph" discipline `solve()`'s own `blocked` set already uses for a
     no-fire district, reused here for a no-fire ASSET. Was previously
     enforced ONLY by a bench row picker (never by the city path), which is
     how three `dtc:Building_11` records reached a live bake manifest
     despite the pack table already naming it.
  6. `H` does not exceed `FIRE_MAX_H_M` (2026-08-31, user policy reviewing
     the live 500 m city: "don't let anything taller than the Amar tower be
     on fire"). This is a SECOND, independent height gate — NOT the same
     knob as the height-CLASS collapse cap in `disaster.urban_fire_spread`
     (`height_class` / `cap_level_for_class`, see "WHERE THE OLD GATE 2
     WENT" below): the class cap only ever restricts HOW BADLY an
     already-burning building may collapse; this gate decides whether
     something structurally this tall gets to be on fire AT ALL.
     `FIRE_MAX_H_M` defaults to 232.0 m, set just above `Amar_Tower`'s own
     measured 231.4 m (`_plans/dtc_buildings.json`) — Amar itself stays a
     legitimate candidate, while the genuine monsters above it (GAC's
     `SM_Building_16` at 312.0 m, `SM_Building_31` at 302.2 m) are refused,
     with a reason naming the measured height and the cap
     (`_height_cap_reason`). `H is None` (an unmeasured asset — the common
     case in a synthetic test that hands `burnable()` an empty `size_of`)
     never trips this gate: it refuses only a building this function
     actually KNOWS is too tall, the same "never invent a reason"
     discipline every gate here already follows. Like gate 5, a refusal
     here is a FIREBREAK — the building never enters `urban_fire_spread.
     solve()`'s graph at all, so it can never be picked as the origin and
     never lit as somebody else's neighbour either.

WHERE THE OLD GATE 2 WENT — HEIGHT CLASS, NOT A DISTRICT BAN (user policy,
2026-08-31, superseding the earlier blanket "no fire in skyscraper
districts"). Gate 2 used to ALSO refuse `typology_at(...) in
NO_FIRE_TYPOLOGIES` ("tower"/"highrise") — a tower or highrise district
could never catch fire at all. That blanket ban is LIFTED: every typology may
now ignite. What replaces it is a COLLAPSE CAP by height class, applied
downstream in `damaged_manifest` (not here — `burnable()` only decides
whether a placement is a candidate, never what LEVEL it ends up at):
`rowhouse`/`lowrise` may fully collapse (F6, F5c); `midrise`/`brick_midrise`
may only partially collapse (F5c, never F6); `tower`/`highrise` — grouped as
one `"skyscraper"` height class, both being the old ban list and their
measured height pools genuinely overlapping — may catch fire but never
collapse at all, not even partially (cap F5, never F5c/F6). See
`disaster.urban_fire_spread`'s "HEIGHT CLASS" docstring section
(`height_class`, `cap_level_for_class`, `enforce_roof_eligibility`) for the
full policy, including the SEPARATE, stricter "roof-affecting outcomes are
rare and low-rise/timber-only" rule `damaged_manifest` also applies there.

`NO_FIRE_TYPOLOGIES` is kept, now EMPTY, purely so `urban_fire_city_
launch_script.py`'s own re-assertion (`t in ufc.NO_FIRE_TYPOLOGIES`) — a
caller this work does not edit — keeps evaluating "no violation" instead of
breaking or raising. (`fire_city_dry_run.check_district_rule` is NOT that
caller: it is rewritten to check the new height-class invariant directly
and no longer reads this constant at all.) `SKYSCRAPER_TYPOLOGIES` is the
real, non-empty set the height-class policy above actually uses;
`no_fire_assets()` now reads from it.
"""

import os
import random
import sys

# ---------------------------------------------------------------------------
# scene_gen on sys.path — see kit_substitute.py's identical helper and its
# docstring for why: `from disaster import gac_fire` needs `scene_gen` (this
# file's grandparent directory) on `sys.path`, which holds when a launch
# script or Kit has already put it there, but NOT when this file is run
# directly (`python3 disaster/urban_fire_city.py`, the way `check()` below is
# verified) — there `sys.path[0]` is `.../scene_gen/disaster`.
# ---------------------------------------------------------------------------
_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _ensure_scene_gen_on_path():
    if _SCENE_GEN not in sys.path:
        sys.path.insert(0, _SCENE_GEN)


# ---------------------------------------------------------------------------
# The district rule — see the module docstring's "WHERE THE OLD GATE 2 WENT"
# ---------------------------------------------------------------------------
#: LEGACY name. EMPTY as of 2026-08-31: the blanket "no fire at all in a
#: tower/highrise block" this used to name is gone (replaced by the height-
#: class collapse cap in `disaster.urban_fire_spread` / `damaged_manifest`
#: below). Left as a real, empty tuple — not deleted, not renamed — purely
#: so `urban_fire_city_launch_script.py`'s own re-assertion (`t in ufc.
#: NO_FIRE_TYPOLOGIES`), a caller this work does not edit, keeps evaluating
#: "no violation" rather than breaking. `burnable()` never reads this, and
#: neither does `fire_city_dry_run.check_district_rule` any more (it now
#: checks the height-class invariant directly); `SKYSCRAPER_TYPOLOGIES`
#: below is the real, non-empty set the current policy uses.
NO_FIRE_TYPOLOGIES = ()

#: the SKYSCRAPER height class's district typologies (`disaster.
#: urban_fire_spread.TYPOLOGY_HEIGHT_CLASS`) — exactly the set
#: `NO_FIRE_TYPOLOGIES` held before the blanket fire ban was lifted. Grouped
#: together (rather than split "tower" into `mid_high`) because their
#: MEASURED height pools overlap — tower 44.7-131 m, highrise 103.7-312 m —
#: so splitting them by height instead of by typology name would be
#: ambiguous exactly where it matters; both are also "fire only, never any
#: collapse" under the new policy, so they share one class either way.
SKYSCRAPER_TYPOLOGIES = ("tower", "highrise")

#: Fallback basenames (no extension) of the `tower`/`highrise` `usds.buildings`
#: pools, current as of `config/asset_sets/urban_gac.yaml` (2026-08-30) — used
#: by `no_fire_assets()` ONLY when the config it is given carries no
#: `usds.buildings.tower`/`highrise` to read live (a synthetic/test config, or
#: one built from a different asset set). NOT the source of truth — see
#: `no_fire_assets()`, which always prefers a live read over this list, so a
#: preset change can never silently drift out of step with it.
FALLBACK_NO_FIRE_ASSETS = frozenset({
    # tower (config/asset_sets/urban_gac.yaml `tower+`)
    "office_tower", "stepped_tower", "slab_tower", "podium_highrise",
    "SM_Building_08", "SM_Building_09", "SM_Building_11", "SM_Building_12",
    "SM_Building_19", "SM_Building_21", "SM_Building_22", "SM_Building_26",
    "SM_Building_27", "SM_Building_28",
    # highrise (config/asset_sets/urban_gac.yaml `highrise`)
    "SM_Building_10", "SM_Building_13", "SM_Building_14", "SM_Building_15",
    "SM_Building_16", "SM_Building_17", "SM_Building_18", "SM_Building_20",
    "SM_Building_23", "SM_Building_31", "Amar_Tower",
})


def _asset_basename(url):
    """`"SM_Building_31"` from a `.usd`/`.usdc`/`.usda` URL or path."""
    base = os.path.basename(str(url))
    for ext in (".usd", ".usdc", ".usda"):
        if base.lower().endswith(ext):
            return base[:-len(ext)]
    return base


def no_fire_assets(config):
    """Basenames of every asset in `config`'s `tower`/`highrise`
    (`SKYSCRAPER_TYPOLOGIES`) `usds.buildings` pools — READ AT CALL TIME from
    the compiled preset (`config["usds"]["buildings"][typ]`, a list of plain
    URL strings or `{"usd": url, ...}` dicts — the same shapes
    `_normalize_usd_list` / `_pool_entries` accept), never a hardcoded
    snapshot: a preset that adds or retires a tower model is picked up on
    the next call with no code change.

    NAME KEPT FOR BACKWARD COMPATIBILITY; the MEANING has moved with the
    2026-08-31 policy — these two typologies are no longer fire-excluded
    (see the module docstring), just capped to the `skyscraper` height class
    (fire only, never any collapse). A caller that still wants "every
    basename in the skyscraper pools" (e.g. to identify them for the
    collapse cap, or for some other skyscraper-specific handling) gets
    exactly that; nothing calls this to gate ignition any more.

    Falls back to `FALLBACK_NO_FIRE_ASSETS` when neither pool is present or
    both are empty (a synthetic config, or an asset set with no such pools) —
    so a caller always gets a non-empty backstop rather than silently
    excluding nothing.
    """
    buildings = ((config or {}).get("usds") or {}).get("buildings") or {}
    out = set()
    for typ in SKYSCRAPER_TYPOLOGIES:
        for entry in (buildings.get(typ) or ()):
            url = entry.get("usd") if isinstance(entry, dict) else entry
            if url:
                out.add(_asset_basename(url))
    return frozenset(out) if out else frozenset(FALLBACK_NO_FIRE_ASSETS)


def typology_at(layout, x, y):
    """The typology name of the block containing `(x, y)`, or `None` when the
    point is not inside any block (a street, a park carved as its own
    typology already answers `"park"`, an unzoned cell). Iterates
    `layout["_typology_of"]` (`{(x0, y0, x1, y1): name}`, written by
    `districts.rezone_blocks`, `districts.py:2655`) exactly as plan section 1
    specifies — a point on more than one rect (should not happen; blocks
    don't overlap) returns whichever the dict iterates first.
    """
    typ_of = (layout or {}).get("_typology_of") or {}
    for (x0, y0, x1, y1), name in typ_of.items():
        if x0 <= x <= x1 and y0 <= y <= y1:
            return name
    return None


# ---------------------------------------------------------------------------
# The bake-kind gate — the one `route()` does not provide
# ---------------------------------------------------------------------------
def bake_kind(usd, W, D, H, btype):
    """`("gac"|"dtc"|"kit", name_or_style)` or `(None, reason)` for `usd`.

    In order:

      1. `usd` starts with a `gac_fire.PACKS[kind]["dir"]` prefix (`"gac"` for
         `gac_fire.GAC_DIR`, `"dtc"` for `gac_fire.DTC_DIR`) -> `(kind, name)`
         with `name` the asset's own basename, extension stripped per that
         pack's `["ext"]`. Checked BEFORE calling `route()` because
         `kit_substitute.pack_of()` has no idea these two Nucleus paths are
         special — to it they are both just `"other"`, the same bucket as an
         AEC brownstone.
      2. otherwise, `kit_substitute.route(usd, W, D, H, btype)`:
           - `('kit', style)` with a real `style` -> `("kit", style)` — a
             `bld_<style>_*.usd` kit build (`style` from its own filename) or
             a `same_art` MCE merged asset matched to a kit twin.
           - `('kit', None)` -> `(None, reason)` — an already-kit-shaped asset
             (`pack_of() == 'kit'`) whose filename does not match the bake's
             own naming convention, so no style name is recoverable for a
             `fire_bake.sh` manifest entry.
           - `('skip', reason)` -> `(None, reason)`, `route`'s own reason
             passed straight through (Muyang DownTown, a `same_art` refusal).
           - `('slice', None)` -> `(None, reason)` — the pack has real,
             unnamed parts (GAC and downtowncity are handled above already,
             so this is always AEC brownstones or `standalone/buildings/...`
             at this point) but `fire_bake.KINDS` has no bake path for it.
    """
    _ensure_scene_gen_on_path()
    from disaster import fire_bake as fb
    from disaster import gac_fire as gf
    from disaster import kit_substitute as ks

    u = str(usd)
    for kind, spec in gf.PACKS.items():
        if u.startswith(spec["dir"]):
            ext = spec["ext"]
            name = _asset_basename(u) if u.lower().endswith(ext.lower()) \
                else os.path.basename(u)
            return kind, name

    action, val = ks.route(u, W, D, H, btype=btype)
    if action == "kit":
        if val is None:
            return None, ("already a kit-shaped asset (pack_of()=='kit') but "
                           "no style name could be parsed from its filename "
                           "-- no fire_bake style name is available: "
                           "{0}".format(u))
        return "kit", val
    if action == "skip":
        return None, val
    # action == "slice": a pack with real, unnamed parts that isn't GAC or
    # downtowncity (both handled above) -- has no fire_bake.KINDS entry.
    return None, ("kit_substitute.route() says 'slice' (a pack with real, "
                  "unnamed parts) but fire_bake.KINDS={0} has no bake kind "
                  "for it -- AEC brownstones and standalone/buildings/... "
                  "land here and must be REFUSED, not silently dropped: "
                  "{1}".format(fb.KINDS, u))


# ---------------------------------------------------------------------------
# The burnable-set predicate
# ---------------------------------------------------------------------------
def _lookup_wdh(size_of, usd):
    """`(W, D, H)` for `usd` from `size_of`, or `(None, None, None)`.

    `size_of` may be a plain `{usd: (W, D, H)}` mapping (what the tests pass)
    or a callable `usd -> (W, D, H) | None` (what a host-side dry run wires a
    `SizeResolver.get` lookup through as, per the plan's own note that "the
    dry run will pass SizeResolver").
    """
    got = size_of(usd) if callable(size_of) else (size_of or {}).get(usd)
    if got is None:
        return None, None, None
    W, D, H = got
    return W, D, H


def _pack_blacklist_reason(kind, name):
    """`reason` (a non-empty string) if `name` is blacklisted in its OWN
    pack (`gac_fire.PACKS[kind]["blacklist"]`, a tuple of PREFIXES -- see
    that table's own comment: "the Carved_* blocks read as the same
    building ... never pick them for a fire row OR A CITY POOL"), else
    `None`.

    2026-08-31: the user blacklisted `Building_11` the same way ("I don't
    like that building"); `PACKS["dtc"]["blacklist"]` already carried both
    prefixes, but only a BENCH row picker ever consulted it -- the CITY
    path (`burnable()`/`kit_substitute.route()`) had no gate for it at all,
    so a `dtc:Building_11` placement sailed straight into the manifest.
    Reads `PACKS` LIVE from `gac_fire` (lazy import, same convention
    `bake_kind()` already uses) rather than duplicating the tuples here --
    a pack that adds or retires a blacklist entry is picked up with no code
    change on this side. `kind`s with no `"blacklist"` entry at all (`gac`,
    `kit`) always return `None`: prefix-matching a style name against a
    pack table that was never meant to cover it is not this function's
    job.
    """
    _ensure_scene_gen_on_path()
    from disaster import gac_fire as gf

    prefixes = (gf.PACKS.get(kind) or {}).get("blacklist") or ()
    name = str(name or "")
    for pfx in prefixes:
        if name.startswith(pfx):
            return ("asset blacklisted in its pack ({0!r} matches the "
                    "{1!r} prefix in PACKS[{2!r}]['blacklist'], gac_fire.py) "
                    "-- refused as a firebreak, never picked for ignition"
                    .format(name, pfx, kind))
    return None


# ---------------------------------------------------------------------------
# Gate 6: the max-fire-height cap (2026-08-31 user policy) — see the module
# docstring's gate-6 paragraph for why this is a SEPARATE knob from the
# height-CLASS collapse cap in `disaster.urban_fire_spread`.
# ---------------------------------------------------------------------------
#: Amar_Tower measures 231.4 m (`_plans/dtc_buildings.json`); the cap is set
#: just above it so Amar itself stays a legitimate fire candidate while
#: anything taller is refused. A plain module-level constant (not a
#: preset/config knob yet) so a caller that wants a different cap for a
#: different city can still override it by assignment before calling
#: `burnable()`, the same convention `ROOF_COLLAPSE_MAX_DEFAULT` uses.
FIRE_MAX_H_M = 232.0


def _height_cap_reason(H):
    """`reason` (a non-empty string) if `H` is a known height that exceeds
    `FIRE_MAX_H_M`, else `None`. `H is None` (unmeasured) is NEVER a
    reason to refuse — this gate only fires on a height it actually knows,
    the same discipline every other gate in this module already follows."""
    if H is None:
        return None
    h = float(H)
    if h <= FIRE_MAX_H_M:
        return None
    return ("{0:.1f} m tall -- taller than the fire-height cap "
            "(FIRE_MAX_H_M={1:.1f} m, set just above Amar_Tower's own "
            "measured ~231.4 m) -- refused as a firebreak, the same "
            "discipline gate 5's pack blacklist already uses: this building "
            "never enters urban_fire_spread.solve()'s graph at all, so it "
            "can never be the origin and never lit as anyone else's "
            "neighbour").format(h, FIRE_MAX_H_M)


def burnable(layout, placement, size_of):
    """`(True, record)` | `(False, reason)` — the six gates of plan
    section 1 (gate 5 added 2026-08-31, the pack blacklist; gate 6 added
    2026-08-31, the max-fire-height cap), applied in order; see the module
    docstring. `size_of(usd) -> (W, D, H)` (or a `{usd: (W, D, H)}` dict)
    is injected rather than measured here, so this function stays pure
    python.

    EVERY TYPOLOGY IS A CANDIDATE, INCLUDING `tower`/`highrise` — the
    blanket district-wide fire ban is LIFTED (2026-08-31 policy; see the
    module docstring's "WHERE THE OLD GATE 2 WENT"). What used to be gate
    2's second half (`typ in NO_FIRE_TYPOLOGIES`) is gone; a `tower`/
    `highrise` placement now only has to clear the SAME three gates every
    other typology does. The collapse cap that replaces the ban is a LEVEL
    policy (`disaster.urban_fire_spread.cap_level_for_class` /
    `enforce_roof_eligibility`), applied downstream in `damaged_manifest` —
    this function only ever decides candidacy, never level, so it has
    nothing to enforce here.

    On success, `record` carries the STATIC facts about the building — the
    geometry and the bake routing — that `damaged_manifest` later merges with
    the spread solve's TEMPORAL facts (level/origin/sides/timing):
    `{"usd", "x", "y", "yaw_deg", "z", "kind", "asset", "style", "typology",
    "W", "D", "H", "cell"}` (`asset`/`style`: whichever `bake_kind` did not
    return is `None`, matching plan section 2's schema, which carries both
    keys on every record). `cell` is `placement.get("prim_path")` — the prim
    `apply_placements` (`scene_generator.py:4161`) already wrote onto the
    placement dict by the time a real dry run gets here; a synthetic test
    placement supplies it directly.
    """
    if placement.get("category") != "house":
        return False, ("not a building placement (category={0!r})"
                        .format(placement.get("category")))

    x = float(placement.get("x_m", placement.get("x", 0.0)))
    y = float(placement.get("y_m", placement.get("y", 0.0)))
    # ORIGINAL (pre-crop, full-city) position, when the placement carries
    # one -- `tools/fc_dump_crop.py` stamps `x_m_orig`/`y_m_orig` on every
    # placement it keeps, alongside the RE-CENTRED `x_m`/`y_m` this
    # function keys everything else off. Falls back to `x`/`y` when absent
    # (every non-cropped dump/live build), so `x_orig == x` there and this
    # is a pure no-op. See `urban_fire_city_launch_script.py`'s
    # `FC_CROP_WINDOW`/`resolve_cell` for the one consumer that needs the
    # distinction: the ASSEMBLY launcher never translates its stage, so
    # matching a manifest record back to a live Kit prim has to compare
    # against the ORIGINAL coordinate, not the solve's re-centred one.
    x_orig = float(placement.get("x_m_orig", x))
    y_orig = float(placement.get("y_m_orig", y))
    typ = typology_at(layout, x, y)
    if typ is None:
        return False, ("outside every zoned block (street/park/unzoned) at "
                        "({0:.1f}, {1:.1f})".format(x, y))

    _ensure_scene_gen_on_path()
    from disaster import kit_substitute as ks

    usd = placement.get("usd")
    W, D, H = _lookup_wdh(size_of, usd)
    btype = placement.get("btype")
    action, val = ks.route(usd, W, D, H, btype=btype)
    if action not in ("kit", "slice"):
        return False, "kit_substitute.route refused: {0}".format(val)

    kind, name_or_reason = bake_kind(usd, W, D, H, btype)
    if kind is None:
        return False, name_or_reason

    blacklist_reason = _pack_blacklist_reason(
        kind, name_or_reason if kind in ("gac", "dtc") else None)
    if blacklist_reason is not None:
        return False, blacklist_reason

    height_reason = _height_cap_reason(H)
    if height_reason is not None:
        return False, height_reason

    record = {
        "usd": usd, "x": x, "y": y, "x_orig": x_orig, "y_orig": y_orig,
        "yaw_deg": float(placement.get("yaw_deg", 0.0)),
        "z": float(placement.get("z_m", placement.get("z", 0.0))),
        "kind": kind,
        "asset": name_or_reason if kind in ("gac", "dtc") else None,
        "style": name_or_reason if kind == "kit" else None,
        "typology": typ, "W": W, "D": D, "H": H,
        "cell": placement.get("prim_path") or placement.get("cell"),
    }
    return True, record


# ---------------------------------------------------------------------------
# The manifest -- static burnable() facts + the spread solve's own results
# ---------------------------------------------------------------------------
#: default for `damaged_manifest`'s `roof_collapse_max` -- see that
#: function's docstring and `disaster.urban_fire_spread`'s "HEIGHT CLASS"
#: section. Sized to a ~26-building city manifest (`downtown_fire_500`'s own
#: `--n`); a caller solving a very different scale should pass its own.
ROOF_COLLAPSE_MAX_DEFAULT = 2


def damaged_manifest(layout, placements, plan_records, seed_base,
                     roof_collapse_max=ROOF_COLLAPSE_MAX_DEFAULT):
    """`(manifest, refused)` -- plan section 2's JSON schema, assembled from
    `placements` (the city's own placement list; supplies geometry and
    `cell`) and `plan_records` (the spread solve's per-building results; one
    dict per building that solve considered, at least `{"i": <index into
    placements>, "level", "origin", "sides", "t_ignite_s", "age_s", "via",
    "how"}`, plus the `W`/`D`/`H` the solve was fed for it).

    `plan_records` are expected to already be the solve's BURNT set (the
    solve's own `buildings` input is burnable-only, per plan section 2), but
    `burnable()` is re-run here anyway rather than trusted -- the single
    source of truth for `kind`/`asset`/`style`/`typology` stays one function,
    and a building substituted or moved on `placements` since the solve ran
    is caught here instead of shipping a stale kind. A record whose index is
    out of range, or that `burnable()` now refuses, goes to `refused` WITH A
    REASON rather than being silently dropped -- the same discipline every
    gate in this module already follows.

    `seed = seed_base + 31 * i` where `i` is the record's ORDINAL POSITION IN
    THE OUTPUT MANIFEST (0-based) -- `fire_bake.parse_entry`'s own convention
    for its `index` argument, so bake `i` of this manifest reproduces the same
    draw a per-building re-bake of manifest entry `i` would.

    THIS IS THE AUTHORITATIVE GATE FOR THE HEIGHT-CLASS POLICY (`disaster.
    urban_fire_spread`'s "HEIGHT CLASS" docstring section) -- re-applied here
    rather than trusted from whatever `plan_records["level"]` says, for the
    same reason `burnable()` is re-run rather than trusted: a manifest built
    from a stale or hand-edited plan must still come out honest. Two layers,
    in order, using EACH RECORD'S OWN `typology` (the field `burnable()` just
    put on it, always present for a real gate-2 pass):

      1. THE RANK CAP (`urban_fire_spread.cap_level_for_class`) -- a `low`
         building may fully collapse, `mid_high` only partially (never F6),
         `skyscraper` never at all (cap F5).
      2. ROOF ELIGIBILITY (`urban_fire_spread.enforce_roof_eligibility`) --
         `F5c`/`F6` (`ROOF_LEVELS`) are further restricted to the `low`
         class ONLY, regardless of what the rank cap alone would still
         allow (this is what makes `mid_high`'s own F5c ceiling rarely
         reached in practice: a real fire there degrades one step under
         the rank cap, F6 -> F5c, and then a second step here, F5c -> F5).
      3. THE ROOF-OUTCOME SHARE BUDGET (`roof_collapse_max`, default
         `ROOF_COLLAPSE_MAX_DEFAULT`) -- roof collapse must stay RARE across
         the whole city, not just restricted to the timber class: at most
         `roof_collapse_max` records in the OUTPUT manifest may show a
         `ROOF_LEVELS` outcome at all. Selection is a DETERMINISTIC choice
         seeded off `seed_base` (so a given plan always produces the same
         manifest) that prefers the fire's own ORIGIN first (`via is None`,
         matching `fire_city_dry_run._enforce_target_f5c`'s "prefer the
         origin" convention) and then fills the rest of the budget with a
         seeded shuffle of the remaining eligible records; anyone left over
         degrades to `"F5"` -- never to F0, and never silently dropped from
         the manifest.

    Every degradation from either layer is recorded verbatim on the entry
    (`entry["level"]` is the FINAL, capped value); `entry["height_class"]`
    names which class did the capping, for a reader (the markdown report,
    a test) that wants to show the "before" without re-deriving it.
    """
    manifest, refused = [], []
    for rec in (plan_records or []):
        i = rec.get("i")
        if i is None or not isinstance(i, int) or not (0 <= i < len(placements)):
            refused.append({"i": i, "reason": ("plan record names an "
                                                "out-of-range placement "
                                                "index: {0!r}".format(i))})
            continue
        placement = placements[i]
        size_of = {}
        if rec.get("W") is not None:
            size_of = {placement.get("usd"): (rec.get("W"), rec.get("D"),
                                              rec.get("H"))}
        ok, result = burnable(layout, placement, size_of)
        if not ok:
            refused.append({"i": i, "usd": placement.get("usd"),
                            "reason": result})
            continue
        entry = dict(result)
        entry["i"] = i
        entry["level"] = rec.get("level")
        entry["origin"] = rec.get("origin")
        entry["sides"] = list(rec.get("sides") or [])
        entry["t_ignite_s"] = rec.get("t_ignite_s")
        entry["age_s"] = rec.get("age_s")
        entry["via"] = rec.get("via")
        entry["how"] = rec.get("how")
        entry["seed"] = int(seed_base) + 31 * len(manifest)
        manifest.append(entry)

    _apply_height_class_policy(manifest, seed_base, roof_collapse_max)
    return manifest, refused


def _apply_height_class_policy(manifest, seed_base, roof_collapse_max):
    """Mutates `manifest` IN PLACE: the rank cap, then roof eligibility,
    then the roof-outcome share budget -- see `damaged_manifest`'s own
    docstring for what each layer does and why they run in this order."""
    _ensure_scene_gen_on_path()
    from disaster import urban_fire_spread as ufs

    for entry in manifest:
        cls = ufs.height_class(typology=entry.get("typology"))
        entry["height_class"] = cls
        entry["level"] = ufs.cap_level_for_class(entry["level"], cls)
        entry["level"] = ufs.enforce_roof_eligibility(entry["level"], cls)

    if roof_collapse_max is None:
        return
    eligible = [e for e in manifest if e["level"] in ufs.ROOF_LEVELS]
    if len(eligible) <= roof_collapse_max:
        return
    origin_first = [e for e in eligible if e.get("via") is None]
    rest = [e for e in eligible if e.get("via") is not None]
    rng = random.Random(int(seed_base))
    rng.shuffle(rest)
    keep_ids = {id(e) for e in (origin_first + rest)[:roof_collapse_max]}
    for e in eligible:
        if id(e) not in keep_ids:
            e["level"] = "F5"


def entry_string(record):
    """`kind:name:level:origin:sides:seed` -- the `fire_bake.sh` manifest
    entry for one `damaged_manifest` record. `name` is `record["asset"]` for
    `gac`/`dtc`, `record["style"]` for `kit`. Round-trips through
    `fire_bake.parse_entry` (all six fields are always written explicitly,
    including a possibly-empty `origin`/`sides`, so `parse_entry` never falls
    back to its own default-seed formula and reads back exactly this
    record's `seed`)."""
    kind = record["kind"]
    name = record.get("asset") if kind in ("gac", "dtc") else record.get("style")
    origin = record.get("origin")
    sides = record.get("sides") or ()
    return "{0}:{1}:{2}:{3}:{4}:{5}".format(
        kind, name, record["level"],
        "" if origin is None else int(origin),
        ",".join(str(s) for s in sides),
        int(record["seed"]))


def _check_height_class_manifest_policy():
    """Host-side check of `damaged_manifest`'s height-class layer: the rank
    cap, roof eligibility, and the roof-outcome share budget, together --
    called by `check()`. Returns a list of complaints (empty = ok)."""
    _ensure_scene_gen_on_path()
    from disaster import gac_fire as gf

    usd = gf.GAC_DIR + "SM_Building_04.usd"
    layout = {"_typology_of": {
        (0.0, 0.0, 100.0, 100.0): "rowhouse",
        (100.0, 0.0, 250.0, 100.0): "midrise",
        (250.0, 0.0, 400.0, 100.0): "highrise",
    }}

    def _p(i, x, y):
        return {"category": "house", "usd": usd, "x_m": x, "y_m": y,
               "yaw_deg": 0.0, "z_m": 0.0,
               "prim_path": "/World/stage/generated/house_{0}".format(i)}

    # i0-3: rowhouse (low, roof-eligible); i4: midrise (mid_high); i5:
    # highrise (skyscraper). Every one of them names F6 -- the raw, uncapped
    # "full collapse" -- so what survives to the final manifest is entirely
    # the height-class policy's doing.
    placements = [_p(0, 50.0, 50.0), _p(1, 20.0, 20.0), _p(2, 70.0, 20.0),
                 _p(3, 20.0, 70.0), _p(4, 150.0, 50.0), _p(5, 300.0, 50.0)]

    def _plan(i, via):
        return {"i": i, "level": "F6", "origin": 0, "sides": ["S"],
               "t_ignite_s": float(i), "age_s": 12000.0, "via": via,
               "how": ("origin" if via is None else "attached")}

    plan_records = [_plan(0, None), _plan(1, 0), _plan(2, 0), _plan(3, 0),
                    _plan(4, 0), _plan(5, 0)]

    bad = []
    manifest, refused = damaged_manifest(layout, placements, plan_records,
                                         4242, roof_collapse_max=2)
    if refused:
        return ["_check_height_class_manifest_policy: unexpected refusals: "
                "{0}".format(refused)]
    by_i = {m["i"]: m for m in manifest}

    # mid_high / skyscraper: F6 is capped, then roof-degraded to F5 -- NEVER
    # a ROOF_LEVELS outcome, regardless of what the rank cap alone allows
    # (mid_high's own cap permits F5c; this is the policy that says it
    # should not actually show it).
    if by_i[4]["height_class"] != "mid_high" or by_i[4]["level"] != "F5":
        bad.append("midrise record should end at F5, got {0}".format(by_i[4]))
    if by_i[5]["height_class"] != "skyscraper" or by_i[5]["level"] != "F5":
        bad.append("highrise record should end at F5, got {0}".format(by_i[5]))

    # low: F6 is ELIGIBLE, but the SHARE BUDGET (2) still caps the COUNT
    # across the four rowhouse records -- exactly 2 keep F6, the other 2
    # degrade to F5, and the fire's own origin (i=0) is always kept.
    rowhouse = [by_i[i] for i in (0, 1, 2, 3)]
    for r in rowhouse:
        if r["height_class"] != "low":
            bad.append("rowhouse record {0} should be low, got {1}".format(
                r["i"], r["height_class"]))
    kept = [r["i"] for r in rowhouse if r["level"] == "F6"]
    demoted = [r["i"] for r in rowhouse if r["level"] == "F5"]
    if len(kept) != 2 or len(demoted) != 2:
        bad.append("roof_collapse_max=2 should keep exactly 2 of 4 eligible "
                  "rowhouse records at F6, got kept={0} demoted={1}".format(
                      kept, demoted))
    if by_i[0]["level"] != "F6":
        bad.append("the fire's own origin should always be kept as a roof "
                  "outcome ahead of the seeded shuffle")

    # deterministic given the same seed_base
    manifest2, _ = damaged_manifest(layout, placements, plan_records, 4242,
                                    roof_collapse_max=2)
    if [(m["i"], m["level"]) for m in manifest2] != \
            [(m["i"], m["level"]) for m in manifest]:
        bad.append("damaged_manifest's roof-share choice is not deterministic "
                  "for a given seed_base")

    # a bigger budget keeps all four eligible; a budget of exactly zero keeps
    # NONE, including the origin -- "at most N" means N, not "N plus the
    # origin for free".
    manifest_big, _ = damaged_manifest(layout, placements, plan_records, 4242,
                                       roof_collapse_max=4)
    if sum(1 for m in manifest_big if m["i"] in (0, 1, 2, 3)
          and m["level"] == "F6") != 4:
        bad.append("roof_collapse_max=4 should keep all four eligible "
                  "rowhouse records at F6")
    manifest_zero, _ = damaged_manifest(layout, placements, plan_records, 4242,
                                        roof_collapse_max=0)
    if any(m["level"] == "F6" for m in manifest_zero if m["i"] in (0, 1, 2, 3)):
        bad.append("roof_collapse_max=0 should demote every rowhouse record, "
                  "including the origin")
    return bad


# ---------------------------------------------------------------------------
# Host-side self-test
# ---------------------------------------------------------------------------
def check(verbose=True):
    """Host-side: no Kit, no Nucleus. `python3 disaster/urban_fire_city.py`."""
    bad = []
    _ensure_scene_gen_on_path()
    from disaster import fire_bake as fb
    from disaster import gac_fire as gf

    layout = {"_typology_of": {
        (0.0, 0.0, 100.0, 100.0): "lowrise",
        (100.0, 0.0, 250.0, 120.0): "brick_midrise",
        (250.0, 0.0, 400.0, 150.0): "tower",
    }}

    # --- typology_at ------------------------------------------------------
    if typology_at(layout, 50.0, 50.0) != "lowrise":
        bad.append("typology_at missed the lowrise block")
    if typology_at(layout, 300.0, 50.0) != "tower":
        bad.append("typology_at missed the tower block")
    if typology_at(layout, 900.0, 900.0) is not None:
        bad.append("typology_at returned a typology for a point in no block")

    # --- bake_kind, one representative case per pack ----------------------
    gac_usd = gf.GAC_DIR + "SM_Building_04.usd"
    kind, name = bake_kind(gac_usd, 31.5, 28.0, 42.3, "urm")
    if (kind, name) != ("gac", "SM_Building_04"):
        bad.append("bake_kind on a GAC asset: got {0}".format((kind, name)))

    dtc_usd = gf.DTC_DIR + "Building_09.usdc"
    kind, name = bake_kind(dtc_usd, 25.0, 25.0, 40.0, "rc")
    if (kind, name) != ("dtc", "Building_09"):
        bad.append("bake_kind on a downtowncity asset: got {0}".format((kind, name)))

    kit_usd = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/"
              "SEI-COA/scene_gen/assets/archetype/bld_apartment_DG0.usd")
    kind, name = bake_kind(kit_usd, 20.0, 20.0, 30.0, "urm")
    if (kind, name) != ("kit", "apartment"):
        bad.append("bake_kind on an already-kit asset: got {0}".format((kind, name)))

    same_art_ok = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/"
                  "SEI-COA/ModernCityEnvironment/Collected_Building01/"
                  "SM_MERGED_BP_MBuilding01.usd")
    kind, name = bake_kind(same_art_ok, 28.5, 18.5, 29.0, None)
    if (kind, name) != ("kit", "commercial_mid"):
        bad.append("bake_kind on a matchable same_art asset: got "
                  "{0}".format((kind, name)))

    same_art_bad = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/"
                    "SEI-COA/ModernCityEnvironment/Collected_Building02/"
                    "SM_MERGED_BP_MBuilding02.usd")
    kind, reason = bake_kind(same_art_bad, 60.0, 140.0, 302.0, None)
    if kind is not None or not reason:
        bad.append("bake_kind on an unmatchable 302 m same_art tower did not "
                  "refuse: {0}".format((kind, reason)))

    aec_usd = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/NVIDIA/Demos/"
              "AEC/Buildings/Brownstone/Brownstone_01.usd")
    kind, reason = bake_kind(aec_usd, 20.0, 15.0, 18.0, "urm")
    if kind is not None or "fire_bake.KINDS" not in (reason or ""):
        bad.append("bake_kind on an AEC brownstone did not give a "
                  "fire_bake.KINDS reason: {0}".format((kind, reason)))

    muyang_usd = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Muyang/DownTown/Assets/BG_Building_A.usd"
    kind, reason = bake_kind(muyang_usd, 45.0, 45.0, 90.0, None)
    if kind is not None or "unburnable" not in (reason or ""):
        bad.append("bake_kind on Muyang DownTown did not give an unburnable "
                  "reason: {0}".format((kind, reason)))

    # --- burnable(): the four gates ----------------------------------------
    p_gac = {"category": "house", "usd": gac_usd, "x_m": 50.0, "y_m": 50.0,
             "yaw_deg": 90.0, "z_m": 0.0, "prim_path": "/World/stage/generated/house_0_10"}
    ok, rec = burnable(layout, p_gac, {})
    if not ok or rec["kind"] != "gac" or rec["asset"] != "SM_Building_04" \
            or rec["style"] is not None or rec["typology"] != "lowrise":
        bad.append("burnable() on a lowrise GAC placement: got {0}".format((ok, rec)))

    p_not_house = dict(p_gac, category="tree")
    ok, reason = burnable(layout, p_not_house, {})
    if ok or "not a building placement" not in reason:
        bad.append("burnable() did not refuse a non-house category: {0}".format((ok, reason)))

    p_street = dict(p_gac, x_m=900.0, y_m=900.0)
    ok, reason = burnable(layout, p_street, {})
    if ok or "outside every zoned block" not in reason:
        bad.append("burnable() did not refuse a street placement: {0}".format((ok, reason)))

    # tower/highrise districts are burnable now (2026-08-31 policy: fire
    # only, capped downstream in `damaged_manifest` -- see below) --
    # `burnable()` itself no longer discriminates by district at all.
    p_tower = dict(p_gac, x_m=300.0, y_m=50.0)
    ok, rec = burnable(layout, p_tower, {})
    if not ok or rec["typology"] != "tower" or rec["kind"] != "gac":
        bad.append("burnable() should now accept a tower-block placement: "
                  "got {0}".format((ok, rec)))

    p_muyang = dict(p_gac, usd=muyang_usd, x_m=60.0, y_m=60.0)
    ok, reason = burnable(layout, p_muyang, {})
    if ok or "route refused" not in reason or "unburnable" not in reason:
        bad.append("burnable() did not refuse Muyang DownTown: {0}".format((ok, reason)))

    p_aec = dict(p_gac, usd=aec_usd, x_m=170.0, y_m=70.0)
    ok, reason = burnable(layout, p_aec, {})
    if ok or "fire_bake.KINDS" not in reason:
        bad.append("burnable() did not refuse an AEC brownstone: {0}".format((ok, reason)))

    # --- gate 5: the pack blacklist (2026-08-31) ----------------------------
    p_b11 = dict(p_gac, usd=gf.DTC_DIR + "Building_11.usdc", x_m=150.0, y_m=60.0)
    ok, reason = burnable(layout, p_b11, {})
    if ok or "blacklisted" not in reason or "Building_11" not in reason:
        bad.append("burnable() did not refuse a blacklisted Building_11: "
                  "{0}".format((ok, reason)))

    p_carved = dict(p_gac, usd=gf.DTC_DIR + "Carved_04.usdc", x_m=150.0, y_m=60.0)
    ok, reason = burnable(layout, p_carved, {})
    if ok or "blacklisted" not in reason or "Carved_" not in reason:
        bad.append("burnable() did not refuse a blacklisted Carved_04: "
                  "{0}".format((ok, reason)))

    p_amar = dict(p_gac, usd=gf.DTC_DIR + "Amar_Tower.usdc", x_m=150.0, y_m=60.0)
    ok, rec = burnable(layout, p_amar, {})
    if not ok or rec["kind"] != "dtc" or rec["asset"] != "Amar_Tower":
        bad.append("burnable() should still accept a non-blacklisted dtc "
                  "asset: {0}".format((ok, rec)))

    if _pack_blacklist_reason("dtc", "Building_11") is None:
        bad.append("_pack_blacklist_reason missed Building_11")
    if _pack_blacklist_reason("dtc", "Carved_17") is None:
        bad.append("_pack_blacklist_reason missed a Carved_ prefix match")
    if _pack_blacklist_reason("dtc", "Building_09") is not None:
        bad.append("_pack_blacklist_reason false-positived on Building_09")
    if _pack_blacklist_reason("gac", "Building_11") is not None:
        bad.append("_pack_blacklist_reason should not touch gac (no "
                  "blacklist entry in PACKS['gac'])")

    # --- gate 6: the max-fire-height cap (2026-08-31) -----------------------
    # Amar_Tower itself (231.4 m, the height the cap is set just above)
    # must stay burnable -- the whole point of the +0.6 m margin.
    p_amar_h = dict(p_gac, usd=gf.DTC_DIR + "Amar_Tower.usdc",
                    x_m=150.0, y_m=60.0)
    ok, rec = burnable(layout, p_amar_h, {p_amar_h["usd"]: (42.3, 48.8, 231.4)})
    if not ok or rec["H"] != 231.4:
        bad.append("burnable() should accept Amar_Tower at its own measured "
                  "height (231.4 m), just under FIRE_MAX_H_M: {0}".format(
                      (ok, rec)))

    # a genuine monster (GAC's SM_Building_31, 302.2 m) must be refused, WITH
    # a reason naming both the measured height and the cap.
    p_tall = dict(p_gac, x_m=150.0, y_m=60.0)
    ok, reason = burnable(layout, p_tall, {p_tall["usd"]: (60.3, 142.2, 302.2)})
    if ok or "taller than the fire-height cap" not in reason \
            or "302.2" not in reason:
        bad.append("burnable() did not refuse a 302.2 m building over "
                  "FIRE_MAX_H_M: {0}".format((ok, reason)))

    # the boundary itself: exactly FIRE_MAX_H_M is allowed, one mm over is not.
    p_boundary = dict(p_gac, x_m=150.0, y_m=60.0)
    ok, _rec = burnable(layout, p_boundary,
                        {p_boundary["usd"]: (20.0, 20.0, FIRE_MAX_H_M)})
    if not ok:
        bad.append("burnable() should accept a building exactly at "
                  "FIRE_MAX_H_M")
    ok, reason = burnable(layout, p_boundary,
                          {p_boundary["usd"]: (20.0, 20.0, FIRE_MAX_H_M + 0.01)})
    if ok:
        bad.append("burnable() should refuse a building 1 cm over FIRE_MAX_H_M")

    # an UNMEASURED height (H=None, the common case for an empty size_of in
    # every OTHER test above) must never be refused by this gate -- it only
    # fires on a height it actually knows.
    if _height_cap_reason(None) is not None:
        bad.append("_height_cap_reason(None) should never refuse -- an "
                  "unmeasured height is not a known-too-tall height")
    if _height_cap_reason(FIRE_MAX_H_M) is not None:
        bad.append("_height_cap_reason at the cap itself should not refuse")
    if _height_cap_reason(FIRE_MAX_H_M + 0.01) is None:
        bad.append("_height_cap_reason just over the cap should refuse")

    # --- the district no longer gates candidacy, mutation-checked ----------
    # only the record's own `typology` field should change; a tower-block
    # placement is still just as burnable as a lowrise one.
    p_moved = dict(p_gac)
    ok_before, rec_before = burnable(layout, p_moved, {})
    p_moved["x_m"], p_moved["y_m"] = 300.0, 50.0   # into the tower block
    ok_after, rec_after = burnable(layout, p_moved, {})
    if not ok_before or not ok_after or rec_before["typology"] != "lowrise" \
            or rec_after["typology"] != "tower" \
            or rec_before["kind"] != rec_after["kind"]:
        bad.append("moving a burnable placement into the tower block should "
                  "only change its typology, not its candidacy: before={0} "
                  "after={1}".format((ok_before, rec_before),
                                     (ok_after, rec_after)))

    # --- entry_string round-trips through fire_bake.parse_entry ------------
    rec_full = {"kind": "gac", "asset": "SM_Building_04", "style": None,
               "level": "F4", "origin": 2, "sides": ["S", "E"], "seed": 1013}
    parsed = fb.parse_entry(entry_string(rec_full))
    if (parsed["kind"], parsed["name"], parsed["level"], parsed["origin"],
            parsed["sides"], parsed["seed"]) != (
                "gac", "SM_Building_04", "F4", 2, ("S", "E"), 1013):
        bad.append("entry_string round-trip (full) mismatch: {0}".format(parsed))

    rec_kit = {"kind": "kit", "asset": None, "style": "commercial_mid",
              "level": "F1", "origin": None, "sides": None, "seed": 55}
    parsed = fb.parse_entry(entry_string(rec_kit))
    if (parsed["kind"], parsed["name"], parsed["level"], parsed["origin"],
            parsed["sides"], parsed["seed"]) != (
                "kit", "commercial_mid", "F1", None, None, 55):
        bad.append("entry_string round-trip (no origin/sides) mismatch: {0}".format(parsed))

    # --- damaged_manifest: seeds, merge, and refusal reporting -------------
    placements = [
        dict(p_gac),
        dict(p_gac, usd=dtc_usd, x_m=150.0, y_m=50.0,
            prim_path="/World/stage/generated/house_1_11"),
        dict(p_gac, x_m=300.0, y_m=50.0,
            prim_path="/World/stage/generated/house_2_12"),   # in the tower block
    ]
    plan_records = [
        {"i": 0, "level": "F4", "origin": 0, "sides": ["S"], "t_ignite_s": 0.0,
         "age_s": 900.0, "via": None, "how": "origin"},
        {"i": 1, "level": "F2", "origin": 0, "sides": ["W"], "t_ignite_s": 300.0,
         "age_s": 600.0, "via": 0, "how": "radiation"},
        # a tower-block record naming F6 (full collapse) -- the height-class
        # policy must cap this to F5 (skyscraper: fire only, never any
        # collapse) rather than refuse the record outright.
        {"i": 2, "level": "F6", "origin": 0, "sides": ["N"], "t_ignite_s": 400.0,
         "age_s": 500.0, "via": 0, "how": "attached"},
        {"i": 99, "level": "F1", "origin": 0, "sides": (), "t_ignite_s": 10.0,
         "age_s": 10.0, "via": 0, "how": "spot"},
    ]
    manifest, refused = damaged_manifest(layout, placements, plan_records, 1000)
    if [m["i"] for m in manifest] != [0, 1, 2]:
        bad.append("damaged_manifest kept the wrong indices: {0}".format(
            [m["i"] for m in manifest]))
    if [m["seed"] for m in manifest] != [1000, 1031, 1062]:
        bad.append("damaged_manifest seeds are not seed_base + 31*i: "
                  "{0}".format([m["seed"] for m in manifest]))
    if len(refused) != 1 or refused[0]["i"] != 99:
        bad.append("damaged_manifest refused list should hold only the "
                  "out-of-range record now that tower is burnable, got "
                  "{0}".format(refused))
    tower_entry = manifest[2]
    if tower_entry["height_class"] != "skyscraper" or tower_entry["level"] != "F5":
        bad.append("damaged_manifest should cap the tower record's F6 down "
                  "to F5 (skyscraper: fire only, never any collapse), got "
                  "{0}".format(tower_entry))

    # --- height-class cap + roof eligibility + the roof-outcome share cap --
    bad.extend(_check_height_class_manifest_policy())

    # --- no_fire_assets: live read + fallback -------------------------------
    live_cfg = {"usds": {"buildings": {
        "tower": [{"usd": "a/b/office_tower.usdc"}],
        "highrise": ["a/b/SM_Building_99.usd"],
    }}}
    got = no_fire_assets(live_cfg)
    if got != frozenset({"office_tower", "SM_Building_99"}):
        bad.append("no_fire_assets live read mismatch: {0}".format(got))
    got = no_fire_assets({})
    if got != frozenset(FALLBACK_NO_FIRE_ASSETS) or "Amar_Tower" not in got:
        bad.append("no_fire_assets fallback mismatch: {0}".format(sorted(got)[:5]))

    if verbose:
        print("[urban_fire_city] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
    return bad


if __name__ == "__main__":
    raise SystemExit(1 if check() else 0)
