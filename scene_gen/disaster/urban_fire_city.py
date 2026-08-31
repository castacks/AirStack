"""urban_fire_city — the CITY-SCALE burnable-set predicate and bake-kind
mapper for the 500 m urban fire (`scene_gen/_plans/urban_fire_city_plan.md`,
work item #3 of its `## 6. Work breakdown`).

Pure python, no `pxr` at import time (checked: this module's own top-level
imports are `os`, `sys` only) — every sibling `disaster.*` module it needs
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

THE FOUR GATES, IN ORDER (plan section 1). `burnable()` applies exactly these,
and stops at the first one that fails — a placement refused at gate 2 never
reaches `kit_substitute.route()` at gate 3, so a district-excluded tower is
never even asked whether it CAN be baked:

  1. `placement["category"] == "house"` — only building placements are
     candidates at all (street furniture, trees, vehicles, debris are not).
  2. `typology_at(layout, x, y)` is not `None` and not in `NO_FIRE_TYPOLOGIES`
     — off the block map entirely (a street, a park, an unzoned cell) or
     inside a skyscraper district refuses here. THE NO-FIRE RULE LIVES ON THE
     BLOCK, NOT ON THE ASSET — a lowrise-district GAC tower burns; the same
     asset inside a `tower`/`highrise` block does not.
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

`NO_FIRE_ASSETS` (via `no_fire_assets(config)`) is a BACKSTOP, not a fifth
gate of `burnable()`: it names the `tower`/`highrise` pool assets by their own
basename, for a caller (the dry run, the city launcher) that wants to keep an
asset out of IGNITION CONSIDERATION entirely — e.g. before feeding a
`buildings` list to `urban_fire_spread.solve` — even against a `bleed: 0.12`
re-roll that could otherwise plant a `tower`/`highrise` pool asset on a
lowrise block. `burnable()` itself does not need it: gate 2 already refuses
every placement actually standing in a `tower`/`highrise` block regardless of
which asset it is.
"""

import os
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
# The district rule
# ---------------------------------------------------------------------------
#: the skyscraper districts — see the module docstring's gate 2.
NO_FIRE_TYPOLOGIES = ("tower", "highrise")

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
    `usds.buildings` pools — READ AT CALL TIME from the compiled preset
    (`config["usds"]["buildings"][typ]`, a list of plain URL strings or
    `{"usd": url, ...}` dicts — the same shapes `_normalize_usd_list` /
    `_pool_entries` accept), never a hardcoded snapshot: a preset that adds or
    retires a tower model is picked up on the next call with no code change.

    Falls back to `FALLBACK_NO_FIRE_ASSETS` when neither pool is present or
    both are empty (a synthetic config, or an asset set with no such pools) —
    so a caller always gets a non-empty backstop rather than silently
    excluding nothing.
    """
    buildings = ((config or {}).get("usds") or {}).get("buildings") or {}
    out = set()
    for typ in NO_FIRE_TYPOLOGIES:
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


def burnable(layout, placement, size_of):
    """`(True, record)` | `(False, reason)` — the four gates of plan section 1,
    applied in order; see the module docstring. `size_of(usd) -> (W, D, H)`
    (or a `{usd: (W, D, H)}` dict) is injected rather than measured here, so
    this function stays pure python.

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
    typ = typology_at(layout, x, y)
    if typ is None:
        return False, ("outside every zoned block (street/park/unzoned) at "
                        "({0:.1f}, {1:.1f})".format(x, y))
    if typ in NO_FIRE_TYPOLOGIES:
        return False, ("in a no-fire district ({0!r}) -- the skyscraper "
                        "districts stay untouched".format(typ))

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

    record = {
        "usd": usd, "x": x, "y": y,
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
def damaged_manifest(layout, placements, plan_records, seed_base):
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
    return manifest, refused


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

    dtc_usd = gf.DTC_DIR + "Building_12.usdc"
    kind, name = bake_kind(dtc_usd, 25.0, 25.0, 40.0, "rc")
    if (kind, name) != ("dtc", "Building_12"):
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

    p_tower = dict(p_gac, x_m=300.0, y_m=50.0)
    ok, reason = burnable(layout, p_tower, {})
    if ok or "no-fire district" not in reason:
        bad.append("burnable() did not refuse a tower-block placement: {0}".format((ok, reason)))

    p_muyang = dict(p_gac, usd=muyang_usd, x_m=60.0, y_m=60.0)
    ok, reason = burnable(layout, p_muyang, {})
    if ok or "route refused" not in reason or "unburnable" not in reason:
        bad.append("burnable() did not refuse Muyang DownTown: {0}".format((ok, reason)))

    p_aec = dict(p_gac, usd=aec_usd, x_m=170.0, y_m=70.0)
    ok, reason = burnable(layout, p_aec, {})
    if ok or "fire_bake.KINDS" not in reason:
        bad.append("burnable() did not refuse an AEC brownstone: {0}".format((ok, reason)))

    # --- district rule, mutation-checked ------------------------------------
    p_moved = dict(p_gac)
    ok_before, _ = burnable(layout, p_moved, {})
    p_moved["x_m"], p_moved["y_m"] = 300.0, 50.0   # into the tower block
    ok_after, reason_after = burnable(layout, p_moved, {})
    if not ok_before or ok_after or "no-fire district" not in reason_after:
        bad.append("moving a burnable placement into the tower block was not "
                  "refused by the district rule: before={0} after={1}"
                  .format(ok_before, (ok_after, reason_after)))

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
        {"i": 2, "level": "F1", "origin": 0, "sides": ["N"], "t_ignite_s": 400.0,
         "age_s": 500.0, "via": 0, "how": "attached"},
        {"i": 99, "level": "F1", "origin": 0, "sides": (), "t_ignite_s": 10.0,
         "age_s": 10.0, "via": 0, "how": "spot"},
    ]
    manifest, refused = damaged_manifest(layout, placements, plan_records, 1000)
    if [m["i"] for m in manifest] != [0, 1]:
        bad.append("damaged_manifest kept the wrong indices: {0}".format(
            [m["i"] for m in manifest]))
    if [m["seed"] for m in manifest] != [1000, 1031]:
        bad.append("damaged_manifest seeds are not seed_base + 31*i: "
                  "{0}".format([m["seed"] for m in manifest]))
    if len(refused) != 2:
        bad.append("damaged_manifest refused list should hold the tower-block "
                  "and out-of-range records, got {0}".format(refused))
    elif not any("no-fire district" in r["reason"] for r in refused):
        bad.append("damaged_manifest did not report the tower-block refusal "
                  "with a reason: {0}".format(refused))

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
