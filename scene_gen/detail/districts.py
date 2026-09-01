"""
districts.py — zoning: what gets built where, and how tall.

Additive. `build_city` is never modified; its placement list is rewritten.

Exports
-------
* `assign()`        — radial ring lookup, used by `city_detail` to thin
                      street furniture toward the edge.
* `zone_field()`    — ``zone_at(x, y) -> typology dict``. The zoning map.
                      `city_layout` may call this to size blocks per zone
                      (see BLOCK SIZE, below).
* `remap_buildings()` — the pipeline hook: rezones every block, then runs
                      infill and the park pass.
* `free_rects()`, `park_blocks()`, `block_inset()` — helpers `parks.py`
                      shares.

WHY POST-PROCESS RATHER THAN CONFIGURE
--------------------------------------
The building pools and the packing that fits them into lots both live inside
`build_city`, and this work is under a no-edits constraint. Packing has already
solved the geometry by the time it returns, so its output can be rewritten:
footprints are known, blocks are known, and a placement is just a dict.

ZONING, NOT A HEIGHT GRADIENT
-----------------------------
A radial height gradient alone produces a city where every block is the same
KIND of place and only the storey count changes. Real cities zone by *type* —
a district of party-wall row houses, a district of mid-rise apartment blocks, a
downtown of towers — and the boundaries are patchy rather than concentric.

So a **typology** (`districts.typologies.*`) owns a building pool, a height
profile and a preferred block size, and the radial rings only supply the
*mixture* of typologies likely at that distance from the centre. The zoning map
itself is a jittered-grid Voronoi over the region: contiguous multi-block
patches, ragged boundaries, and a patch of one type can land inside a ring
dominated by another. `zoning.bleed` additionally re-rolls a fraction of blocks
straight from the ring mix, which is what puts a mid-rise on a row-house street
the way a real corner apartment building sits in a terrace district.

MORPHOLOGY
----------
Typologies differ in how their blocks are built, not only in what fills them:

* `terrace` — party-wall rows along the block's long faces, laid END TO END
  WITH NO GAP, full face length, back yards/alley left in the middle. This is
  the whole point of a row house: one dropped into a gap on its own reads as a
  detached house with a missing neighbour, which is exactly wrong. A terrace
  asset is therefore never in `buildings.intact` (where `build_city` would pick
  it uniformly and scatter it) — it lives in its own pool and is only ever
  placed by this pass, in a run.
* `pack` — guillotine packing from the typology's pool, as downtown does.

BLOCK SIZE
----------
Measured, the library's footprints span 15.6 x 24.3 m to 91.1 x 96.1 m. Six of
the ten stock buildings are 50-81 m wide, against a ~46 m usable interior on
the blocks the subdivider was producing — so those blocks could not be built on
at all and stayed bare pavement. Block size has to follow the typology: a
row-house block wants ~48 m (two 21 m terraces back to back plus an alley), a
tower block wants ~100 m.

The subdivider lives in `city_layout.py`, which this work may not edit, so
`zone_field()` is exposed for it to consult. Until that lands, every typology
choice is validated against the block it actually got and downgraded to one
whose pool fits — the scene degrades to "smaller buildings than intended"
rather than to "empty block".
"""

import json
import math
import os
import random

from scene_generator import _in_exclusion, _normalize_usd_list

# Footprint slack when testing whether a building still fits its lot. Small and
# negative-biased on purpose: fractionally larger risks clipping a neighbour.
_FIT_TOL_M = 0.05

# Geometry below an asset's own origin (areaways, basement window wells, the
# sunken front court of a brownstone) is authored to sit BELOW grade. The
# generator's `z = fp["base"]` rule lifts the building until that clears the
# ground, which floats the stoop. Anything shallower than this is sunk back to
# grade; deeper, and the origin probably isn't the ground floor.
_BELOW_GRADE_MAX_M = 1.2


# ---------------------------------------------------------------------------
# burnability-aware pool selection — "I know we aren't using some buildings
# for fire at all, we need to account for that in the layout gen" (user,
# 2026-08-31). An unburnable asset (AEC brownstones / `standalone/buildings/
# ...` / Muyang DownTown — always refused by `kit_substitute.route`; a GAC/
# DTC pack-blacklisted name or a >232 m building — conditionally, per
# `disaster.gac_fire.PACKS[...]['blacklist']` and `urban_fire_city.
# FIRE_MAX_H_M`) standing next to another one fragments `urban_fire_spread`'s
# graph into a firebreak the fire silently routes around; the census this
# module's own docstring history is built on (`fire_city_dry_run.
# gather_burnable`) found 8 of 75 `downtown_fire_500` seed-4 houses refused
# this way.
#
# The checked-in table (`config/harvested/burnability_table.json`, generated
# by `tools/gen_burnability_table.py`) is the boolean verdict of the REAL
# gate (`disaster.urban_fire_city.burnable`), called once per pool asset,
# offline. This module only ever does a plain dict lookup against it —
# `districts.py` is imported for every scene, fire disaster or not, and must
# not drag the fire stack (`disaster.gac_fire`/`kit_substitute`/`quake`/
# `urban_fire_city`) into a build that never touches fire.
# ---------------------------------------------------------------------------

#: Matches `disaster.urban_fire_city.FIRE_MAX_H_M` — kept as a literal, not
#: an import, for the same reason the table below is a lookup and not a live
#: call. `tools/gen_burnability_table.py`'s docstring is the regeneration
#: point if that constant ever moves.
_LANDMARK_H_M = 232.0

#: The smaller-of-two-areas / larger-of-two-areas ratio a substitute's raw
#: footprint must clear against the entry it is replacing — see
#: `_burnable_substitute`. Same default as `districts.pack_area_band`
#: (`_pack_free`'s own "close enough to compete for one slot" band), reused
#: here rather than invented fresh.
_SUBSTITUTE_AREA_BAND = 0.55

#: How much a substitute's rotated footprint may exceed the entry it is
#: replacing, per axis, before `_burnable_substitute` refuses it (the
#: OVERLAP defect, 2026-08-31 review) — floating-point slack, not a real
#: allowance: a substitute that is even slightly bigger than what the
#: guillotine reserved for the original can spill into a neighbour.
_SUBSTITUTE_FIT_TOL_M = 0.05

_BURNABILITY_TABLE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "harvested", "burnability_table.json")

_burnability_cache = None


def _burnability_table() -> dict:
    """`{typology: {basename: bool}}`, loaded once and cached for the life
    of the process. A missing or unreadable file, or a typology the table
    has never seen, resolves to `{}`, which makes `_is_unburnable` fail
    OPEN (every asset treated as burnable) — a scene that never ran `gen_
    burnability_table.py`, or whose pool/typology map has grown since, must
    never have its fill rate regress because of a lookup miss.

    KEYED BY TYPOLOGY, NOT ASSET ALONE (2026-08-31 review fix) — `usds.
    buildings` pool KEYS are not typology names (the `midrise` TYPOLOGY
    draws from the `midrise_v2` POOL; a `midrise` pool also exists but no
    typology in `downtown_fire_500` references it), so a table keyed
    purely by basename risked auditing the wrong pool for a typology and
    would silently mismatch the day a gate starts reading typology at all.
    `tools/gen_burnability_table.py` resolves each typology's REAL pool
    list from `districts.typologies.<name>.pools`, exactly as `rezone_
    blocks`/`infill_blocks` do, and writes one sub-table per typology.
    Measured (that tool's own `--prove`): every verdict is IDENTICAL to the
    single-table version for `downtown_fire_500` seed 4 — candidacy is
    asset-and-material intrinsic, never typology-dependent, matching
    `urban_fire_city`'s own self-test ("moving a placement into the tower
    block should only change its typology, not its candidacy"). Keeping
    the per-typology shape anyway: correct is correct even when it doesn't
    move today's numbers.
    """
    global _burnability_cache
    if _burnability_cache is None:
        try:
            with open(_BURNABILITY_TABLE_PATH) as fh:
                _burnability_cache = dict(json.load(fh).get("assets") or {})
        except (OSError, ValueError):
            _burnability_cache = {}
    return _burnability_cache


def _asset_basename(usd: str) -> str:
    base = str(usd).rsplit("/", 1)[-1]
    for ext in (".usd", ".usdc", ".usda"):
        if base.lower().endswith(ext):
            return base[: -len(ext)]
    return base


def _is_unburnable(usd: str, typology) -> bool:
    """`True` only for a (typology, name) pair the table KNOWS and marks
    `False` — an unrecognised typology or name (an asset/typology added
    since the table was last regenerated, or a non-building prop that
    never belonged in it) is burnable by default, never a reason to swap
    or refuse a placement."""
    per_typ = _burnability_table().get(typology) or {}
    return per_typ.get(_asset_basename(usd)) is False


def _is_landmark(entry) -> bool:
    """The >232 m exemption (`SM_Building_16`/`SM_Building_31` today,
    whatever else clears the same bar tomorrow): unburnable by the same
    height cap `urban_fire_city.burnable` enforces, but a landmark, not
    firebreak clutter, so the swap rule below must leave it alone. Pool
    membership already confines these to whichever typology lists them
    (`highrise` in `urban_gac.yaml` today, never `tower` despite the user's
    "tower/skyscraper-typology" phrasing — that pool's own name in THIS
    asset-set just isn't the tall one); this predicate only decides whether
    the swap applies, never where an asset may be drawn from."""
    fp = entry[3] if len(entry) > 3 else {}
    return float(fp.get("sz", 0.0)) > _LANDMARK_H_M


def _rotated_wh(fp, yaw):
    """(bw, bh) — *fp*'s own (sx, sy) swapped when *yaw* is an odd multiple
    of 90, exactly as `_pack_free`'s own candidate-shape rotation does (see
    `_pool_entries`'s module-level docstring on the sx/sy swap). Shared by
    `_substitute_fits_street` and `_burnable_substitute`'s own footprint-fit
    check so both compute "this entry's world-space footprint at this
    already-decided yaw" the same way.
    """
    sx, sy = fp.get("sx", 0.0), fp.get("sy", 0.0)
    if round(yaw) % 180 == 90:
        return sy, sx
    return sx, sy


def _substitute_fits_street(cand, yaw, cx, cy, block_rect):
    """`False` only when *cand* HAS a measured front, *block_rect* is known,
    and that front (at the SAME candidate yaw the original slot already
    solved for) would land on a real street the candidate's own footprint
    does not actually reach — i.e. reusing the original position/yaw for a
    NEWLY front-tagged substitute would recreate exactly the `house_16_223`
    defect on different geometry.

    Only fires when `cand`'s own `front0` is not `None`. Relaxing
    `_burnable_substitute`'s front-match rule so an UNTAGGED target
    (`front0 is None`, most of `tower`'s "any" filler stock) accepts a
    TAGGED candidate (`podium_highrise` <- `SM_Building_27`, 2026-08-31
    review) reuses a candidate-yaw that was never solved with that
    candidate's front in mind at all -- this is the check that makes that
    still safe. A target that already HAD a matching front0 needs no
    re-check here: `front0` being equal by construction means `_rot_side`
    of it at this same yaw reproduces the SAME world direction the slot was
    actually scored against, whichever entry carries it.
    """
    cmeta = cand[5] if len(cand) > 5 else {}
    cfront0 = cmeta.get("front0")
    if cfront0 is None or block_rect is None:
        return True
    fpc = cand[3] if len(cand) > 3 else {}
    bw, bh = _rotated_wh(fpc, yaw)
    px, py = cx - bw / 2.0, cy - bh / 2.0
    fsides = _street_sides(block_rect, px, py, bw, bh)
    if not fsides:
        return True
    return _rot_side(cfront0, yaw) in fsides


def _burnable_substitute(entry, pool, typology, cx=None, cy=None, yaw=None,
                         block_rect=None):
    """A pool-mate to stand in for *entry* — deterministic, no `rng` draw,
    so a swap never perturbs any OTHER decision's random stream (seed-4
    byte-identity depends on that). *cx*, *cy*, *yaw*, *block_rect* are the
    ALREADY-DECIDED position/orientation *entry* would have been placed at
    — needed only to re-verify a front-tagged candidate against real street
    geometry when the target itself was untagged (`_substitute_fits_street`
    below); omit them (the default) to skip that re-check entirely, which
    is exactly right for a caller with no facing context at all.

    NOT matched on raw `yaw-offset` — measured on `midrise_v2`, the pool
    `downtown_fire_500` seed 4's own violations came from (`Building_11`/
    `Building_12`, both `yaw-offset: 270`): every burnable entry in it
    carries `yaw-offset: 0`, so a same-yaw-offset requirement finds zero
    substitutes for either and both stay put. What actually has to match is
    `front0` — the asset's front direction ALREADY NORMALISED TO THE
    MODULE'S OWN CANONICAL "W" by `_pool_entries` (`_rot_side(front,
    yaw_offset)`, see that function and the `urban-layout` skill) — because
    the candidate yaw the packer already solved for *entry* was chosen to
    put THAT normalised direction on the street; reusing it for a
    substitute with the same `front0` points it exactly the same way
    regardless of what its own raw `yaw-offset` happens to be.

    `front0 is None` (no measured front at all — most of `tower`'s "any"
    filler stock, `podium_highrise`/`office_tower`/etc.) has NO direction to
    preserve, so it is NOT required to match a candidate's `front0` — an
    untagged entry accepts ANY candidate's front0, tagged or not. Requiring
    an exact `None == None` match here (the original rule) silently refused
    every measured-front GAC building as a substitute for an untagged
    "tower" entry, which is exactly why `podium_highrise` (1,176 m2, real
    Nucleus size) had no substitute even though `SM_Building_27` (1,216 m2,
    `front0="W"`) is a near-perfect footprint match: it was skipped purely
    for HAVING a front tag the target never needed. A candidate WITH a
    measured front still only substitutes for an entry that ALSO has one
    when they agree — a real facing preference is never silently dropped
    the other way around.

    `blank0` must be a SUBSET of *entry*'s own (never MORE blank sides than
    what already cleared `_pack_free`'s hard reject at this slot) and
    `place` must be no more restrictive (`any` always qualifies; otherwise
    an exact match). Footprint must be a similar CLASS too — the smaller of
    the two raw, unrotated areas at least `_SUBSTITUTE_AREA_BAND` (0.55, the
    same default `districts.pack_area_band` already uses for "close enough
    to compete for one slot") of the larger, a RATIO test rather than a
    symmetric one because the pool this rule actually had to fix
    (`midrise_v2`) has exactly one front0/blank0/place match for
    `Building_12` (828 m2) at `SM_Building_30` (1204 m2) — 45% bigger, which
    a symmetric +/-40% window rejects but a same-ballpark ratio test should
    not.

    NEITHER ROTATED EXTENT MAY EXCEED *entry*'s OWN (2026-08-31 review: the
    overlap defect). Matching only on AREA lets a substitute be, say, 40%
    taller and narrower than the entry it replaces while keeping the same
    area ratio — reused at *entry*'s own (cx, cy, yaw), it then spills past
    whatever the guillotine actually reserved for *entry*'s footprint and
    interpenetrates a neighbour the packer placed in good faith right next
    to it. MEASURED, `Building_11` (35.4 x 30.9 m) swapped for `SM_Building_
    30` (28.4 x 42.4 m) at unchanged (cx, cy, 0 deg) — same ballpark area,
    11.5 m taller — overlapped `bld_apartment_tall_DG0` by 85.6 m2 in both
    the seeded host build AND the Kit dump it was meant to match. Requiring
    both `bw`/`bh` (at the SAME *yaw* — a candidate's own rotated shape) to
    fit within *entry*'s own is strictly TIGHTER than the area ratio and
    subsumes it for this purpose; the area ratio is kept as the
    "not needlessly small" floor. `None` when nothing in *pool* qualifies —
    the caller then keeps the unburnable draw rather than leaving a hole or
    gambling on a wildly different size/orientation for a bare "something
    has to stand here" swap.
    """
    if not pool:
        return None
    fp0 = entry[3] if len(entry) > 3 else {}
    area0 = float(fp0.get("sx", 0.0)) * float(fp0.get("sy", 0.0))
    if area0 <= 0.0:
        return None
    meta0 = entry[5] if len(entry) > 5 else {}
    front0 = meta0.get("front0")
    blank0 = meta0.get("blank0") or frozenset()
    place0 = meta0.get("place", "any")
    bw0 = bh0 = None
    if yaw is not None:
        bw0, bh0 = _rotated_wh(fp0, yaw)
    for cand in pool:
        if cand[0] == entry[0]:
            continue
        if _is_landmark(cand) or _is_unburnable(cand[0], typology):
            continue
        cmeta = cand[5] if len(cand) > 5 else {}
        if front0 is not None and cmeta.get("front0") != front0:
            continue
        if (cmeta.get("blank0") or frozenset()) - blank0:
            continue
        cplace = cmeta.get("place", "any")
        if cplace != "any" and cplace != place0:
            continue
        if yaw is not None and not _substitute_fits_street(
                cand, yaw, cx, cy, block_rect):
            continue
        fpc = cand[3] if len(cand) > 3 else {}
        areac = float(fpc.get("sx", 0.0)) * float(fpc.get("sy", 0.0))
        if areac <= 0.0:
            continue
        if min(area0, areac) / max(area0, areac) < _SUBSTITUTE_AREA_BAND:
            continue
        if bw0 is not None:
            bwc, bhc = _rotated_wh(fpc, yaw)
            if bwc > bw0 + _SUBSTITUTE_FIT_TOL_M or bhc > bh0 + _SUBSTITUTE_FIT_TOL_M:
                continue
        return cand
    return None


def _log_swap(swap_log, entry, sub, cx, cy, yaw):
    """Appends one `_BurnabilityGuard` swap decision to *swap_log* (a plain
    list; `rezone_blocks`/`infill_blocks` share ONE such list for the whole
    scene via `layout["_burn_swap_log"]`) -- the record `repair_overlaps`
    consults to revert a substitute whose real footprint turned out to
    overlap a neighbour back to the original draw the packer actually
    validated for this slot. Keyed by the placement's own (cx, cy, yaw),
    since that is the one thing `repair_overlaps` can still match against a
    FINAL placement dict (which carries no pool-entry identity at all).
    """
    swap_log.append({
        "cx": cx, "cy": cy, "yaw": yaw,
        "original_usd": entry[0], "original_scale": entry[1],
        "original_axis_up": entry[2],
        "substitute_usd": sub[0], "substitute_scale": sub[1],
        "substitute_axis_up": sub[2],
    })


class _BurnabilityGuard:
    """Per-block bookkeeping for the "no firebreak cluster" rule: an
    unburnable asset may stand at most ONCE per block; a second (or later)
    draw is swapped, deterministically, for a burnable pool-mate of similar
    footprint class (`_burnable_substitute`). A landmark (`_is_landmark`)
    never counts against the block's one-unburnable allowance and is never
    itself swapped.

    ONE FRESH INSTANCE PER BLOCK — `rezone_blocks` and `infill_blocks` each
    create one where their own per-block loop begins and run every
    placement that block produces through `filter_laid`/`filter_one` before
    it becomes a placement, so the count resets at each block boundary
    exactly as "at most once per block" requires. *block_rect* is that same
    block's own inset rect (the `rect` both callers already pass to
    `_pack_free`) — threaded through to `_burnable_substitute` so a
    front-tagged candidate standing in for an untagged target is checked
    against the REAL street geometry at that position, not just accepted
    on faith (`_substitute_fits_street`). *swap_log*, if given (a list;
    `layout["_burn_swap_log"]`), records every actual substitution so
    `repair_overlaps` can revert one whose real footprint overlaps a
    neighbour back to the original draw (`_log_swap`).
    """

    def __init__(self):
        self.used = False

    def filter_one(self, entry, pool, typology, cx=None, cy=None, yaw=None,
                   block_rect=None, swap_log=None):
        if _is_landmark(entry) or not _is_unburnable(entry[0], typology):
            return entry
        if not self.used:
            self.used = True
            return entry
        sub = _burnable_substitute(entry, pool, typology, cx, cy, yaw,
                                   block_rect)
        if sub is None:
            return entry
        if swap_log is not None and cx is not None:
            _log_swap(swap_log, entry, sub, cx, cy, yaw)
        return sub

    def filter_laid(self, laid, pool, typology, block_rect=None,
                    swap_log=None):
        """Same rule as `filter_one`, applied across a whole block's
        `laid` list at once -- and, unlike calling `filter_one` per entry
        in order, choosing WHICH unburnable entry gets the block's one free
        pass instead of always handing it to whichever happens to come
        first.

        WHY THIS MATTERS: measured on `downtown_fire_500` seed 4's `tower`
        typology with real (Nucleus-seeded) sizes, one block's guillotine
        pass laid `stepped_tower` (5,071 m2) before `podium_highrise`
        (1,176 m2). A first-come-first-free allocation hands the pass to
        `stepped_tower` — which HAS a same-footprint-class substitute
        (`SM_MERGED_BP_MBuilding02`, 8,752 m2) sitting unused in the pool —
        and then fails to find anything small enough to stand in for
        `podium_highrise`, leaving TWO unburnable buildings in the block
        despite a substitute existing for one of them. Computing every
        unburnable entry's candidate substitute FIRST and reserving the
        free pass for one that has NONE (falling back to positional order
        only when every entry has a substitute, or none needs to choose)
        lets `stepped_tower` take the swap it can afford and `podium_
        highrise` keep the slot nothing else fits.

        `self.used` (already spent by an earlier call for this SAME block —
        `rezone_blocks` and `infill_blocks` share one guard via `layout
        ["_burn_guard_by_block"]`) is respected exactly as before: no free
        pass is granted at all once it is already gone, regardless of which
        entries have substitutes this time.
        """
        unb_idx = [i for i, (e, _cx, _cy, _yaw) in enumerate(laid)
                  if not _is_landmark(e) and _is_unburnable(e[0], typology)]
        if not unb_idx:
            return laid
        subs = {}
        for i in unb_idx:
            e, cx, cy, yaw = laid[i]
            subs[i] = _burnable_substitute(e, pool, typology, cx, cy, yaw,
                                           block_rect)
        free_idx = None
        if not self.used:
            no_sub = [i for i in unb_idx if subs[i] is None]
            free_idx = no_sub[0] if no_sub else unb_idx[0]
        out = list(laid)
        for i in unb_idx:
            if i == free_idx:
                continue
            e, cx, cy, yaw = out[i]
            sub = subs[i]
            if sub is None:
                continue
            if swap_log is not None:
                _log_swap(swap_log, e, sub, cx, cy, yaw)
            out[i] = (sub, cx, cy, yaw)
        self.used = True
        return out


# ---------------------------------------------------------------------------
# compass rotation — the frame every `front:`/`blank:` tag is written in
# ---------------------------------------------------------------------------

# CCW about +Z, matching the yaw the placer applies: +90 walks E -> N -> W ->
# S -> E. `tools/faces_to_yaml.py` derives each asset's `yaw-offset` by
# solving "rotate `front` onto W" against this exact sequence (front W needs
# 0, N needs 90, E needs 180, S needs 270), so any rotation done here has to
# agree with that table or the two halves of the facing system disagree with
# each other while each individually looks correct.
_COMPASS = ("E", "N", "W", "S")


def _rot_side(letter: str, deg: float) -> str:
    """Rotate a compass letter (``"N"``/``"E"``/``"S"``/``"W"``) CCW about +Z
    by *deg*, which must be a multiple of 90 (placement yaws always are).

    This is the one piece of arithmetic the whole facing system hangs off:
    `_pool_entries` uses it to fold a `blank:` tag through an asset's own
    `yaw-offset` into `blank0`, `_pack_free`/`_lay_terrace` use it again to
    fold `blank0` through the placement yaw into WORLD sides, and the
    `--audit` pass in `plan_png.py` uses it a third time, straight from the
    raw tag, through the placement's final `yaw_deg`. All three have to mean
    the same rotation or a building that measures fine in isolation ends up
    wrong on the ground.
    """
    i = _COMPASS.index(letter)
    steps = int(round(deg / 90.0)) % 4
    return _COMPASS[(i + steps) % 4]


def _rot_sides(sides, deg: float) -> frozenset:
    """`_rot_side` over a whole set of letters. Empty in, empty out — the
    no-metadata case this whole mechanism has to be a no-op for."""
    return frozenset(_rot_side(s, deg) for s in sides)


def assign(config: dict, layout: dict):
    """Return ``(district_at, districts)``.

    ``district_at(x, y)`` gives the ring dict for a world position, or ``None``
    when zoning is off. Rings are concentric fractions of the region's
    half-diagonal — they no longer decide what gets built, only the mixture of
    typologies and how much street furniture `city_detail` thins out.
    """
    cfg = config.get("districts") or {}
    rings = cfg.get("rings") or []
    if not cfg.get("enabled") or not rings:
        return (lambda x, y: None), []

    x0, y0, x1, y1 = layout.get("region", (0.0, 0.0, 0.0, 0.0))
    w, h = abs(x1 - x0), abs(y1 - y0)
    if min(w, h) < float(cfg.get("min_region_m", 0.0)):
        print(f"[districts] region {w:.0f}x{h:.0f} m is below "
              f"min_region_m={cfg.get('min_region_m')} — single district")
        return (lambda x, y: None), []

    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    half_diag = math.hypot(w, h) / 2.0 or 1.0
    ordered = sorted(rings, key=lambda r: float(r.get("to_frac", 1.0)))

    def district_at(x, y):
        f = math.hypot(x - cx, y - cy) / half_diag
        for r in ordered:
            if f <= float(r.get("to_frac", 1.0)):
                return r
        return ordered[-1]

    names = ", ".join(str(r.get("name", "?")) for r in ordered)
    print(f"[districts] {len(ordered)} rings over {w:.0f}x{h:.0f} m: {names}")
    return district_at, ordered


def block_lookup(district_at):
    """Adapt `district_at` to the block-rect callback `city_detail` expects."""
    def district_of(block):
        x0, y0, x1, y1 = block
        return district_at((x0 + x1) / 2.0, (y0 + y1) / 2.0)
    return district_of


# ---------------------------------------------------------------------------
# the zoning map
# ---------------------------------------------------------------------------

def _region_of(config: dict, region=None):
    if region is not None:
        return tuple(region)
    r = (config.get("layout") or {}).get("region_m", [400.0, 400.0])
    w, h = float(r[0]), float(r[1])
    return (-w / 2.0, -h / 2.0, w / 2.0, h / 2.0)


def _pick(mix: dict, rnd: random.Random):
    total = sum(max(0.0, float(v)) for v in mix.values()) or 1.0
    r = rnd.random() * total
    for k, v in mix.items():
        r -= max(0.0, float(v))
        if r <= 0.0:
            return k
    return next(iter(mix))


class _AreaMap:
    """Land use as a few irregular contiguous areas grown from nuclei.

    Concentric rings are Burgess (1925) — the textbook caricature. Real cities
    look closer to Harris & Ullman's multiple-nuclei model (1945), and that is
    what OSM landuse polygons show: a handful of irregular contiguous patches
    around one dominant CBD, not bands. So typology is decided by growing
    regions outward from a small number of nuclei on a coarse lattice, and the
    radial rings survive only as the PRIOR on where those nuclei land — the
    downtown still tends to sit centrally without the city reading as banded.

    Growth is Dijkstra from every nucleus at once with jittered edge costs,
    which is what makes a boundary wander instead of coming out as the straight
    bisector a plain Voronoi would give.
    """

    def __init__(self, config: dict, region=None):
        cfg = config.get("districts") or {}
        zcfg = cfg.get("zoning") or {}
        self.typologies = cfg.get("typologies") or {}
        self.rings = sorted(cfg.get("rings") or [],
                            key=lambda r: float(r.get("to_frac", 1.0)))
        self.bleed = float(zcfg.get("bleed", 0.12))
        self.blend_m = float(zcfg.get("blend_m", 150.0))
        cell = max(30.0, float(zcfg.get("cell_m", 120.0)))

        x0, y0, x1, y1 = _region_of(config, region)
        self.x0, self.y0, self.cell = x0, y0, cell
        self.cx, self.cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        self.half_diag = math.hypot(x1 - x0, y1 - y0) / 2.0 or 1.0
        self.nx = max(1, int(math.ceil((x1 - x0) / cell)))
        self.ny = max(1, int(math.ceil((y1 - y0) / cell)))

        # Own RNG: the map must not shift because some other pass happened to
        # draw a different number of randoms before it.
        rnd = random.Random(int(config.get("seed", 0)) * 7919 + 104729)
        self.grid = self._grow(zcfg, rnd)
        self._ladder(zcfg)

    # -- lattice helpers
    def _ij(self, x, y):
        return (min(self.nx - 1, max(0, int((x - self.x0) / self.cell))),
                min(self.ny - 1, max(0, int((y - self.y0) / self.cell))))

    def _xy(self, i, j):
        return (self.x0 + (i + 0.5) * self.cell,
                self.y0 + (j + 0.5) * self.cell)

    def ring_of(self, x, y):
        f = math.hypot(x - self.cx, y - self.cy) / self.half_diag
        for r in self.rings:
            if f <= float(r.get("to_frac", 1.0)):
                return r
        return self.rings[-1] if self.rings else {}

    def _grow(self, zcfg, rnd):
        """Place nuclei, then flood the lattice outward from all of them."""
        import heapq

        # One nucleus per `nucleus_per_m2` of region, split between typologies
        # by the ring mix at each candidate site — so a tower nucleus is much
        # likelier to be drawn near the centre and a row-house one at the edge.
        area = self.nx * self.ny * self.cell * self.cell
        n_nuc = max(2, int(round(area / max(1.0, float(
            zcfg.get("area_m2_per_nucleus", 120_000.0))))))
        heap, grid = [], {}
        for k in range(n_nuc):
            for _try in range(24):
                # Multiple-nuclei still has ONE dominant CBD. Left to chance the
                # innermost ring is a small target and most seeds land outside
                # it, so a city comes out with no downtown at all — nucleus 0 is
                # therefore forced into the core ring.
                if k == 0 and zcfg.get("downtown", True):
                    i = int(self.nx * rnd.uniform(0.38, 0.62))
                    j = int(self.ny * rnd.uniform(0.38, 0.62))
                else:
                    i = rnd.randrange(self.nx)
                    j = rnd.randrange(self.ny)
                i = min(self.nx - 1, max(0, i))
                j = min(self.ny - 1, max(0, j))
                if (i, j) in grid:
                    continue
                x, y = self._xy(i, j)
                mix = (self.ring_of(x, y) or {}).get("mix") or {}
                name = _pick(mix, rnd) if mix else None
                if name is None:
                    continue
                grid[(i, j)] = name
                heapq.heappush(heap, (0.0, k, i, j, name))
                break

        while heap:
            d, k, i, j, name = heapq.heappop(heap)
            if grid.get((i, j)) not in (None, name) and d > 0.0:
                continue
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ni, nj = i + di, j + dj
                if not (0 <= ni < self.nx and 0 <= nj < self.ny):
                    continue
                if (ni, nj) in grid:
                    continue
                grid[(ni, nj)] = name
                heapq.heappush(heap, (d + rnd.uniform(0.6, 1.6), k, ni, nj,
                                      name))
        return grid

    def _ladder(self, zcfg):
        """Forbid a rank jump greater than one between neighbouring areas.

        Transitional zoning, which is how real codes stop a tower from abutting
        a two-storey terrace: where the jump is too big the intermediate
        typology is inserted instead. Also what keeps block sizes from having
        to blend across an impossible gap.
        """
        if not zcfg.get("adjacency_ladder", True):
            return
        rank = {n: float(t.get("rank", 0.0))
                for n, t in self.typologies.items()}
        by_rank = sorted(rank, key=lambda n: rank[n])
        for _sweep in range(3):
            changed = False
            for (i, j), name in list(self.grid.items()):
                for di, dj in ((1, 0), (0, 1)):
                    other = self.grid.get((i + di, j + dj))
                    if other is None or other == name:
                        continue
                    a, b = rank.get(name, 0.0), rank.get(other, 0.0)
                    if abs(a - b) <= 1.0 + 1e-9:
                        continue
                    mid = min(by_rank,
                              key=lambda n: abs(rank[n] - (a + b) / 2.0))
                    self.grid[(i + di, j + dj) if b > a else (i, j)] = mid
                    changed = True
            if not changed:
                break

    # -- lookups
    def name_at(self, x, y):
        return self.grid.get(self._ij(x, y))

    def bleed_name(self, x, y, rng):
        """A typology drawn straight from the ring mix, ignoring the area.

        Clean zoning is the giveaway; real cities leak. This is what puts a
        mid-rise apartment block in the middle of a terrace street.
        """
        mix = (self.ring_of(x, y) or {}).get("mix") or {}
        return _pick(mix, rng) if mix else None

    def targets_at(self, x, y):
        """Blended block-size targets ``(short_lo, short_hi, long_lo, long_hi)``.

        Sampled over a small cross rather than at the point alone, so block size
        GRADES across an area boundary over `blend_m` instead of snapping from
        a 40 m residential block to a 110 m downtown one in a single street.
        """
        # BLEND ONLY WITHIN A TYPOLOGY. Averaging a cell with neighbours of a
        # DIFFERENT typology does not grade the seam, it destroys the target:
        # measured, row-house blocks configured at 53-58 m came out 77-92 m
        # because most row-house cells are boundary cells and every one was
        # being pulled toward the 80-115 m mid-rise and 70-110 m tower figures
        # around it. A terrace pair needs 52.7 m, so those blocks could only
        # ever be 40% built. A hard seam in block size is a far smaller problem
        # than a typology that cannot build what it is for.
        here_name = self.name_at(x, y)
        acc, n = [0.0, 0.0, 0.0, 0.0], 0
        r = self.blend_m
        for dx, dy in ((0.0, 0.0), (r, 0.0), (-r, 0.0), (0.0, r), (0.0, -r)):
            name = self.name_at(x + dx, y + dy)
            if name != here_name:
                continue
            t = self.typologies.get(name or "") or {}
            s = t.get("block_short_m")
            lg = t.get("block_long_m")
            if not s or not lg:
                continue
            acc[0] += float(s[0])
            acc[1] += float(s[1])
            acc[2] += float(lg[0])
            acc[3] += float(lg[1])
            n += 1
        if not n:
            return None
        out = tuple(v / n for v in acc)
        # A typology may run its blocks ACROSS the city's grain. `city_layout`
        # maps (short, long) onto (x, y) by one global `long_axis`, so every
        # block in the scene is elongated the same way — which is most of why a
        # terrace district reads as more of the same. Swapping the pair here
        # turns that typology's blocks a quarter turn, and because the terrace
        # rows follow the block's own long face they turn with it.
        here = self.typologies.get(self.name_at(x, y) or "") or {}
        if _grain_flip(here.get("cross_grain"), x, y):
            out = (out[2], out[3], out[0], out[1])
        return out

    def __call__(self, x, y):
        """Typology dict at a position, with its ``name`` filled in."""
        name = self.name_at(x, y)
        if name is None:
            return None
        t = dict(self.typologies.get(name) or {})
        t["name"] = name
        return t


def _grain_flip(cross_grain, x: float, y: float) -> bool:
    """Whether the block grain turns a quarter at this position.

    ``true`` turns the whole typology, which makes every terrace block in the
    scene run the same way — the same monotony the flag exists to break, just
    rotated. A NUMBER instead turns it by section: the plane is cut into squares
    of that many metres and alternate ones flip, so one row-house district runs
    with the city grain and the next runs across it.

    Sections are coarse on purpose. The grain has to hold across a whole strip
    or its blocks come out inconsistent, so the size should comfortably exceed
    one district; a section boundary falling inside a strip is the one case to
    watch. Position, not `rng` — the subdivider samples this many times per
    cell and a random answer would differ between samples of the same block.
    """
    if not cross_grain:
        return False
    if cross_grain is True:
        return True
    try:
        step = float(cross_grain)
    except (TypeError, ValueError):
        return True
    if step <= 0.0:
        return True
    ix = int(math.floor(x / step))
    iy = int(math.floor(y / step))
    return ((ix * 73856093) ^ (iy * 19349663)) % 2 == 0


def zone_field(config: dict, region=None):
    """``zone_at(x, y) -> typology dict | None``.

    Public so `city_layout` can size each block to the area it falls in; see
    BLOCK SIZE in the module docstring. Returns a None-yielding callable when
    typology zoning is not configured, so callers need no feature test.
    """
    cfg = config.get("districts") or {}
    if not (cfg.get("enabled") and cfg.get("typologies") and cfg.get("rings")):
        return lambda x, y: None
    return _AreaMap(config, region)


# ---------------------------------------------------------------------------
# building library
# ---------------------------------------------------------------------------

def _entry_meta(tags: frozenset, yaw_offset: float) -> dict:
    """Turn one asset's raw `tags` frozenset into the placement-facing dict
    every consumer below reads: ``place``, ``front``, ``blank`` (all in the
    asset's OWN unrotated frame, exactly as measured/authored), ``blank0``
    (the same blank set folded through this asset's `yaw-offset`, i.e. the
    frame the pool's assets share once every yaw-offset has been applied —
    see the module docstring's per-asset `yaw-offset` note), ``front0`` (the
    same fold applied to `front` — always "W" for a correctly-authored
    `yaw-offset`, since that offset's whole job is to turn the front to -X,
    but computed rather than assumed so a bad `yaw-offset` fails as a wrong
    score instead of silently trusting a convention it violates), and
    ``never_corner`` (`place_never_corner` — see `_pack_free`'s scoring
    docstring). A pool entry with no tags at all gets ``place="any"``, empty
    `front`/`blank`/`blank0`/`front0` and `never_corner=False` — every
    consumer of this dict has to treat that as "impose no constraint", which
    is what makes the whole facing system a no-op for a library nobody has
    measured yet.
    """
    place = ("mid" if "place_mid" in tags else
             "end" if "place_end" in tags else
             "corner" if "place_corner" in tags else
             "none" if "place_none" in tags else "any")
    never_corner = "place_never_corner" in tags
    front = None
    blank: frozenset = frozenset()
    for t in tags:
        if t.startswith("front:"):
            front = t.split(":", 1)[1] or None
        elif t.startswith("blank:"):
            blank = frozenset(s for s in t.split(":", 1)[1].split(",") if s)
    return {"place": place, "front": front, "blank": blank,
            "blank0": _rot_sides(blank, yaw_offset),
            "front0": _rot_side(front, yaw_offset) if front else None,
            "never_corner": never_corner}


def _pool_entries(config: dict, resolver, key: str):
    """``[(usd, scale, axis_up, footprint, yaw_offset, meta)]`` for one
    ``usds.buildings`` pool.

    *footprint* is the resolver's measured ``{sx, sy, sz, base}`` for
    *yaw_offset* 0, SWAPPED on sx/sy when *yaw_offset* is an odd multiple of
    90 — a yaw-offset that turns a building a quarter turn also turns which
    of its two horizontal extents ends up along world X once it is placed,
    and every consumer downstream (`_lay_terrace`'s terrace depth,
    `_pack_free`'s guillotine fit, `_fits_block`, `min_side`) needs the
    footprint IT WILL ACTUALLY STAND AT, not the one the resolver measured
    before any rotation. Concretely: `SM_Building_24` measures 29.0 x 58.0 m
    with `yaw-offset: 90`; its true depth in a terrace is 58 m. A copy of the
    dict is swapped, never the resolver's own cached one — that cache is
    shared across every pool that references the same asset.
    """
    usds = config.get("usds") or {}
    bld = usds.get("buildings") or usds.get("houses") or {}
    raw = bld.get(key) or []
    default_scale = float(config.get("asset_scale", 1.0))
    asset_root = str(config.get("asset_root", "") or "")
    paths, sc_ovr, au_ovr, yaw_ovr, tag_ovr = _normalize_usd_list(
        raw, default_scale, asset_root)

    out, seen = [], set()
    for p in paths:
        if p in seen:
            continue
        seen.add(p)
        sc = sc_ovr.get(p, default_scale)
        au = au_ovr.get(p, "Z")
        yaw_offset = float(yaw_ovr.get(p, 0.0))
        fp = dict(resolver.get(p, "house", scale=sc, axis_up=au))
        if round(yaw_offset) % 180 == 90:
            fp["sx"], fp["sy"] = fp["sy"], fp["sx"]
        meta = _entry_meta(tag_ovr.get(p) or frozenset(), yaw_offset)
        out.append((p, sc, au, fp, yaw_offset, meta))
    return out


def _asset_meta_table(config: dict) -> dict:
    """``{usd_path: {"front", "blank", "place", "never_corner", ...}}`` for
    every unique asset across every ``usds.buildings`` pool — pure yaml-tag
    derivation (`_entry_meta`), no resolver/geometry needed. Keyed by the
    FULLY RESOLVED usd path (post `asset_root` join via `_normalize_usd_
    list`), matching what every placement's own `"usd"` field already
    carries.

    Built for `repair_facing`/`repair_overlaps`: a post-hoc pass over the
    FINAL placements list has no pool-entry object to read `meta` off of
    (a placement dict is just usd/x/y/yaw/scale — the whole point of
    `_new_placement` collapsing an entry to a plain dict), so recovering
    "does this placement's asset have a measured front" means re-deriving
    it from the SAME yaml tags the original pool-selection pass read,
    independent of which pool actually drew it.
    """
    usds = config.get("usds") or {}
    bld = usds.get("buildings") or usds.get("houses") or {}
    default_scale = float(config.get("asset_scale", 1.0))
    asset_root = str(config.get("asset_root", "") or "")
    out: dict = {}
    for raw in bld.values():
        if not isinstance(raw, list):
            continue
        paths, _sc, _au, yaw_ovr, tag_ovr = _normalize_usd_list(
            raw, default_scale, asset_root)
        for p in paths:
            if p in out:
                continue
            yaw_offset = float(yaw_ovr.get(p, 0.0))
            out[p] = _entry_meta(tag_ovr.get(p) or frozenset(), yaw_offset)
    return out


def _pools_for(config, resolver, names, cache):
    """Merge the named ``usds.buildings`` pools, tallest first."""
    out, seen = [], set()
    for n in names or ():
        if n not in cache:
            cache[n] = _pool_entries(config, resolver, n)
        for e in cache[n]:
            if e[0] not in seen:
                seen.add(e[0])
                out.append(e)
    out.sort(key=lambda e: -e[3]["sz"])
    return out


def _place_z(fp):
    """Ground height for a footprint — see `_BELOW_GRADE_MAX_M`."""
    return 0.0 if 0.0 < fp["base"] <= _BELOW_GRADE_MAX_M else fp["base"]


def _new_placement(entry, x, y, yaw, category="house"):
    """*yaw* is the PLACEMENT yaw the packer/terrace code computed — it does
    not know about `yaw-offset`, which lives on the entry, not the slot. The
    offset is added here, once, at the only point every pool entry funnels
    through on its way into `placements`. Tolerated as a 4-tuple (no offset,
    no meta) defensively, but every entry this module itself builds is now a
    6-tuple (`_pool_entries`) — see that function before relying on the
    fallback.
    """
    usd, sc, au, fp = entry[0], entry[1], entry[2], entry[3]
    yaw_offset = entry[4] if len(entry) > 4 else 0.0
    return {"usd": usd, "x_m": x, "y_m": y, "z_m": _place_z(fp),
            "yaw_deg": (yaw + yaw_offset) % 360.0,
            "roll_deg": 90.0 if au == "Y" else 0.0,
            "pitch_deg": 0.0, "scale": sc, "category": category,
            "axis_up": au}


# ---------------------------------------------------------------------------
# height field
# ---------------------------------------------------------------------------

class _Skyline:
    """Samples a building height, per typology, and decides WHICH model.

    Three effects compose. The typology supplies a log-normal height target —
    right-skewed, so most draws are low and a few are tall. `neighbour_weight`
    then pulls the draw toward the mean of nearby buildings already decided, so
    height varies gradually along a street instead of independently per lot.
    Finally the draw among the models that MATCH that height is damped twice
    over, by how often each one is already standing and by whether one of them
    is standing within sight — see `_pick`.
    """

    def __init__(self, cfg: dict, rng):
        self.rng = rng
        self.w = float(cfg.get("neighbour_weight", 0.5))
        self.r2 = float(cfg.get("neighbour_radius_m", 45.0)) ** 2
        # Wider than the typology's own sigma on purpose: with a coarse library
        # an exact match is usually unavailable, and always taking the nearest
        # re-quantises the distribution onto a handful of heights.
        self.pick_sigma = float(cfg.get("pick_sigma", 0.30))
        # -- GLOBAL REPEAT: how much of the city is this one model? -----------
        # 0 restores pure height matching; 1 makes an asset's odds inversely
        # proportional to how many times it is already standing in the city.
        # ABOVE 1 it is superlinear, which is the point: at the historical 0.6
        # a model already used five times still drew at 35% of an unused one's
        # odds, so with a small library the same few kept winning. The library
        # is no longer small (`asset_sets/urban_v2.yaml` takes tower to 32 and
        # midrise to 93), and a penalty that actually bites is what converts
        # that into buildings on the ground rather than entries in a file.
        #
        # THE DEFAULT IS THE HISTORICAL 0.6 and the presets carry the real
        # value — `downtown.yaml` has always set 1.0, `downtown_1000.yaml` sets
        # 2.2. Changing the default would silently restyle every existing
        # downtown scene, including the earthquake one, which is not this
        # change's business.
        self.repeat = float(cfg.get("repeat_penalty", 0.6))
        # -- LOCAL REPEAT: is this model already standing within sight? -------
        # The global count says nothing about WHERE, and where is what a viewer
        # sees: two identical towers across one street read as a copy-paste no
        # matter how balanced the city-wide histogram is. Every instance inside
        # `repeat_radius_m` multiplies the model's weight by
        # `repeat_local_penalty`, so a second instance next door is a ~50x
        # handicap and a third is ~2500x.
        #
        # A MULTIPLIER AND NOT A BAN, deliberately. A gap whose only fitting
        # model is one already nearby has to be filled with something, and a
        # hard exclusion there either leaves a hole or falls back to a rule
        # nobody can see. At 0.02 the near model still wins when it is the only
        # candidate and effectively never wins when it is not.
        #
        # OFF BY DEFAULT — `repeat_radius_m: 0` makes the whole term inert —
        # for the same reason `repeat_penalty` keeps its old default: a scene
        # opts in. `downtown_1000.yaml` does.
        self.local_r2 = float(cfg.get("repeat_radius_m", 0.0)) ** 2
        self.local_pen = float(cfg.get("repeat_local_penalty", 0.02))
        # ...and GRADED BY DISTANCE inside that radius, because a flat count is
        # not what the eye does: a twin 30 m away across the street and one
        # 145 m away at the far end of the district are not the same defect and
        # a binary test scores them identically. Each copy contributes
        # `1 + falloff * (1 - d/R)` to the exponent, so at the default 2.0 an
        # adjacent twin costs pen^3 (a 125,000x handicap) and one at the rim
        # costs pen^1 (50x), with everything between graded smoothly. 0
        # restores the flat count.
        self.local_falloff = float(cfg.get("repeat_local_falloff", 2.0))
        # -- HARD REPEAT BAN: never THIS close, no matter the weight ----------
        # `repeat_local_penalty` is a multiplier, deliberately, because a slot
        # whose only fitting model is already standing nearby still has to be
        # filled. But a multiplier can still lose the draw to a heavily
        # height-favoured repeat, and at close range there is no distance at
        # which a viewer reads two copies of the same building touching or
        # near-touching as anything but a rendering glitch — the case the
        # multiplier alone was letting through. Inside `repeat_hard_radius_m`
        # a copy of itself is REMOVED from the candidate list outright rather
        # than just down-weighted; only if that removal would empty the list
        # does the soft multiplier path run instead, so a slot is still filled
        # when the hard-excluded model is the only thing that fits at all.
        #
        # 0 disables it, which is the default — it is a stricter version of
        # `repeat_radius_m` and every existing scene was tuned against the
        # soft penalty alone.
        self.hard_r2 = float(cfg.get("repeat_hard_radius_m", 0.0)) ** 2
        # -- LANDMARKS: the tall tail the height model cannot reach -----------
        # A downtown has a few buildings well above its own norm, and the
        # log-normal cannot produce them here for two compounding reasons,
        # both measured on `downtown_1000` seed 42 over 141 tower slots:
        #
        #   * the target never gets high enough. Median 85 m and sigma 0.45
        #     should reach 150 m occasionally, but `neighbour_weight` blends
        #     each draw halfway (in log space) toward the mean height already
        #     standing within `neighbour_radius_m` — which is the midrise
        #     carpet — so the OBSERVED targets ran p50 74 m, p90 104 m, max
        #     140 m. Not one of 141 reached 150 m. It is a feedback loop:
        #     short neighbours beget shorter targets.
        #   * `pick_sigma` is a Gaussian in LOG height, so the falloff is
        #     exponential. Against a 74 m target the 231 m Amar_Tower scored
        #     0.000533 — a 0.01% share of the draw. Over every slot it reached,
        #     its mean win probability was 0.33%, i.e. 0.04 expected placements.
        #
        # Raising the cap does nothing (the cap was reached once in 141), and
        # raising the median lifts the WHOLE skyline rather than giving it a
        # tail. So a landmark is authored as a budget instead of hoped for as a
        # tail event: `landmark_count` slots may ignore the height match and
        # the packer's area band, and take a tall model outright.
        #
        # 0 disables it, which is the default — `downtown.yaml` and the
        # earthquake preset keep the pure log-normal they were tuned with.
        self.landmark_budget = int(cfg.get("landmark_count", 0))
        self.landmark_min_h = float(cfg.get("landmark_min_height_m", 110.0))
        # -- TALL SEPARATION: two skyscrapers must not stand shoulder to
        # shoulder ------------------------------------------------------------
        # `repeat_hard_radius_m` keeps two copies of the SAME model apart;
        # this is about any two TALL buildings regardless of which models
        # they are — a 312 m tower and a 302 m tower of DIFFERENT models
        # ending up adjacent reads as one mass exactly like two copies of one
        # model do (user, looking at the built scene).
        #
        # Measured between FOOTPRINT RECTANGLES, not centres — see
        # `_tall_ok`. This library runs 42 x 42 m to 60 x 142 m, and a
        # centre-to-centre rule would treat a slender tower next to a broad
        # one completely differently from the reverse; "immediately next to
        # each other" is about the space between the buildings, which the
        # footprints define and the centres do not.
        #
        # A HARD exclusion, not a multiplier like the repeat penalties:
        # unlike a repeated MODEL, a pool always has SHORTER members to fall
        # back to, and a shorter building next to a tower is exactly the
        # right outcome — there is no equivalent of "the only thing that
        # fits is already standing here" to rescue with a soft weight. Only
        # when the filter would empty the candidate list entirely does the
        # slot fall through to whatever is left (`tall_fallback` counts it,
        # so a scene where every tall slot is falling through shows up in
        # the log instead of silently building a wall of towers anyway).
        #
        # BOTH default to 0 (off): a building is never "tall" until
        # `tall_min_h_m` says so, which is what keeps every scene that has
        # not set both knobs unmoved.
        self.tall_min_h = float(cfg.get("tall_min_h_m", 0.0))
        self.tall_gap = float(cfg.get("tall_min_gap_m", 0.0))
        self.tall_fallback = 0

        self.used: dict = {}            # usd -> times placed
        self.at: dict = {}              # usd -> [(x, y)] of each placement
        self.placed: list = []          # (x, y, height_m, footprint_or_None)

    def landmark_picks(self, fits):
        """The tall entries in *fits* if a landmark is still owed, else ().

        *fits* is the packer's PRE-BAND list, and that is deliberate: a tall
        tower is usually slender (Amar_Tower is 2,064 m2 against MBuilding02's
        8,752) and the area band drops it from 92% of the slots it fits in
        before the skyline is ever consulted. A landmark that only competed
        inside the band would be a landmark that never gets built.
        """
        if self.landmark_budget <= 0:
            return ()
        return tuple(f for f in fits if f[0][3]["sz"] >= self.landmark_min_h)

    def took_landmark(self):
        self.landmark_budget -= 1

    def target(self, typ: dict, x: float, y: float) -> float:
        med = float((typ or {}).get("height_median_m", 0.0))
        if med <= 0.0:
            return 0.0
        t = med * math.exp(self.rng.gauss(0.0, float(typ.get("height_sigma", 0.4))))
        cap = typ.get("height_max_m")
        if cap:
            t = min(t, float(cap))
        if self.w > 0.0:
            near = [h for (px, py, h, _fp) in self.placed
                    if (px - x) ** 2 + (py - y) ** 2 <= self.r2]
            if near:
                mean = sum(near) / len(near)
                t = math.exp((1.0 - self.w) * math.log(max(t, 0.5))
                             + self.w * math.log(max(mean, 0.5)))
        return t

    def choose(self, candidates, target: float, x: float = 0.0, y: float = 0.0):
        """Draw one model for a slot at *(x, y)*.

        *(x, y)* is the corner the building will be seated at, which is exactly
        where the packer puts it (`cx = x0 + bw/2`), so it is the true position
        to within half a footprint. That is well inside `repeat_radius_m` and is
        the best available answer: which model lands here is not decided yet, so
        neither is its centre.
        """
        if not candidates:
            return None
        if target <= 0.0:
            return self._pick(candidates, [1.0] * len(candidates), x, y)
        lt = math.log(target)
        w = [math.exp(-((math.log(max(e[3]["sz"], 0.5)) - lt) ** 2)
                      / (2.0 * self.pick_sigma ** 2)) for e in candidates]
        if sum(w) <= 1e-12:
            # Nothing is anywhere near the target height. Falling straight to
            # the nearest match here is what used to hand every such slot to the
            # SAME model, so the penalties get a say: rank by height distance,
            # then let `_pick` choose among the close ones.
            w = [1.0 / (1.0 + abs(math.log(max(e[3]["sz"], 0.5)) - lt))
                 for e in candidates]
        return self._pick(candidates, w, x, y)

    def _near(self, usd, x, y):
        """Distance-graded count of copies of *usd* inside `repeat_radius_m`.

        Returns a float: each copy contributes `1 + local_falloff * (1 - d/R)`,
        so it is >= the plain count and rises as the copies get closer. Used as
        the exponent on `repeat_local_penalty`.
        """
        w = 0.0
        for (px, py) in self.at.get(usd, ()):
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 <= self.local_r2:
                d = math.sqrt(d2 / self.local_r2) if self.local_r2 else 0.0
                w += 1.0 + self.local_falloff * (1.0 - d)
        return w

    def _within(self, usd, x, y, r2):
        return any((px - x) ** 2 + (py - y) ** 2 < r2
                   for (px, py) in self.at.get(usd, ()))

    def _tall_ok(self, sx, sy, x, y):
        """False if a footprint *(sx, sy)* centred at *(x, y)* would land
        within `tall_min_gap_m` of an already-placed TALL building's own
        footprint.

        PLAN-DISTANCE BETWEEN THE TWO RECTANGLES, not between centres — the
        standard axis-aligned-box gap: 0 along an axis where the boxes'
        extents already overlap, the leftover span where they do not. A
        neighbour recorded with no footprint (`record`'s `footprint=None` —
        every pre-existing call site) is treated as a point, i.e. `(0, 0)`
        extent, which is the same "conservative for a point, exact for a
        box" contract `_street_sides`'s tolerance uses elsewhere.
        """
        for (px, py, h, fp) in self.placed:
            if h < self.tall_min_h:
                continue
            psx, psy = fp if fp is not None else (0.0, 0.0)
            dx = max(0.0, abs(x - px) - (sx + psx) / 2.0)
            dy = max(0.0, abs(y - py) - (sy + psy) / 2.0)
            if math.hypot(dx, dy) < self.tall_gap - 1e-6:
                return False
        return True

    def _pick(self, candidates, w, x, y):
        """Weighted draw, damped by how often each model is used and by whether
        one is already standing within sight of *(x, y)*.

        Height match alone concentrates the city on a handful of models: an
        asset is eligible whenever it FITS, so small footprints win far more
        slots than large ones and one model ends up a fifth of every building in
        the scene. Two dampers act on the height weight rather than replacing
        it, so the height distribution survives and only the choice WITHIN a
        height band moves:

        * `(1 + uses) ** -repeat_penalty` — the city-wide histogram. Superlinear
          at the default 1.5, so the tail of a big library actually gets used.
        * `repeat_local_penalty ** (copies within repeat_radius_m)` — the thing
          a viewer sees. This is the strong one by design: a model with one copy
          across the street is at 2% of its odds, with two at 0.04%.

        ROW HOUSES NEVER REACH HERE, and that is the exemption. A terrace is
        laid by `_lay_terrace`/`_tile_run`, which never consult the skyline —
        a brownstone row IS the same house repeated, that is what a terrace is,
        and penalising the repeat would destroy the one typology whose whole
        character is uniformity.
        """
        if self.tall_min_h > 0.0 and self.tall_gap > 0.0:
            # TALL SEPARATION, a hard drop like the repeat-radius one below
            # rather than a multiplier — see `__init__`. `(x, y)` is the
            # slot's corner (same convention `choose` documents); each
            # candidate's own footprint gives a per-candidate centre
            # estimate, which is as precise an answer as exists before the
            # packer has picked an orientation for it.
            #
            # THIS IS ALSO THE LANDMARK PATH. `_pack_free` calls
            # `sky.choose(...)` on the landmark shortlist exactly the same
            # way it does on the ordinary band, and `choose` always funnels
            # into `_pick` — so a landmark candidate is tested here too,
            # without a second copy of this filter at the landmark call
            # site. That matters: `landmark_min_height_m` sits well above
            # `tall_min_h_m` in every scene that sets both, so EVERY
            # landmark candidate is "tall" by this test, which is exactly
            # the case a viewer notices most — two 300 m towers stacked
            # from the landmark budget alone.
            tall = []
            for e, wi in zip(candidates, w):
                sz = e[3]["sz"]
                if sz >= self.tall_min_h:
                    sx, sy = e[3]["sx"], e[3]["sy"]
                    if not self._tall_ok(sx, sy, x + sx / 2.0, y + sy / 2.0):
                        continue          # would stand shoulder to shoulder
                tall.append((e, wi))
            if tall:
                candidates, w = (list(t) for t in zip(*tall))
            else:
                # Every tall candidate is too close to one already standing.
                # NOT rescued by falling back to "place it anyway" — that is
                # the multiplier's job and this is deliberately not a
                # multiplier. Fall through to the ORIGINAL list so a shorter
                # member of the pool can still fill the slot, and count it.
                self.tall_fallback += 1
        if self.hard_r2 > 0.0:
            hard = [(e, wi) for e, wi in zip(candidates, w)
                    if not self._within(e[0], x, y, self.hard_r2)]
            if hard:
                candidates, w = (list(t) for t in zip(*hard))
            # else: every candidate already has a copy inside the hard
            # radius — fall through to the soft path below with the ORIGINAL
            # lists, because something still has to fill this slot.
        if self.repeat > 0.0:
            w = [wi / ((1.0 + self.used.get(e[0], 0)) ** self.repeat)
                 for e, wi in zip(candidates, w)]
        if 0.0 <= self.local_pen < 1.0 and self.local_r2 > 0.0:
            w = [wi * (self.local_pen ** self._near(e[0], x, y))
                 for e, wi in zip(candidates, w)]
        total = sum(w)
        if total <= 1e-12:
            # Every candidate is penalised into the floor — the slot's only
            # fitting models are all standing nearby. Something has to go here,
            # so fall back deterministically rather than to chance: FEWEST
            # COPIES NEARBY first, global count only as the tie-break. That
            # order is the whole point — a twin 40 m away is what a viewer sees
            # and a flat city-wide histogram is not, so the visible term has to
            # win when the two disagree.
            chosen = min(candidates, key=lambda e: (self._near(e[0], x, y),
                                                    self.used.get(e[0], 0)))
            return self._take(chosen, x, y)
        r = self.rng.random() * total
        for e, wi in zip(candidates, w):
            r -= wi
            if r <= 0.0:
                return self._take(e, x, y)
        return self._take(candidates[-1], x, y)

    def _take(self, e, x, y):
        """Book a model as used at *(x, y)* and return it."""
        self.used[e[0]] = self.used.get(e[0], 0) + 1
        self.at.setdefault(e[0], []).append((float(x), float(y)))
        return e

    def record(self, x, y, height, footprint=None):
        """*footprint*, when given, is ``(sx, sy)`` in world plan dimensions
        — used only by the tall-separation filter (`_tall_ok`). ``None``
        (the default) treats the building as a point; every call site that
        predates the tall-separation feature still works unchanged."""
        self.placed.append((x, y, height, footprint))

    def note(self, usd, x, y):
        """Book a model that some EARLIER pass already stood at *(x, y)*.

        `infill_blocks` builds a second `_Skyline` over a city `rezone_blocks`
        has already filled, and without this that instance starts with an empty
        histogram and an empty position index: both repeat penalties see 123
        infill buildings and none of the 297 already standing, so the smallest
        model that fits a leftover gap wins every leftover gap in the city. The
        heights were already being carried across (`record`); this carries the
        identities, which is what the penalties are actually about.
        """
        if not usd:
            return
        self.used[usd] = self.used.get(usd, 0) + 1
        self.at.setdefault(usd, []).append((float(x), float(y)))

    def diversity(self):
        """``(models_used, placements, share_of_the_most_used)`` — the numbers
        the two penalties exist to move, so a run can be judged without a
        render."""
        n = sum(self.used.values())
        return (len(self.used), n,
                (max(self.used.values()) / n) if n else 0.0)


# ---------------------------------------------------------------------------
# free-space decomposition
# ---------------------------------------------------------------------------

def free_rects(rect, obstacles, min_side: float = 1.0):
    """Decompose *rect* minus *obstacles* into axis-aligned free rectangles.

    Sweeps the obstacle x-edges into columns, takes each column's free
    y-intervals, then merges columns sharing an interval — so a strip of open
    ground comes back as one wide rectangle rather than a row of slivers.
    """
    x0, y0, x1, y1 = rect
    if x1 - x0 < min_side or y1 - y0 < min_side:
        return []
    obs = [o for o in obstacles
           if o[2] > x0 and o[0] < x1 and o[3] > y0 and o[1] < y1]

    xs = sorted({x0, x1} | {min(max(v, x0), x1)
                            for o in obs for v in (o[0], o[2])})
    cells: list = []
    for cx0, cx1 in zip(xs, xs[1:]):
        if cx1 - cx0 < 1e-6:
            continue
        blocked = sorted((o[1], o[3]) for o in obs
                         if o[0] < cx1 - 1e-6 and o[2] > cx0 + 1e-6)
        cursor, spans = y0, []
        for by0, by1 in blocked:
            if by0 > cursor:
                spans.append((cursor, min(by0, y1)))
            cursor = max(cursor, by1)
            if cursor >= y1:
                break
        if cursor < y1:
            spans.append((cursor, y1))
        cells += [[cx0, a, cx1, b] for a, b in spans if b - a > 1e-6]

    merged: list = []
    for c in cells:
        for m in merged:
            if (abs(m[2] - c[0]) < 1e-6 and abs(m[1] - c[1]) < 1e-6
                    and abs(m[3] - c[3]) < 1e-6):
                m[2] = c[2]
                break
        else:
            merged.append(c)
    return [tuple(m) for m in merged
            if m[2] - m[0] >= min_side and m[3] - m[1] >= min_side]


def _rect_of(p, resolver, category=None, margin=0.0):
    cat = category or p.get("category")
    fp = resolver.get(p["usd"], cat, scale=p.get("scale"),
                      axis_up=p.get("axis_up", "Z"))
    sx, sy = fp["sx"], fp["sy"]
    if abs((float(p.get("yaw_deg", 0.0)) % 180.0) - 90.0) < 45.0:
        sx, sy = sy, sx
    x, y = float(p["x_m"]), float(p["y_m"])
    return (x - sx / 2.0 - margin, y - sy / 2.0 - margin,
            x + sx / 2.0 + margin, y + sy / 2.0 + margin)


def _placement_rects(placements, resolver, categories, margin: float = 0.0):
    return [_rect_of(p, resolver, margin=margin) for p in placements
            if p.get("category") in categories]


def park_blocks(layout: dict, placements: list):
    """The blocks that are parks.

    Preferred source is `city_layout.PARK_RESERVES` — superblocks the
    subdivider carved out whole, with streets on their sides and no road
    through them. Falling back to trail-tile detection covers a run where the
    subdivider is the built-in one and `build_city` chose the parks itself.

    "Has no buildings" is deliberately NOT the test: a block where nothing in
    the library fit is also empty, and that block needs building on, not a
    footpath through it.
    """
    blocks = layout.get("blocks", [])
    try:
        from layout import city_layout
        reserved = [b for b in city_layout.PARK_RESERVES if b in blocks]
    except ImportError:
        reserved = []
    if reserved:
        return reserved
    trails = [(float(p["x_m"]), float(p["y_m"])) for p in placements
              if p.get("category") == "trail"]
    return [b for b in blocks
            if any(b[0] <= x <= b[2] and b[1] <= y <= b[3]
                   for x, y in trails)]


def block_inset(config: dict, resolver):
    """How far `build_city` holds block content off the block edge: a verge
    plus a sidewalk one tile wide. Recomputed here because the generator keeps
    the inset local to its packing loop."""
    tiles = (config.get("usds") or {}).get("tiles") or {}
    raw = tiles.get("sidewalk") or tiles.get("brick") or []
    default = float(config.get("asset_scale", 1.0))
    paths, sc_ovr, _au, _yaw, _tags = _normalize_usd_list(
        raw, default, str(config.get("asset_root", "") or ""))
    sw = 0.0
    if paths:
        fp = resolver.get(paths[0], "sidewalk", scale=sc_ovr.get(paths[0], default))
        sw = max(fp["sx"], fp["sy"])
    return float(config.get("frontage", {}).get("verge_m", 0.0)) + sw


# ---------------------------------------------------------------------------
# morphology: terrace
# ---------------------------------------------------------------------------

def _tile_run(length: float, pool, rng, jitter=0.35, no_repeat: bool = False):
    """Choose row assets whose lengths sum as close to *length* as possible.

    Longest-first with a random pick among the top few, so consecutive block
    faces don't all come out as the same sequence. *pool* entries are measured
    with their long axis on y.

    *no_repeat* — NEVER THE SAME USD TWICE IN A ROW. A HARD guarantee, not a
    preference: if the only entry that still fits the remaining span is the
    one just placed, the run ENDS there rather than repeating it — the first
    version of this fell back to placing it anyway ("something has to fill
    the space") and still produced Brownstone02-Brownstone02 pairs in 6 of
    25 multi-house runs on `downtown`, because a narrow leftover span after
    a long house is often only wide enough for the pool's SHORTEST member,
    twice. A run ending a little short of `length` is what `_lay_terrace`'s
    end-of-face centring already tolerates; a forced duplicate is not
    tolerable at any length. Beyond that, a USD not yet anywhere in this run
    is preferred over one already used, when both still fit: `_tile_run`
    never reaches `_Skyline` (ROW HOUSES NEVER REACH HERE — see that class's
    docstring), so nothing else in the pipeline damps a repeat inside one
    run, and two identical houses back to back inside a run is the single
    worst case a repeat can be — they are touching.

    OFF BY DEFAULT (`districts.terrace_no_repeat`, read once by
    `rezone_blocks`), and the two branches below are kept as two SEPARATE
    loops rather than one with the filtering made conditional, on purpose:
    narrowing the candidate list changes not just WHICH entry a draw picks
    but whether `rng.random()` is even called at all (the `len(fits) == 1`
    short-circuit below), which perturbs the RNG stream for every scene that
    reaches a terrace, including one with no facing metadata whatsoever —
    `downtown`'s eight-brownstone rowhouse pool went 216 -> 223 buildings
    with this rule unconditional, because the shifted stream propagates to
    every later draw in the pipeline. `downtown_earthquake` builds on
    `downtown` and is mid-flight; a silent restyle there is not this
    feature's business, so OFF has to reproduce the ORIGINAL algorithm
    exactly, draw for draw, not merely "the same choices most of the time."
    """
    by_len = sorted(pool, key=lambda e: -e[3]["sy"])
    if not by_len:
        return []
    shortest = by_len[-1][3]["sy"]
    out, rem = [], length
    if not no_repeat:
        while rem >= shortest - 1e-6:
            fits = [e for e in by_len if e[3]["sy"] <= rem + 1e-6]
            if not fits:
                break
            k = (1 if len(fits) == 1 or rng.random() > jitter
                 else min(len(fits), 2))
            e = fits[rng.randrange(k)]
            out.append(e)
            rem -= e[3]["sy"]
        return out
    used = set()
    while rem >= shortest - 1e-6:
        fits = [e for e in by_len if e[3]["sy"] <= rem + 1e-6]
        if not fits:
            break
        last = out[-1][0] if out else None
        not_last = [e for e in fits if e[0] != last]
        if not not_last:
            # The ONLY thing that still fits the remaining span is the piece
            # just placed — MEASURED to fire repeatedly on the brownstone
            # pool: after a longer row eats most of `want`, the leftover is
            # often inside [shortest, 2*shortest), where only the shortest
            # model (6.8 m Brownstone02) fits at all, twice. Falling back to
            # "place it again anyway" is what first shipped here, and it
            # still produced Brownstone02-Brownstone02 pairs in 6 of 25
            # multi-house runs — visibly worse than the plain length-driven
            # algorithm was ever accused of, since these two are TOUCHING.
            # Ending the run here instead leaves it short of `length`, which
            # `_lay_terrace`'s end-of-face centring already tolerates; a
            # forced duplicate is not tolerable at any length.
            break
        fresh = [e for e in not_last if e[0] not in used]
        cand = fresh or not_last
        k = 1 if len(cand) == 1 or rng.random() > jitter else min(len(cand), 2)
        e = cand[rng.randrange(k)]
        out.append(e)
        used.add(e[0])
        rem -= e[3]["sy"]
    return out


def _tile_run_ended(length: float, pool, rng, jitter: float = 0.35,
                    no_repeat: bool = False):
    """Build one terrace run END-FIRST: the two houses that will stand at
    the run's two open ends are chosen BEFORE the interior, from stock that
    may legally stand there (`place` != `mid`), and the interior is filled
    only with whatever room is left between them.

    WHY END-FIRST, NOT END-FILTERED. `_tile_run` picks purely by length, and
    only afterward did `_order_run` ask whether the result has two
    end-capable pieces for its ends — dropping the whole run if not. Against
    a pool that is mostly `mid` (measured: `brick_midrise`'s first cut was
    8 of 11 members `mid`, because 09/21/22/08 are 44-86 m deep, too deep to
    terrace at all, leaving three shallow end-capable buildings in the whole
    library) most length-driven draws come back all-`mid` and get thrown
    away wholesale — the run either vanishes or degenerates to whatever thin
    slice of end-capable stock survived. MEASURED: that produced a "terrace
    district" of 6 buildings on 3 blocks, 2 per block, the exact defect this
    function exists to fix — end-capacity is a constraint ON the tiling now,
    not a filter applied after it.

    ALGORITHM. `end_capable` is every entry whose `place` is not `mid`
    (`end`/`corner`/`any`). `_face_runs` only calls this function when the
    pool has a `mid` member at all (see `ends_matter` there) — an untagged
    pool, or one with no `mid`, has nothing here to protect and takes the
    plain `_tile_run` path unchanged.

      1. Can the two SMALLEST end-capable entries even fit together in
         *length*? If not, no bracketed (2+-piece) run is possible at this
         length, so the run is a single `place_any` piece if one fits — the
         existing "a run of ONE only takes `any`" rule — or nothing.
      2. FIRST END: chosen to leave room for an interior AND a close when
         that is at all achievable — NOT just the longest thing that fits,
         which is what `_tile_run`'s own style would do. MEASURED on
         `brick_midrise` (2 `any` at 28.4/42.4 m, 1 `end` at 28.9 m, 8 `mid`
         from 14.4-28.8 m): picking the 42.4 m `any` first because it is
         longest left only 9.7 m of interior budget against a 14.4 m
         shortest `mid` — every run came back a bracket with NO interior, or
         a lone `any`, never a real row. So `first` draws PREFERENTIALLY
         from end-capable stock short enough to leave room for the close
         AND at least one interior piece; only when nothing end-capable
         leaves that much room does it fall back to "leaves room for the
         close alone" (which `2*min_end <= length`, checked above,
         guarantees is non-empty).
      3. INTERIOR: filled preferentially from `mid` stock (that is what an
         interior house's blank flanks are FOR — covered by a neighbour on
         each side), longest-first-with-jitter, falling back to any pool
         member when no `mid` piece fits the remaining gap so a fillable gap
         is never left bare for want of the "right" class. Stops once what
         remains would leave no room for the reserved close.
      4. CLOSE: the second end, drawn from whatever end-capable stock fits
         whatever is ACTUALLY left after the interior — non-empty by the
         reservation in step 2/3, so this step never fails.

    Only step 1 (no bracket possible) or an empty `end_capable`/no fitting
    `any` can end the run before it starts. That is the whole point: "only
    drop a run when it genuinely cannot be laid," never because a
    length-driven draw happened to pick badly.

    *no_repeat* narrows each of the three draws exactly as `_tile_run`
    does — never the immediately-preceding USD, and a USD not yet anywhere
    in the run preferred over one already used, when either still fits.
    """
    def sy(e):
        return e[3]["sy"]

    def draw(cands, last_usd, used_here):
        by_len = sorted(cands, key=lambda e: -sy(e))
        if no_repeat:
            not_last = [e for e in by_len if e[0] != last_usd]
            base = not_last or by_len
            fresh = [e for e in base if e[0] not in used_here]
            cand = fresh or base
        else:
            cand = by_len
        k = (1 if len(cand) == 1 or rng.random() > jitter
             else min(len(cand), 2))
        return cand[rng.randrange(k)]

    def lone_any(budget):
        any_fits = [e for e in pool if e[5].get("place", "any") == "any"
                   and sy(e) <= budget + 1e-6]
        return [draw(any_fits, None, set())] if any_fits else []

    end_capable = [e for e in pool if e[5].get("place", "any") != "mid"]
    if not end_capable:
        return lone_any(length)          # nothing to bracket with at all

    min_end = min(sy(e) for e in end_capable)
    if 2.0 * min_end > length + 1e-6:
        return lone_any(length)          # cannot bracket two ends here

    shortest_pool = min(sy(e) for e in pool)
    # Tier A leaves room for the close AND >= 1 interior piece; tier B (the
    # old, sole test) leaves room for the close alone. A is tried first.
    first_fits = [e for e in end_capable
                 if sy(e) <= length - min_end - shortest_pool + 1e-6]
    if not first_fits:
        first_fits = [e for e in end_capable
                     if sy(e) <= length - min_end + 1e-6]
    if not first_fits:
        # `2*min_end <= length` already guarantees the piece achieving
        # `min_end` itself qualifies tier B, so this is unreachable outside
        # floating-point boundary noise — kept as a defensive fallback
        # rather than trusted to never fire.
        return lone_any(length)

    first = draw(first_fits, None, set())
    out = [first]
    used = {first[0]}
    rem = length - sy(first)

    # INTERIOR, reserving `min_end` off the top so the close always has
    # somewhere to land.
    budget = rem - min_end
    while budget >= shortest_pool - 1e-6:
        mid_fits = [e for e in pool if e[5].get("place", "any") == "mid"
                   and sy(e) <= budget + 1e-6]
        fits = mid_fits or [e for e in pool if sy(e) <= budget + 1e-6]
        if not fits:
            break
        piece = draw(fits, out[-1][0], used)
        out.append(piece)
        used.add(piece[0])
        budget -= sy(piece)
        rem -= sy(piece)

    # CLOSE. `rem` >= `min_end` by construction (the interior loop only ever
    # spends out of `budget = rem - min_end`, never touching the reserve),
    # so `close_fits` is never empty.
    close_fits = [e for e in end_capable if sy(e) <= rem + 1e-6]
    close = draw(close_fits, out[-1][0], used)
    out.append(close)
    return out


def _perimeter_rects(rect, depth: float, min_side: float):
    """The block's frontage BAND, as up to four rects, centre left out.

    Guillotine-packing the whole block interior lets a building sit anywhere in
    it, so a block often ends up built at the back with a deep apron of bare
    paving at the street. From the pavement that apron reads as an enormously
    wide sidewalk — it is not sidewalk at all, it is unbuilt block interior.

    A dense urban block is built to its edges with the slack in the MIDDLE,
    which is what this produces: pack the band, leave the courtyard. Returns the
    whole rect when it is too shallow to have a distinct middle.
    """
    x0, y0, x1, y1 = rect
    w, h = x1 - x0, y1 - y0
    if depth <= 0.0 or w <= 2.0 * depth + min_side or h <= 2.0 * depth + min_side:
        return [rect]
    return [(x0, y0, x1, y0 + depth),               # south frontage
            (x0, y1 - depth, x1, y1),               # north
            (x0, y0 + depth, x0 + depth, y1 - depth),   # west, between them
            (x1 - depth, y0 + depth, x1, y1 - depth)]   # east


def _split_along(blk, spans, axis):
    """The block cut into the pieces BETWEEN *spans*, spans left as gaps."""
    x0, y0, x1, y1 = blk
    lo, hi = (y0, y1) if axis == "x" else (x0, x1)
    cuts = sorted(spans)
    out, edge = [], lo
    for a, b in cuts + [(hi, hi)]:
        if a - edge > 1.0:
            out.append((x0, edge, x1, a) if axis == "x"
                       else (edge, y0, a, y1))
        edge = b
    return out


def _add_internal_street(layout, rect, span, axis, road_w, typ):
    """Register the terrace superblock's own street as a real corridor.

    `apply_ground_planes` lays asphalt and lane markings from
    ``layout["road_corridors"]``, and it runs AFTER this pass, so appending
    here is enough to have the street actually built rather than drawn as a
    gap in the paving. It is a residential street by construction: two lanes,
    parking both sides, which is what a row-house street is.
    """
    lo, hi = span
    x0, y0, x1, y1 = rect
    park_w = float(typ.get("street_park_w_m", 2.4))
    car_lo, car_hi = lo + park_w, hi - park_w
    c = {"n_lanes": 2, "carriage": [car_lo, car_hi], "park_w": park_w,
         "parking": "both", "zone": "edge", "road_class": "local",
         "internal": True}
    if axis == "x":                       # street runs along X, spans in Y
        c.update({"x0": x0, "x1": x1, "y0": lo, "y1": hi, "dir": "ew"})
    else:
        c.update({"x0": lo, "x1": hi, "y0": y0, "y1": y1, "dir": "ns"})
    layout.setdefault("road_corridors", []).append(c)


def _terrace_quad(rect, depth: float, alley_m: float, road_w: float):
    """Terrace rows on their OWN finer grid, or ``None``.

        row | alley | row  ||street||  row | alley | row  ||street||  ...

    As many back-to-back pairs as the block will take, each pair separated from
    the next by a street of its own. A terrace pair on a deep block leaves its
    back row fronting nothing, and filling the leftover with unrelated buildings
    is what makes brownstones look like they are sharing someone else's block.
    Real row-house districts answer this by being cut FINER than the rest of the
    city — smaller blocks, more streets — so the superblock brings its own.

    Returns ``(strips, [road_span, ...], axis)``. Each strip carries its own
    facing, so every pair fronts outward to whatever bounds it.
    """
    x0, y0, x1, y1 = rect
    w, h = x1 - x0, y1 - y0
    along_x = w >= h
    short = h if along_x else w
    pair = 2.0 * depth + alley_m
    # Most pairs that fit, ending on a pair rather than a street.
    n = int((short + road_w) // (pair + road_w))
    if n < 2:
        return None                       # a single pair is not a superblock
    need = n * pair + (n - 1) * road_w
    base = (y0 if along_x else x0) + (short - need) / 2.0

    strips, roads = [], []
    for i in range(n):
        p0 = base + i * (pair + road_w)
        outer, inner = p0, p0 + depth + alley_m
        if along_x:
            strips.append(((x0, outer, x1, outer + depth), "x", True))
            strips.append(((x0, inner, x1, inner + depth), "x", False))
        else:
            strips.append(((outer, y0, outer + depth, y1), "y", True))
            strips.append(((inner, y0, inner + depth, y1), "y", False))
        if i < n - 1:
            r0 = p0 + pair
            roads.append((r0, r0 + road_w))
    return strips, roads, ("x" if along_x else "y")


def _next_typology(typologies: dict, name: str):
    """The next typology up the intensity ladder, for a refused block.

    Excludes every terrace-morphology typology, not just *name* itself — a
    refused terrace should never fall back to another terrace, with two of
    them (`rowhouse` and a second one) that now matters for real. It also
    cannot return *name*: the `rank > here` test is strict, so a candidate
    with *name*'s own rank (itself included) never qualifies regardless of
    morphology.
    """
    here = (typologies.get(name) or {}).get("rank", 0)
    up = sorted(((t.get("rank", 0), n) for n, t in typologies.items()
                 if t.get("rank", 0) > here
                 and str(t.get("morphology", "pack")) != "terrace"))
    return up[0][1] if up else None


def _terrace_fit_pool(pool, block_short: float, alley_m: float):
    """Narrow *pool* to the entries a terrace pair on a block of short side
    *block_short* can actually hold, and the deepest depth among them —
    ``(fitting_pool, depth)``, or ``([], 0.0)`` when nothing fits at all.

    `_lay_terrace` used to size the WHOLE admissible-block band off the
    pool's single deepest member (`max(e[3]["sx"] for e in pool)`), so
    appending one deep building moved the band for every block in the scene.
    MEASURED: appending 31.5 m GreatAmericanCity stock to a pool of 21.1 m
    AEC brownstones moved the band to [65, 89] m and refused all eight
    existing brownstone blocks outright, with nothing reporting why beyond
    `rowhouse_refused=8`. Filtering to what fits THIS block's short side
    before taking the max keeps the shallow blocks building shallow stock and
    only admits the deep stock on blocks that can actually hold it — a block
    is disqualified by its own size now, never by an unrelated pool member's.
    """
    fitting = [e for e in pool if 2.0 * e[3]["sx"] + alley_m <= block_short]
    if not fitting:
        return [], 0.0
    return fitting, max(e[3]["sx"] for e in fitting)


def _terrace_band(depth: float, alley_m: float, alley_max_m: float = 0.0):
    """The admissible block-short-side band a terrace PAIR needs —
    ``(lo, hi)`` — factored out of `_terrace_strips` so `_select_probe_blocks`
    tests a candidate block against the EXACT same arithmetic rather than a
    second copy of it that can drift. *depth* is the pool's deepest member
    (`sx`, already post yaw-offset swap — see `_pool_entries`), tight end at
    ``2*depth + alley_m``, loose end at ``2*depth + alley_max_m`` (or
    ``2*depth + alley_m*2.5`` when no `alley_max_m` is configured).
    """
    lo = 2.0 * depth + alley_m
    hi = 2.0 * depth + (alley_max_m if alley_max_m > alley_m else alley_m * 2.5)
    return lo, hi


def _terrace_strips(rect, depth: float, alley_m: float, alley_max_m: float = 0.0):
    """Frontage strips for a perimeter terrace block, or NOTHING.

    A terrace block is two rows back to back with a service alley between them
    and nothing else. Both other outcomes this used to produce are wrong:

    * ONE row down the middle of a narrow block. It leaves the block's own
      frontage unbuilt on both sides, which reads as an absurdly wide pavement.
    * Two rows on the FACES of a block far deeper than two terraces need, which
      leaves a courtyard-sized void between their backs rather than an alley.

    So the block short side has to land in a band — 2*depth + alley_m at the
    tight end, 2*depth + alley_max_m at the loose end — and a block outside it
    is refused. The caller falls back to another morphology, which is correct:
    a block that cannot take a terrace pair is not a terrace block.
    """
    x0, y0, x1, y1 = rect
    w, h = x1 - x0, y1 - y0
    lo, hi = _terrace_band(depth, alley_m, alley_max_m)
    short = min(w, h)
    if short < lo:
        return []                               # cannot fit a pair at all
    if short > hi:
        # Deeper than a terrace pair needs. This used to keep both rows against
        # ONE face and hand the remainder back to the packer, which is why the
        # pavement came out 2.00 m on one side of the block and 25.30 m on the
        # other — MEASURED, on the 84-92 m blocks. Neither way out is any good:
        # rows on opposite faces leave a courtyard where an alley belongs, and
        # rows on one face leave that lopsided strip. So refuse, exactly as the
        # too-narrow case does, and let the block build as the next typology up.
        # A block that cannot take a terrace pair is not a terrace block.
        return []
    if w >= h:                                  # faces run along X
        return [((x0, y0, x1, y0 + depth), "x", True),
                ((x0, y1 - depth, x1, y1), "x", False)]
    return [((x0, y0, x0 + depth, y1), "y", True),
            ((x1 - depth, y0, x1, y1), "y", False)]


def _face_runs(face_len: float, pool, rng, med: float, sigma: float,
               gap_lo: float, gap_hi: float, no_repeat: bool = False):
    """Break a block face into terrace runs separated by gaps.

    MEASURED (Philadelphia, Boston Back Bay, Baltimore; contiguous party-wall
    runs at a 1.0 m wall gap): a run is 6 houses / 43-48 m of frontage at the
    median and 17-29 houses / 105-130 m at p90 — NOT a whole block face. Filling
    the face end to end makes a 120 m unbroken wall, which is why runs are
    sampled log-normally instead. 93-97% of row-house stock sits in runs of >=3,
    so single houses only ever appear closing out a run.

    *no_repeat* passes straight through to whichever tiler is used — see
    `_tile_run`'s docstring for why OFF has to be the literal original
    algorithm rather than a conditional inside one shared loop.

    ENDS_MATTER decides which tiler: `_tile_run_ended` (end-capacity as a
    construction constraint) when *pool* has any `place_mid` member, plain
    `_tile_run` (length only) when it does not. Computed ONCE per strip, not
    per run, because the pool does not change within a strip. A pool with no
    `mid` — including every pool with NO place tags at all, since the
    untagged default is `any` — has nothing for the end-anchored
    construction to protect, so it never reaches that code path: this is
    what keeps a metadata-less pool (`downtown`'s brownstones) on the
    ORIGINAL `_tile_run` call, unchanged, draw for draw.
    """
    shortest = min(e[3]["sy"] for e in pool)
    ends_matter = any(e[5].get("place", "any") == "mid" for e in pool)
    out, cursor = [], 0.0
    while face_len - cursor >= shortest:
        want = min(face_len - cursor, med * math.exp(rng.gauss(0.0, sigma)))
        chosen = (_tile_run_ended(want, pool, rng, no_repeat=no_repeat)
                  if ends_matter else
                  _tile_run(want, pool, rng, no_repeat=no_repeat))
        if not chosen:
            break
        used = sum(e[3]["sy"] for e in chosen)
        out.append((cursor, chosen))
        cursor += used + rng.uniform(gap_lo, gap_hi)
    return out


def _order_run(chosen, rng):
    """Reorder one terrace run so a `mid` piece never ends up at either end,
    or refuse the run outright when it cannot be arranged legally.

    A `place_mid` asset is blank on BOTH flanks (`gac_faces.py`'s 1-modelled-
    side case) — it only ever reads correctly with a neighbour covering each
    side, i.e. strictly inside a run. `end`/`corner`/`any` have at least one
    flank covered (or none needed) and can close a run out. A run of exactly
    ONE shows both its flanks to the open air, so it only accepts `any` — a
    lone `mid` or `end` would bare a flank nothing is there to hide.

    Documented in `urban_gac.yaml` as `districts._order_run` before this
    function existed to do it: "orders each terrace run by these and DROPS a
    run it cannot lay legally, rather than laying a blank wall against a
    street." Returns the reordered list, or ``[]`` — the caller must skip a
    ``[]`` run rather than lay it, exactly as it already skips a block whose
    terrace band the short side falls outside: a hole in a terrace is a
    vacant lot, which is real; a blank wall at the end of a row is not.

    THIS IS NOW AN INVARIANT CHECK, NOT THE MECHANISM. `_tile_run_ended`
    builds a run end-first — it puts a legal end-capable piece at both ends
    BEFORE it ever touches the interior — so by the time a run reaches here
    it should ALREADY satisfy everything below, and this function is just
    confirming that rather than repairing it. It is kept anyway as the
    single place that enforces the rule, in front of every source of a
    `chosen` list this pass has (including plain `_tile_run`, which knows
    nothing about `place` at all). A run this function actually has to
    DROP or REORDER for cause — as opposed to the harmless cosmetic
    reshuffling it still does among interchangeable end-capable pieces —
    means the construction step upstream produced something illegal, which
    is a real bug in `_tile_run_ended`, not routine attrition to expect in
    normal operation.
    """
    n = len(chosen)
    if n == 0:
        return []
    places = [(e[5].get("place", "any") if len(e) > 5 else "any")
             for e in chosen]
    if n == 1:
        return [chosen[0]] if places[0] == "any" else []
    # NOTHING TO ORDER when no piece in the run is `mid` — every entry can
    # legally stand at an end, so `_tile_run`'s own sequence is already
    # legal and reordering it would only spend an `rng.shuffle` proving
    # that. This is what keeps a pool nobody has tagged (`place="any"`
    # everywhere, the untagged default) byte-identical to before this
    # function existed: it never draws from `rng` and never touches the
    # sequence for exactly the pools this feature has nothing to say about.
    if "mid" not in places:
        return list(chosen)
    idx = list(range(n))
    end_ok = [i for i in idx if places[i] != "mid"]
    if len(end_ok) < 2:
        return []              # not enough flank-covered stock to close it
    rng.shuffle(end_ok)
    first_i, last_i = end_ok[0], end_ok[1]
    middle_i = [i for i in idx if i not in (first_i, last_i)]
    rng.shuffle(middle_i)
    return [chosen[i] for i in ([first_i] + middle_i + [last_i])]


def _lay_terrace(rect, pool, rng, facing_deg: float, alley_m: float,
                 run_median_m: float = 45.0, run_sigma: float = 0.70,
                 run_gap_m=(3.0, 9.0), alley_max_m: float = 0.0,
                 strips=None, no_repeat: bool = False):
    """Place party-wall rows around *rect*. Returns ``[(entry, x, y, yaw)]``.

    Rows inside a run butt end to end with no gap — a terrace shares walls, and
    the 2.5 m packing gap between them is exactly what makes a row read as
    detached houses. Between runs there IS a gap: a side alley or a vacant lot.

    DEPTH IS THE POOL'S DEEPEST, NOT ITS SHALLOWEST, and that only started to
    matter when the pool stopped being eight brownstones of identical depth.
    `_terrace_strips` uses this number twice — for the strip itself and for the
    admissible block-short-side band `[2*depth + alley, 2*depth + alley_max]` —
    while each house below is then seated on the street edge at ITS OWN depth.
    With `urban_v2`'s seven standalone terrace houses in the pool (3.9-8.4 m
    against the brownstones' 21.1 m) the minimum put that band at [10, 18] m,
    which no downtown block is in: every terrace block would have been refused
    and rebuilt as mid-rise, and the rowhouse typology would have come out at
    ZERO with nothing anywhere reporting why. The maximum leaves the band
    exactly where the brownstones tuned it and lets a shallow house sit on the
    same frontage line with a deeper back alley behind it, which is what a
    mixed-depth terrace actually looks like.

    FACING: each strip has ONE outward yaw for every building in it (a row's
    whole point is a shared, consistent front), so the pool is filtered once
    per strip to entries whose `blank:` sides do not include that outward
    direction — an entry that would show a blank wall to this particular
    street cannot be used on THIS strip even if it is fine on the opposite
    one. `facing_deg` 0 assumes -X (WEST) is the canonical front every
    asset's `yaw-offset` has already been turned to (see the module
    docstring's per-asset `yaw-offset` note and `tools/faces_to_yaw.py`'s
    `FRONT_TO_YAW` table), so the outward letter for a given strip yaw is
    ``_rot_side("W", yaw)`` — the same rotation `_pack_free` applies to
    `blank0`, just solved from the strip's geometry instead of a candidate
    footprint's.
    """
    x0r, y0r, x1r, y1r = rect
    pool, depth = _terrace_fit_pool(pool, min(x1r - x0r, y1r - y0r), alley_m)
    if not pool:
        return []
    out = []
    for (sx0, sy0, sx1, sy1), axis, near_lo in (
            strips if strips is not None
            else _terrace_strips(rect, depth, alley_m, alley_max_m)):
        face_len = (sx1 - sx0) if axis == "x" else (sy1 - sy0)
        if axis == "x":
            yaw = 90.0 + (facing_deg if near_lo else 180.0 + facing_deg)
        else:
            yaw = facing_deg if near_lo else 180.0 + facing_deg
        outward = _rot_side("W", yaw)
        strip_pool = [e for e in pool
                     if outward not in _rot_sides(
                         (e[5].get("blank0") if len(e) > 5 else None)
                         or frozenset(), yaw)]
        if not strip_pool:
            continue           # nothing in the pool may legally face here
        runs = _face_runs(face_len, strip_pool, rng, run_median_m, run_sigma,
                          run_gap_m[0], run_gap_m[1], no_repeat=no_repeat)
        if not runs:
            continue
        # Centre the whole sequence on the face so the end gaps match. Uses
        # the RAW runs (pre-`_order_run`), which is correct — reordering a
        # run's contents does not change its total length.
        span = runs[-1][0] + sum(e[3]["sy"] for e in runs[-1][1])
        shift = (face_len - span) / 2.0
        for start, chosen in runs:
            chosen = _order_run(chosen, rng)
            if not chosen:
                # This run cannot be laid with a legal flank arrangement —
                # dropped, not laid wrong. Leaves a gap in the frontage,
                # which is a vacant lot; that is a real thing a block face
                # has, a `place_mid` house standing alone at the end of a
                # row is not.
                continue
            cursor = start + shift
            positions = []          # (entry, cx, cy, world_w, world_h)
            for e in chosen:
                ln, dp = e[3]["sy"], e[3]["sx"]
                if axis == "x":
                    # Depth may be less than the strip; sit the row on the
                    # street edge, the face nearer the block boundary.
                    cx = sx0 + cursor + ln / 2.0
                    cy = sy0 + dp / 2.0 if near_lo else sy1 - dp / 2.0
                    w, h = ln, dp
                else:
                    cy = sy0 + cursor + ln / 2.0
                    cx = sx0 + dp / 2.0 if near_lo else sx1 - dp / 2.0
                    w, h = dp, ln
                positions.append((e, cx, cy, w, h))
                cursor += ln
            # THE TWO ENDS OF THE RUN, AND ONLY THEM, CAN HAVE AN EXPOSED
            # FLANK. Every interior house's flanks are covered by its two
            # neighbours by construction, so the per-strip `outward`-only
            # filter above is sufficient for them — but a house at either
            # END of a run has one side open to whatever lies beyond the
            # run, and when a run ends at a block CORNER that open side can
            # itself be a street the outward check never looks at (it only
            # ever tests the row's single shared front/back direction).
            # MEASURED: this is what a `place_end` piece with `front:S,
            # blank:N,W` produced at a superblock corner — 3 of 3 real
            # violations traced to this pass were exactly this shape. Full
            # `_street_sides` on just these two houses catches it; checking
            # every house in the run would too, but redundantly.
            ok = True
            for e, cx, cy, w, h in (positions[0], positions[-1]):
                blank0 = (e[5].get("blank0") if len(e) > 5 else None) \
                    or frozenset()
                if not blank0:
                    continue
                fx0, fy0 = cx - w / 2.0, cy - h / 2.0
                fsides = _street_sides(rect, fx0, fy0, w, h)
                if _rot_sides(blank0, yaw) & fsides:
                    ok = False
                    break
            if not ok:
                # Dropped for the same reason an unlayable `_order_run` is:
                # the run's own composition is fine, but where it LANDS puts
                # a blank flank on a street its outward face was never
                # tested against. A shorter frontage here, not a bad one.
                continue
            for e, cx, cy, w, h in positions:
                out.append((e, cx, cy, yaw))
    return out


# ---------------------------------------------------------------------------
# morphology: pack
# ---------------------------------------------------------------------------

def _street_reach(block_rect, max_m: float):
    """``reach(x0, y0, w, h) -> bool`` — does this footprint touch a street?

    A building needs a face on the public way. Nothing in the packer knew that:
    it fills whatever rectangle it is handed, and the second row it stacks
    inside a frontage band, or anything `infill_blocks` drops in a courtyard,
    is a building with no way in. Measured on `downtown_1000` seed 42 before
    this existed: 122 of 419 packed buildings (29%) stood more than 10 m in
    from every edge of their own block and 75 more than 25 m — landlocked.

    *block_rect* is the block ALREADY INSET by `block_inset`, i.e. the sidewalk
    line, so a building seated hard against it measures 0. `max_m` is how much
    setback still counts as fronting the street; 0 disables the whole test.
    """
    if max_m <= 0.0:
        return None
    bx0, by0, bx1, by1 = block_rect

    def reach(x0, y0, w, h):
        return min(x0 - bx0, y0 - by0, bx1 - (x0 + w), by1 - (y0 + h)) <= max_m
    return reach


def _street_sides(block_rect, x0: float, y0: float, w: float, h: float,
                  tol_m: float = 6.0) -> frozenset:
    """Which of a footprint's four WORLD faces look at a street.

    *block_rect* is already inset to the sidewalk line (see `block_inset`),
    same as `_street_reach`'s. A face looks at a street when ITS OWN edge — not
    the footprint's centre, not the whole footprint — sits within *tol_m* of
    the matching block edge; the other three faces may still be well inside
    the block (a corner lot's back two sides) without that counting against
    them. `tol_m` defaults to 6 m: wider than `_street_reach`'s frontage test
    needs to be exact, because a face is either basically on the sidewalk line
    or basically not — there is no "partially on the street" the way there is
    a graded "how far in from it".
    """
    bx0, by0, bx1, by1 = block_rect
    sides = set()
    if x0 - bx0 <= tol_m:
        sides.add("W")
    if bx1 - (x0 + w) <= tol_m:
        sides.add("E")
    if y0 - by0 <= tol_m:
        sides.add("S")
    if by1 - (y0 + h) <= tol_m:
        sides.add("N")
    return frozenset(sides)


def _place_ok(place: str, n_street_sides: int) -> bool:
    """Coarse version of the same rule `blank:` enforces exactly: how many
    street sides a slot has decides which `place` classes may stand there.
    2+ sides is a corner slot (`corner`/`any` only); exactly 1 is an end/mid
    slot with one flank exposed and one covered (`end`/`corner`/`any`); 0 is
    interior, where nothing is street-facing and every class is fine.

    Used only as a FALLBACK when an entry has no `blank:` tag to test
    directly — see `_pack_free`. An entry with `place="any"` (the untagged
    default) always passes, which is what keeps this a no-op for a pool
    nobody has measured.
    """
    if n_street_sides >= 2:
        return place in ("corner", "any")
    if n_street_sides == 1:
        return place in ("end", "corner", "any")
    return True


def _typ_gap(typ: dict, default: float) -> float:
    """The clear gap `_pack_free` leaves between buildings, PER TYPOLOGY.

    `packing.building_gap_m` is one global number, and 2.5 m is right for a
    mid-rise block built to its edges. It is wrong for a tower district, and
    that turned out to be the only lever that actually separates tall
    buildings: `tall_min_gap_m` can only redirect a slot to SHORTER stock, and
    the `highrise` pool is 134-312 m throughout — every candidate is tall, so
    the filter empties the list and falls through. Measured, no value of it
    changed the layout above ~6 m.

    The packer's own gap has no such escape hatch: it is the guillotine step,
    so it holds by construction. MEASURED on the probe at the default 2.5 m,
    towers stood 38-42 m apart centre to centre on 42-86 m wide plans — i.e.
    touching.
    """
    return float(typ.get("building_gap_m", default))


def _has_facing_pref(meta: dict) -> bool:
    """Whether *meta* carries enough to prefer one yaw over another at all —
    gates the extra candidate `_pack_free` generates per footprint shape
    (0 vs 180, 90 vs 270) and whether a shape's survivors get COLLAPSED to
    one by `_yaw_score` rather than left as separate draws.

    Broader than "has a `blank:` tag" on purpose. `SM_Building_22`
    (`front:W`, no `blank:`) used to get no say in its own orientation at
    all: the old gate was `bool(meta.get("blank"))`, so an entry with only a
    `front:` tag never even had its 180-degree twin generated, and the
    `front:` tag was dead weight in `_pack_free` — read by nothing there.
    `place` and `never_corner` are included too, since either can eliminate
    a candidate on its own (the corner check in `_pack_free`) even with no
    `blank:`/`front:` at all.

    False for a pool nobody has measured (`place="any"`, `front=None`,
    `blank` empty, `never_corner=False`) — the exact case this predicate has
    to stay a no-op for, so `downtown`/`downtown_1000`'s untagged pools draw
    exactly as many candidates as they always have.
    """
    return bool(meta.get("front") or meta.get("blank")
               or meta.get("place", "any") not in ("any",)
               or meta.get("never_corner"))


def _frontage_len(block_rect, side) -> float:
    """Length of the BLOCK's own edge on *side* — a W or E edge runs
    north-south (parallel to Y), so its length is the block's Y extent; a N
    or S edge runs east-west, so it is the X extent. ``None`` (no side, i.e.
    nothing to measure) returns 0.

    Used only to break a tie between two streets a corner slot can front
    (`_yaw_score`'s "longest frontage" term) — `house_26_707`:
    SM_Building_26's 28.4 m face belongs on the long street it was standing
    across from, not the short one it was defaulting to. The block's own
    plan size is the only measure of "how long is this street" available
    this early in the pipeline, before an actual road width is decided.
    """
    if side not in ("N", "E", "S", "W"):
        return 0.0
    bx0, by0, bx1, by1 = block_rect
    return (by1 - by0) if side in ("W", "E") else (bx1 - bx0)


def _depth_into_block(bw: float, bh: float, fsides: frozenset) -> bool:
    """True when THIS candidate's longer footprint extent runs perpendicular
    to one of its street sides — the short face meets the sidewalk, the
    long one runs back into the block.

    A W/E street's own edge runs north-south (parallel to Y), so putting the
    depth into the block wants the LONGER extent along X there; a N/S
    street's edge runs east-west, so it wants the longer extent along Y.
    `house_42_734` (SM_Building_02, 28.0 x 14.4 m, front the 14.4 m face):
    only correct when the 28.0 m runs into the block and the windowed
    14.4 m meets the street — which coincides with `_yaw_score`'s tier 1
    (front on street) by construction whenever that tier is satisfied, so
    this only has independent teeth as the LAST tiebreak, among candidates
    that all failed to put a front on any street at all.
    """
    if not fsides:
        return False
    for side in fsides:
        if side in ("W", "E") and bw >= bh:
            return True
        if side in ("N", "S") and bh >= bw:
            return True
    return False


def _yaw_score(bw: float, bh: float, yaw: float, block_rect, meta: dict,
              fsides: frozenset):
    """4-tier score for one candidate orientation that has ALREADY cleared
    the hard reject (no blank side on the street, no illegal `place`/corner
    combination) — a higher tuple wins `_pack_free`'s per-entry vote.

    Replaces a VETO ("first yaw that is not illegal") with a SCORE, per the
    four counter-examples the user gave reviewing a built scene (2026-08-29):
    a building can be LEGAL at more than one yaw and still be WRONG at all
    but one of them — "this isn't a hard rule, you have to change
    orientation based on where the streets are to the building."

      1. FRONT ON THE STREET. `front0` is the asset's front direction in the
         WORLD frame at placement yaw 0 (mirrors `blank0` — see
         `_pool_entries`); rotating it by *yaw* gives the direction THIS
         candidate actually points. `house_32_698` (SM_Building_22): the one
         elevation without windows was left facing the street, because
         nothing here ever preferred the alternative — `front:` was parsed
         and then never consulted.
      2. LONGEST STREET FRONTAGE, among the front-on-street candidates —
         `house_26_707`: the long face belongs on the long street, not
         whichever one a blind veto happened to land on first.
      3. MOST DETAILED ELEVATIONS ON THE STREET. Every surviving candidate
         already has NO blank side on a street — that is the hard reject —
         so this is just how many street sides it actually engages: a
         corner candidate that reaches both streets outranks one that only
         reaches one of them.
      4. DEPTH INTO THE BLOCK — see `_depth_into_block`. Genuinely the last
         tiebreak: whenever tier 1 holds this is already implied by
         construction, so it only decides anything among candidates that
         all failed to put a front on the street.
    """
    front = meta.get("front0")
    front_world = _rot_side(front, yaw) if front else None
    on_street = bool(front_world) and front_world in fsides
    frontage = _frontage_len(block_rect, front_world) if on_street else 0.0
    return (on_street, frontage, len(fsides),
            _depth_into_block(bw, bh, fsides))


def _pack_free(rect, pool, gap: float, min_side: float, rng, sky, typ,
               area_band: float = 0.55, reach=None, block_rect=None,
               street_tol_m: float = 6.0, justify: bool = True):
    """Guillotine-pack *pool* into *rect*; returns ``([(entry, cx, cy, yaw)], refused)``.

    Candidates are tried largest-footprint-first so a gap closes with one big
    building rather than several small ones, and the orientation putting the
    long axis along the gap's long axis is preferred. Which of the near-largest
    lands there is the skyline's choice, not the packer's.

    *area_band* is how far below the largest fitting footprint an asset may be
    and still compete. At 0.85 the band is often a single asset — the largest
    thing that fits wins every equivalent gap in the city, which is why two
    models took 66% of all buildings and why the skyline's repeat penalty had
    nothing to act on. Widening it puts real choice in the band; the packer
    still prefers big, it just no longer decides alone.

    IT IS THE PACKER'S GREED KNOB and it is now settable per scene
    (`districts.pack_area_band`). The trade is real in both directions: too
    narrow and the biggest fitting model wins every equivalent gap however hard
    the skyline penalises repeats, because there is nothing else in the band to
    give it; too wide and a small building wins a big gap, leaving slack that
    reads as unbuilt block interior. What the band must NOT be used for is
    diversity on its own — that is the skyline's two penalties. This just has
    to leave them something to choose between.

    *block_rect* is the block's own inset rect (the same one `_street_reach`
    is built from) and turns on the FACING test, run per candidate
    ORIENTATION rather than per asset:

      1. HARD REJECT — unchanged discipline, now unconditional rather than a
         fallback: a `place_mid` asset (one modelled elevation) or one
         tagged `place_never_corner` is refused at any slot with 2+ street
         sides regardless of whether it also carries a `blank:` tag; then,
         as before, a candidate whose `blank:` sides (rotated to this yaw)
         land on the street is dropped, or — when it carries no `blank:` at
         all — one whose `place` class cannot legally stand there
         (`_place_ok`).
      2. SCORE the survivors (`_yaw_score`) and keep the single best
         orientation PER ENTRY — see that function for the four tiers. Only
         entries carrying enough metadata to prefer one yaw over another
         (`_has_facing_pref`) are collapsed this way; an entry with none is
         left with exactly as many candidates as it always had, which is
         what keeps a pool nobody has tagged drawing byte-identical to
         before this scoring existed.

    When every candidate for a sub-rectangle fails the hard reject the
    rectangle is left EMPTY rather than building the least-bad option — the
    same discipline `reach` already applies to frontage — and counted in the
    returned *refused* tally so a run that is quietly refusing everything
    shows up in the `[districts]` log instead of just being a smaller city.
    ``None`` (the default) skips the whole test, which is what keeps this a
    no-op for any caller that has not been updated to pass a block rect.

    *justify* (`districts.pack_justify`, default on) hands a residue no
    library building could ever use back as gap between the buildings already
    placed, instead of banking it at the block's far edge — see `_justify` for
    the measurement that made this necessary. `justify=False` reproduces the
    original corner-anchored packing placement for placement.
    """
    out, stack, refused = [], [tuple(rect)], 0
    while stack:
        x0, y0, x1, y1 = stack.pop()
        w, h = x1 - x0, y1 - y0
        if w < min_side or h < min_side:
            continue
        fits = []
        for e in pool:
            meta = e[5] if len(e) > 5 else {}
            if meta.get("place") == "none":
                continue           # all-blank stock: never a pack candidate
            sx, sy = e[3]["sx"], e[3]["sy"]
            # The 180-degree-flipped facing (and, on the swapped shape, the
            # 270 twin) is only worth generating as a SEPARATE candidate when
            # there is something to prefer one over the other — otherwise it
            # is a geometrically-identical duplicate and would silently
            # double that entry's odds in `sky.choose` for nothing. See
            # `_has_facing_pref` — broader than "has a `blank:` tag" now, so
            # a `front:`-only entry (SM_Building_22) finally gets a say too.
            pref = block_rect is not None and _has_facing_pref(meta)
            if sx <= w and sy <= h:
                fits.append((e, sx, sy, 0.0))
                if pref:
                    fits.append((e, sx, sy, 180.0))
            if sy <= w and sx <= h and abs(sx - sy) > 1e-6:
                fits.append((e, sy, sx, 90.0))
                if pref:
                    fits.append((e, sy, sx, 270.0))
        # A LANDMARK SLOT SKIPS THE AREA BAND. Asked before the band because
        # that is what the band would remove — see `_Skyline.landmark_picks`.
        # Still subject to `reach`: a 231 m tower with no street frontage is
        # the same defect as a 12 m one with none.
        marks = sky.landmark_picks(fits)
        if reach is not None:
            # A CANDIDATE THAT CANNOT REACH THE STREET IS NOT A CANDIDATE, and
            # if none of them can, this sub-rectangle stays bare. Falling back
            # to "place something anyway" is what produced the landlocked
            # courtyard buildings in the first place: a gap with no frontage is
            # a courtyard, and a courtyard is a legitimate thing for a block to
            # have. Cost is real and intended — more open block interior.
            fits = [f for f in fits if reach(x0, y0, f[1], f[2])]
        if not fits:
            continue
        if block_rect is not None:
            hard = []          # [(f, fsides)] — survivors of the hard reject
            for f in fits:
                e, bw, bh, yaw = f
                meta = e[5] if len(e) > 5 else {}
                fsides = _street_sides(block_rect, x0, y0, bw, bh,
                                       street_tol_m)
                n_sides = len(fsides)
                if n_sides >= 2 and (meta.get("place") == "mid"
                                     or meta.get("never_corner")):
                    # A one-sided (`mid`) asset shows a blank flank on
                    # whichever of its two uncovered sides a neighbour is
                    # NOT standing against, and a corner needs BOTH covered
                    # at once — no yaw ever supplies that, so this is
                    # refused unconditionally rather than left to the
                    # `blank:`/`_place_ok` test below, which only fires when
                    # `blank:` is unset. `place_never_corner` is the same
                    # rule for an asset that is otherwise fine on a straight
                    # block face but was authored as wrong at a corner
                    # regardless.
                    continue
                blank = meta.get("blank") or frozenset()
                if blank:
                    bad = _rot_sides(meta.get("blank0") or frozenset(), yaw) \
                        & fsides
                    ok = not bad
                else:
                    ok = _place_ok(meta.get("place", "any"), n_sides)
                if ok:
                    hard.append((f, fsides))
            if not hard:
                # Something fit AND reached the street, but every surviving
                # orientation would show a blank wall, an illegal `place`
                # class, or a one-sided asset at a corner. Leaving the
                # rectangle bare is the same call `reach` makes for a
                # landlocked gap — a blank wall on a street is worse than an
                # empty lot, not better.
                refused += 1
                continue
            # COLLAPSE TO ONE ORIENTATION PER ENTRY, but only for an entry
            # that actually has a preference to express (`_has_facing_pref`)
            # — an untagged entry may legally have TWO surviving candidates
            # here (its two fitting SHAPES, not two facings of one shape),
            # and both have to stay in play for the ORIGINAL area-band/
            # `along` selection below to choose between them exactly as it
            # always has. This is also where `_yaw_score`'s tiers 3-4 get
            # their only real say: within one shape, 0 and 180 share the
            # same `(bw, bh)` and hence the same `fsides`, so only tier 1
            # (front) can ever separate them there — tiers 3-4 only matter
            # comparing ACROSS an entry's two shapes, which happens here.
            # `hard` is never empty at this point, so every entry that
            # contributes to it lands in `kept` one way or the other — via
            # this loop directly, or via `best_by_entry` below.
            best_by_entry: dict = {}
            kept = []
            for f, fsides in hard:
                e, bw, bh, yaw = f
                meta = e[5] if len(e) > 5 else {}
                if not _has_facing_pref(meta):
                    kept.append(f)
                    continue
                score = _yaw_score(bw, bh, yaw, block_rect, meta, fsides)
                cur = best_by_entry.get(id(e))
                if cur is None or score > cur[0]:
                    best_by_entry[id(e)] = (score, f, fsides)
            # A MEASURED FRONT THAT CANNOT REACH A REAL STREET HERE AT ANY
            # SURVIVING ORIENTATION is refused outright rather than kept at
            # its least-bad tie. `_yaw_score`'s tiers 2-4 only mean to break
            # ties AMONG candidates that already put a front on a street
            # (the docstring's own framing: "every surviving candidate
            # already has NO blank side on a street"); an entry that carries
            # no `blank:` tag at all (an "any"-class building with only a
            # `front:` tag — SM_Building_26 is 28.4 m across its modelled
            # face, 14.3 m deep) was never blank-gated, so when its OWN
            # aspect ratio means neither yaw fitting a narrow guillotine
            # remainder ever faces that remainder's real street, tier 1 is
            # False for every one of its candidates and tiers 3-4 (which
            # reward more street sides / more depth, oblivious to whether the
            # FRONT is among them) can still make it win the slot outright —
            # `house_16_223`, 2026-08-31. Only fires when the winning
            # candidate's own `fsides` is non-empty (a street genuinely
            # exists here); an entry whose best candidate has none at all is
            # a true interior slot, nothing to get wrong, and is kept as
            # before.
            for score, f, fsides in best_by_entry.values():
                e = f[0]
                meta = e[5] if len(e) > 5 else {}
                if meta.get("front0") and fsides and not score[0]:
                    continue
                kept.append(f)
            fits = kept
            if not fits:
                # Every survivor of the hard reject carried a front that
                # cannot reach this slot's real street at any orientation
                # its own footprint fits — the same "leave it bare" call
                # `if not hard` already makes, just discovered one step
                # later, after the per-entry facing collapse rather than
                # before it.
                refused += 1
                continue
        if marks:
            marks = [f for f in marks if f in fits]      # survived `reach` + facing
        if marks:
            # The tallest band, drawn among themselves so the repeat penalties
            # still decide WHICH landmark — two identical towers facing each
            # other is no better for being tall.
            oriented = marks
            chosen = sky.choose([f[0] for f in oriented], 0.0, x0, y0)
            sky.took_landmark()
        else:
            best = max(f[1] * f[2] for f in fits)
            top = [f for f in fits if f[1] * f[2] >= best * area_band]
            along = w >= h
            oriented = [f for f in top if (f[1] >= f[2]) == along] or top
            chosen = sky.choose([f[0] for f in oriented],
                                sky.target(typ, x0, y0), x0, y0)
        e, bw, bh, yaw = next(f for f in oriented if f[0] is chosen)

        # WHERE IN THE SUB-RECTANGLE — the low corner unless the residue is
        # unbuildable, in which case the slack is handed back as gap. See
        # `_justify`; `px, py == x0, y0` reproduces the original packing
        # exactly, which is what `justify=False` forces.
        px, py = (_justify(x0, y0, x1, y1, bw, bh, pool, gap, block_rect,
                           e[5] if len(e) > 5 else {}, yaw, street_tol_m)
                  if justify else (x0, y0))

        cx, cy = px + bw / 2.0, py + bh / 2.0
        out.append((e, cx, cy, yaw))
        sky.record(cx, cy, e[3]["sz"], (bw, bh))

        right_w, top_h = x1 - (px + bw) - gap, y1 - (py + bh) - gap
        if right_w >= top_h:
            if right_w > 0:
                stack.append((px + bw + gap, y0, x1, y1))
            if top_h > 0:
                # From the sub-rect's ORIGINAL low edge, not the justified
                # one: the strip the shift opened up beside the building is
                # still free ground above it, and anchoring this child at
                # `px` would abandon it.
                stack.append((x0, py + bh + gap, px + bw, y1))
        else:
            if top_h > 0:
                stack.append((x0, py + bh + gap, x1, y1))
            if right_w > 0:
                stack.append((px + bw + gap, y0, x1, py + bh))
    return out, refused


def _pool_fits(pool, w: float, h: float) -> bool:
    """Could ANY member of *pool* stand in a `w` x `h` rectangle, either way
    round? The question `_justify` needs answered about a leftover.

    Deliberately coarser than `_pack_free`'s own candidate loop: it ignores
    `reach`, the blank-wall reject and the area band, so it only ever says
    "something might fit here", never "something will". That is the safe
    direction — a leftover this says nothing fits in is one NO run of the
    packer could have used, so handing it back as gap cannot cost a building.
    """
    if w <= 0.0 or h <= 0.0:
        return False
    for e in pool:
        sx, sy = e[3]["sx"], e[3]["sy"]
        if (sx <= w and sy <= h) or (sy <= w and sx <= h):
            return True
    return False


def _justify(x0, y0, x1, y1, bw, bh, pool, gap, block_rect, meta, yaw,
             street_tol_m, eps: float = 0.5):
    """Where the building actually goes inside its sub-rectangle.

    THE HOLE AT THE FAR EDGE OF EVERY BLOCK. The guillotine anchors every
    building at its sub-rectangle's low corner and pushes the residue to the
    far side, so a block's unusable slack — the metres left over once no
    library building fits any more — accumulates as ONE contiguous band along
    the block's north and east edges. MEASURED on `downtown_fire_500` seed 4
    before this existed: 27,771 m2 of open ground at least 8 m across inside
    ten blocks (15.3% of all block interior), and every non-highrise block
    carried a 8-19 m deep band on a STREET frontage — which is precisely the
    "enormously wide sidewalk" artefact `_perimeter_rects` was written to
    stop, arriving by the other axis. `infill_blocks` cannot close it: the
    narrowest member of this library is 13.5 m across, so an 11 m band can
    never hold a building however many passes look at it.

    So when the residue on an axis is too small for anything in *pool*, the
    building is JUSTIFIED to the far end of its sub-rectangle and the residue
    is handed back as extra gap BETWEEN buildings instead — which is exactly
    what `packing.building_gap_m`'s own preset comment says the slack is for
    ("A bigger gap spreads the same buildings across the whole block, putting
    the slack BETWEEN them instead of all of it at the edge"). The street wall
    closes on the block edge; the slack lands inside, where a courtyard is a
    real thing for a block to have.

    Returns ``(px, py)``, the building's low corner. Identical to ``(x0, y0)``
    whenever a residue could still hold something, so a block the packer fills
    exactly is untouched.

    THE FACING RE-CHECK is not optional. Moving a footprint toward the block
    edge can put a face on a street that was interior when the candidate was
    scored, and a `blank:` side arriving on a street is the one defect this
    module refuses buildings outright to avoid. The shift is dropped on that
    axis rather than reasoned about.
    """
    dx = x1 - x0 - bw
    dy = y1 - y0 - bh
    if dx > eps and _pool_fits(pool, dx - gap, y1 - y0):
        dx = 0.0
    if dy > eps and _pool_fits(pool, bw, dy - gap):
        dy = 0.0
    dx = dx if dx > eps else 0.0
    dy = dy if dy > eps else 0.0
    if not dx and not dy:
        return x0, y0
    px, py = x0 + dx, y0 + dy
    if block_rect is not None:
        fsides = _street_sides(block_rect, px, py, bw, bh, street_tol_m)
        n_sides = len(fsides)
        if n_sides >= 2 and (meta.get("place") == "mid"
                             or meta.get("never_corner")):
            return x0, y0
        blank = meta.get("blank") or frozenset()
        if blank:
            if _rot_sides(meta.get("blank0") or frozenset(), yaw) & fsides:
                return x0, y0
        elif not _place_ok(meta.get("place", "any"), n_sides):
            return x0, y0
        # A MEASURED FRONT THAT MISSES THE STREET THE SHIFT JUST CREATED is
        # the same defect as a `blank:` side landing there, just invisible to
        # the check above for an entry that carries no `blank:` tag at all
        # (an "any"-class building with only a `front:` tag, e.g.
        # SM_Building_26). `_pack_free`'s own candidate generation and
        # `_yaw_score` both run against the PRE-shift sub-rectangle, so a
        # slot with 0 street sides at scoring time (nothing to prefer, tier 1
        # moot, ties broken by insertion order) can still be the ONLY shape
        # this entry's footprint fits into a narrow guillotine remainder --
        # SM_Building_26 is 28.4 m across its modelled face and 14.3 m deep,
        # so a remainder narrower than 28.4 m never even generates the
        # candidate that would face perpendicular to it. `house_16_223`
        # (2026-08-31): exactly this shape, the tie's arbitrary winner (front
        # W) pushed flush against a real N street it was never scored
        # against. Declining the shift leaves the residue as a gap, same as
        # the far-edge case above.
        front = meta.get("front0")
        if front and fsides and _rot_side(front, yaw) not in fsides:
            return x0, y0
    return px, py


def _lay_terrace_end_caps(rect, depth: float, pool, rng, sky, typ,
                          area_band: float, max_depth_m: float, gap: float,
                          reach, street_tol_m: float = 6.0):
    """Compact stock at the two ends of a plain terrace pair's alley —
    ``[(entry, cx, cy, yaw)]``, `_lay_terrace`'s own return shape, so the
    caller appends it the same way.

    No `alley_m` parameter, deliberately — unlike `_terrace_strips`, which
    needs it to judge whether a block qualifies for a terrace pair at all,
    this only runs AFTER two rows have already been laid successfully, so
    the alley that matters here is whatever is ACTUALLY left between them:
    exactly `rect`'s own short side minus `2 * depth`, with no need to go
    back to the config value that merely bounded it during sizing.

    `_lay_terrace` builds two rows along a block's LONG faces and leaves the
    alley between them open on purpose — right for the alley itself, a
    terrace's back yards and service lane are supposed to stay paved and
    empty. But a block generous enough to leave a WIDE alley also leaves its
    own SHORT edges — themselves streets, the two rows never reach them —
    entirely unbuilt. `house_25_621` (user, 2026-08-29): "this house is fine
    but it's block is 2 rows... lots of free space in the middle... The
    smaller edges can place house 42 there since it's very compact and we
    can afford depth in a block like that."

    OPT-IN, per typology (`districts.typologies.<name>.terrace_end_caps`) —
    see `rezone_blocks`, which only calls this when a scene has set it.

    IMPLEMENTED BY REUSING `_pack_free` rather than a bespoke placer: the
    alley's two ends are ordinary free rectangles against the SAME
    `block_rect` (the terrace block itself), so `_street_sides`/`_yaw_score`
    already know these ends front the block's short edges and will orient
    whatever lands there outward, exactly as they would for any other
    packed slot — nothing about facing needs re-deriving here.

    *max_depth_m* caps how far EACH cap zone reaches down the alley's own
    LENGTH (parallel to the rows, away from the short edge it fronts) — not
    the alley's cross-width, which is fixed by the rows already laid and is
    simply handed to `_pack_free` whole, exactly as any other free rect's
    height is. 0 falls back to the pool's own deepest member, which is what
    keeps a single row of caps from becoming a THIRD row of buildings
    marching down the alley toward the middle: capped this way, `_pack_free`
    still guillotine-packs the zone (so more than one compact building can
    stand side by side along the short edge if the pool has room for it),
    but nothing can extend past roughly one building's own depth toward the
    alley's centre, which is what keeps the middle open. Also hard-clamped
    to under half the alley's length so the two ends can never meet.
    """
    x0r, y0r, x1r, y1r = rect
    w, h = x1r - x0r, y1r - y0r
    along_x = w >= h                    # matches `_terrace_strips`'s own test
    cross = (h if along_x else w) - 2.0 * depth     # the alley's own width
    if cross <= 0.0:
        return []                       # no alley at all — nothing to cap
    min_side = min(min(e[3]["sx"], e[3]["sy"]) for e in pool)
    if cross < min_side:
        return []                       # too narrow for anything in the pool
    length = w if along_x else h        # the alley's own length, end to end
    reach_m = max_depth_m if max_depth_m > 0.0 \
        else max(e[3]["sx"] for e in pool)
    reach_m = min(reach_m, length / 2.0 - 1e-6)
    if reach_m < min_side:
        return []                       # the cap budget can't fit anything
    out = []
    if along_x:
        lo, hi = y0r + depth, y1r - depth               # the alley band
        ends = [(x0r, lo, x0r + reach_m, hi),            # west end
                (x1r - reach_m, lo, x1r, hi)]             # east end
    else:
        lo, hi = x0r + depth, x1r - depth
        ends = [(lo, y0r, hi, y0r + reach_m),            # south end
                (lo, y1r - reach_m, hi, y1r)]             # north end
    for fr in ends:
        got, _refused = _pack_free(fr, pool, gap, min_side, rng, sky, typ,
                                   area_band, reach, block_rect=rect,
                                   street_tol_m=street_tol_m)
        out += got
    return out


# ---------------------------------------------------------------------------
# the pass
# ---------------------------------------------------------------------------

def _fits_block(pool, rect):
    w, h = rect[2] - rect[0], rect[3] - rect[1]
    return any(min(e[3]["sx"], e[3]["sy"]) <= min(w, h) + _FIT_TOL_M
               and max(e[3]["sx"], e[3]["sy"]) <= max(w, h) + _FIT_TOL_M
               for e in pool)


# Minimum centre-to-centre separation between two `districts.probe` picks,
# including two picks for DIFFERENT named typologies. MEASURED (the
# urban-layout skill's BLOCK SIZE section): footprints and the blocks sized
# for them span roughly 40-100 m short side across every library this
# generator has been pointed at, so a separation comfortably past the widest
# of those guarantees at least one full unzoned block between any two probe
# picks — "spread them out, do not take three adjacent blocks."
_PROBE_SPREAD_M = 120.0


def _select_probe_blocks(layout: dict, parks: set, pool_of: dict,
                         typologies: dict, inset: float, probe_cfg: dict,
                         rng):
    """Which blocks a `districts.probe` config zones, and to what.

    The FIT TEST is per morphology, because "the block short side is inside
    `block_short_m`" is the wrong question for a terrace typology — what
    actually decides whether a terrace pair can be laid is `_terrace_strips`'
    band, `[2*depth + alley_m, 2*depth + alley_max_m]`, measured against the
    BUILDABLE rect (the block inset by `block_inset()` on both sides), with
    *depth* the pool's deepest member's `sx` — already swapped for
    yaw-offset, see `_pool_entries`. Picking a terrace probe block by
    `block_short_m`/`_fits_block` alone (the pack test) is exactly what
    zoned three "brick_midrise" blocks whose bands then refused them at
    build time, forcing the next-typology-up fallback and putting a THIRD,
    uninvited typology in a scene meant to show two. `_terrace_band` is the
    same arithmetic `_terrace_strips` itself uses, not a second copy of it.

    A PACK typology keeps the original test: `_fits_block` for "can stand at
    all", `block_short_m` as a soft preference (a block outside it can still
    take the typology's smallest member, so it is a fallback candidate, not
    excluded). A terrace typology has no such fallback tier — either the
    block's short side is in the band or a pair cannot be laid there at all,
    so every terrace candidate ranks equally and there is nothing to prefer
    between them beyond the spread rule below.

    Picks greedily off that ranking, refusing a candidate closer than
    `_PROBE_SPREAD_M` to any pick already made — for this typology or an
    earlier one in *probe_cfg*, so two different probe typologies do not end
    up sharing a corner either — then relaxes that spacing on a second pass
    rather than hand back fewer blocks than asked.

    Returns ``({block: typology_name}, {typology_name: n_qualified})`` — the
    second dict is how many blocks passed the fit test at all, BEFORE the
    spread rule or the count cap trimmed it, so the caller can say plainly
    when fewer blocks qualify than were asked for rather than silently
    zoning fewer than it looks like it did. Every block missing from the
    first dict is the caller's to leave unzoned — `rezone_blocks` treats a
    block missing from this map as "not built", and the `doomed` pass it
    already runs before zoning has stripped whatever `build_city` put there,
    so an unpicked block comes out genuinely empty rather than merely
    unlabelled.
    """
    blocks = [b for b in layout.get("blocks", []) if b not in parks]
    chosen: dict = {}
    qualified: dict = {}
    centers: list = []
    for name, count in probe_cfg.items():
        count = int(count)
        pool = pool_of.get(name)
        if count <= 0 or not pool:
            qualified[name] = 0
            continue
        t = typologies.get(name) or {}
        is_terrace = str(t.get("morphology", "pack")) == "terrace"
        if is_terrace:
            depth = max(e[3]["sx"] for e in pool)
            alley_m = float(t.get("alley_m", 6.0))
            alley_max_m = float(t.get("alley_max_m", 0.0))
            lo, hi = _terrace_band(depth, alley_m, alley_max_m)
        else:
            band = t.get("block_short_m")
        cands = []
        for blk in blocks:
            if blk in chosen:
                continue
            rect = (blk[0] + inset, blk[1] + inset,
                    blk[2] - inset, blk[3] - inset)
            w, h = rect[2] - rect[0], rect[3] - rect[1]
            if w < 4.0 or h < 4.0:
                continue
            short = min(w, h)
            if is_terrace:
                if not (lo <= short <= hi):
                    continue                  # a pair cannot be laid here
                rank = 0
            else:
                if not _fits_block(pool, rect):
                    continue
                in_band = bool(band) and float(band[0]) <= short <= float(band[1])
                rank = 0 if in_band else 1
            cands.append((rank, blk,
                         (blk[0] + blk[2]) / 2.0, (blk[1] + blk[3]) / 2.0))
        qualified[name] = len(cands)
        rng.shuffle(cands)                    # break ties within a rank
        cands.sort(key=lambda c: c[0])        # stable: shuffle order survives
        picked = 0
        for relax in (False, True):
            if picked >= count:
                break
            for _rank, blk, cx, cy in cands:
                if picked >= count or blk in chosen:
                    continue
                if not relax and any(
                        math.hypot(cx - pcx, cy - pcy) < _PROBE_SPREAD_M
                        for pcx, pcy in centers):
                    continue
                chosen[blk] = name
                centers.append((cx, cy))
                picked += 1
    return chosen, qualified


def rezone_blocks(config: dict, layout: dict, placements: list, resolver, rng,
                  zone_at) -> dict:
    """Rebuild every non-park block to its zone's typology.

    Rebuilding rather than swapping in place because morphology, not just
    height, is what distinguishes a row-house block from a tower block — and a
    swap can only ever replace a building with a smaller one, so it can never
    turn a mid-rise lot into a tower lot.

    Disaster art survives untouched: ruins from the damaged/destroyed pools and
    their debris stay exactly where `build_city` put them and are treated as
    obstacles, so re-zoning cannot quietly repair a bombed city.
    """
    cfg = config.get("districts") or {}
    typologies = cfg.get("typologies") or {}
    if not typologies:
        return {}

    cache: dict = {}
    intact = {e[0] for e in _pools_for(config, resolver, ["intact"], cache)}
    order = sorted(typologies,
                   key=lambda n: -float(typologies[n].get("rank", 0.0)))
    pool_of = {n: _pools_for(config, resolver,
                             typologies[n].get("pools") or [n], cache)
               for n in typologies}

    inset = block_inset(config, resolver)
    gap = float(config.get("packing", {}).get("building_gap_m", 2.5))
    bleed = float((cfg.get("zoning") or {}).get("bleed", 0.12))
    exclusions = config.get("exclusions") or []
    sky = _Skyline(cfg, rng)
    # The packer's greed, per scene. See `_pack_free`.
    area_band = float(cfg.get("pack_area_band", 0.55))
    # Hand an unbuildable residue back as gap rather than banking it at the
    # block's far edge — see `_justify`. ON by default because it fixes a
    # measured defect (a 8-19 m bare band on a street frontage in every
    # block); `districts.pack_justify: false` restores the original packing.
    justify = bool(cfg.get("pack_justify", True))
    # How far in from the sidewalk line a building may sit and still count as
    # fronting the street. 0 disables the test. See `_street_reach`.
    frontage_max = float(cfg.get("frontage_max_m", 0.0))
    # Suppress a terrace run repeating the same USD back to back — see
    # `_tile_run`. OFF BY DEFAULT: it perturbs the whole scene's `rng` stream
    # even for a pool with no facing metadata (`downtown`'s brownstones have
    # none), and `downtown_earthquake` builds on `downtown` mid-flight.
    # A scene opts in explicitly; `downtown_gac.yaml` is the first to.
    no_repeat = bool(cfg.get("terrace_no_repeat", False))
    parks = set(park_blocks(layout, placements))

    # Ruins and their debris are immovable; everything else already standing
    # inside a block is an obstacle too. Street furniture is absent — city_detail
    # runs after this pass.
    keep_cats = {"play_structure", "trail", "tree", "bus_stop", "traffic_light",
                 "debris_pile", "debris"}
    survivors = [p for p in placements
                 if p.get("category") != "house" or p.get("usd") not in intact]

    # `districts.probe`: zone only the NAMED typologies below, onto at most
    # that many blocks each, and leave every other block unzoned — see
    # `_select_probe_blocks`. A probe run replaces the zone-map-driven
    # selection entirely rather than layering on top of it, because the
    # whole point is reviewing a typology on a handful of blocks in
    # isolation, with nothing else in the city competing for attention.
    probe_cfg = cfg.get("probe") or {}
    probe_map: dict = {}
    if probe_cfg:
        probe_map, probe_qualified = _select_probe_blocks(
            layout, parks, pool_of, typologies, inset, probe_cfg, rng)
        got = {}
        for n in probe_map.values():
            got[n] = got.get(n, 0) + 1
        want = "  ".join(f"{n}={got.get(n, 0)}/{int(c)}"
                         for n, c in probe_cfg.items())
        n_nonpark = len(layout.get("blocks", [])) - len(parks)
        print(f"[districts] probe: {want}  "
              f"({len(probe_map)} of {n_nonpark} non-park blocks zoned; "
              f"the rest stay empty)")
        # SAY SO PLAINLY when fewer blocks qualified than were asked for,
        # rather than silently handing back a smaller probe than it looks
        # like was requested — `got[n]/count` above already shows the
        # shortfall in the COUNT, this says WHY: not enough blocks passed
        # the fit test at all, as opposed to the spread rule crowding them
        # out (which the two-pass relax already recovers from).
        short = [f"{n} (found {probe_qualified.get(n, 0)}, wanted {int(c)})"
                for n, c in probe_cfg.items()
                if probe_qualified.get(n, 0) < int(c)]
        if short:
            print(f"[districts] probe: not enough qualifying blocks for "
                  f"{', '.join(short)}")

    # Terrace superblocks are deliberately rare: they are the distinctive part
    # of the city, and each one eats a large block and adds a street. THE
    # BUDGET IS PER TYPOLOGY, not one shared counter — `rowhouse` and a second
    # terrace typology (e.g. `brick_midrise`) must not exhaust each other's
    # allowance depending only on which one a given block happens to zone as
    # first. Read off each typology's own `max_superblocks` at the point its
    # block is built; default 2, the historical single-counter value, so a
    # scene with exactly one terrace typology — every scene before a second
    # one existed — sees no change.
    n_super_by_typ: dict = {}
    splits = []          # (original block, [sub-blocks], typology)

    doomed = set()
    for i, p in enumerate(placements):
        if p.get("category") != "house" or p.get("usd") not in intact:
            continue
        x, y = float(p["x_m"]), float(p["y_m"])
        for b in layout.get("blocks", []):
            if b not in parks and b[0] <= x <= b[2] and b[1] <= y <= b[3]:
                doomed.add(i)
                break
    if doomed:
        placements[:] = [p for i, p in enumerate(placements) if i not in doomed]

    obstacles = [_rect_of(p, resolver, margin=gap)
                 for p in placements
                 if p.get("category") in keep_cats or p.get("category") == "house"]

    typ_of, counts, added, blank_refused, probe_refused = {}, {}, 0, 0, 0
    # WHAT THE BLOCK WAS ACTUALLY BUILT FROM, which is not always what it was
    # ZONED as. A terrace block outside its alley band is rebuilt from the next
    # typology up (`rowhouse_refused` below) and `typ_of` keeps reporting the
    # zoned name — deliberately, because everything downstream of this module
    # keys damage policy and road width off `layout["_typology_of"]` and a
    # block silently changing district would move those too.
    #
    # `infill_blocks` is the one consumer that needs the OTHER answer.
    # MEASURED on `downtown_fire_500` seed 4: both `brick_midrise` blocks were
    # terrace-refused and packed with `tower` stock, then infill read
    # `typ_of`, saw `morphology: terrace`, and skipped them entirely as "back
    # yards and service alley" — 3,943 m2 of leftover on two blocks that had
    # no terrace in them at all, including a 1,190 m2 hole. So the built name
    # is published separately and infill prefers it.
    built_typ_of: dict = {}
    paved: list = []                 # terrace-block interiors to un-pave
    for blk in layout.get("blocks", []):
        if blk in parks:
            typ_of[blk] = "park"
            counts["park"] = counts.get("park", 0) + 1
            continue
        rect = (blk[0] + inset, blk[1] + inset, blk[2] - inset, blk[3] - inset)
        if rect[2] - rect[0] < 4.0 or rect[3] - rect[1] < 4.0:
            continue

        if probe_cfg:
            # PROBE MODE: the block was already picked (or wasn't) by
            # `_select_probe_blocks`. No zone map, no bleed, no rank ladder —
            # a block this pass did not name for a typology stays unzoned,
            # and the `doomed` pass above has already stripped whatever
            # `build_city` put on it, so it comes out genuinely empty.
            name = probe_map.get(blk)
            if name is None:
                continue
        else:
            bcx, bcy = (blk[0] + blk[2]) / 2.0, (blk[1] + blk[3]) / 2.0
            here = zone_at(bcx, bcy) or {}
            # `remap_buildings` passes `zone_field`, which yields a TYPOLOGY,
            # so `.get("name")` is the typology name. The `mix` branch is for
            # a caller that hands in `assign`'s RING instead — a ring name is
            # not a key in pool_of and would silently fall through to "first
            # typology that fits".
            name = (_pick(here["mix"], rng) if here.get("mix")
                    else here.get("name"))
            if rng.random() < bleed and hasattr(zone_at, "bleed_name"):
                name = zone_at.bleed_name(bcx, bcy, rng) or name
            # A typology whose pool cannot fit this block would leave it
            # empty; step down through the ranks until something can be built.
            for cand in [name] + [n for n in order if n != name]:
                if cand in pool_of and _fits_block(pool_of[cand], rect):
                    name = cand
                    break
            else:
                continue
        typ = dict(typologies.get(name) or {})
        typ["name"] = name
        typ_of[blk] = name
        counts[name] = counts.get(name, 0) + 1
        # SHARED with `infill_blocks` via `layout` (same dict, same `blk`
        # key) -- both passes build houses onto this SAME block, and the
        # "at most one unburnable per block" rule has to see every house
        # either one adds, not just its own.
        burn_guard = layout.setdefault(
            "_burn_guard_by_block", {}).setdefault(blk, _BurnabilityGuard())

        local = [o for o in obstacles
                 if o[2] > rect[0] and o[0] < rect[2]
                 and o[3] > rect[1] and o[1] < rect[3]]
        pool = pool_of[name]

        if str(typ.get("morphology", "pack")) == "terrace" and not local:
            gaps = typ.get("run_gap_m") or (3.0, 9.0)
            alley = float(typ.get("alley_m", 6.0))
            # The pool NARROWED TO WHAT THIS BLOCK CAN HOLD, and the deepest
            # depth among what's left — see `_terrace_fit_pool`. A quad sized
            # off the pool's single deepest member, unfiltered, is what used
            # to let one deep building disqualify blocks a shallower member
            # would have fit fine.
            depth0_pool, depth0 = _terrace_fit_pool(
                pool, min(rect[2] - rect[0], rect[3] - rect[1]), alley)
            road_w = float(typ.get("street_w_m", 11.4))
            quad = None
            # A terrace SUPERBLOCK: four rows around an internal street, the
            # row-house district cut on its own finer grid. Budgeted, because
            # the whole point is that these are a distinctive few rather than
            # the default — and because each one consumes a large block.
            # PER TYPOLOGY, not global — see the budget comment above.
            max_super_here = int(typ.get("max_superblocks", 2))
            n_super_here = n_super_by_typ.get(name, 0)
            if depth0_pool and n_super_here < max_super_here:
                quad = _terrace_quad(rect, depth0, alley, road_w)
            if quad:
                strips, road_spans, road_axis = quad
                laid = _lay_terrace(rect, pool, rng,
                                    float(typ.get("facing_offset_deg", 0.0)),
                                    alley,
                                    float(typ.get("run_median_m", 45.0)),
                                    float(typ.get("run_sigma", 0.70)),
                                    (float(gaps[0]), float(gaps[1])),
                                    float(typ.get("alley_max_m", 0.0)),
                                    strips=strips, no_repeat=no_repeat)
                if laid:
                    n_super_by_typ[name] = n_super_here + 1
                    for span in road_spans:
                        _add_internal_street(layout, rect, span, road_axis,
                                             road_w, typ)
                    # SPLIT THE BLOCK along the new streets. apply_ground_planes
                    # draws one ground mesh PER BLOCK at z=0.01, over the
                    # asphalt base at z=0 — so a corridor inside an unchanged
                    # block rect is simply buried under that block's own grass.
                    # Splitting leaves the street as a gap between blocks, which
                    # is exactly how every other road in the scene shows
                    # through.
                    splits.append((blk, _split_along(blk, road_spans,
                                                     road_axis), name))
                    counts["terrace_superblock"] = \
                        counts.get("terrace_superblock", 0) + 1
                    counts["terrace_rows"] = counts.get("terrace_rows", 0) \
                        + len(strips)
            else:
                laid = _lay_terrace(rect, pool, rng,
                                    float(typ.get("facing_offset_deg", 0.0)),
                                    alley,
                                    float(typ.get("run_median_m", 45.0)),
                                    float(typ.get("run_sigma", 0.70)),
                                    (float(gaps[0]), float(gaps[1])),
                                    float(typ.get("alley_max_m", 0.0)),
                                    no_repeat=no_repeat)
            # The interior stays PAVED. A row-house block in NYC or Boston has
            # a paved service alley behind it, not lawn — exposing the grass
            # plane here is what made these blocks read as suburban houses with
            # back gardens.
            if not laid:
                if probe_cfg:
                    # PROBE MODE: refusing here is exactly the discipline a
                    # probe exists to demonstrate — nothing else may compete
                    # for the isolation the probe promised, so a block whose
                    # terrace band refuses it stays EMPTY rather than
                    # quietly becoming an uninvited third typology. Outside
                    # probe mode the fallback below is correct and unchanged
                    # — "refusing AND leaving it empty would just trade one
                    # artefact for another" is only true when something else
                    # in the scene would otherwise draw the eye there; a
                    # probe scene has nothing else to draw it to.
                    probe_refused += 1
                else:
                    # The block is outside the terrace band, so it is not a
                    # terrace block. Build it as the next typology up rather
                    # than leaving a hole — refusing here is the whole
                    # point, but refusing AND leaving it empty would just
                    # trade one artefact for another.
                    alt = _next_typology(typologies, name)
                    if alt and pool_of.get(alt):
                        name, typ, pool = alt, dict(typologies[alt]), pool_of[alt]
                        typ["name"] = alt
                        built_typ_of[blk] = alt
                        counts[alt] = counts.get(alt, 0) + 1
                        counts[  # the refused terrace no longer counts as one
                            "rowhouse_refused"] = counts.get(
                                "rowhouse_refused", 0) + 1
                        min_side = min(min(e[3]["sx"], e[3]["sy"]) for e in pool)
                        reach = _street_reach(rect, frontage_max)
                        for fr in free_rects(rect, local, min_side):
                            got, refused = _pack_free(
                                fr, pool, _typ_gap(typ, gap), min_side, rng,
                                sky, typ, area_band, reach, block_rect=rect,
                                justify=justify)
                            laid += got
                            blank_refused += refused
            elif quad:
                # A SUPERBLOCK IS ENTIRELY ITS OWN. Four rows and their street
                # are the whole composition, so nothing else is built here —
                # that is the point of giving the row-house district its own
                # grid. Packing the leftover with mid-rise is exactly the
                # "brownstones sharing a block with other buildings" the
                # superblock exists to stop.
                pass
            else:
                # NOTHING BEHIND THE PAIR. Two rows is the whole composition on
                # a plain terrace block; building the leftover out with mid-rise
                # reads as a third row of brownstones and is the same
                # sharing-a-block artefact the superblock exists to avoid. The
                # space stays open — rear gardens, which is what is actually
                # behind a terrace.
                #
                # UNLESS the typology opts into `terrace_end_caps` — compact
                # stock at the alley's own two ends, fronting the block's
                # SHORT edges (themselves streets the two long-face rows
                # never reach at all). `house_25_621`: a block wide enough to
                # leave a generous alley leaves those short edges bare, and
                # "we can afford depth in a block like that." OFF BY
                # DEFAULT — every existing rowhouse block is unchanged unless
                # a scene sets this per typology.
                ec_cfg = typ.get("terrace_end_caps")
                if ec_cfg:
                    ec_pool = _pools_for(config, resolver,
                                         ec_cfg.get("pools") or [], cache)
                    if ec_pool:
                        caps = _lay_terrace_end_caps(
                            rect, depth0, ec_pool, rng, sky, typ,
                            area_band, float(ec_cfg.get("max_depth_m", 0.0)),
                            _typ_gap(typ, gap),
                            _street_reach(rect, frontage_max))
                        laid += caps
                        counts["terrace_end_caps"] = \
                            counts.get("terrace_end_caps", 0) + len(caps)
        else:
            laid = []
            min_side = min(min(e[3]["sx"], e[3]["sy"]) for e in pool)
            band = float(typ.get("perimeter_depth_m", 0.0))
            # Measured against the BLOCK, not against the band rectangle it is
            # packing: the band's inner edge is a courtyard boundary, not a
            # street, and a building seated against it fronts nothing.
            reach = _street_reach(rect, frontage_max)
            for outer in _perimeter_rects(rect, band, min_side):
                for fr in free_rects(outer, local, min_side):
                    got, refused = _pack_free(
                        fr, pool, _typ_gap(typ, gap), min_side, rng, sky, typ,
                        area_band, reach, block_rect=rect, justify=justify)
                    laid += got
                    blank_refused += refused

        laid = burn_guard.filter_laid(
            laid, pool, typ["name"], rect,
            swap_log=layout.setdefault("_burn_swap_log", []))
        for entry, cx, cy, yaw in laid:
            if exclusions and _in_exclusion(cx, cy, exclusions):
                continue
            placements.append(_new_placement(entry, cx, cy, yaw))
            obstacles.append(_rect_of(placements[-1], resolver, margin=gap))
            sky.record(cx, cy, entry[3]["sz"], (entry[3]["sx"], entry[3]["sy"]))
            added += 1

    n_paving = 0
    if paved:
        keep = []
        for p in placements:
            if p.get("category") == "concrete" and any(
                    r[0] <= p["x_m"] <= r[2] and r[1] <= p["y_m"] <= r[3]
                    for r in paved):
                n_paving += 1
                continue
            keep.append(p)
        placements[:] = keep

    # Rewrite the block list so the superblocks' internal streets read as gaps.
    # Done after the loop rather than during it because the loop iterates
    # layout["blocks"].
    if splits:
        blocks = list(layout.get("blocks") or [])
        for original, pieces, tname in splits:
            if original in blocks:
                i = blocks.index(original)
                blocks[i:i + 1] = pieces
            was_built = built_typ_of.pop(original, None)
            typ_of.pop(original, None)
            for p in pieces:
                typ_of[p] = tname
                if was_built:
                    built_typ_of[p] = was_built
        layout["blocks"] = blocks
        print(f"[districts] {len(splits)} terrace superblock(s) split into "
              f"{sum(len(p) for _o, p, _t in splits)} blocks around their "
              f"own streets")

    if blank_refused:
        # SAME LINE the rest of the run summary is in, on purpose — a run
        # that is quietly refusing every candidate on some blocks produces a
        # smaller city with no other symptom, and this is the number that
        # would otherwise only show up as "fewer buildings than expected"
        # after a render.
        counts["blank_wall_refused"] = blank_refused
    if probe_refused:
        # A probe block whose terrace band refused it — left EMPTY rather
        # than rebuilt as the next typology up, which is the whole point of
        # a probe (nothing else may compete for the isolation it promised).
        # Reported so the log still says what happened instead of a probe
        # scene quietly zoning fewer blocks than it looked like it asked
        # for.
        counts["probe_refused"] = probe_refused
    layout["_typology_of"] = typ_of
    layout["_built_typology_of"] = built_typ_of
    detail = "  ".join(f"{k}={v}" for k, v in sorted(counts.items()))
    print(f"[districts] rezoned {len(typ_of)} blocks -> {added} buildings "
          f"({len(doomed)} replaced, {n_paving} paving tiles removed from "
          f"terrace yards)   {detail}")
    # WHAT THE TWO REPEAT PENALTIES ACTUALLY DID. "the city looks copy-pasted"
    # has cost several rounds of guessing, and it is one number: the share of
    # every packed building taken by the single most-used model. Terraces are
    # not in it — they are laid by `_lay_terrace`, which never draws from here.
    n_models, n_packed, top_share = sky.diversity()
    if n_packed:
        pool_size = len({e[0] for n in typologies
                         for e in (pool_of.get(n) or ())
                         if str((typologies[n] or {}).get("morphology",
                                                          "pack")) != "terrace"})
        print(f"[districts] model diversity: {n_models} of {pool_size} models "
              f"used across {n_packed} packed buildings, "
              f"top model {100.0 * top_share:.1f}% "
              f"(repeat_penalty={sky.repeat}, "
              f"repeat_radius_m={math.sqrt(sky.local_r2):.0f}, "
              f"repeat_local_penalty={sky.local_pen}, "
              f"repeat_local_falloff={sky.local_falloff}, "
              f"repeat_hard_radius_m={math.sqrt(sky.hard_r2):.0f}, "
              f"tall_min_h_m={sky.tall_min_h:.0f}, "
              f"tall_min_gap_m={sky.tall_gap:.0f}, "
              f"pack_area_band={area_band})")
        if sky.tall_fallback:
            # Every tall candidate was too close to one already standing, on
            # this many slots — the slot still filled (from the pool's
            # shorter members, or a tall repeat if that is truly all that
            # fits), but the separation rule did not get its way there. A
            # scene where this is large is a scene asking for more towers
            # than its blocks have room to space out.
            print(f"[districts] tall_fallback={sky.tall_fallback} slot(s) "
                  f"where every tall candidate was too close to an existing "
                  f"tall building")
        # PER-MODEL HISTOGRAM AND THE UNUSED LIST. "a lot of the building
        # assets are not being used" (user) is otherwise only visible by
        # counting distinct colours in a render. `sky.used` already IS this
        # histogram — it is what the two repeat penalties above read from —
        # so this is printing state that already existed, not computing new
        # state. Terrace-pool models are excluded from `models_unused`
        # (`pool_size` above already excludes them from its denominator, for
        # the same reason: ROW HOUSES NEVER REACH THE SKYLINE, so an unused
        # brownstone is not a diversity defect the way an unused mid-rise is
        # — ­`_lay_terrace`/`_tile_run` have their own repeat rule instead).
        top_models = sorted(sky.used.items(), key=lambda kv: -kv[1])[:15]
        hist = "  ".join(f"{os.path.basename(u)}={n}" for u, n in top_models)
        print(f"[districts] model histogram (top {len(top_models)} of "
              f"{n_models}): {hist}")
        pack_names = {os.path.basename(e[0]) for n in typologies
                     for e in (pool_of.get(n) or ())
                     if str((typologies[n] or {}).get("morphology",
                                                      "pack")) != "terrace"}
        used_names = {os.path.basename(u) for u in sky.used}
        unused = sorted(pack_names - used_names)
        if unused:
            print(f"[districts] models_unused ({len(unused)} of "
                  f"{pool_size}): {', '.join(unused)}")
    # Per-typology accounting, because "I don't see any X" has cost several
    # rounds of guessing. A typology can come out at zero for four different
    # reasons and they are indistinguishable in the viewport: no block was
    # zoned for it, its pool did not resolve, its pool did not fit the block it
    # got, or the block fell outside the terrace band and was refused. Say
    # which.
    for name in sorted(typologies):
        pool = pool_of.get(name) or []
        blocks_here = sum(1 for v in typ_of.values() if v == name)
        built = counts.get(name, 0)
        note = ""
        if not pool:
            note = "  <- POOL EMPTY (asset paths unresolved?)"
        elif blocks_here == 0:
            note = "  <- no block zoned for it"
        elif built == 0:
            note = "  <- zoned but nothing built"
        print(f"[districts]   {name:9s} pool={len(pool):2d} "
              f"blocks={blocks_here:2d} built={built:2d}{note}")
    if counts.get("rowhouse_refused"):
        print(f"[districts]   terrace refused on "
              f"{counts['rowhouse_refused']} block(s) outside the alley band "
              f"— rebuilt as the next typology up")
    if counts.get("terrace_rows") or counts.get("rowhouse") or \
            counts.get("rowhouse_refused"):
        print(f"[districts]   terrace_no_repeat={no_repeat} "
              f"(districts.terrace_no_repeat; off reproduces the original "
              f"_tile_run draw for draw)")
    return typ_of


def infill_blocks(config: dict, layout: dict, placements: list, resolver,
                  rng, zone_at) -> int:
    """Build on whatever block area is still bare pavement.

    `build_city`'s packer abandons a leftover the moment no library building
    fits it, and with `packing.placeholders.enabled` false those leftovers keep
    the block's concrete tiles and nothing else — which is the "the sidewalk is
    enormous here" effect: it is not sidewalk, it is unbuilt paved interior.

    Draws from one pool merged across every non-terrace typology by
    default, or from each block's OWN typology's pool when
    `districts.infill.per_block_pool` is set — see the flag's own comment
    below for why it defaults off and what it fixes.
    """
    cfg = (config.get("districts") or {}).get("infill") or {}
    if not cfg.get("enabled", True):
        return 0
    dcfg = config.get("districts") or {}
    cache: dict = {}
    # NEVER infill from a terrace pool. A row house only works in a party-wall
    # run laid along a block face; dropped into a leftover gap by the packer it
    # lands on its own, spaced by its own 21 m DEPTH, so a column of them reads
    # as houses stacked front-to-back. Terrace typologies are placed by
    # `_lay_terrace` and by nothing else.
    typs = dcfg.get("typologies") or {}
    names = [n for n, t in typs.items()
             if str(t.get("morphology", "pack")) != "terrace"]
    explicit_pools = cfg.get("pools")
    # PER-BLOCK TYPOLOGY POOL: infill a block from ITS OWN typology's stock,
    # not one pool merged from every non-terrace typology in the scene. OFF
    # BY DEFAULT (`districts.infill.per_block_pool`) — it moves which model
    # can land in which block's leftover gap, and every scene that has not
    # opted in must not restyle for it. MEASURED on `downtown_gac_probe`,
    # where only `highrise` and `brick_midrise` were zoned at all: the
    # merged pool put an 11.8 m Dmytro factory shed and two Muyang mid-rises
    # inside HIGHRISE blocks (134-312 m glass towers) — invisible in the
    # typology counts, because the BLOCK still reports as `highrise`, and
    # exactly what the six-typology ladder's district separation exists to
    # prevent. `districts.infill.pools`, when a scene sets one, still wins
    # over per-block selection either way — it already overrides the merged
    # pool today, and an explicit override is still explicit.
    per_block = bool(cfg.get("per_block_pool", False)) and not explicit_pools
    # A GAP THIS BLOCK'S OWN STOCK CANNOT FILL MAY DROP A RANK. `per_block_pool`
    # is what stops an 11.8 m Dmytro shed landing between two 300 m towers, and
    # it is right — but it also means a leftover narrower than the district's
    # own narrowest model stays bare FOREVER, however many passes look at it.
    # MEASURED on `downtown_fire_500` seed 4: the `highrise` pool's slimmest
    # member is 29.9 m across and five of its nine leftovers were 8-19 m deep,
    # so the pass reported nine gaps and built three buildings.
    #
    # So a gap nothing in the block's own pool fits may be offered to the
    # typologies BELOW it, nearest rank first, up to this many ranks down.
    # One step keeps the district legible — a tower block may take a
    # `tower`-rank infill, never a shed — and 0 restores the strict per-block
    # behaviour exactly. Terrace pools are excluded here as everywhere else in
    # this pass (`names`).
    rank_fallback = max(0, int(cfg.get("rank_fallback", 1)))
    pool_by_typ: dict = {}
    if per_block:
        # Sized off the UNION of every non-terrace typology's pool, exactly
        # as the merged-pool path is below, so `min_gap`/`gap`/`margin` do
        # not themselves become a second, harder-to-see behaviour change —
        # only WHICH pool `_pack_free` draws from, per block, moves.
        sizing_pool = _pools_for(config, resolver, names or ["intact"], cache)
    else:
        pool = _pools_for(config, resolver,
                          (explicit_pools or names or ["intact"]), cache)
        sizing_pool = pool
    if not sizing_pool:
        return 0

    smallest = min(min(e[3]["sx"], e[3]["sy"]) for e in sizing_pool)
    min_gap = float(cfg.get("min_gap_m", 0.0)) or smallest
    gap = float(cfg.get("gap_m", config.get("packing", {})
                        .get("building_gap_m", 2.5)))
    margin = float(cfg.get("clearance_m", gap))
    inset = block_inset(config, resolver)
    exclusions = config.get("exclusions") or []

    obstacles = _placement_rects(
        placements, resolver,
        {"house", "play_structure", "trail", "tree", "bus_stop",
         "traffic_light", "debris_pile"}, margin)
    sky = _Skyline(dcfg, rng)
    # NO LANDMARKS IN INFILL. This is a second `_Skyline` over the same config,
    # so it would otherwise be handed its own full `landmark_count` and double
    # the tall towers — and a landmark is by definition the thing a block was
    # zoned for, not something dropped into whatever gap was left over.
    sky.landmark_budget = 0
    area_band = float(dcfg.get("pack_area_band", 0.55))
    justify = bool(dcfg.get("pack_justify", True))
    frontage_max = float(dcfg.get("frontage_max_m", 0.0))
    # INFILL USED TO UNDO THE PERIMETER MORPHOLOGY. `rezone_blocks` packs a
    # `perimeter_depth_m` band and leaves the middle open on purpose — "a dense
    # urban block is built to its edges with the slack in the MIDDLE" — and
    # then this pass ran `free_rects` over the WHOLE block and filled that
    # middle back in. Every building it put there was landlocked by
    # construction. With this on, infill sees the same band the block was built
    # from and the courtyard survives.
    #
    # DEFAULT OFF, like the repeat penalties and `frontage_max_m`: turning it on
    # takes `downtown.yaml` from 223 buildings to 190 (its infill drops from 43
    # gaps / 40 buildings to 5 / 7), and restyling that scene — and the
    # earthquake one built on it — is not this change's business.
    # `downtown_1000.yaml` sets it true.
    perimeter_only = bool(dcfg.get("infill", {}).get("perimeter_only", False))
    for p in placements:
        if p.get("category") == "house":
            fp = resolver.get(p["usd"], "house", scale=p.get("scale"),
                              axis_up=p.get("axis_up", "Z"))
            # WORLD footprint, not the resolver's raw one — swapped exactly
            # as `_rect_of` swaps it, so a pre-existing house's recorded
            # extent matches how it actually sits on the ground and the
            # tall-separation gap test (`_Skyline._tall_ok`) measures the
            # real footprint, not the pre-rotation one.
            psx, psy = fp["sx"], fp["sy"]
            if abs((float(p.get("yaw_deg", 0.0)) % 180.0) - 90.0) < 45.0:
                psx, psy = psy, psx
            sky.record(float(p["x_m"]), float(p["y_m"]), fp["sz"], (psx, psy))
            # ...and WHICH model, and where. See `_Skyline.note`.
            sky.note(str(p.get("usd") or ""), float(p["x_m"]), float(p["y_m"]))

    parks = set(park_blocks(layout, placements))
    typ_of = layout.get("_typology_of") or {}
    # THE TYPOLOGY THE BLOCK WAS BUILT FROM, not the one it was zoned as — see
    # `built_typ_of` in `rezone_blocks`. A terrace block the alley band refused
    # was packed from the next typology up and is not a terrace block at all;
    # reading `typ_of` here skipped it as "back yards" and left its leftovers
    # bare. Falls back to `typ_of` for every block that was built as zoned,
    # which is all of them in a scene with no refusal.
    built_of = layout.get("_built_typology_of") or {}
    added = gaps = blank_refused = rank_fill = 0
    gap_area = 0.0

    # Non-terrace typologies below a given rank, nearest first — the ladder
    # `rank_fallback` walks. Built once; `names` is already terrace-free.
    def _rank_of(n):
        return float((typs.get(n) or {}).get("rank", 0.0))

    def _lower_ranks(tname):
        r = _rank_of(tname)
        lower = sorted((n for n in names if n != tname and _rank_of(n) < r),
                       key=_rank_of, reverse=True)
        return lower[:rank_fallback]

    def _pool_for(tname):
        if tname not in pool_by_typ:
            tc = (typs.get(tname) or {})
            pool_by_typ[tname] = _pools_for(config, resolver,
                                            tc.get("pools") or [tname], cache)
        return pool_by_typ[tname]
    for blk in layout.get("blocks", []):
        if blk in parks:
            continue
        tname = built_of.get(blk) or typ_of.get(blk)
        if tname is None:
            # No typology reached this block in `rezone_blocks` — too small,
            # no pool fit it, or (`districts.probe`) deliberately left
            # unzoned. Any of those means infill has no business here: it
            # used to fall through to an EMPTY `typ` dict and the general
            # infill pool, which is exactly the "probe leaves the rest of the
            # city empty" guarantee breaking — a block with no typology is
            # not a gap inside a typed block.
            continue
        # A terrace block's middle is its back yards and service alley, not a
        # gap to be filled.
        tcfg = (dcfg.get("typologies") or {}).get(tname) or {}
        if str(tcfg.get("morphology", "pack")) == "terrace":
            continue
        # SHARED with `rezone_blocks` (see its own comment) -- infill adds
        # MORE houses to a block `rezone_blocks` already built, so the guard
        # must carry that pass's count forward rather than start over.
        burn_guard = layout.setdefault(
            "_burn_guard_by_block", {}).setdefault(blk, _BurnabilityGuard())
        if per_block:
            this_pool = _pool_for(tname)
            if not this_pool:
                # This typology's OWN pool resolved to nothing — the merged
                # pool used to paper over that with whatever else the scene
                # had; per-block selection means a block with no library of
                # its own gets no infill rather than a stray building from
                # somewhere else's district.
                continue
        else:
            this_pool = pool
        rect = (blk[0] + inset, blk[1] + inset, blk[2] - inset, blk[3] - inset)
        typ = dict(tcfg)
        typ["name"] = tname
        # A DISTRICT THAT RESERVES CLEAR GROUND KEEPS IT. `highrise` sets
        # `building_gap_m: 30` precisely so a tower block is one or two towers
        # with a plaza between them — that void is the design, not a packing
        # residue, and `city_detail`'s plaza pass is what dresses it. Offering
        # it to a lower rank filled it with mid-rises instead: MEASURED, the
        # three `highrise` blocks went from 9 buildings and 13,949 m2 of open
        # ground to 16 and 7,877, i.e. the rank ladder ate half the plazas.
        # So the fallback only applies where the typology is content with the
        # scene's own `building_gap_m`.
        allow_fallback = (per_block and rank_fallback > 0
                          and _typ_gap(tcfg, gap) <= gap + 1e-6)
        reach = _street_reach(rect, frontage_max)
        band = float(tcfg.get("perimeter_depth_m", 0.0)) if perimeter_only else 0.0
        for outer in _perimeter_rects(rect, band, min_gap):
            for fr in free_rects(outer, obstacles, min_gap):
                gaps += 1
                gap_area += (fr[2] - fr[0]) * (fr[3] - fr[1])
                # PER-TYPOLOGY GAP HERE TOO. Without it infill undoes the
                # separation `rezone_blocks` just established: measured on the
                # probe, 11 of the 21 buildings standing in highrise blocks
                # came from this pass, packed at the global gap, and the
                # closest tower pair was 2.0 m apart however `building_gap_m`
                # was set on the typology.
                use_pool = this_pool
                if allow_fallback and not _pool_fits(
                        this_pool, fr[2] - fr[0], fr[3] - fr[1]):
                    for alt in _lower_ranks(tname):
                        alt_pool = _pool_for(alt)
                        if _pool_fits(alt_pool, fr[2] - fr[0],
                                      fr[3] - fr[1]):
                            use_pool, rank_fill = alt_pool, rank_fill + 1
                            break
                got, refused = _pack_free(
                    fr, use_pool, _typ_gap(tcfg, gap), min_gap, rng, sky,
                    typ, area_band, reach, block_rect=rect, justify=justify)
                blank_refused += refused
                got = burn_guard.filter_laid(
                    got, use_pool, tname, rect,
                    swap_log=layout.setdefault("_burn_swap_log", []))
                for entry, cx, cy, yaw in got:
                    if exclusions and _in_exclusion(cx, cy, exclusions):
                        continue
                    placements.append(_new_placement(entry, cx, cy, yaw))
                    obstacles.append(_rect_of(placements[-1], resolver,
                                              margin=margin))
                    added += 1

    n_models, n_packed, top_share = sky.diversity()
    refused_note = f"; blank_wall_refused={blank_refused}" if blank_refused else ""
    per_block_note = "; per_block_pool=True" if per_block else ""
    if rank_fill:
        per_block_note += f"; rank_fallback filled {rank_fill} gap(s)"
    tall_note = (f"; tall_fallback={sky.tall_fallback}" if sky.tall_fallback
                else "")
    print(f"[districts] infill: {gaps} unbuilt gaps ({gap_area:,.0f} m2) "
          f"-> {added} buildings; {n_models} models in play across "
          f"{n_packed} (the city so far), top model "
          f"{100.0 * top_share:.1f}%{refused_note}{per_block_note}{tall_note}")
    return added


def _fp_of(resolver, p):
    return resolver.get(p["usd"], p.get("category", "house"),
                        scale=p.get("scale", 1.0), axis_up=p.get("axis_up", "Z"))


def _house_box(resolver, p):
    """(x0, y0, x1, y1) — *p*'s ACTUAL final world footprint, from its own
    real (x_m, y_m, yaw_deg, usd, scale, axis_up), not whatever the packer
    thought it was reserving. Shared by `repair_facing`'s overlap check and
    `repair_overlaps`."""
    fp = _fp_of(resolver, p)
    bw, bh = _rotated_wh(fp, float(p["yaw_deg"]))
    x, y = float(p["x_m"]), float(p["y_m"])
    return (x - bw / 2.0, y - bh / 2.0, x + bw / 2.0, y + bh / 2.0)


def _boxes_overlap(a, b, tol: float = 0.2) -> bool:
    """True when *a* and *b* overlap by MORE than a *tol* m touch — two
    buildings sharing a party wall to the centimetre are not a defect;
    genuinely occupying the same ground is."""
    ox = min(a[2], b[2]) - max(a[0], b[0])
    oy = min(a[3], b[3]) - max(a[1], b[1])
    return ox > tol and oy > tol


def repair_facing(config: dict, layout: dict, placements: list, resolver,
                  street_tol_m: float = 6.0) -> dict:
    """Final per-block facing REPAIR — runs once, at the very end of
    `remap_buildings`, on every "house" placement's ACTUAL FINAL (x, y, yaw,
    usd). This is deliberately NOT another packing-time preference: it is a
    gate on the geometry the resolver really returned, so the same code is
    correct whether the sizes behind it came from the host's offline caches,
    a seeded real-Nucleus cache, or Kit's own live resolver — the 2026-08-31
    decision to stop chasing host/Kit packing parity for facing specifically
    (`SM_Building_13` violated 3 times, reproducibly, in a Kit-only
    guillotine geometry the host packer never explores, despite `_pack_free`
    /`_justify`'s own facing fix being byte-identical and violation-free on
    every host reconstruction tried).

    For every front-tagged placement whose front does not address a real
    street (the SAME inset-corrected test the dump-side checker uses):

      (a) `yaw += 180` if the OPPOSITE side addresses a street — the
          footprint is unchanged (a 180 flip never changes which extent is
          which), so this is always geometrically legal.
      (b) else `yaw +/- 90` if the ROTATED footprint's front then addresses
          a street AND the rotated footprint does not overlap any other
          house (`_boxes_overlap`) — tried both directions, first that
          works wins, deterministically (90 before -90).
      (c) else left alone, logged as unrepairable.

    Deterministic and IDEMPOTENT: reads only already-final placement fields
    and a pure function of them, no `rng` — running this twice makes no
    further change the second time (a fixed placement re-checks as already
    correct; an unrepairable one re-logs the identical verdict).
    """
    meta_table = _asset_meta_table(config)
    inset = block_inset(config, resolver)
    blocks = [(x0 + inset, y0 + inset, x1 - inset, y1 - inset)
             for (x0, y0, x1, y1) in (layout.get("_typology_of") or {}).keys()]

    def block_of(x, y):
        for b in blocks:
            if b[0] <= x <= b[2] and b[1] <= y <= b[3]:
                return b
        return None

    houses = [p for p in placements if p.get("category") == "house"]
    checked = repaired_180 = repaired_90 = unrepairable = 0

    for p in houses:
        meta = meta_table.get(p.get("usd"))
        front = meta.get("front") if meta else None
        if not front:
            continue
        checked += 1

        fp = _fp_of(resolver, p)
        yaw = float(p["yaw_deg"])
        cx, cy = float(p["x_m"]), float(p["y_m"])
        block = block_of(cx, cy)
        if block is None:
            continue          # not inside any zoned block -- nothing to check

        bw, bh = _rotated_wh(fp, yaw)
        px, py = cx - bw / 2.0, cy - bh / 2.0
        fsides = _street_sides(block, px, py, bw, bh, street_tol_m)
        front_world = _rot_side(front, yaw)
        if not fsides or front_world in fsides:
            continue          # already correct, or a true interior slot

        # (a) 180 flip -- same footprint, always legal.
        if _rot_side(front_world, 180.0) in fsides:
            p["yaw_deg"] = (yaw + 180.0) % 360.0
            repaired_180 += 1
            continue

        # (b) +/-90 -- only if the re-oriented footprint both reaches a
        # street with its front AND clears every other house.
        fixed = False
        others = [q for q in houses if q is not p]
        other_boxes = [_house_box(resolver, q) for q in others]
        for delta in (90.0, -90.0):
            new_yaw = (yaw + delta) % 360.0
            nbw, nbh = _rotated_wh(fp, new_yaw)
            npx, npy = cx - nbw / 2.0, cy - nbh / 2.0
            nfsides = _street_sides(block, npx, npy, nbw, nbh, street_tol_m)
            if _rot_side(front, new_yaw) not in nfsides:
                continue
            nbox = (npx, npy, npx + nbw, npy + nbh)
            if any(_boxes_overlap(nbox, ob) for ob in other_boxes):
                continue
            p["yaw_deg"] = new_yaw
            repaired_90 += 1
            fixed = True
            break
        if fixed:
            continue

        unrepairable += 1
        print(f"[districts] facing: unrepairable {_asset_basename(p['usd'])} "
             f"at ({cx:.1f}, {cy:.1f}) -- front faces {front_world}, only "
             f"{sorted(fsides)} is a real street here, and neither a 180 "
             f"flip nor a +/-90 (checked against overlap) reaches it")

    print(f"[districts] facing repair: checked={checked} "
         f"repaired_180={repaired_180} repaired_90={repaired_90} "
         f"unrepairable={unrepairable}")
    return {"checked": checked, "repaired_180": repaired_180,
           "repaired_90": repaired_90, "unrepairable": unrepairable}


def repair_overlaps(config: dict, layout: dict, placements: list, resolver,
                    tol: float = 0.2) -> dict:
    """Final safety net, run immediately after `repair_facing` (which can
    itself change a footprint's rotated extent): detect every house-house
    footprint overlap beyond a *tol* m touch and resolve it deterministically
    — NEVER leaves two buildings interpenetrating, whatever produced the
    overlap.

    Root cause, diagnosed 2026-08-31 (fresh Kit dump + a byte-identical
    seeded host reconstruction both reproduced it): a `_BurnabilityGuard`
    substitute whose real (seeded/Kit) footprint is bigger, in one or both
    extents, than the entry it replaced — `_burnable_substitute` reused the
    SAME (cx, cy, yaw) the packer solved for the SMALLER original, so the
    oversized substitute spills into whatever the packer put right next to
    it. `_burnable_substitute` itself is now fixed (a substitute may never
    exceed the original's own rotated extents), which is why this pass
    finds nothing to do on a clean run — it exists for whatever that fix
    does not cover, and never trusts that.

    For each overlapping pair, in order:
      1. If one side is a recorded `_BurnabilityGuard` swap
         (`layout["_burn_swap_log"]`, matched by its own (x, y, yaw) — a
         placement dict carries no pool-entry identity to check directly),
         revert it to the ORIGINAL draw, which the packer already validated
         fits this exact slot. If that still overlaps (the original itself
         changed something else in the interim), the revert is undone.
      2. Otherwise (or if the revert did not resolve it), the SMALLER-
         footprint member of the pair is DROPPED (removed from
         *placements* entirely) — an empty lot is a real thing a block can
         have; two buildings occupying the same ground is not.

    Deterministic: the pair search always resumes from the front of a
    freshly rebuilt list, and each iteration permanently reverts one
    specific swap or permanently drops one specific placement, so the
    number of iterations is bounded and identical run to run.
    """
    swap_by_key = {}
    for s in layout.get("_burn_swap_log") or []:
        key = (round(s["cx"], 2), round(s["cy"], 2), round(s["yaw"] % 360.0, 1))
        swap_by_key[key] = s

    def find_overlap():
        houses = [(i, p) for i, p in enumerate(placements)
                 if p.get("category") == "house"]
        boxes = [(i, _house_box(resolver, p)) for i, p in houses]
        for a in range(len(boxes)):
            for b in range(a + 1, len(boxes)):
                i, bi = boxes[a]
                j, bj = boxes[b]
                if _boxes_overlap(bi, bj, tol):
                    return i, j, bi, bj
        return None

    checked = reverted = dropped = 0
    dropped_idx: set = set()
    while True:
        found = find_overlap()
        if found is None:
            break
        i, j, bi, bj = found
        checked += 1
        pi, pj = placements[i], placements[j]

        fixed = False
        for p, other in ((pi, pj), (pj, pi)):
            key = (round(float(p["x_m"]), 2), round(float(p["y_m"]), 2),
                  round(float(p["yaw_deg"]) % 360.0, 1))
            s = swap_by_key.get(key)
            if s is None or s["substitute_usd"] != p.get("usd"):
                continue
            saved = (p["usd"], p.get("scale"), p.get("axis_up"))
            p["usd"] = s["original_usd"]
            p["scale"] = s["original_scale"]
            p["axis_up"] = s["original_axis_up"]
            if _boxes_overlap(_house_box(resolver, p),
                              _house_box(resolver, other), tol):
                p["usd"], p["scale"], p["axis_up"] = saved
                continue
            reverted += 1
            fixed = True
            print(f"[districts] overlap: reverted "
                 f"{_asset_basename(saved[0])} back to "
                 f"{_asset_basename(s['original_usd'])} at "
                 f"({p['x_m']:.1f}, {p['y_m']:.1f})")
            break
        if fixed:
            continue

        area_i = (bi[2] - bi[0]) * (bi[3] - bi[1])
        area_j = (bj[2] - bj[0]) * (bj[3] - bj[1])
        drop_i = area_i <= area_j
        drop_idx = i if drop_i else j
        keep_idx = j if drop_i else i
        dropped_idx.add(drop_idx)
        dropped += 1
        dp, kp = placements[drop_idx], placements[keep_idx]
        print(f"[districts] overlap: unrepairable, dropped "
             f"{_asset_basename(dp['usd'])} at "
             f"({dp['x_m']:.1f}, {dp['y_m']:.1f}) -- overlapped "
             f"{_asset_basename(kp['usd'])} at "
             f"({kp['x_m']:.1f}, {kp['y_m']:.1f})")
        # Physically remove it now so `find_overlap` never reconsiders a
        # dropped placement (index-in-`placements` stays stable for
        # everything ELSE since a `while True` re-derives `houses`/`boxes`
        # from `placements` fresh every iteration).
        placements[drop_idx] = dict(placements[drop_idx], category="_dropped_overlap")

    if dropped_idx:
        placements[:] = [p for i, p in enumerate(placements)
                         if p.get("category") != "_dropped_overlap"]

    print(f"[districts] overlap repair: checked={checked} "
         f"reverted={reverted} dropped={dropped}")
    return {"checked": checked, "reverted": reverted, "dropped": dropped}


def remap_buildings(config: dict, layout: dict, placements: list, resolver,
                    rng, district_at) -> int:
    """Pipeline hook: rezone, infill, then rebuild the parks.

    WHY THIS DRIVES THE OTHER PASSES: `generate_scene` owns the pipeline and
    is off-limits to this work, and this is the only hook it offers that
    receives the placement list. `layout` carries a done-flag so a caller that
    grows explicit calls later gets a no-op rather than a doubled scene.
    """
    cfg = config.get("districts") or {}
    if not cfg.get("enabled") or layout.get("_districts_done"):
        return 0
    layout["_districts_done"] = True

    zone_at = zone_field(config, layout.get("region"))
    typ_of = rezone_blocks(config, layout, placements, resolver, rng, zone_at)
    infill_blocks(config, layout, placements, resolver, rng, zone_at)
    try:
        from detail import parks
    except ImportError:
        pass
    else:
        parks.build(config, layout, placements, resolver, rng)

    # FINAL REPAIR, on the placement list every earlier pass has now
    # finished mutating (`parks.build` can remove houses; nothing after
    # this point adds or moves one) — see `repair_facing`/`repair_
    # overlaps`'s own docstrings for why this replaces trying to make the
    # host packer reproduce Kit bit for bit. Facing first: it can change a
    # placement's yaw (and, on a +/-90 repair, its rotated footprint
    # extent), so overlap detection has to run on what facing repair
    # actually leaves behind, not before it.
    repair_facing(config, layout, placements, resolver)
    repair_overlaps(config, layout, placements, resolver)
    return len(typ_of)
