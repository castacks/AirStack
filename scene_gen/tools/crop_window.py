#!/usr/bin/env python3
"""crop_window.py — HOST-SIDE (no Kit, no pxr) window crop over a generated
city, for the "generate 1.5 km, crop to 1 km, not always centred" mechanic
(see `downtown_fire_1500.yaml`'s own header and
`.agents/skills/freeze-disaster-dataset/SKILL.md`'s matrix section: each
baseline LEVEL is a different LAYOUT, and here that means a different (seed,
window) draw over one bigger generator rather than three native builds).

`crop_layout()` is the one function everything else in this file (and
`fc_dump_crop.py`, the FC-dump-specific adapter) is built on. It takes
whatever `(layout, placements)` pair `scene_generator.build_city` /
`tools/plan_png.build()` already produce (host-side, pxr-free) and a window
rect, and returns the SAME SHAPE, filtered and re-centred — a caller
downstream (`tools/plan_png.draw()`, `tools/fire_city_dry_run.py`, a future
Kit-side consumer) sees an ordinary smaller city, not a "cropped" one.

THE THREE RULES, in order of how much they surprised the first draft:

1. **A building whose footprint pokes outside the window is DROPPED WHOLE,
   never cut.** A structural archetype is a single rigid mesh; slicing one at
   an arbitrary window edge is a fracture problem this crop step has no
   business solving, and "half a building" is not a thing the fire/quake/
   people pipelines downstream know how to interpret. The footprint test uses
   the AABB of the ROTATED W x D rect (`_rot_aabb`), which is a superset of
   the true silhouette — the conservative direction: a building that just
   grazes the boundary on its diagonal is dropped, never left with a corner
   hanging over the line.

2. **A BLOCK is CLIPPED, never dropped whole, and that is what trims roads
   at the boundary for free.** `disaster.fire_people.derive_layout` (the
   thing that turns an FC dump back into a `GroundClass`-shaped layout for
   the people planner) computes road corridors as "the exact rectangular
   complement of the blocks inside the region" — so a block rect must never
   extend past the window bound or that complement goes wrong, and clipping
   the last block on an edge to a shallower rect is *exactly* "the street
   grid keeps going, the last block is just shallower", which is what a real
   crop of a real city looks like. Roads themselves are never placements in
   an FC dump at all (`urban_fire_city_launch_script.dump_city_placements`
   writes house placements only) — there is nothing else to trim.

3. **A prop tied to a DROPPED building by its `of` IDENTITY TAG is dropped
   too, even if the prop's own point is still inside the window.**
   `detail.gac_props._place()` stamps every roof/wall prop with
   `"of": "<building usd stem>@<x>,<y>"` (`tools/fc_prop_orphan_probe.py`'s
   `tag_of()` is that exact formula, reused here rather than re-derived — see
   that tool's own docstring for the floating-fire-escape bug this discipline
   already fixed once). A prop with no `of` (street furniture, park
   furniture — nothing outside `detail/gac_props.py` tags a placement at
   all) is just tested by its own point, the same as any other non-building
   placement.

CLIPPING A BLOCK CAN INVALIDATE A BUILDING'S FACING THAT WAS CORRECT AGAINST
THE FULL CITY — measured, not guessed: a window cut through a real
`brick_midrise` block once moved that block's own edge 93 m inward, and a
building sitting 109 m from that edge pre-crop (comfortably interior) was 16 m
from it post-crop — inside `districts.repair_facing`'s own street-proximity
tolerance, so its FRONT/BLANK tags now read as facing a "street" that is
really just where the window happened to cut. This is never a new geometric
fact (the crop only filters and translates) — it is a fact that was true
against the full block and stopped being true against the block's cropped
remainder. `repair_after_crop()` below re-runs `districts.repair_facing` then
`districts.repair_overlaps` — the SAME "final repair, whatever produced the
geometry" pass `remap_buildings` already applies once at the end of a normal
build, for the analogous host/Kit-packing-parity reason — against the cropped
output; `tools/baseline_layouts.py` calls it on every candidate (seed,
window) draw before accepting it. It needs a compiled `config` and a real
resolver, so it is NOT inside `crop_layout()` itself (kept schema-agnostic
and dependency-free) and is NOT available to `fc_dump_crop.py`'s FC-dump-only
path — see that module's docstring for the same gap stated where it actually
bites the pod deliverable.

Everything else (streetlights, trees, cars, park furniture, ...) is a POINT,
not a footprint — it has no silhouette to slice, so "inside the window" is a
plain point test.

RE-CENTRING moves the window's own CENTRE to (0, 0) — never a corner — which
is what lets `downtown_fire_1500.yaml`'s committed `epicenter: [0.0, 0.0]`
keep meaning "the middle of the visible plate" after a crop with no per-level
edit: the fire spec (epicenter/heading/wind/duration/start_offset_frac) is
read straight off the ORIGINAL 1500 m preset's raw YAML by
`fire_city_dry_run._raw_fire_spec` regardless of which window was cropped, and
none of those five keys is coordinate-dependent except epicenter, which this
recentring keeps valid by construction.

WHAT THIS DOES NOT SOLVE — the crop/assembly gap, said out loud rather than
silently assumed away: `urban_fire_city_launch_script.resolve_cell` matches a
fire manifest record back to a REAL Kit-built prim by its recorded `i` (index
into the SAME preset+seed's full placement list) verified against that
placement's OWN (x, y) within 0.5 m, or failing that, the nearest same-usd
house within 2 m of the record's (x, y). Kit rebuilding the full 1500 m city
for that lookup will place buildings at their ORIGINAL, un-shifted world
positions — so a cropped/re-centred record's (x_m, y_m) will NOT match
route 1 or route 3 of `resolve_cell` any more; only route 2 (the `cell` prim
path, carried through unchanged by `fc_dump_crop.py`) still works. Every
cropped placement therefore also carries `x_m_orig`/`y_m_orig` (this
module's own addition, ignored by every existing reader) so a future patch
to the Kit-side launcher can either use those for the position check, or
filter its own rebuilt placement list by the same window before matching.
Fire solve and people placement (`fire_city_dry_run.py`,
`fire_people_dry_run.py`, `disaster.fire_people`) are UNAFFECTED by this gap
— both operate purely on the dump's own coordinates and never touch a live
Kit stage — so this is an ASSEMBLY-time follow-on, not a blocker for the
baseline layouts this preset exists to produce.
"""
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
for _p in (_HERE, _SCENE_GEN):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from fc_prop_orphan_probe import tag_of  # noqa: E402 -- gac_props' own "of" tag, verbatim

BUILDING_CATEGORIES = ("house", "building")


def _rot_aabb(cx, cy, W, D, yaw_deg):
    """Axis-aligned bounding box of a W x D rect centred at (cx, cy) and
    rotated by yaw_deg about its own centre. A conservative SUPERSET of the
    true silhouette by construction (it is the AABB of the four rotated
    corners) -- see rule 1 in the module docstring for why over-dropping,
    not under-dropping, is the safe direction here."""
    hw, hd = W / 2.0, D / 2.0
    rad = math.radians(yaw_deg)
    c, s = math.cos(rad), math.sin(rad)
    xs, ys = [], []
    for sx, sy in ((hw, hd), (hw, -hd), (-hw, hd), (-hw, -hd)):
        xs.append(cx + sx * c - sy * s)
        ys.append(cy + sx * s + sy * c)
    return min(xs), min(ys), max(xs), max(ys)


def _clip_rect(rect, window):
    """Axis-aligned rect intersection, or `None` if they do not overlap
    (or only touch along an edge -- a zero-area sliver is not a block)."""
    x0, y0, x1, y1 = rect
    wx0, wy0, wx1, wy1 = window
    nx0, ny0 = max(x0, wx0), max(y0, wy0)
    nx1, ny1 = min(x1, wx1), min(y1, wy1)
    if nx1 <= nx0 or ny1 <= ny0:
        return None
    return (nx0, ny0, nx1, ny1)


def _point_in(x, y, window, tol=1e-6):
    wx0, wy0, wx1, wy1 = window
    return (wx0 - tol) <= x <= (wx1 + tol) and (wy0 - tol) <= y <= (wy1 + tol)


def _shift(p, dx, dy):
    """A copy of placement `p` with `x_m`/`y_m` translated by (dx, dy), and
    the PRE-shift values preserved as `x_m_orig`/`y_m_orig` -- see the module
    docstring's "what this does not solve" section for who needs them."""
    x = float(p.get("x_m", 0.0))
    y = float(p.get("y_m", 0.0))
    q = dict(p)
    q["x_m_orig"] = x
    q["y_m_orig"] = y
    q["x_m"] = x + dx
    q["y_m"] = y + dy
    return q


def crop_layout(layout, placements, window, *, footprint_of,
                is_building=None, recenter=True, min_block_area_m2=1.0):
    """Filter+re-centre `(layout, placements)` to `window = (x0, y0, x1, y1)`.

    `footprint_of(p) -> (W, D) | None` gives a BUILDING placement's plan
    footprint for the AABB test (rule 1); `None` degrades that one placement
    to a point test. `is_building(p) -> bool` defaults to
    `p.get("category") in BUILDING_CATEGORIES` (the `house`/`building`
    convention `scene_generator`/`plan_png` already use) -- pass your own for
    a placement shape that does not carry `category`.

    Returns `(new_layout, new_placements, report)`. `new_layout` keeps every
    key of the input `layout` (a shallow copy), with `region`, `blocks`,
    `road_corridors` and `_typology_of` replaced. `report` is a plain dict of
    kept/dropped counts per population, JSON-serialisable, meant to be
    printed or asserted on -- see `tools/baseline_layouts.py` for the former
    and `tests/test_crop_window.py` for the latter.
    """
    wx0, wy0, wx1, wy1 = (float(v) for v in window)
    if wx1 <= wx0 or wy1 <= wy0:
        raise ValueError("crop_layout: degenerate window {0!r}".format(window))
    win = (wx0, wy0, wx1, wy1)
    cx, cy = (wx0 + wx1) / 2.0, (wy0 + wy1) / 2.0
    dx, dy = (-cx, -cy) if recenter else (0.0, 0.0)

    if is_building is None:
        is_building = lambda p: p.get("category") in BUILDING_CATEGORIES  # noqa: E731

    building_recs, other_recs = [], []
    for p in placements:
        (building_recs if is_building(p) else other_recs).append(p)

    dropped_building_tags = set()
    kept_buildings = []
    for p in building_recs:
        x, y = float(p.get("x_m", 0.0)), float(p.get("y_m", 0.0))
        wd = footprint_of(p)
        if wd is None:
            ok = _point_in(x, y, win)
        else:
            W, D = wd
            bx0, by0, bx1, by1 = _rot_aabb(x, y, float(W), float(D),
                                           float(p.get("yaw_deg", 0.0)))
            ok = (bx0 >= wx0 - 1e-6 and by0 >= wy0 - 1e-6
                 and bx1 <= wx1 + 1e-6 and by1 <= wy1 + 1e-6)
        if ok:
            kept_buildings.append(p)
        else:
            dropped_building_tags.add(tag_of(p.get("usd", ""), x, y))

    n_orphan_props = 0
    kept_props = []
    for p in other_recs:
        of = p.get("of")
        if of is not None and of in dropped_building_tags:
            n_orphan_props += 1
            continue
        x, y = float(p.get("x_m", 0.0)), float(p.get("y_m", 0.0))
        if not _point_in(x, y, win):
            continue
        kept_props.append(p)

    new_placements = ([_shift(p, dx, dy) for p in kept_buildings]
                      + [_shift(p, dx, dy) for p in kept_props])

    typ_of = layout.get("_typology_of") or {}
    new_blocks = []
    new_typ_of = {}
    n_blk_dropped = 0
    for b in layout.get("blocks", []):
        rect = tuple(b)
        clipped = _clip_rect(rect, win)
        if clipped is None or ((clipped[2] - clipped[0])
                               * (clipped[3] - clipped[1]) < min_block_area_m2):
            n_blk_dropped += 1
            continue
        shifted = (clipped[0] + dx, clipped[1] + dy,
                  clipped[2] + dx, clipped[3] + dy)
        new_blocks.append(list(shifted))
        t = typ_of.get(rect) or typ_of.get(b)
        if t is not None:
            new_typ_of[shifted] = t

    new_corridors = []
    n_cor_dropped = 0
    for c in layout.get("road_corridors", []):
        rect = (c["x0"], c["y0"], c["x1"], c["y1"])
        clipped = _clip_rect(rect, win)
        if clipped is None:
            n_cor_dropped += 1
            continue
        nc = dict(c)
        nc["x0"], nc["y0"] = clipped[0] + dx, clipped[1] + dy
        nc["x1"], nc["y1"] = clipped[2] + dx, clipped[3] + dy
        new_corridors.append(nc)

    new_region = (wx0 + dx, wy0 + dy, wx1 + dx, wy1 + dy)
    new_layout = dict(layout)
    new_layout["region"] = new_region
    new_layout["blocks"] = new_blocks
    new_layout["road_corridors"] = new_corridors
    new_layout["_typology_of"] = new_typ_of

    report = {
        "window": [wx0, wy0, wx1, wy1], "shift": [dx, dy],
        "buildings_kept": len(kept_buildings),
        "buildings_dropped": len(building_recs) - len(kept_buildings),
        "props_kept": len(kept_props),
        "props_dropped": len(other_recs) - len(kept_props),
        "props_orphan_dropped": n_orphan_props,
        "blocks_kept": len(new_blocks), "blocks_dropped": n_blk_dropped,
        "corridors_kept": len(new_corridors), "corridors_dropped": n_cor_dropped,
    }
    return new_layout, new_placements, report


def repair_after_crop(config, layout, placements, resolver,
                      street_tol_m=6.0, overlap_tol_m=0.2):
    """Re-run `districts.repair_facing` then `districts.repair_overlaps`
    against a CROPPED `(layout, placements)` -- see the module docstring's
    "clipping a block can invalidate a building's facing" section for why
    this is needed and measured, not a defensive reflex.

    Mutates `placements` in place (every step below does) and returns
    `{"facing": <repair_facing's own dict, plus "dropped_unrepairable_
    blank">, "overlaps": <repair_overlaps's own dict>}`. Call this once,
    immediately after `crop_layout`, on a HOST-PACKER `(layout, placements)`
    where a compiled `config` and a resolver (`plan_png.build()`'s `cfg`/
    `res`) are already on hand -- see the module docstring for why
    `fc_dump_crop.py` cannot do the same today.

    A REMAINING BLANK-WALL VIOLATION AFTER `repair_facing` IS DROPPED, NOT
    LEFT: `repair_facing` only reorients a placement's FRONT to face a real
    street; it has no opinion on a BLANK side, so a `place_mid` building
    whose FRONT already faced a street correctly survives that pass
    unchanged even when the crop has just removed the FLANK NEIGHBOUR that
    used to cover its blank side (measured on a real seed -- a `place_mid`
    GAC building's front correctly rotated onto a newly-street-facing block
    edge, while its blank side ALSO now faced a street because its run
    neighbour on that flank had been cropped away). That neighbour is gone
    precisely because the crop dropped it, and `detail.districts._order_run`
    's own doctrine already covers this exact shape: "a hole in a terrace is
    a vacant lot, which is a real thing; a blank wall facing a street is
    not." Dropping the now-exposed building is the SAME rule the packer
    already applies everywhere else, not a new one invented for cropping.
    """
    from detail import districts
    import plan_png as _plan_png

    facing = districts.repair_facing(config, layout, placements, resolver,
                                     street_tol_m=street_tol_m)

    violations = _plan_png._blank_wall_violations(config, layout, placements,
                                                   resolver)
    dropped = 0
    if violations:
        drop_keys = {(name, round(x, 3), round(y, 3))
                    for name, x, y, _yaw, _bad in violations}
        kept = []
        for p in placements:
            key = (os.path.basename(str(p.get("usd", ""))),
                  round(float(p.get("x_m", 0.0)), 3),
                  round(float(p.get("y_m", 0.0)), 3))
            if p.get("category") == "house" and key in drop_keys:
                dropped += 1
                continue
            kept.append(p)
        placements[:] = kept
    facing["dropped_unrepairable_blank"] = dropped

    overlaps = districts.repair_overlaps(config, layout, placements, resolver,
                                         tol=overlap_tol_m)
    return {"facing": facing, "overlaps": overlaps}
