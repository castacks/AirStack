#!/usr/bin/env python3
"""fire_city_force_blocks.py — force two "brownstone" mini-blocks into fire
on the `downtown_fire_500` city (seed 4 dump / seed 138 manifest), for a
user preview ("I wanna force some 'brownstone' mini blocks and see them on
fire").

HOST-SIDE, no Isaac, no pxr. Reads the REAL dump (`fc_dump_500.json`, 88
houses) and a REAL manifest (default: the CURRENT `_plans/fire_city_500m_
39.json`) and emits a CANDIDATE pair — never overwrites `scene_gen/_plans/`.

THIS FILE'S OWN BUG, FIXED 2026-08-31 (read before touching NEW_RECORDS_SPEC
or REAL_FOOTPRINT_M again)
------------------------------------------------------------------------
The first version of this tool ran the overlap SAT check with each new
record's `W`/`D` set to its CELL's own true dump size (the correct, existing
convention for the *schema* fields — see the class docstring on
`build_new_records` — because that is what `check_footprint`/
`fire_people._manifest_matches_dump` are keyed on). But the overlap/road-
clearance check needs the ARCHETYPE'S OWN REAL BUILT SIZE for any record
whose `style` diverges from the cell's true asset — the schema `W`/`D`
describe the LOT, not what gets built on it — and using the cell's (often
narrower) dims there let `i=264` pass with `style="brownstone_row"`
(real built width 39.6 m) dropped onto a 23.5 m-wide `apartment` cell 15 m
from the "lowrise" block's own eastern edge. It baked exactly as asked
(confirmed clean from the bake log/sidecar bbox below — 39.57 m wide, no
Nucleus asset substitution) and stuck ~4.5 m out past the block's true
boundary into the road. `REAL_FOOTPRINT_M` + `road_clearance_table` /
`fits_real_footprint` below are the fix: every check that decides whether a
building FITS now uses the archetype's real built size, never the cell's.

MEASURING A KIT STYLE'S REAL BUILT FOOTPRINT — the reliable, working method
------------------------------------------------------------------------
`detail.urban_building.footprint(STYLES[style])` **works** host-side (pure
Python, no pxr) but only if you pass it the SPEC DICT
(`ub.footprint(ub.STYLES["brownstone_row"])`, not the style NAME as a
string — a bare string blows up on `spec["bands"]`, which is almost
certainly what failed for a caller that tried `ub.footprint("brownstone_
row")`). Even called correctly, it returns the NOMINAL design-target size
from the style's own band/module math ("38 x 14" for `brownstone_row`),
which UNDERSTATES the real built footprint — measured here at 39.57 x 16.28
(+1.57 m W, **+2.28 m D**) — because roof ledges, corner returns and the
authored pieces' own small overhangs (`PIECES` table entries like
`SM_MBuilding03_RoofLedge_Corner`'s `(1.78, 1.78, ...)`) sit outside the
bands-only rectangle `footprint()` computes.

The RELIABLE number, and the one this tool uses, comes from the DUMP
itself: `fc_dump_500.json` already carries 6 real `bld_brownstone_DG0.usd`
and 5 real `bld_brownstone_row_DG0.usd` placements — genuine kit-archetype
builds the CITY GENERATOR measured at Kit-build time (`dump_city_
placements` records the composed prim's own bbox, not a design target).
All 6 brownstone entries agree to 23.57 x 16.28 x 18.26 m and all 5
brownstone_row entries agree to 39.57 x 16.28 x 21.26 m, to 2 decimal
places — cross-validated across independent builds at different seeds, so
this is measurement, not a single sample. `REAL_FOOTPRINT_M` below is
exactly these numbers. Recipe for a NEW style this tool has not yet used:
find >= 2 real dump placements whose `usd` is that style's own
`bld_<style>_DG0.usd`, and read their `W`/`D`/`H`.

BAKE-LOG CONFIRMATION THIS WAS NOT A NUCLEUS ISSUE
------------------------------------------------------------------------
`~/docker/isaac-sim/logs/city_138_kit_brownstone_row_F2_o2_S_s9104.log`:
`[fb] kit:brownstone_row F2 seed 9104 -> .../city_138` and `[scene_gen]
Applied 208 placements under '/World/bake/k35/parts' (11 unique USDs...)` —
the INTENDED archetype was requested and built (11 unique USDs is exactly
family-03's own piece count, not e.g. a single fallback cube). The
sidecar's own bbox is `[-19.783, -11.559, ..., 19.783, 7.783, 19.82]` — an
X-span of exactly 39.566 m, matching the real `brownstone_row` measurement
above to 3 decimals. A failed Nucleus pull composes NOTHING (the reference
resolves empty), not a wrong-but-real asset at the wrong size — this is a
correctly-built `brownstone_row`, on a lot sized for something narrower.

STREET SIDE EVIDENCE (why "S" is the confirmed street side for both rows,
not a guess): `urban_fire_spread.side_facing`/`entry_side` are in the
building's own LOCAL frame (`front == -Y == "S"` at yaw 0, both rows are
yaw 0). Two SPOT ignitions in the real 66-record manifest — the one
mechanism that only ever crosses an open street (`edges()`'s own
docstring) — land on local "S" for a building in EACH row (#258 via a spot
brand from #278, entry_side "S"; #266 via a spot brand from #271, entry_side
"S"), which is independent, real-geometry evidence that "S" is the
street-facing elevation for every yaw-0 member of both rows.

ROAD-EDGE CONVENTION — `dump["typology"]["blocks"]` (10 real block rects,
the same ones `disaster.fire_people.derive_layout` turns into
`road_corridors = rect_complement(region, blocks)`) is the host-side way to
know where roads are without Kit. A footprint corner outside its own block
rect is a HARD violation ("on the road", 0 m margin allowed — this is what
caught i=264). `SIDEWALK_MARGIN_M=5.0` (`fire_assembly_lib.
APRON_ROAD_CLEARANCE_M`'s own documented "5 m for the sidewalk a real block
still keeps between its building line and the kerb") is used as an
ADVISORY-only inner band, reported but not enforced as a hard failure,
because several genuinely pre-existing, untouched placements in row B
(e.g. #270, unchanged since before this tool existed) already sit only
3.5-4.3 m off that same block's own edge — enforcing 5 m as a hard gate
would flag placements this tool never touched and did not cause.

AREA-RATIO NOTE (deliberate, reported deviation, re-proven under the REAL
footprint below, not just the cell one): `check_footprint`'s
`area_ratio_max=1.3` compares each kit record's own CELL area against its
style's NOMINAL DG0 archetype area (`archetypes.json`, 22x14 for
`brownstone`). Three of the 7 records now use narrow `brownstone` on an
oversized cell (`#261`, `#264` post-fix, `#269` — all apartment/dw_terrace
cells) and miss it (~1.41-1.49 vs the 1.3 cap). There is NO footprint
overlap and NO road-edge violation for any of the three under the REAL
23.57 x 16.28 m footprint (re-verified below) — this remains a cosmetic
"the building is smaller than the plot it stands in" mismatch, reported
exactly like the pre-existing, accepted `has_f1` failure, never hidden.

USAGE
    python3 scene_gen/tools/fire_city_force_blocks.py [--out-dir DIR]
                                                       [--manifest PATH]
"""
import argparse
import copy
import hashlib
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
for _p in (_HERE, _SCENE_GEN):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fire_city_dry_run as fcd          # noqa: E402  (host-side, no pxr)
import fire_city_manifest as fcm         # noqa: E402
from disaster import fire_bake as fb     # noqa: E402
from disaster import quake as q          # noqa: E402

DEFAULT_DUMP = os.path.join(_SCENE_GEN, "_plans", "fc_dump_500.json")
# The CURRENT manifest (39 records: the base 32 + this tool's own 7 forced
# records, already adopted upstream — see the module docstring). Patched,
# never appended-to blindly: any of MY_SEEDS found here is REPLACED with a
# freshly regenerated (and now real-footprint-checked) record, not skipped.
DEFAULT_MANIFEST = os.path.join(_SCENE_GEN, "_plans", "fire_city_500m_39.json")
ARCH_DIR = os.path.join(_SCENE_GEN, "assets", "archetype")

#: real, MEASURED (not nominal) built footprint per kit style — see the
#: module docstring's "MEASURING A KIT STYLE'S REAL BUILT FOOTPRINT".
#: (W, D, H) in the style's own canonical (unrotated) frame.
REAL_FOOTPRINT_M = {
    "brownstone": (23.57, 16.28, 18.26),
    "brownstone_row": (39.57, 16.28, 21.26),
}
_BROWNSTONE_N_STOREYS = {"brownstone": 5, "brownstone_row": 6}
#: advisory only (see module docstring) -- reported in the clearance table,
#: never a hard failure by itself.
SIDEWALK_MARGIN_M = 5.0

MY_SEEDS = {9101, 9102, 9103, 9104, 9105, 9106, 9107}

# ---------------------------------------------------------------------------
# The 7 forced records. `style`/`level`/`entry_side`/`sides`/`origin_frac`/
# `via`/`how`/`seed` are authored here; `usd`/`cell`/`x`/`y`/`z`/`yaw_deg`
# and the SCHEMA `W`/`D`/`H` (the CELL's own true dump size — this is the
# `check_footprint`/`_manifest_matches_dump` convention, NOT the real built
# size used for the fit checks below) are pulled LIVE from the real dump by
# `i` (never transcribed) so a stale/incorrect copy can never drift from the
# dump this runs against.
#
# i=264 FIX (2026-08-31): was `style="brownstone_row"` (seed 9104) — real
# built width 39.57 m on a 23.5 m apartment cell only 15.25 m from the
# block's own east edge, 4.54 m HARD violation (see `road_clearance_table`
# output). Swapped to the narrow `brownstone` style per the fix ladder in
# `fit_record` (style swap tried before a position nudge, before a drop) --
# real built width 23.57 m clears the same edge by +3.46 m. Kept seed 9104
# (no collision: the OLD stem was `kit_brownstone_row_..._s9104`, the NEW
# one is `kit_brownstone_..._s9104` — the style segment alone already
# disambiguates them, and the old stem's bake is simply orphaned, not
# collided with).
# ---------------------------------------------------------------------------
NEW_RECORDS_SPEC = [
    # BLOCK 1 -- row A, y=~157m, yaw 0, street side "S" (see module doc).
    # 260 (existing, unchanged) -> 261 -> 262(F5c) -> 263 -> 264
    dict(i=261, style="brownstone",     level="F2",  sides=("S",),
         via=260, seed=9101),
    dict(i=262, style="brownstone_row", level="F5c", sides=("S", "E"),
         via=261, seed=9102),
    dict(i=263, style="brownstone",     level="F3",  sides=("S", "W"),
         via=262, seed=9103),
    dict(i=264, style="brownstone",     level="F2",  sides=("S",),
         via=263, seed=9104),               # <- FIXED, was brownstone_row
    # BLOCK 2 -- row B, y=~231m, yaw 0, street side "S". #267 (existing,
    # unchanged, already brownstone/F4) is the 4th visual member.
    # 267 (existing, unchanged) -> 268 -> 269 -> 270
    dict(i=268, style="brownstone_row", level="F3",  sides=("S", "W"),
         via=267, seed=9105),
    dict(i=269, style="brownstone",     level="F2",  sides=("S",),
         via=268, seed=9106),
    dict(i=270, style="brownstone_row", level="F3",  sides=("S", "E"),
         via=269, seed=9107),
]
ORIGIN_FRAC = 0.45   # radiation entry, low-mid compartment -- the
                     # convention every real "via=radiation" record in
                     # this manifest already uses (verified against the
                     # 66-record manifest before this tool was written).
F5C_INDEX = 262
BTYPE = "urm"        # masonry -- both brownstone kit styles ("family 03"),
                     # per the 2026-08-31 lead note.
TYPOLOGY = "lowrise"


def _load(path):
    with open(path) as fh:
        return json.load(fh)


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------------------
# Real-footprint geometry: block lookup + rotated-corner fit test
# ---------------------------------------------------------------------------
def dump_blocks(dump_doc):
    """`[(x0,y0,x1,y1,name), ...]` -- the REAL block rects a road corridor
    is the complement of (`disaster.fire_people.derive_layout`)."""
    out = []
    for b in (dump_doc.get("typology") or {}).get("blocks") or []:
        r = b.get("rect")
        if r and len(r) == 4:
            x0, y0, x1, y1 = r
            out.append((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1),
                       b.get("name")))
    return out


def block_at(blocks, x, y):
    for x0, y0, x1, y1, name in blocks:
        if x0 <= x <= x1 and y0 <= y <= y1:
            return (x0, y0, x1, y1, name)
    return None


def _corners(cx, cy, W, D, yaw_deg):
    a = math.radians(yaw_deg)
    ca, sa = math.cos(a), math.sin(a)
    hw, hd = W / 2.0, D / 2.0
    return [(cx + ca * dx - sa * dy, cy + sa * dx + ca * dy)
            for dx, dy in ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd))]


def real_wd(rec):
    """The REAL built (W, D) for one record's fit-checking purposes --
    `REAL_FOOTPRINT_M[style]` for a kit record in that table (regardless of
    whether `style` matches the cell's own true asset), else the record's
    own schema `W`/`D` (a non-diverging / non-kit record: cell size IS the
    real size)."""
    if rec.get("kind") == "kit" and rec.get("style") in REAL_FOOTPRINT_M:
        w, d, _h = REAL_FOOTPRINT_M[rec["style"]]
        return w, d
    return float(rec["W"]), float(rec["D"])


def block_margins(blocks, x, y, W, D, yaw_deg):
    """`(margins, block)` -- `margins = {"E":..,"W":..,"N":..,"S":..}`,
    the clear distance from the REAL rotated footprint's own extent to the
    containing block's edge on each side (negative = past the edge, onto
    the road)."""
    blk = block_at(blocks, x, y)
    if blk is None:
        return None, None
    bx0, by0, bx1, by1, _name = blk
    cs = _corners(x, y, W, D, yaw_deg)
    xs = [c[0] for c in cs]
    ys = [c[1] for c in cs]
    margins = {"W": min(xs) - bx0, "E": bx1 - max(xs),
              "S": min(ys) - by0, "N": by1 - max(ys)}
    return margins, blk


def fits_real_footprint(blocks, rec, extra=()):
    """`(ok, margins, block, overlap_pairs)` for one record's REAL built
    footprint: `ok` is False on a HARD block-edge violation (any margin
    < 0) OR a real-footprint overlap against `extra` (other records/dump
    placements to test against, each `{"i","x","y","W","D","yaw_deg"}` --
    already using REAL wd for any kit-divergent style)."""
    W, D = real_wd(rec)
    margins, blk = block_margins(blocks, rec["x"], rec["y"], W, D,
                                 rec["yaw_deg"])
    hard_violation = margins is not None and min(margins.values()) < 0.0
    my_corners = _corners(rec["x"], rec["y"], W, D, rec["yaw_deg"])
    overlaps = []
    for other in extra:
        if other["i"] == rec["i"]:
            continue
        oc = _corners(other["x"], other["y"], other["W"], other["D"],
                     other["yaw_deg"])
        if fcd._obb_overlap(my_corners, oc):
            overlaps.append(other["i"])
    ok = (not hard_violation) and (not overlaps)
    return ok, margins, blk, overlaps


# ---------------------------------------------------------------------------
# Record construction + the fix ladder
# ---------------------------------------------------------------------------
def _build_one(spec, p):
    style = spec["style"]
    n_storeys = _BROWNSTONE_N_STOREYS[style]
    storey = max(0, min(n_storeys - 1,
                        int(round(ORIGIN_FRAC * (n_storeys - 1)))))
    rec = {
        "i": spec["i"],
        "level": spec["level"],
        "origin": storey,
        "sides": list(spec["sides"]),
        "t_ignite_s": 0.0,
        "age_s": {"F2": 900.0, "F3": 1800.0, "F5c": 9000.0}[spec["level"]],
        "via": spec["via"],
        "how": "radiation",
        "W": p["W"], "D": p["D"], "H": p["H"],   # CELL's own true size --
                                                 # schema convention, see
                                                 # module docstring.
        "kind": "kit",
        "asset": None,
        "style": style,
        "typology": TYPOLOGY,
        "cell": p["cell"],
        "btype": BTYPE,
        "entry_side": spec["sides"][0],
        "origin_frac": ORIGIN_FRAC,
        "n_storeys": n_storeys,
        "height_class": "low",
        "seed": spec["seed"],
        "usd": p["usd"],
        "x": p["x_m"], "y": p["y_m"], "yaw_deg": p["yaw_deg"],
        "z": p["z_m"],
    }
    assert set(rec.keys()) == {
        "D", "H", "W", "age_s", "asset", "btype", "cell", "entry_side",
        "height_class", "how", "i", "kind", "level", "n_storeys",
        "origin", "origin_frac", "seed", "sides", "style",
        "t_ignite_s", "typology", "usd", "via", "x", "y", "yaw_deg",
        "z"}, "schema drift: {0}".format(sorted(rec.keys()))
    return rec


#: fix ladder order for a style that overflows its cell: try the NARROW
#: style before the wide one (never the reverse -- widening never helps).
_NARROWER = {"brownstone_row": "brownstone"}
#: bounded nudge search, metres, tried along the record's own local WIDTH
#: axis (world +/-X at yaw 0/180, +/-Y at yaw 90/270) toward the block's
#: interior -- only reached if a style swap alone does not clear the edge.
_NUDGE_STEPS_M = (1.0, 2.0, 3.0, 4.0, 5.0)


def fit_record(spec, p, blocks, others_by_i, log):
    """Returns `(record_or_None, note)`. Tries, in order: the SPEC's own
    style as given; the narrower style (if any) at the same position;
    a bounded nudge INTO the block at the narrower/original style; drop
    (`None`) with a reason. `others_by_i` -- every OTHER record/placement
    real-footprint entry to check neighbour overlap against (never
    includes this record's own `i`)."""
    candidates = [dict(spec)]
    if spec["style"] in _NARROWER:
        alt = dict(spec)
        alt["style"] = _NARROWER[spec["style"]]
        candidates.append(alt)

    for cand in candidates:
        rec = _build_one(cand, p)
        others = [v for k, v in others_by_i.items() if k != rec["i"]]
        ok, margins, blk, overlaps = fits_real_footprint(blocks, rec, others)
        if ok:
            tag = ("as specified" if cand is candidates[0]
                  else "SWAPPED style {0} -> {1} (fix)".format(
                      spec["style"], cand["style"]))
            log.append((spec["i"], tag, margins, overlaps))
            return rec, tag

    # nudge: try the narrowest candidate we have, moved along the local
    # width axis toward the block interior.
    base = candidates[-1]
    # width axis in world space at this yaw: (cos(yaw), sin(yaw))
    ax = math.cos(math.radians(p["yaw_deg"]))
    ay = math.sin(math.radians(p["yaw_deg"]))
    W, D, _h = REAL_FOOTPRINT_M[base["style"]]
    rec0 = _build_one(base, p)
    margins0, blk0 = block_margins(blocks, rec0["x"], rec0["y"], W, D,
                                   rec0["yaw_deg"])
    if margins0 is None:
        log.append((spec["i"], "NO BLOCK FOUND at this position", None, []))
        return None, "dropped: no containing block"
    # nudge toward whichever side is currently negative (or smallest)
    worst_side = min(margins0, key=lambda k: margins0[k])
    sign = {"E": -1, "W": 1, "N": -1, "S": 1}[worst_side]
    step_axis = (ax, ay) if worst_side in ("E", "W") else (-ay, ax)
    for step in _NUDGE_STEPS_M:
        dx = sign * step * step_axis[0]
        dy = sign * step * step_axis[1]
        cand = dict(base)
        rec = _build_one(cand, p)
        rec["x"] += dx
        rec["y"] += dy
        others = [v for k, v in others_by_i.items() if k != rec["i"]]
        ok, margins, blk, overlaps = fits_real_footprint(blocks, rec, others)
        if ok:
            tag = "NUDGED {0:.1f} m toward block interior (fix)".format(step)
            log.append((spec["i"], tag, margins, overlaps))
            return rec, tag

    log.append((spec["i"], "DROPPED -- no style/nudge clears the block edge "
               "or a neighbour", margins0, []))
    return None, "dropped: no fix found"


def build_new_records(dump_by_i, blocks):
    """The 7 records, each run through `fit_record`'s ladder. `others_by_i`
    accumulates as records are accepted so later records in the same block
    see earlier ones' REAL footprints for the neighbour-overlap half of the
    check, exactly like a real placement pass."""
    others_by_i = {}
    log = []
    out = []
    dropped = []
    for spec in NEW_RECORDS_SPEC:
        i = spec["i"]
        p = dump_by_i[i]
        rec, note = fit_record(spec, p, blocks, others_by_i, log)
        if rec is None:
            dropped.append((i, note))
            continue
        out.append(rec)
        W, D = real_wd(rec)
        others_by_i[i] = {"i": i, "x": rec["x"], "y": rec["y"],
                          "W": W, "D": D, "yaw_deg": rec["yaw_deg"]}
    return out, dropped, log


# ---------------------------------------------------------------------------
# Verification
# ---------------------------------------------------------------------------
def full_88_footprint_census(dump_placements, manifest_records):
    """One footprint per dump index -- dump geometry, overridden by
    whatever the manifest currently burns there, using the REAL built (W,D)
    for any kit record whose style diverges (`real_wd`) -- for the
    "against all 88" SAT pass."""
    by_i = {}
    for p in dump_placements:
        by_i[p["i"]] = {"i": p["i"], "x": p["x_m"], "y": p["y_m"],
                        "W": p["W"], "D": p["D"], "yaw_deg": p["yaw_deg"]}
    for r in manifest_records:
        w, d = real_wd(r)
        by_i[r["i"]] = {"i": r["i"], "x": r["x"], "y": r["y"],
                        "W": w, "D": d, "yaw_deg": r["yaw_deg"]}
    return list(by_i.values())


def road_clearance_table(blocks, records):
    rows = []
    for r in records:
        w, d = real_wd(r)
        margins, blk = block_margins(blocks, r["x"], r["y"], w, d,
                                     r["yaw_deg"])
        hard = margins is not None and min(margins.values()) < 0.0
        warn = (not hard) and margins is not None and (
            min(margins.values()) < SIDEWALK_MARGIN_M)
        rows.append({"i": r["i"], "style": r.get("style"), "W": w, "D": d,
                    "block": blk[4] if blk else None,
                    "margins": margins, "hard_violation": hard,
                    "advisory_warn": warn})
    return rows


def run_verification(dump_doc, manifest_doc, new_records, blocks):
    report = {}

    census = full_88_footprint_census(dump_doc["placements"],
                                      manifest_doc["records"])
    fake = {"records": census}
    ok_overlap, detail_overlap = fcd.check_footprint(fake)
    report["overlap_vs_all_88"] = (ok_overlap, detail_overlap)

    report["road_clearance"] = road_clearance_table(
        blocks, [r for r in manifest_doc["records"] if r["i"] in
                {s["i"] for s in NEW_RECORDS_SPEC}])

    full_manifest = q.load_manifest(ARCH_DIR)
    manifest_dg0 = {style: rec for (style, level), rec in full_manifest.items()
                    if level == "DG0"}
    checks = fcd.run_all_checks(manifest_doc, manifest_dg0=manifest_dg0)
    report["dry_run_checks"] = checks

    out_dir = (os.environ.get("FB_OUT")
              or os.path.expanduser(
                  "~/docker/isaac-sim/cache/main/fire_bakes/city_138"))
    my_rows = fcm.classify(new_records, out_dir)
    other_records = [r for r in manifest_doc["records"]
                     if r["i"] not in {s["i"] for s in NEW_RECORDS_SPEC}]
    other_rows = fcm.classify(other_records, out_dir)
    report["new_stems"] = my_rows
    report["other_have"] = sum(1 for r in other_rows if r["status"] == "HAVE")
    report["other_total"] = len(other_rows)
    report["out_dir"] = out_dir

    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dump", default=DEFAULT_DUMP)
    ap.add_argument("--manifest", default=DEFAULT_MANIFEST)
    ap.add_argument("--out-dir", default=os.path.join(
        "/tmp/claude-1000/-home-krrishjain-SEI-COA-disaster-dataset/"
        "16ec6066-0093-43cb-86a8-200733668b11/scratchpad/brownstone"))
    args = ap.parse_args()

    dump_doc = _load(args.dump)
    manifest_doc = _load(args.manifest)
    dump_by_i = {p["i"]: p for p in dump_doc["placements"]}
    blocks = dump_blocks(dump_doc)

    my_targets = {s["i"] for s in NEW_RECORDS_SPEC}
    base_by_i = {r["i"]: r for r in manifest_doc["records"]}
    diff_old = {}
    for i in my_targets:
        if i not in dump_by_i:
            print("*** i={0} is not in the dump at all".format(i))
            return 2
        existing = base_by_i.get(i)
        if existing is not None:
            if existing.get("seed") not in MY_SEEDS:
                print("*** i={0} already has a FOREIGN record (seed={1}) "
                      "in {2} -- refusing to clobber real content"
                      .format(i, existing.get("seed"), args.manifest))
                return 2
            diff_old[i] = existing   # mine, from an earlier run -- may
                                    # be patched/replaced below

    new_records, dropped, fit_log = build_new_records(dump_by_i, blocks)

    # ---- assemble the corrected candidate: base manifest MINUS my old
    #      records (if present) PLUS the freshly fitted ones -------------
    kept_base = [r for r in manifest_doc["records"] if r["i"] not in my_targets]
    candidate = copy.deepcopy(manifest_doc)
    candidate["records"] = kept_base + new_records
    candidate["n"] = len(candidate["records"])
    candidate["n_achieved"] = len(candidate["records"])
    candidate["note"] = (
        (manifest_doc.get("note") or "")
        + " | brownstone-block records (i={0}) patched by "
          "fire_city_force_blocks.py (real-footprint road-clearance fix, "
          "2026-08-31), base dump sha256={1}".format(
              ",".join(str(i) for i in sorted(my_targets)),
              _sha256(args.dump)[:12]))

    report = run_verification(dump_doc, candidate, new_records, blocks)

    # ---- write the candidate pair -----------------------------------
    os.makedirs(args.out_dir, exist_ok=True)
    cand_dump_path = os.path.join(args.out_dir, "fc_dump_500_brownstone.json")
    cand_manifest_path = os.path.join(
        args.out_dir, "fire_city_500m_39_brownstone_fixed.json")
    with open(cand_dump_path, "w") as fh:
        json.dump(dump_doc, fh, indent=1)      # UNCHANGED, written for a
                                               # self-contained pair
    with open(cand_manifest_path, "w") as fh:
        json.dump(candidate, fh, indent=1)

    # ---- report -------------------------------------------------------
    print("=" * 78)
    print("fire_city_force_blocks: CORRECTED candidate written")
    print("  dump      -> {0} (byte-identical copy, 0 changes)".format(
        cand_dump_path))
    print("  manifest  -> {0} ({1} kept + {2} refitted = {3} records)"
          .format(cand_manifest_path, len(kept_base), len(new_records),
                  len(candidate["records"])))
    print("=" * 78)

    def stem_of(rec):
        _entry, stem = fcm.build_entry_and_stem(rec, 0)
        return stem

    print("\n-- diff: old stem -> new stem / dropped --")
    for i in sorted(my_targets):
        old = diff_old.get(i)
        old_stem = stem_of(old) if old else "(none)"
        new = next((r for r in new_records if r["i"] == i), None)
        if new is None:
            drop_reason = next((rn for ri, rn in dropped if ri == i), "?")
            print("  i={0:<4} {1:<45} -> DROPPED ({2})".format(
                i, old_stem, drop_reason))
            continue
        new_stem = stem_of(new)
        tag = "unchanged" if old_stem == new_stem else "CHANGED"
        print("  i={0:<4} {1:<45} -> {2:<45} [{3}]".format(
            i, old_stem, new_stem, tag))

    print("\n-- fit ladder log (per record) --")
    for i, note, margins, overlaps in fit_log:
        m = ("E={E:.2f} W={W:.2f} N={N:.2f} S={S:.2f}".format(**margins)
            if margins else "n/a")
        print("  i={0:<4} {1:<50} margins[{2}] overlaps={3}".format(
            i, note, m, overlaps))

    print("\n-- road clearance table (all 7 target records, REAL footprint) --")
    print("  {0:<5}{1:<16}{2:>7}{3:>7}  {4:<10}{5:>7}{6:>7}{7:>7}{8:>7}  "
          "{9:<5}{10:<8}".format(
              "i", "style", "W", "D", "block", "E", "W", "N", "S",
              "HARD", "warn<5m"))
    for row in report["road_clearance"]:
        m = row["margins"] or {}
        print("  {0:<5}{1:<16}{2:>7.2f}{3:>7.2f}  {4:<10}{5:>7.2f}{6:>7.2f}"
              "{7:>7.2f}{8:>7.2f}  {9:<5}{10:<8}".format(
                  row["i"], row["style"], row["W"], row["D"],
                  str(row["block"]), m.get("E", 0), m.get("W", 0),
                  m.get("N", 0), m.get("S", 0),
                  "YES" if row["hard_violation"] else "no",
                  "YES" if row["advisory_warn"] else "no"))

    print("\n-- bake stems for the (refitted) 7 -- lead must bake any "
          "marked NEED (~2 min each on GPU) --")
    for row in report["new_stems"]:
        print("  i={0}\n    entry: {1}\n    stem:  {2}.usd/.json  [{3}]"
              .format(row["record"]["i"], row["entry"], row["stem"],
                      row["status"]))

    print("\n-- other {0}/{1} manifest records still HAVE their bake "
          "(untouched by this patch) --".format(
              report["other_have"], report["other_total"]))

    ok_overlap, detail_overlap = report["overlap_vs_all_88"]
    print("\n-- REAL-FOOTPRINT overlap SAT vs all 88 dump slots --")
    print("  ok={0}  overlapping pairs={1}".format(
        ok_overlap, detail_overlap["overlaps"]))

    print("\n-- dry-run checks (candidate = corrected {0}-record manifest) "
          "--".format(len(candidate["records"])))
    for name, (ok, detail) in report["dry_run_checks"].items():
        tag = "PASS" if ok else "FAIL"
        print("  [{0}] {1}".format(tag, name))
        if not ok:
            if name == "level_distribution" and not detail.get("has_f1"):
                print("      (expected/accepted: no F1 record -- same "
                      "known failure as the un-forced candidate)")
            elif name == "footprint" and detail.get("area_ratio_violations"):
                print("      area_ratio_violations (expected/accepted, "
                      "re-proven under the REAL footprint above -- see "
                      "module docstring): {0}".format(
                          detail["area_ratio_violations"]))
            else:
                print("      UNEXPECTED: {0}".format(detail))

    if dropped:
        print("\n-- {0} record(s) DROPPED (no fix found) --".format(
            len(dropped)))
        for i, reason in dropped:
            print("  i={0}: {1}".format(i, reason))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
