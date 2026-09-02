#!/usr/bin/env python3
"""city_layout_audit.py — OFFLINE audit of a built city: does every building
sit inside a typology block (off the roads), which blocks are empty, and are
a damage manifest's records in the SAME coordinate frame as the city they
are composed into?  Pure python, no Kit, no pxr.

    python3 scene_gen/tools/city_layout_audit.py \\
        --gt final_disaster_dataset/Fire/Urban/level_1/1/GT_hints.json \\
        --dump scene_gen/_plans/baseline_l1_dump.json \\
        --manifest scene_gen/_plans/baseline_l1_manifest.json \\
        [--layout <host-built layout json with blocks + road_corridors>] \\
        [--png ~/fire_previews/road_overlap/level1.png] [--json report.json]

THE 2026-09-01 BASELINE INCIDENT THIS EXISTS FOR.  The three urban-fire
baseline cells (`Fire/Urban/level_{1,2,3}/1`) were built from manifests
solved on RE-CENTRED 1 km crops of a 1.5 km dump (`fc_dump_crop.py`, shift
= minus the window centre), then composed into the FULL 1.5 km stage
WITHOUT `FC_CROP_WINDOW` — and `urban_fire_city_launch_script.load_fire`
only shifted the records back when that env var was set.  Every burnt
substitute therefore landed one window-centre away from the intact building
it replaced: (+180, -180) m in level 1, (-100, -150) m in level 2,
(-20, -230) m in level 3.  The intact cell was hidden (an EMPTY LOT) and the
burnt shell dropped wherever the shifted coordinate fell — on roads, across
other buildings.  Measured from the shipped GT: 23 / 41 / 86 records
overhang a block at the shipped coordinates, 0 / 0 / 0 at the original-
frame ones (`record_xy`).  The user's report: "buildings are spawning on
top of the road, they are empty blocks".

WHAT IT MEASURES

  `audit_footprints(fps, blocks)`   fraction of each footprint inside its
                                    best-overlapping block; offenders below
                                    `min_frac`; blocks with nothing in them.
  `road_overlap(rect, corridors)`   fraction of a footprint lying on road
                                    corridors (needs a layout with
                                    `road_corridors`, i.e. a host build or
                                    a Kit layout dump — the placement dump
                                    carries blocks only).
  `centre_in_road(x, y, corridors)` a placement's own centre inside a
                                    corridor (needs `--layout`, same as above).
  `building_overlap_pairs(records)` BUILDING-VS-BUILDING footprint overlap —
                                    a proper separating-axis test over each
                                    record's REAL, yaw-ROTATED box (needs no
                                    block/road/layout input, only the dump's
                                    own `x_m`/`y_m`/`W`/`D`/`yaw_deg`). See
                                    its own docstring: THE 2026-09-01
                                    "SYSTEMIC LAYOUT DEFECT" investigation —
                                    a report of ~29-43 "overlapping" building
                                    pairs and 6-11 "empty blocks" per
                                    baseline level did not reproduce against
                                    this tool's rotation-correct footprints
                                    (0/0/0 real overlaps, 0/0/0 non-boundary
                                    empty blocks, 0/0/0 buildings overhanging
                                    a block on all three shipped dumps —
                                    `tests/test_city_layout_audit.py`'s
                                    parametrized `test_shipped_baseline_
                                    dumps_have_*` tests are that gate). It DOES
                                    reproduce, in the same order of
                                    magnitude, from a NAIVE axis-aligned
                                    check that skips the yaw rotation the
                                    packer applies everywhere else
                                    (`detail.districts._pool_entries`/
                                    `_rotated_wh`) — every "overlap" it finds
                                    is a legitimately touching pair standing
                                    at a relative 90-degree turn (a terrace
                                    end, two guillotine-packed neighbours
                                    sharing a party wall), not a placement
                                    defect. The packer already sizes/spaces
                                    every lot from each asset's REAL
                                    resolver-measured footprint (never a
                                    nominal constant — `_pool_entries`/
                                    `_fp_of` both call `resolver.get()`), and
                                    `districts.repair_overlaps` already runs
                                    on that real geometry (verified: `checked
                                    =0` on all three baseline seeds, host-
                                    rebuilt) — this function exists so the
                                    OFFLINE AUDIT agrees with the packer's
                                    own verdict instead of re-litigating it
                                    with a rotation-blind box.
  `manifest_frame_check(...)`       per record: distance between where the
                                    launcher WOULD place the bake and the
                                    intact cell it replaces, for both the
                                    shipped rule (`rec["x"]`) and the fixed
                                    one (`record_xy`).

The CLI always runs `building_overlap_pairs` and a dump-self block audit
(buildings vs. the dump's own typology blocks, in the dump's own frame) —
no `--gt`/`--manifest`/`--layout` needed, so a dump can be gated the moment
it exists. `--layout` (a host-built layout with `road_corridors`) adds
`centre_in_road`.

`record_xy(rec)` is the ONE coordinate rule: `x_orig`/`y_orig` when the
record carries them (stamped by `urban_fire_city.burnable()` from a cropped
dump's `x_m_orig`), else `x`/`y`.  The launcher defines the same function;
`tests/test_city_layout_audit.py` executes the launcher's own source and
checks the two agree on every record shape.
"""
import argparse
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import fire_city_dry_run as _fdr        # noqa: E402 -- _obb_corners/_obb_overlap

MIN_FRAC_IN_BLOCK = 0.97     # a building this far inside its block is "on it"
CELL_MATCH_TOL_M = 0.5       # bake holder vs intact cell, same as resolve_cell
EMPTY_BLOCK_MIN_SIDE_M = 15.0  # thinner than this is a road-edge sliver


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------
def record_xy(rec):
    """The full-city (stage-frame) coordinate of a manifest record.

    `x_orig`/`y_orig` WIN when present: they are the cropped dump's own
    `x_m_orig`/`y_m_orig`, i.e. exactly where Kit authored the intact cell.
    Only a record that never went through a crop has neither, and there
    `x`/`y` already ARE the stage frame."""
    if "x_orig" in rec and "y_orig" in rec:
        return float(rec["x_orig"]), float(rec["y_orig"])
    return float(rec.get("x", 0.0)), float(rec.get("y", 0.0))


def footprint_rect(x, y, W, D, yaw_deg):
    """Axis-aligned footprint of a W x D building at (x, y) with yaw.

    Exact for yaw in {0, 90, 180, 270} (every city placement — the packer
    only yaws by 0/90 and the pack's own yaw-offset is a multiple of 90);
    the AABB of the rotated corners otherwise (conservative)."""
    W, D = float(W), float(D)
    r = (float(yaw_deg) % 360.0)
    if abs(r - 90.0) < 1e-6 or abs(r - 270.0) < 1e-6:
        W, D = D, W
    elif abs(r) > 1e-6 and abs(r - 180.0) > 1e-6:
        a = math.radians(r)
        c, s = abs(math.cos(a)), abs(math.sin(a))
        W, D = W * c + D * s, W * s + D * c
    return (x - W / 2.0, y - D / 2.0, x + W / 2.0, y + D / 2.0)


def _overlap_area(a, b):
    ox = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    oy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    return ox * oy


def rect_frac_inside(rect, blocks):
    """`(fraction_of_rect_inside_best_block, best_block_index)`; (0, None)
    for a degenerate rect."""
    area = (rect[2] - rect[0]) * (rect[3] - rect[1])
    if area <= 0.0:
        return 0.0, None
    best, best_i = 0.0, None
    for i, b in enumerate(blocks):
        a = _overlap_area(rect, b[:4])
        if a > best:
            best, best_i = a, i
    return best / area, best_i


def road_overlap(rect, corridors):
    """Fraction of `rect` lying on any road corridor (corridors are dicts
    with x0/y0/x1/y1 as `scene_generator._subdivide_region_metric` returns,
    or plain 4-tuples).  Corridors overlap each other at intersections, so
    the sum is clipped to 1."""
    area = (rect[2] - rect[0]) * (rect[3] - rect[1])
    if area <= 0.0:
        return 0.0
    tot = 0.0
    for c in corridors:
        cr = ((c["x0"], c["y0"], c["x1"], c["y1"]) if isinstance(c, dict)
              else tuple(c[:4]))
        tot += _overlap_area(rect, cr)
    return min(1.0, tot / area)


def centre_in_road(x, y, corridors):
    """True when point *(x, y)* (a building's own placement centre, not its
    footprint) lies inside any road corridor — the strict "on the road"
    reading `road_overlap`'s footprint-fraction test does not give you by
    itself (a building can graze a corridor with 4% of its footprint while
    its own centre stands nowhere near it)."""
    for c in corridors:
        cr = ((c["x0"], c["y0"], c["x1"], c["y1"]) if isinstance(c, dict)
              else tuple(c[:4]))
        if cr[0] <= x <= cr[2] and cr[1] <= y <= cr[3]:
            return True
    return False


# ---------------------------------------------------------------------------
# building-vs-building overlap — SAT over REAL, yaw-rotated footprints
# ---------------------------------------------------------------------------
def building_overlap_pairs(records, tol=None):
    """Every genuine footprint interpenetration among *records*
    (`{"x_m", "y_m", "W", "D", "yaw_deg"}`-shaped dicts — a placements dump's
    own `placements` list needs no adapting) via a proper separating-axis
    test over each building's REAL, yaw-ROTATED oriented box — reusing
    `fire_city_dry_run._obb_corners` / `_obb_overlap` / `OVERLAP_TOL_M`, the
    SAME tolerance/definition the fire manifest solver already gates every
    damaged record's footprint on (`check_footprint`), so "clean" here means
    the same thing it means there.

    WHY THIS MUST ROTATE BY YAW, NOT JUST OFFSET BY W/2, D/2: a placement's
    `W`/`D` are the asset's extents in ITS OWN frame; at yaw 90/270 the
    footprint standing in the WORLD is D wide x W deep, not W x D — exactly
    the swap `detail.districts._pool_entries`/`_rotated_wh` apply everywhere
    else in the packer. A naive axis-aligned check that skips this rotation
    reads every legitimately touching pair standing at a relative 90-degree
    turn (a terrace end, two guillotine-packed neighbours sharing a party
    wall) as "overlapping".

    MEASURED, 2026-09-01, on the three shipped `downtown_fire_1500[_lvl2/3]`
    baseline dumps: a naive W x D box (no yaw rotation) reports 112 / 82 / 79
    "overlapping" pairs per level — the example that motivated this function,
    `SM_Building_03` (yaw 180) at (-335.09, 126.89) against
    `SM_Building_06_Small` (yaw 90) at (-313.27, 126.94), among them, at a
    naively-computed 7.1 m of "overlap". Rotated correctly by each building's
    own `yaw_deg`, that same pair's footprints are 0.02 m apart — a party
    wall, not a defect — and every naive "overlap" in all three dumps
    resolves to a legitimate touch: this function reports 0 / 0 / 0 pairs on
    the same three dumps. The packer already sizes and spaces every lot from
    each asset's REAL resolver-measured footprint (`detail.districts.
    _pool_entries`/`_fp_of` both call `resolver.get()`, never a nominal
    config constant), and `districts.repair_overlaps` already runs on that
    same real geometry and independently finds nothing to repair — this
    function exists so the OFFLINE AUDIT agrees with the packer's own
    verdict instead of re-litigating it with a rotation-blind box.

    Returns `[(i, j), ...]` — index pairs into *records*, its own order."""
    tol = _fdr.OVERLAP_TOL_M if tol is None else tol
    corners = [_fdr._obb_corners(float(r["x_m"]), float(r["y_m"]),
                                 float(r["W"]), float(r["D"]),
                                 float(r.get("yaw_deg", 0.0)))
              for r in records]
    pairs = []
    for a in range(len(records)):
        ca = corners[a]
        for b in range(a + 1, len(records)):
            if _fdr._obb_overlap(ca, corners[b], tol):
                pairs.append((a, b))
    return pairs


# ---------------------------------------------------------------------------
# inputs
# ---------------------------------------------------------------------------
def unshifted_blocks(dump):
    """A placement dump's typology blocks in the FULL-city frame (a cropped
    dump's blocks are in the re-centred frame; `crop.shift` undoes it)."""
    sx, sy = 0.0, 0.0
    crop = dump.get("crop") or {}
    if crop.get("shift"):
        sx, sy = float(crop["shift"][0]), float(crop["shift"][1])
    out = []
    for b in (dump.get("typology") or {}).get("blocks") or []:
        r = b["rect"]
        out.append((r[0] - sx, r[1] - sy, r[2] - sx, r[3] - sy,
                    b.get("name")))
    return out


def hints_footprints(gt):
    """Building footprints off a `GT_hints.json` (on-stage bboxes)."""
    fps = []
    for h in gt.get("hints") or []:
        if h.get("class") not in ("Building", "Damaged building"):
            continue
        if "bbox_min" not in h or "bbox_max" not in h:
            # Many pristine "Building" hints carry no bbox at all (only
            # prim_path/style/yaw) -- not every hint recorder computes one.
            # Skipping them here (rather than crashing) means this function
            # audits whatever subset of hints DOES carry geometry, which is
            # still useful signal, instead of failing the whole audit run.
            continue
        x0, y0 = h["bbox_min"][0], h["bbox_min"][1]
        x1, y1 = h["bbox_max"][0], h["bbox_max"][1]
        fps.append({"name": h.get("prim_path", "").split("/")[-1],
                    "prim_path": h.get("prim_path", ""),
                    "rect": (x0, y0, x1, y1),
                    "centre": ((x0 + x1) / 2.0, (y0 + y1) / 2.0),
                    "cls": h.get("class"), "style": h.get("style"),
                    "substitute": h.get("prim_path", "").startswith("/World/fire/")})
    return fps


def manifest_footprints(records, use=record_xy):
    """Footprints of manifest records at the coordinate `use(rec)` gives."""
    fps = []
    for r in records:
        x, y = use(r)
        fps.append({"name": r.get("cell", "").split("/")[-1],
                    "prim_path": r.get("cell", ""),
                    "rect": footprint_rect(x, y, r["W"], r["D"],
                                           r.get("yaw_deg", 0.0)),
                    "centre": (x, y), "cls": "record",
                    "style": r.get("style") or r.get("asset"),
                    "substitute": True})
    return fps


# ---------------------------------------------------------------------------
# the audits
# ---------------------------------------------------------------------------
def audit_footprints(fps, blocks, min_frac=MIN_FRAC_IN_BLOCK, window=None,
                     margin_m=0.0):
    """`{"n", "offenders": [...], "empty_blocks": [...], "by_block": {...}}`.

    `window` + `margin_m`: only judge footprints whose centre is at least
    `margin_m` inside `window` (a CROPPED dump only carries the blocks the
    window kept, so a building just outside it has no block to be in)."""
    def _judged(fp):
        if window is None:
            return True
        cx, cy = fp["centre"]
        return (window[0] + margin_m < cx < window[2] - margin_m
                and window[1] + margin_m < cy < window[3] - margin_m)

    offenders, by_block, n = [], {}, 0
    for fp in fps:
        if not _judged(fp):
            continue
        n += 1
        frac, bi = rect_frac_inside(fp["rect"], blocks)
        if bi is not None:
            cx, cy = fp["centre"]
            b = blocks[bi]
            if b[0] <= cx <= b[2] and b[1] <= cy <= b[3]:
                by_block[bi] = by_block.get(bi, 0) + 1
        if frac < min_frac:
            offenders.append(dict(fp, frac_in=frac, block=bi))
    offenders.sort(key=lambda o: o["frac_in"])
    empty = []
    for i, b in enumerate(blocks):
        if by_block.get(i, 0):
            continue
        w, h = b[2] - b[0], b[3] - b[1]
        if min(w, h) < EMPTY_BLOCK_MIN_SIDE_M:
            continue                     # road-edge sliver, not a block
        if len(b) > 4 and b[4] == "park":
            continue
        if window is not None:
            # a block the crop clipped: judge only if it is well inside
            if not (window[0] + margin_m < (b[0] + b[2]) / 2.0 < window[2] - margin_m
                    and window[1] + margin_m < (b[1] + b[3]) / 2.0 < window[3] - margin_m):
                continue
        empty.append({"block": i, "rect": tuple(b[:4]),
                      "name": b[4] if len(b) > 4 else None,
                      "size_m": (w, h)})
    return {"n": n, "offenders": offenders, "empty_blocks": empty,
            "by_block": by_block}


def manifest_frame_check(records, dump, tol_m=CELL_MATCH_TOL_M):
    """For every record with a cell in `dump`: how far the bake holder would
    be from the intact cell under the SHIPPED rule (`rec["x"]`) and under
    `record_xy`.  Returns `{"n", "shipped": {"max_d", "n_bad"},
    "fixed": {...}, "rows": [...]}`."""
    bycell = {p["cell"]: p for p in dump.get("placements") or []}
    rows = []
    for r in records:
        p = bycell.get(r.get("cell"))
        if p is None:
            continue
        cx = float(p.get("x_m_orig", p["x_m"]))
        cy = float(p.get("y_m_orig", p["y_m"]))
        sx, sy = float(r.get("x", 0.0)), float(r.get("y", 0.0))
        fx, fy = record_xy(r)
        rows.append({"cell": r["cell"],
                     "d_shipped": math.hypot(sx - cx, sy - cy),
                     "d_fixed": math.hypot(fx - cx, fy - cy)})

    def _summ(key):
        ds = [q[key] for q in rows]
        return {"max_d": max(ds) if ds else 0.0,
                "n_bad": sum(1 for d in ds if d > tol_m)}
    return {"n": len(rows), "shipped": _summ("d_shipped"),
            "fixed": _summ("d_fixed"), "rows": rows}


def compare_blocks(host_blocks, dump_blocks, tol_m=0.05):
    """Host-built blocks vs a dump's (un-shifted) blocks: every dump block
    must match a host block to `tol_m` on all four edges.  Returns
    `(n_matched, n_dump, unmatched)`; a Kit dump's blocks are clipped to the
    crop window, so a dump block that is a clipped host block also counts."""
    unmatched = []
    n = 0
    for db in dump_blocks:
        ok = False
        for hb in host_blocks:
            if all(abs(db[k] - hb[k]) <= tol_m for k in range(4)):
                ok = True
                break
            # clipped: the dump rect must be inside the host rect and share
            # every edge the window did not cut
            if (hb[0] - tol_m <= db[0] and db[2] <= hb[2] + tol_m
                    and hb[1] - tol_m <= db[1] and db[3] <= hb[3] + tol_m
                    and sum(abs(db[k] - hb[k]) <= tol_m for k in range(4)) >= 2):
                ok = True
                break
        if ok:
            n += 1
        else:
            unmatched.append(tuple(db[:4]))
    return n, len(dump_blocks), unmatched


# ---------------------------------------------------------------------------
# the figure
# ---------------------------------------------------------------------------
def draw(out_path, blocks, corridors, intact, shipped, fixed, title="",
         region=None, window=None):
    """blocks / corridors / footprint dict lists as above.  `shipped` and
    `fixed` are the SAME records at the two coordinate rules; an arrow joins
    each pair."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(13, 13))
    if region:
        ax.add_patch(Rectangle((region[0], region[1]), region[2] - region[0],
                               region[3] - region[1], fc="#4a4a4a", ec="none"))
    for c in corridors or []:
        r = ((c["x0"], c["y0"], c["x1"], c["y1"]) if isinstance(c, dict)
             else tuple(c[:4]))
        ax.add_patch(Rectangle((r[0], r[1]), r[2] - r[0], r[3] - r[1],
                               fc="#3a3a3a", ec="none"))
    for b in blocks:
        ax.add_patch(Rectangle((b[0], b[1]), b[2] - b[0], b[3] - b[1],
                               fc="#d9d2c2", ec="#8a8378", lw=0.5))
    for fp in intact:
        r = fp["rect"]
        ax.add_patch(Rectangle((r[0], r[1]), r[2] - r[0], r[3] - r[1],
                               fc="#9a9a9a", ec="#555555", lw=0.3))
    for fp in shipped:
        r = fp["rect"]
        ax.add_patch(Rectangle((r[0], r[1]), r[2] - r[0], r[3] - r[1],
                               fc="#e0443a", ec="#7a1d17", lw=0.6, alpha=0.9))
    for fp in fixed:
        r = fp["rect"]
        ax.add_patch(Rectangle((r[0], r[1]), r[2] - r[0], r[3] - r[1],
                               fc="#3fae5a", ec="#1d6b32", lw=0.6, alpha=0.9))
    for a, b in zip(shipped, fixed):
        ax.annotate("", xy=b["centre"], xytext=a["centre"],
                    arrowprops=dict(arrowstyle="->", color="#ffd23f", lw=0.8,
                                    alpha=0.8))
    if window:
        ax.add_patch(Rectangle((window[0], window[1]), window[2] - window[0],
                               window[3] - window[1], fc="none", ec="#1e90ff",
                               lw=1.5, ls="--"))
    ax.set_aspect("equal")
    if region:
        ax.set_xlim(region[0] - 20, region[2] + 20)
        ax.set_ylim(region[1] - 20, region[3] + 20)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title, fontsize=10)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(fc="#3a3a3a", label="road corridor"),
                       Patch(fc="#d9d2c2", label="block"),
                       Patch(fc="#9a9a9a", label="intact building (GT bbox)"),
                       Patch(fc="#e0443a", label="burnt substitute, SHIPPED position"),
                       Patch(fc="#3fae5a", label="burnt substitute, corrected (record_xy)"),
                       Patch(fc="none", ec="#1e90ff", label="level's 1 km crop window")],
              loc="upper right", fontsize=8)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    return out_path


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _load(path):
    with open(path) as fh:
        return json.load(fh)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--gt", help="GT_hints.json of the built cell")
    ap.add_argument("--dump", required=True,
                    help="placement dump the manifest was solved on")
    ap.add_argument("--manifest", help="fire/tornado manifest json")
    ap.add_argument("--layout", help="host-built layout json "
                    "(blocks + road_corridors, full-city frame)")
    ap.add_argument("--png", help="write the figure here")
    ap.add_argument("--json", help="write the report here")
    ap.add_argument("--min-frac", type=float, default=MIN_FRAC_IN_BLOCK)
    args = ap.parse_args(argv)

    dump = _load(args.dump)
    blocks = unshifted_blocks(dump)

    # BUILDING-VS-BUILDING SAT overlap, over the dump's own REAL, Kit-
    # measured W/D (rotated by each record's own yaw_deg) -- independent of
    # --gt/--manifest/--layout, since the placements dump alone carries
    # everything this needs. See `building_overlap_pairs`'s own docstring
    # for why this must be yaw-rotated and what a naive axis-aligned check
    # gets wrong.
    house_recs = [p for p in (dump.get("placements") or [])
                 if p.get("W") is not None and p.get("D") is not None]
    overlap_pairs = building_overlap_pairs(house_recs)
    print("[audit] building-building footprint overlaps (SAT, real W/D, "
         "yaw-correct): {0} pair(s) of {1} building(s)"
         .format(len(overlap_pairs), len(house_recs)))
    for i, j in overlap_pairs[:10]:
        ri, rj = house_recs[i], house_recs[j]
        print("[audit]    {0} @ ({1:.1f},{2:.1f}) x {3} @ ({4:.1f},{5:.1f})"
              .format(os.path.basename(ri.get("usd", "")), ri["x_m"], ri["y_m"],
                      os.path.basename(rj.get("usd", "")), rj["x_m"], rj["y_m"]))
    report = {"dump": args.dump, "building_overlap_pairs": len(overlap_pairs)}

    # SELF-AUDIT: the dump's OWN buildings against its OWN typology blocks,
    # in the dump's OWN (possibly re-centred-by-crop) frame -- no --gt/
    # --manifest/--layout needed, so this gates a dump the moment it exists,
    # before any manifest is solved or any cell is baked. A crop's own
    # window (`dump["crop"]["window"]`, already re-centred by
    # `dump["crop"]["shift"]` the same way `x_m`/`y_m` are) is used with the
    # same 70 m margin `main()`'s --gt path applies below, so a boundary
    # sliver the crop legitimately left thin is not reported as "empty" or
    # "overhanging" -- see `tools/crop_window.py`'s drop-not-cut / clip-not-
    # drop rules for why the crop edge always leaves slivers like this.
    self_blocks = [(b["rect"][0], b["rect"][1], b["rect"][2], b["rect"][3],
                    b.get("name")) for b in (dump.get("typology") or {}).get("blocks") or []]
    self_fps = [{"name": os.path.basename(r.get("usd", "")),
                "rect": footprint_rect(r["x_m"], r["y_m"], r["W"], r["D"],
                                       r.get("yaw_deg", 0.0)),
                "centre": (r["x_m"], r["y_m"])} for r in house_recs]
    self_window = tuple(dump.get("crop", {}).get("window")) if dump.get("crop") else None
    if self_window:
        sx, sy = dump["crop"].get("shift") or (0.0, 0.0)
        self_window = (self_window[0] + sx, self_window[1] + sy,
                      self_window[2] + sx, self_window[3] + sy)
    a_self = audit_footprints(self_fps, self_blocks, args.min_frac,
                              window=self_window, margin_m=70.0 if self_window else 0.0)
    print("[audit] dump self-audit: {0} building(s), {1} block(s) -- "
         "{2} overhang(ing) a block, {3} empty (non-boundary) block(s)"
         .format(a_self["n"], len(self_blocks), len(a_self["offenders"]),
                 len(a_self["empty_blocks"])))
    for e in a_self["empty_blocks"][:10]:
        print("[audit]    empty block {0} {1} {2:.0f} x {3:.0f} m"
              .format(e["block"], e["name"], *e["size_m"]))
    report["dump_self_overhang_offenders"] = len(a_self["offenders"])
    report["dump_self_empty_blocks"] = len(a_self["empty_blocks"])

    corridors, region = [], None
    if args.layout:
        lay = _load(args.layout)
        corridors = lay.get("road_corridors") or []
        region = lay.get("region")
        hb = [tuple(b) for b in lay["blocks"]]
        n_ok, n_dump, unmatched = compare_blocks(hb, blocks)
        print("[audit] host-built blocks vs dump blocks: {0}/{1} match"
              .format(n_ok, n_dump))
        if unmatched:
            print("[audit]   unmatched (dump frame): {0}".format(unmatched[:6]))
        blocks = [tuple(b) + (None,) for b in hb]     # the full set
        # CENTRE-IN-ROAD: a stricter reading than "> 3% of footprint on a
        # road" -- the placement's own centre standing inside a corridor,
        # over the dump's real (Kit-measured) footprints in the FULL-city
        # frame (x_m_orig/y_m_orig when the dump is a crop, else x_m/y_m --
        # `record_xy`-style: this loop reads the dump's own placement dicts,
        # which carry `x_m_orig`/`y_m_orig`, not a manifest record's `x_orig`
        # /`y_orig`, so it is spelled out here rather than reusing
        # `record_xy`).
        n_centre_road = sum(
            1 for r in house_recs
            if centre_in_road(float(r.get("x_m_orig", r["x_m"])),
                              float(r.get("y_m_orig", r["y_m"])), corridors))
        print("[audit] building placement CENTRES inside a road corridor: {0}"
              .format(n_centre_road))
        report["building_centres_on_road"] = n_centre_road
    crop = dump.get("crop") or {}
    window = crop.get("window")
    margin = 70.0 if (window and not args.layout) else 0.0

    report.update({"n_blocks": len(blocks)})
    intact, shipped_fp, fixed_fp = [], [], []
    if args.gt:
        gt = _load(args.gt)
        fps = hints_footprints(gt)
        intact = [f for f in fps if not f["substitute"]]
        subs = [f for f in fps if f["substitute"]]
        a_int = audit_footprints(intact, blocks, args.min_frac,
                                 window=window if not args.layout else None,
                                 margin_m=margin)
        a_sub = audit_footprints(subs, blocks, args.min_frac,
                                 window=window if not args.layout else None,
                                 margin_m=margin)
        print("[audit] GT intact buildings: {0} judged, {1} overhang a block"
              .format(a_int["n"], len(a_int["offenders"])))
        print("[audit] GT burnt substitutes: {0} judged, {1} overhang a block"
              .format(a_sub["n"], len(a_sub["offenders"])))
        for o in a_sub["offenders"][:10]:
            print("[audit]    {0:5.2f} inside  {1}".format(o["frac_in"], o["name"]))
        if corridors:
            on_road = [(road_overlap(f["rect"], corridors), f) for f in subs]
            n_road = sum(1 for fr, _ in on_road if fr > 0.03)
            print("[audit] substitutes with > 3% of footprint ON A ROAD: {0}"
                  .format(n_road))
            report["substitutes_on_road"] = n_road
        print("[audit] empty blocks (no building centre inside): {0}"
              .format(len(a_int["empty_blocks"])))
        for e in a_int["empty_blocks"][:10]:
            print("[audit]    block {0} {1} {2:.0f} x {3:.0f} m".format(
                e["block"], e["name"], *e["size_m"]))
        report.update({"gt_intact_offenders": len(a_int["offenders"]),
                       "gt_substitute_offenders": len(a_sub["offenders"]),
                       "empty_blocks": a_int["empty_blocks"]})
        shipped_fp = subs
    if args.manifest:
        man = _load(args.manifest)
        recs = man.get("records") or []
        fc = manifest_frame_check(recs, dump)
        print("[audit] manifest frame: {0} record(s) matched to a dump cell; "
              "bake-vs-cell distance  shipped rule max {1:.1f} m ({2} > {3} m)"
              "  |  record_xy max {4:.2f} m ({5} > {3} m)".format(
                  fc["n"], fc["shipped"]["max_d"], fc["shipped"]["n_bad"],
                  CELL_MATCH_TOL_M, fc["fixed"]["max_d"], fc["fixed"]["n_bad"]))
        fixed_fp = manifest_footprints(recs, record_xy)
        a_fix = audit_footprints(fixed_fp, blocks, args.min_frac)
        a_shp = audit_footprints(manifest_footprints(
            recs, lambda r: (float(r.get("x", 0.0)), float(r.get("y", 0.0)))),
            blocks, args.min_frac)
        print("[audit] record footprints overhanging a block: shipped rule "
              "{0}/{1}, record_xy {2}/{3}".format(
                  len(a_shp["offenders"]), a_shp["n"],
                  len(a_fix["offenders"]), a_fix["n"]))
        if corridors:
            n_road_fix = sum(1 for f in fixed_fp
                             if road_overlap(f["rect"], corridors) > 0.03)
            print("[audit] record_xy footprints ON A ROAD: {0}".format(n_road_fix))
            report["fixed_on_road"] = n_road_fix
        report.update({"frame": {k: fc[k] for k in ("n", "shipped", "fixed")},
                       "record_offenders_shipped": len(a_shp["offenders"]),
                       "record_offenders_fixed": len(a_fix["offenders"])})
        if not shipped_fp:
            shipped_fp = manifest_footprints(
                recs, lambda r: (float(r.get("x", 0.0)), float(r.get("y", 0.0))))
    if args.png:
        # pair shipped/fixed by cell for the arrows
        by_cell = {f["prim_path"]: f for f in fixed_fp}
        pairs_s, pairs_f = [], []
        if args.gt and args.manifest:
            # GT substitutes carry the holder name, not the cell; pair by
            # nearest shipped-rule record position instead
            man_shipped = manifest_footprints(
                _load(args.manifest)["records"],
                lambda r: (float(r.get("x", 0.0)), float(r.get("y", 0.0))))
            for ms, mf in zip(man_shipped, fixed_fp):
                pairs_s.append(ms)
                pairs_f.append(mf)
        else:
            pairs_s, pairs_f = shipped_fp, fixed_fp
        title = ("{0}\nred = burnt substitute where the shipped cell put it, "
                 "green = on its own intact cell (record_xy); arrows join the "
                 "two".format(os.path.basename(args.dump)))
        draw(args.png, blocks, corridors, intact, pairs_s, pairs_f, title,
             region=region or (dump.get("region_m") and
                               [-dump["region_m"][0] / 2.0, -dump["region_m"][1] / 2.0,
                                dump["region_m"][0] / 2.0, dump["region_m"][1] / 2.0]),
             window=window)
        print("[audit] figure -> {0}".format(args.png))
        report["png"] = args.png
    if args.json:
        with open(args.json, "w") as fh:
            json.dump(report, fh, indent=1)
    return report


if __name__ == "__main__":
    main()
