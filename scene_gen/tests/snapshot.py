"""
snapshot.py — canonical, diffable signatures of a generated scene's layout.

WHY THIS EXISTS
---------------
The generator is being refactored into three stages (layout -> detail ->
damage) and the first requirement of that refactor is *preserve behavior*.
There were no tests, so there was nothing to preserve behavior against. This
turns a generation run into a stable text signature that can be committed and
diffed, which is what makes "did that refactor change the scene?" answerable
without launching Isaac Sim.

Runs the pure-Python layout only — no stage, no Nucleus, ~0.1 s per config —
the same substrate `preset_report.py` uses. `preset_report` answers "does this
disaster look right?" with aggregate counts; this answers "did anything move?"
with exact positions. They are complements, not duplicates.

THE THREE SIGNATURES, AND WHY THERE ARE THREE
---------------------------------------------
* `geometry_signature`  — the street plan: region, blocks, road corridors.
  Pure layout, no assets. The strongest statement of "same city".
* `layout_signature`    — where every non-damage prop stands: category and
  pose, damage categories excluded and `usd` deliberately omitted (a damaged
  building is a different asset at the same spot, which is damage, not layout).
* `full_signature`      — everything, including `usd`, roll/pitch and debris,
  in emission order. The regression baseline; catches anything at all moving.

`layout` and `geometry` are sorted, so interleaving new damage placements
cannot produce a false diff. `full` keeps emission order, because prim naming
in `apply_placements` depends on it and a reordering is a real change.

THE ASSERTION THIS EXISTS TO SUPPORT
-------------------------------------
`geometry_signature` and `layout_signature` must be **identical across
severity** for a fixed locale and seed. That is requirement 4a of the refactor:
a locale and a seed fully specify the layout, and damage only adds to it.

It does not hold today, and the test that checks it is expected to fail until
Phase 2 lands. `scene_generator._anchor_ok` gates which block a large building
may anchor into by local damage intensity, so raising severity currently moves
buildings. That failure is the point: it is the regression test for the fix.

USAGE
-----
    python3 tests/snapshot.py --write            # (re)baseline every case
    python3 tests/snapshot.py                    # compare against baselines
    python3 tests/snapshot.py --case earthquake@0.6
    python3 -m pytest tests/ -v                  # the assertions
"""

import argparse
import contextlib
import hashlib
import io
import json
import os
import sys

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_TESTS_DIR)
sys.path.insert(0, _SCENE_GEN_DIR)

SNAPSHOT_DIR = os.path.join(_TESTS_DIR, "snapshots")
PRESET_DIR = os.path.join(_SCENE_GEN_DIR, "config", "presets")

# Categories that only exist because of damage. Excluded from the layout
# signature: their absence at severity 0 is correct, not a layout difference.
DAMAGE_CATEGORIES = {"debris", "debris_pile"}

# Footprint used when not measuring real USD bboxes, so blocks actually pack
# offline. Matches preset_report.py's default for comparability.
HOUSE_FALLBACK = [15.0, 15.0]

# (preset, seed, severity) — the severity sweep at a fixed seed is what proves
# decoupling; the second seed guards against a coincidence at seed 42.
DEFAULT_CASES = [
    ("earthquake", 42, 0.0),
    ("earthquake", 42, 0.4),
    ("earthquake", 42, 0.8),
    ("tornado", 42, 0.0),
    ("tornado", 42, 0.8),
    ("explosion", 42, 0.6),
    ("flood", 42, 0.6),
    ("hurricane", 42, 0.6),
    ("suburban", 7, 0.5),
]


def _round(v, nd=3):
    """Round and normalise -0.0 to 0.0, which json and repr disagree about."""
    r = round(float(v), nd)
    return 0.0 if r == 0.0 else r


def build(preset: str, seed: int, severity: float):
    """Compile *preset* at *severity*/*seed* and run the layout.

    Mirrors `compile_disaster.load_scene_config` exactly (compile_spec ->
    resolve_asset_set -> validate_config) rather than calling it, because the
    severity override has to land on the spec *before* compilation and
    load_scene_config takes a path, not a dict. The preset path stays the
    anchor so asset-set resolution behaves identically.
    """
    import yaml

    import compile_disaster as cd
    import scene_generator as sg

    preset_path = cd.resolve_config_path(preset)
    with open(preset_path) as fh:
        spec = yaml.safe_load(fh)
    if not cd.is_high_level(spec):
        raise ValueError(
            f"{preset!r} is a low-level config; snapshots sweep severity, "
            "which only a high-level spec has")

    spec["severity"] = float(severity)
    spec["seed"] = int(seed)

    with open(cd.DEFAULT_BASE) as fh:
        base = yaml.safe_load(fh)

    # The generator narrates to stdout; a snapshot run does its own reporting.
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        cfg = cd.compile_spec(spec, base)
        cfg = sg.resolve_asset_set(cfg, preset_path)
        cfg = sg.validate_config(cfg, preset_path)

        # Offline footprints: no Nucleus, deterministic, and fast.
        cfg["measure_usds"] = False
        cfg.setdefault("fallback_sizes", {})["house"] = list(HOUSE_FALLBACK)

        resolver = sg._make_resolver(cfg)
        placements, layout = sg.build_city(cfg, resolver)
    return placements, layout


def geometry_signature(layout: dict) -> dict:
    """The street plan: region, blocks, road corridors. Assets play no part.

    `blocks` are bare ``(x0, y0, x1, y1)`` tuples; `road_corridors` are dicts
    carrying ``n_lanes`` and ``dir`` alongside their rect — both are layout
    facts, so both are in the signature.
    """
    blocks = sorted([_round(v) for v in r] for r in (layout.get("blocks") or []))
    corridors = sorted(
        [_round(c["x0"]), _round(c["y0"]), _round(c["x1"]), _round(c["y1"]),
         int(c.get("n_lanes", 0)), str(c.get("dir", ""))]
        for c in (layout.get("road_corridors") or [])
    )
    return {
        "region": [_round(v) for v in layout["region"]],
        "blocks": blocks,
        "road_corridors": corridors,
    }


def layout_signature(placements: list) -> list:
    """Pose of every non-damage placement, sorted.

    `usd` is omitted on purpose: swapping a building for its damaged variant at
    the same spot is damage, not layout, and must not fail the decoupling
    assertion. roll/pitch are omitted for the same reason — toppling a
    streetlight is damage; where the streetlight stands is layout.
    """
    return sorted(
        (p["category"], _round(p["x_m"]), _round(p["y_m"]), _round(p["yaw_deg"], 1))
        for p in placements
        if p["category"] not in DAMAGE_CATEGORIES
    )


def full_signature(placements: list) -> list:
    """Everything, in emission order — the regression baseline.

    Order is preserved because `apply_placements` derives prim paths from the
    placement index, so a reordering renames prims even when nothing moved.
    """
    return [
        [
            p["category"], p["usd"],
            _round(p["x_m"]), _round(p["y_m"]), _round(p["z_m"]),
            _round(p["yaw_deg"], 1), _round(p.get("roll_deg", 0.0), 1),
            _round(p.get("pitch_deg", 0.0), 1),
        ]
        for p in placements
    ]


def _digest(obj) -> str:
    return hashlib.sha256(
        json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()[:16]


def snapshot(preset: str, seed: int, severity: float, detail: bool = False) -> dict:
    """The record for one case: counts, digests, and the street plan.

    The `layout` and `full` signatures are **digested, not stored**. Verbatim
    they run to ~1 MB per case (10k placements x 8 fields), which is both too
    heavy to commit and unreadable as a diff — and unnecessary, because every
    case is reproducible from `(preset, seed, severity)` alone. When a digest
    mismatch needs explaining, `explain()` rebuilds both sides and says what
    moved. Pass *detail* to keep the full arrays in memory for that.

    The street plan *is* stored: it is small (tens of rects) and it is the one
    thing worth eyeballing in a review.
    """
    placements, layout = build(preset, seed, severity)

    counts: dict = {}
    for p in placements:
        counts[p["category"]] = counts.get(p["category"], 0) + 1

    geom = geometry_signature(layout)
    lay = layout_signature(placements)
    full = full_signature(placements)

    snap = {
        "case": case_name(preset, seed, severity),
        "preset": preset,
        "seed": seed,
        "severity": severity,
        "n_placements": len(placements),
        "counts": dict(sorted(counts.items())),
        "digests": {
            "geometry": _digest(geom),
            "layout": _digest(lay),
            "full": _digest(full),
        },
        "geometry": geom,
    }
    if detail:
        snap["_layout"] = lay
        snap["_full"] = full
    return snap


def explain(preset: str, seed: int, sev_a: float, sev_b: float) -> str:
    """Human-readable diff of two cases. Both sides are rebuilt, not loaded."""
    a = snapshot(preset, seed, sev_a, detail=True)
    b = snapshot(preset, seed, sev_b, detail=True)
    la, lb = set(map(tuple, a["_layout"])), set(map(tuple, b["_layout"]))
    moved_from, moved_to = la - lb, lb - la

    by_cat: dict = {}
    for cat, *_ in moved_from:
        by_cat[cat] = by_cat.get(cat, 0) + 1

    lines = [
        f"{preset} seed {seed}:  severity {sev_a:g} -> {sev_b:g}",
        f"  street plan : {'SAME' if a['digests']['geometry'] == b['digests']['geometry'] else 'CHANGED'}",
        f"  layout      : {'SAME' if a['digests']['layout'] == b['digests']['layout'] else 'CHANGED'}",
        f"  non-damage placements: {len(la)} -> {len(lb)}",
        f"  unmoved: {len(la & lb)}   moved/removed: {len(moved_from)}   "
        f"new/relocated: {len(moved_to)}",
    ]
    if by_cat:
        top = sorted(by_cat.items(), key=lambda kv: -kv[1])[:8]
        lines.append("  most-affected categories: "
                     + ", ".join(f"{c}={n}" for c, n in top))
    return "\n".join(lines)


def case_name(preset: str, seed: int, severity: float) -> str:
    return f"{preset}@s{seed}@{severity:g}"


def snapshot_path(preset: str, seed: int, severity: float) -> str:
    return os.path.join(SNAPSHOT_DIR, case_name(preset, seed, severity) + ".json")


def write(preset: str, seed: int, severity: float) -> str:
    os.makedirs(SNAPSHOT_DIR, exist_ok=True)
    snap = snapshot(preset, seed, severity)
    path = snapshot_path(preset, seed, severity)
    with open(path, "w") as fh:
        json.dump(snap, fh, indent=1, sort_keys=False)
        fh.write("\n")
    return path


def load(preset: str, seed: int, severity: float) -> dict:
    with open(snapshot_path(preset, seed, severity)) as fh:
        return json.load(fh)


def _parse_case(text: str):
    """'earthquake', 'earthquake@0.6' or 'earthquake@s7@0.6' -> a case tuple."""
    parts = text.split("@")
    preset = parts[0]
    seed, severity = 42, 0.6
    for p in parts[1:]:
        if p.startswith("s"):
            seed = int(p[1:])
        else:
            severity = float(p)
    return preset, seed, severity


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--write", action="store_true",
                    help="(re)write baselines instead of comparing")
    ap.add_argument("--case", action="append", default=None,
                    help="preset[@sSEED][@SEVERITY]; repeatable")
    ap.add_argument("--explain", nargs=3, metavar=("PRESET", "SEV_A", "SEV_B"),
                    help="rebuild two severities and report what moved")
    ap.add_argument("--seed", type=int, default=42,
                    help="seed for --explain (default 42)")
    args = ap.parse_args()

    if args.explain:
        preset, sa, sb = args.explain
        print(explain(preset, args.seed, float(sa), float(sb)))
        return 0

    cases = ([_parse_case(c) for c in args.case] if args.case
             else list(DEFAULT_CASES))

    if args.write:
        for preset, seed, sev in cases:
            path = write(preset, seed, sev)
            print(f"  wrote {os.path.relpath(path, _SCENE_GEN_DIR)}")
        return 0

    failed = 0
    for preset, seed, sev in cases:
        name = case_name(preset, seed, sev)
        try:
            want = load(preset, seed, sev)
        except FileNotFoundError:
            print(f"  {name:28s} NO BASELINE (run --write)")
            failed += 1
            continue
        got = snapshot(preset, seed, sev)
        diffs = [k for k in ("geometry", "layout", "full")
                 if got["digests"][k] != want["digests"][k]]
        if diffs:
            print(f"  {name:28s} CHANGED: {', '.join(diffs)}")
            failed += 1
        else:
            print(f"  {name:28s} ok  ({got['n_placements']} placements)")

    if failed:
        print(f"\n{failed} case(s) differ from baseline. If intended, "
              f"re-run with --write and commit the diff.")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
