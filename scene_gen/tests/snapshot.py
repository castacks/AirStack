"""
snapshot.py — canonical, diffable signatures of a generated scene's layout.

WHY THIS EXISTS
---------------
The generator is organised in three stages (layout -> detail -> disaster) and the first requirement of that refactor is *preserve behavior*.
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

WHAT THE INVARIANT ACTUALLY IS
------------------------------
*Structure* — buildings, roads, ground tiles — must be positionally identical
at every severity. That is a guarantee: `STRUCTURE_CATEGORIES`.

*Details* must be **generated** at the same positions; the disaster stage may
then topple, tilt, swap or displace them. Post hoc those two are hard to tell
apart, so `run_layout` deliberately stops *before* the disaster pass — which
makes the assertion exact by construction rather than by inference.

`layout_signature` is a strict regression tripwire, not the invariant: it moves
on any legitimate disaster effect too. Use `positions_by_category`.

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
import random
import sys

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_TESTS_DIR)
sys.path.insert(0, _SCENE_GEN_DIR)

SNAPSHOT_DIR = os.path.join(_TESTS_DIR, "snapshots")
PRESET_DIR = os.path.join(_SCENE_GEN_DIR, "config", "presets")

# Categories that only exist because of damage. Excluded from the layout
# signature: their absence at severity 0 is correct, not a layout difference.
DAMAGE_CATEGORIES = {"debris", "debris_pile"}

# Structural categories — the city plan. These must be positionally identical
# at every severity: a locale and a seed fully specify where the buildings and
# the ground surfaces are.
STRUCTURE_CATEGORIES = {"house", "sidewalk", "concrete"}

# Categories the disaster stage is allowed to MOVE out of the position the
# detail stage chose.
#
# READ THIS BEFORE TREATING THE EMPTY SET AS A GUARANTEE. The *guarantee* is
# `STRUCTURE_CATEGORIES` — buildings, roads and ground tiles do not move, ever.
# Details are a different matter: a disaster is entitled to blow a bin down the
# street, and `disaster.trash_cans_scatter_m` is 14 m for a tornado.
#
# The set is empty today for two reasons, only one of them good:
#   1. the RNG leaks are fixed, so nothing is *relaid out* by severity — good,
#      and this test is what keeps it that way;
#   2. almost nothing is actually displaced yet. Scatter exists only in the
#      built-in frontage pass, which the downtown locale switches off in favour
#      of `city_detail` — and `city_detail` props get no disaster effects at
#      all. So downtown street furniture is currently immune to the event.
#
# Fixing (2) will move categories into this set, and that is correct, not a
# regression. What must never happen is a category moving because a
# disaster-only draw was taken from the layout stream, or because two branches
# consumed different numbers of layout draws — that is relayout, and it is what
# this test exists to catch.
DAMAGE_MAY_RELOCATE: set = set()

# Footprint used when not measuring real USD bboxes, so blocks actually pack
# offline. Matches preset_report.py's default for comparability.
HOUSE_FALLBACK = [15.0, 15.0]

# (preset, seed, severity) — the severity sweep at a fixed seed is what proves
# decoupling; the second seed guards against a coincidence at seed 42.
DEFAULT_CASES = [
    # Pristine baselines, both locales and a fast one.
    ("downtown", 42, 0.0),
    ("downtown_small", 42, 0.0),
    ("suburb", 42, 0.0),
    # Every disaster type — the fields differ in shape (radial, path, uniform)
    # and each drives a different mix of effects, so a leak can hide in one.
    ("earthquake", 42, 0.0),
    ("earthquake", 42, 0.4),
    ("earthquake", 42, 0.8),
    ("tornado", 42, 0.0),
    ("tornado", 42, 0.8),
    ("explosion", 42, 0.6),
    ("fire", 42, 0.6),
    ("flood", 42, 0.6),
    ("hurricane", 42, 0.6),
    # The other locale, damaged.
    ("suburb_earthquake", 7, 0.5),
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
        placements, layout = run_layout(cfg, resolver)
    return placements, layout


def run_layout(cfg: dict, resolver):
    """Layout + detail — the PRISTINE scene, stopping before the disaster.

    Delegates to `generate_scene.build_scene`, so the harness exercises the
    same pipeline the sim does rather than a reimplementation of it that can
    drift. Stopping at `detail` is what makes the position-preservation
    assertions exact by construction: at this point severity has had no
    opportunity to affect anything.
    """
    import generate_scene

    placements, layout, _ = generate_scene.build_scene(
        cfg, resolver, stop_after="detail")
    return placements, layout


def run_full(cfg: dict, resolver):
    """All three stages — what an actual scene looks like."""
    import generate_scene

    placements, layout, _ = generate_scene.build_scene(cfg, resolver)
    return placements, layout


def sg_module():
    import scene_generator
    return scene_generator



def build_for_disaster(preset: str, seed: int, severity: float):
    """(config, layout, placements) with the disaster stage NOT yet applied.

    `run_layout` deliberately stops before the disaster pass — that is what
    makes the invariance assertions exact — so a test that wants to exercise
    the pass needs the config and layout handed back with it.
    """
    import yaml

    import compile_disaster as cd
    import scene_generator as sg

    preset_path = cd.resolve_config_path(preset)
    with open(preset_path) as fh:
        spec = yaml.safe_load(fh)
    spec["severity"] = float(severity)
    spec["seed"] = int(seed)
    with open(cd.DEFAULT_BASE) as fh:
        base = yaml.safe_load(fh)

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        cfg = cd.compile_spec(spec, base)
        cfg = sg.resolve_asset_set(cfg, preset_path)
        cfg = sg.validate_config(cfg, preset_path)
        cfg["measure_usds"] = False
        cfg.setdefault("fallback_sizes", {})["house"] = list(HOUSE_FALLBACK)
        placements, layout = run_layout(cfg, sg._make_resolver(cfg))
    return cfg, layout, placements



def build_full_positions(preset: str, seed: int, severity: float) -> dict:
    """`positions_by_category` for a scene built through ALL three stages."""
    cfg, layout, placements = build_for_disaster(preset, seed, severity)
    from disaster import disaster_stage
    disaster_stage.apply_to_buildings(cfg, layout, placements, _resolver_for(cfg))
    disaster_stage.apply(cfg, layout, placements)
    return positions_by_category(placements)


def _resolver_for(cfg):
    import scene_generator as sg
    return sg._make_resolver(cfg)


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
    """Pose of every non-damage placement, sorted. A *regression* signature.

    NOT the decoupling criterion. This is deliberately strict — it moves if
    anything at all changes — which makes it a good tripwire for "did this
    refactor alter the scene?" and a bad statement of "is damage additive?".
    Damage legitimately adds placements, displaces props and re-yaws whatever
    it topples, all of which move this digest. Use `positions_by_category`
    and the tests built on it for the actual invariant.
    """
    return sorted(
        (p["category"], _round(p["x_m"]), _round(p["y_m"]), _round(p["yaw_deg"], 1))
        for p in placements
        if p["category"] not in DAMAGE_CATEGORIES
    )


def positions_by_category(placements: list) -> dict:
    """``{category: {(x, y), ...}}`` — where things were put, ignoring pose.

    The decoupling criterion is about *placement*, not orientation: a felled
    tree is still the same tree in the same spot, and a swapped-in ruin stands
    on the same slab. Dropping yaw/roll/pitch is what separates "damage acted
    on this prop" from "the detail stage put this prop somewhere else".
    """
    out: dict = {}
    for p in placements:
        if p["category"] in DAMAGE_CATEGORIES:
            continue
        out.setdefault(p["category"], set()).add(
            (_round(p["x_m"]), _round(p["y_m"])))
    return out


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
        snap["_positions"] = positions_by_category(placements)
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
