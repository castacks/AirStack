#!/usr/bin/env python3
"""spawn_from_real_people.py — re-validate the two RAVEN test casualties
against the REAL frozen `GT_people.json`, and regenerate SPAWN_CONFIGS.

Called by `scripts/raven_live/validate_freeze.sh`; runnable on its own.

    python3 scripts/raven_live/spawn_from_real_people.py \
        --people _test_freeze/raven_suburb_tornado_250/GT_people.json \
        --out-dir scripts/raven_live/out

WHY. `_plans/raven_test_scene_runbook.md` §1b's spawn pair is a HOST
APPROXIMATION. The host dry run cannot measure `deck_points` (they come off
the BAKED WRECK ARCHETYPES with `Usd.TraverseInstanceProxies` + `BBoxCache`,
which needs a live stage), so it plans on the flat per-level `DEBRIS_Z_M`
deck. The real build's deck is lumpy, which changes a handful of `_Deck`-tilt
accept/refuse decisions near a wreck pile — so the real population may not
hold EXACTLY 19 casualties at EXACTLY those coordinates. §2 step 5 of that
runbook says to re-check before flying. This is that step, automated.

WHAT IS AND IS NOT AFFECTED BY THAT GAP. `deck_points` only changes which
CANDIDATE CASUALTIES are accepted. It does not touch the layout: the houses,
the street graph, the track and the intensity field are the same pure
functions off the same seed. So the spawn search runs against
`tornado_people_dry_run.build_ctx(...)`'s geometry (identical to the real
build's) while the CASUALTIES come from the real file. That is the only
combination that is both correct and reproducible offline.

THE SUBSTITUTION RULE (runbook §2 step 5).
  1. Look for a casualty of the right kind within `--tol-m` of the runbook's
     reference point. Found -> EXACT, no substitution.
  2. Otherwise take the NEAREST casualty of the right kind and say so loudly.
  3. If no casualty of that kind exists anywhere, fall back to
     `tornado_people_dry_run.pick_casualties`' own auto-pick (nearest the
     plate centre) and say so even more loudly.
  4. The two picks must be at least `--min-sep-m` apart, or the two spawn
     rings overlap and the two-drone, two-sector test degenerates into one
     sector. When they are not, the partial pick is re-chosen as the
     FARTHEST qualifying casualty from the exposed one, and that is flagged.

OUTPUT (into `--out-dir`)
    spawns_1robot.json   {"SPAWN_CONFIGS": "<json string>", ...}
    spawns_2robot.json   same, two entries
    spawn_summary.txt    the human-readable version
Exit 0 clean, 2 when it had to substitute (still usable — READ THE FLAGS),
1 when it could not produce a pair at all.
"""

import argparse
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "scene_gen", "tools"))
sys.path.insert(0, os.path.join(_REPO, "scene_gen"))

# The runbook's two reference casualties (host dry run, seed 10).
REF_EXPOSED = (-4.87, -24.86)
REF_PARTIAL = (-16.46, -64.78)


def _d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def load_people(path):
    with open(path, encoding="utf-8") as fh:
        doc = json.load(fh)
    if isinstance(doc, dict):
        people = doc.get("people") or doc.get("records") or []
        meta = doc.get("meta") or {}
    else:
        people, meta = doc, {}
    if not people:
        raise SystemExit("spawn_from_real_people: {0} holds no people records"
                         .format(path))
    return people, meta


def choose(people, ref, want, tol_m, flags, label, partial_lo, partial_hi,
           exclude_idx=None, farthest_from=None):
    """Pick one casualty index. `want` is 'exposed' or 'partial'."""
    if want == "exposed":
        def ok(p):
            return str(p.get("occlusion", "")) == "none"
        crit = "occlusion == 'none'"
    else:
        def ok(p):
            cf = float(p.get("covered_frac", -1.0))
            return partial_lo <= cf <= partial_hi
        crit = "covered_frac in [{0}, {1}]".format(partial_lo, partial_hi)

    cands = [i for i, p in enumerate(people)
             if ok(p) and i != exclude_idx]
    if not cands:
        flags.append(
            "!! NO casualty in the REAL file satisfies {0} ({1}). The scene's "
            "casualty draw moved far enough that this arm of the test has no "
            "target of the intended kind. DO NOT fly the runbook numbers."
            .format(crit, label))
        return None, None

    near = min(cands, key=lambda i: _d((people[i]["x"], people[i]["y"]), ref))
    dist = _d((people[near]["x"], people[near]["y"]), ref)
    if dist <= tol_m:
        return near, dist

    # Substituted.
    if farthest_from is not None:
        pick = max(cands, key=lambda i: _d((people[i]["x"], people[i]["y"]),
                                           farthest_from))
    else:
        pick = near
    p = people[pick]
    flags.append(
        "!! SUBSTITUTED {0}: no casualty satisfying {1} within {2:.1f} m of "
        "the runbook's ({3:.2f}, {4:.2f}). Nearest qualifying is idx {5} at "
        "({6:.2f}, {7:.2f}), {8:.1f} m away — using idx {9} at ({10:.2f}, "
        "{11:.2f}). The .env / mission SPAWN_CONFIGS in the runbook and in "
        "osmo/missions/raven_single_shared_test.yaml are now WRONG; use the "
        "ones this tool writes."
        .format(label, crit, tol_m, ref[0], ref[1], near,
                people[near]["x"], people[near]["y"], dist, pick,
                p["x"], p["y"]))
    return pick, dist


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--people", required=True,
                    help="the REAL GT_people.json / humans_<seed>.json")
    ap.add_argument("--config", default="suburb_tornado_250")
    ap.add_argument("--seed", type=int, default=None,
                    help="layout seed override; default = the preset's own "
                         "(10 for suburb_tornado_250, which is what TOR_SEED "
                         "must have been)")
    ap.add_argument("--out-dir", default=os.path.join(_HERE, "out"))
    ap.add_argument("--spawn-dist-m", type=float, default=20.0)
    ap.add_argument("--tol-m", type=float, default=3.0,
                    help="how far a real casualty may sit from the runbook's "
                         "reference and still count as 'the same one'")
    ap.add_argument("--min-sep-m", type=float, default=30.0,
                    help="minimum distance between the two picked casualties; "
                         "below this the two spawn rings nearly touch and the "
                         "two-sector test degenerates")
    ap.add_argument("--partial-lo", type=float, default=0.30)
    ap.add_argument("--partial-hi", type=float, default=0.55)
    args = ap.parse_args(argv)

    import tornado_people_dry_run as dr

    people, meta = load_people(args.people)
    flags = []      # real substitutions — these change what gets flown
    warnings = []   # worth reading, but the pair is still the intended one

    print("[spawns] people file : {0}".format(args.people))
    print("[spawns] casualties  : {0}".format(len(people)))
    if meta:
        print("[spawns] meta        : seed={0} scene_config={1} epoch_min={2}"
              .format(meta.get("seed"), meta.get("scene_config"),
                      meta.get("epoch_min")))
        if meta.get("note"):
            warnings.append(
                "!! This people file carries a `note` — it is a HOST DRY RUN "
                "file, not the Isaac build's ground truth: {0!r}"
                .format(str(meta["note"])[:120]))
        want_seed = args.seed if args.seed is not None else 10
        if meta.get("seed") not in (None, want_seed):
            warnings.append(
                "!! seed mismatch: the people file says seed={0} but the "
                "spawn geometry is being built at seed={1}. TOR_SEED and the "
                "preset's layout seed must agree."
                .format(meta.get("seed"), want_seed))

    by_occ = {}
    for p in people:
        by_occ[p.get("occlusion", "?")] = by_occ.get(p.get("occlusion", "?"), 0) + 1
    print("[spawns] by occlusion: {0}".format(
        ", ".join("%s=%d" % kv for kv in sorted(by_occ.items()))))

    # --- pick the two ------------------------------------------------------
    idx_a, d_a = choose(people, REF_EXPOSED, "exposed", args.tol_m, flags,
                        "robot_1 (fully exposed)", args.partial_lo,
                        args.partial_hi)
    idx_b, d_b = choose(people, REF_PARTIAL, "partial", args.tol_m, flags,
                        "robot_2 (partially covered)", args.partial_lo,
                        args.partial_hi, exclude_idx=idx_a)

    if idx_a is None and idx_b is None:
        print("\n".join(flags + warnings))
        raise SystemExit("spawn_from_real_people: neither casualty kind "
                         "exists in the real file — nothing to fly")
    if idx_a is None or idx_b is None:
        # One arm survived. Auto-pick the other the way the dry-run tool would.
        try:
            auto_a, auto_b = dr.pick_casualties(
                people, region_center=(0.0, 0.0),
                partial_lo=args.partial_lo, partial_hi=args.partial_hi)
            idx_a = auto_a if idx_a is None else idx_a
            idx_b = auto_b if idx_b is None else idx_b
            flags.append("!! one arm fell back to pick_casualties' auto-pick "
                         "(nearest the plate centre).")
        except ValueError as exc:
            print("\n".join(flags + warnings))
            raise SystemExit("spawn_from_real_people: auto-pick failed too: "
                             "{0}".format(exc))

    sep = _d((people[idx_a]["x"], people[idx_a]["y"]),
             (people[idx_b]["x"], people[idx_b]["y"]))
    if sep < args.min_sep_m:
        flags.append(
            "!! the two picks are only {0:.1f} m apart (< {1:.0f} m): the two "
            "20 m spawn rings nearly touch and the two-drone test becomes a "
            "one-sector test. Re-picking robot_2 as the FARTHEST qualifying "
            "casualty from robot_1's."
            .format(sep, args.min_sep_m))
        alt, _ = choose(people, REF_PARTIAL, "partial", -1.0, [],
                        "robot_2 retry", args.partial_lo, args.partial_hi,
                        exclude_idx=idx_a,
                        farthest_from=(people[idx_a]["x"], people[idx_a]["y"]))
        if alt is not None:
            idx_b = alt
            sep = _d((people[idx_a]["x"], people[idx_a]["y"]),
                     (people[idx_b]["x"], people[idx_b]["y"]))
            if sep < args.min_sep_m:
                flags.append(
                    "!! even the farthest qualifying partial is only {0:.1f} m "
                    "away. Accepting it — but the two sectors overlap."
                    .format(sep))

    # --- the geometry, from the SAME deterministic layout ------------------
    print("[spawns] building the host layout for the spawn search "
          "(config={0}, seed={1}) — this takes ~30-60 s"
          .format(args.config, args.seed if args.seed is not None else "<preset>"))
    built = dr.build_ctx(args.config, seed_override=args.seed)
    houses = built["houses"]
    region = built["region"]
    inten = built["inten"]
    to_track, _u, _v = built["tn"].frame(built["tcfg"])

    def in_any_house(x, y, margin=1.5):
        for (hx, hy, fp) in houses:
            r = 0.5 * fp + margin
            if abs(x - hx) <= r and abs(y - hy) <= r:
                return True
        return False

    def cross_offset(x, y):
        _a, c = to_track(x, y)
        return c

    entries, lines = [], []
    for tag, idx in (("robot_1", idx_a), ("robot_2", idx_b)):
        p = people[idx]
        sp = dr.pick_spawn_point(float(p["x"]), float(p["y"]),
                                 args.spawn_dist_m, region, in_any_house,
                                 cross_offset, inten)
        if sp is None:
            raise SystemExit(
                "spawn_from_real_people: no clear point {0} m from casualty "
                "idx {1} at ({2:.2f}, {3:.2f}) — every angle on the ring is "
                "inside a house footprint or off the plate. Re-run with a "
                "different --spawn-dist-m."
                .format(args.spawn_dist_m, idx, p["x"], p["y"]))
        yaw, _q = dr.yaw_facing(sp["x"], sp["y"], float(p["x"]), float(p["y"]))
        entries.append(dr.spawn_config_entry(sp["x"], sp["y"], yaw))
        lines.append(
            "{0}: casualty idx={1} ({2:.2f},{3:.2f}) occlusion={4} "
            "covered_frac={5:.2f} pose={6} -> spawn ({7:.2f},{8:.2f}) "
            "intensity={9:.2f} yaw={10:.1f} deg"
            .format(tag, idx, p["x"], p["y"], p.get("occlusion"),
                    float(p.get("covered_frac", 0.0)), p.get("pose", "?"),
                    sp["x"], sp["y"], sp["intensity"], yaw))

    os.makedirs(args.out_dir, exist_ok=True)
    payloads = {
        "spawns_1robot.json": {
            "NUM_ROBOTS": 1,
            "SPAWN_CONFIGS": json.dumps(entries[:1]),
            "casualty_indices": [idx_a],
            "casualties": [people[idx_a]],
        },
        "spawns_2robot.json": {
            "NUM_ROBOTS": 2,
            "SPAWN_CONFIGS": json.dumps(entries),
            "casualty_indices": [idx_a, idx_b],
            "casualties": [people[idx_a], people[idx_b]],
        },
    }
    for name, doc in payloads.items():
        doc.update(people_file=os.path.abspath(args.people),
                   config=args.config, spawn_dist_m=args.spawn_dist_m,
                   separation_m=round(sep, 2),
                   substituted=bool(flags), flags=flags,
                   warnings=warnings)
        with open(os.path.join(args.out_dir, name), "w", encoding="utf-8") as fh:
            json.dump(doc, fh, indent=1)
        print("[spawns] -> {0}".format(os.path.join(args.out_dir, name)))

    summary = [
        "RAVEN suburb_tornado_250 — spawn re-validation against the REAL "
        "casualty file",
        "  people file    {0}".format(os.path.abspath(args.people)),
        "  casualties     {0}".format(len(people)),
        "  separation     {0:.1f} m between the two picks".format(sep),
        "",
    ] + ["  " + ln for ln in lines] + [
        "",
        "  SPAWN_CONFIGS (1 robot) = {0}".format(payloads["spawns_1robot.json"]["SPAWN_CONFIGS"]),
        "  SPAWN_CONFIGS (2 robot) = {0}".format(payloads["spawns_2robot.json"]["SPAWN_CONFIGS"]),
        "",
    ]
    if flags:
        summary += ["  SUBSTITUTION FLAGS:"] + ["    " + f for f in flags]
    else:
        summary += ["  clean — both casualties are where the runbook says, "
                    "within {0:.1f} m; no substitution".format(args.tol_m)]
    if warnings:
        summary += ["", "  warnings:"] + ["    " + w for w in warnings]
    text = "\n".join(summary) + "\n"
    with open(os.path.join(args.out_dir, "spawn_summary.txt"), "w",
              encoding="utf-8") as fh:
        fh.write(text)
    print()
    print(text)

    if flags:
        print("=" * 72)
        print("SUBSTITUTION HAPPENED — read the FLAGS above before flying.")
        print("=" * 72)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
