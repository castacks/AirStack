#!/usr/bin/env python3
"""fire_city_lower_origins.py — patch a fire-city manifest's TOP-HEAVY
records down into a low-biased origin, with minimal churn.

    python3 scene_gen/tools/fire_city_lower_origins.py \
        scene_gen/_plans/fire_city_500m_39.json \
        --out /path/to/fire_city_500m_39_lowered.json

HOST-SIDE, no Isaac, no pxr, no Nucleus. Reads a fire-city manifest (the
schema `urban_fire_spread.solve` + `urban_fire_city.damaged_manifest`
write — see `_plans/fire_city_500m_39.json`) and emits a CANDIDATE manifest
— it never overwrites `_plans/` — that rewrites ONLY `origin`/`origin_frac`
on the TOP-HEAVY records, byte-identical everywhere else.

WHY THIS EXISTS ("fire still seems to be on the higher floors rather than
all over", user review, 2026-08-31). `origin_frac = origin / n_storeys` for
every record in `fire_city_500m_39.json`: 11 of 39 are TOP-HEAVY
(`origin_frac > 0.5`), and every single one traces back to
`urban_fire_spread.solve`'s `how == "spot"` mechanism (a brand landing on
the roof, raw `origin_frac = 0.88`) landing on a tall GAC tower. Because
`urban_fire.plan_fire`'s F3/F4/F5(c) bands are windowed by, or run to the
top from, the origin, an origin at 0.85+ on a 20-30 storey mass leaves the
ENTIRE lower building untouched and pins every flame/soot/smoke event to a
handful of storeys under the roof — which is exactly what reads as "fire on
the higher floors" from the one view this dataset is shot from.

The SOLVER is fixed at the source (`urban_fire_spread.ORIGIN_FRAC_CAP`, see
that module) so every FUTURE solve is capped. This tool repairs the CURRENT
manifest already on disk without a full re-solve, because a full re-solve
would renumber/reshuffle every record (different `via`/`how`/`entry_side`
chains, different levels from a different `rng` walk) — churn nobody asked
for when only 11 of 39 records are actually wrong.

WHAT CHANGES, AND WHAT DOES NOT.
  * `origin` (the storey the fire started on) and `origin_frac` (the
    fraction of the mass's height that storey came from) are the ONLY two
    fields touched on a changed record. `level`, `sides`, `entry_side`,
    `how`, `via`, `seed`, `t_ignite_s`, `age_s`, every geometry field
    (`x`/`y`/`yaw_deg`/`W`/`D`/`H`/`cell`/`usd`), `kind`, `asset`/`style`,
    `btype`, `height_class` are carried through byte-identical.
  * `how` STAYS "spot" on every record this tool touches (it was, in every
    single top-heavy record measured). This is deliberate, not an
    oversight: `how` records the GRAPH fact "a brand lit this building",
    which the origin-storey fix does not change — only WHERE on the
    building it started. A `spot` record with a low `origin_frac` is a
    perfectly legitimate state and is exactly what `urban_fire_spread.
    ORIGIN_FRAC_CAP` now guarantees for every future solve too.
  * The new `origin` RENAMES the bake stem (`fire_bake.out_stem`'s
    `_o<origin>_` segment, e.g. `o18` -> `o7`), which invalidates that
    record's existing bake. Every GAC bake this tool can touch is already
    scheduled for a re-bake for unrelated fixes (interiors/pillars/roof
    props) — so a GAC-record origin change costs NOTHING beyond what is
    already happening. A KIT-record origin change is NOT already covered
    (kit bakes are cheap and are usually only re-baked when something
    about THAT record changes), so those are reported separately as the
    "extra re-bakes beyond the 16 GAC".

WHY A LOW-BIASED DRAW, AND WHY THE RECORD'S OWN SEED. Every record already
carries a `seed` (`urban_fire_city_plan.md`'s per-building stable seed —
built into a stable draw from the building's tag/level/origin/sides/mass/
position), so `random.Random(rec["seed"])` reproduces the exact same new
origin on every run of this tool — no separate RNG state to track, and
running the tool twice on the same input is a no-op diff. The reshape is
the bench's own low-bias shape (`urban_fire.plan_fire`'s `u ** 1.7`,
reused here as `urban_fire_spread.ORIGIN_BIAS`) linearly mapped onto the
[0.15, 0.45] band the user/lead asked for, instead of [0, 1] — about 60%
of draws land in the bottom third of THAT band (frac ~0.15-0.25), and the
tail still reaches 0.45, never above it and never below storey 1.
"""
import argparse
import json
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.normpath(os.path.join(_HERE, ".."))
if _SCENE_GEN_DIR not in sys.path:
    sys.path.insert(0, _SCENE_GEN_DIR)

from disaster import urban_fire_spread as ufs             # noqa: E402
from disaster import fire_bake as fb                       # noqa: E402

TOP_HEAVY_THRESHOLD = 0.5      # origin/n_storeys above this is "top-heavy"
LOW_BAND = (0.15, 0.45)        # target band for the redrawn origin_frac
BIAS = ufs.ORIGIN_BIAS         # 1.7 -- the same low-bias exponent the bench
                                # (`urban_fire.plan_fire`) and the solver's
                                # own `pick_origin` already use


def _stem_entry(rec):
    """A record -> the dict `fire_bake.out_stem` expects."""
    kind = rec.get("kind")
    name = rec.get("asset") if kind in ("gac", "dtc") else rec.get("style")
    return {"kind": kind, "name": name, "level": rec.get("level"),
            "origin": rec.get("origin"), "sides": rec.get("sides"),
            "seed": rec.get("seed")}


def draw_low_origin(rec):
    """Deterministic from `rec["seed"]` -> `(new_storey, new_origin_frac)`.

    `frac` is drawn in `LOW_BAND` with the `u ** BIAS` low-bias shape, then
    scaled to a storey index the same way `entry_for_plan_fire` does
    (`round(frac * (n_storeys - 1))`), clamped to `[1, n_storeys - 1]` —
    never storey 0 (the rule handed down: "never below storey 1"), never
    at or past the roof.
    """
    n = int(rec["n_storeys"])
    rng = random.Random(int(rec["seed"]))
    u = rng.random()
    lo, hi = LOW_BAND
    frac = lo + (hi - lo) * (u ** BIAS)
    storey = int(round(frac * (n - 1)))
    storey = max(1, min(n - 1, storey))
    return storey, round(frac, 4)


def lower_top_heavy(manifest, kinds=("gac", "kit"),
                    threshold=TOP_HEAVY_THRESHOLD):
    """Returns `(new_manifest, diff_rows)`.

    `new_manifest` is a deep copy of `manifest` with `origin`/`origin_frac`
    rewritten on every top-heavy record whose `kind` is in `kinds` — every
    other record, and every other field of a changed record, is
    byte-identical to the input. `diff_rows` is one dict per changed
    record: `i`, `kind`, `name`, `level`, `n_storeys`, `old_origin`,
    `new_origin`, `old_frac`, `new_frac`, `old_stem`, `new_stem`.
    """
    out = json.loads(json.dumps(manifest))       # plain deep copy
    diff_rows = []
    for rec in out["records"]:
        n = rec.get("n_storeys")
        origin = rec.get("origin")
        if not n or origin is None or rec.get("kind") not in kinds:
            continue
        frac_now = origin / float(n)
        if frac_now <= threshold:
            continue
        old_stem = fb.out_stem(_stem_entry(rec))
        new_storey, new_frac = draw_low_origin(rec)
        rec["origin"] = new_storey
        rec["origin_frac"] = new_frac
        new_stem = fb.out_stem(_stem_entry(rec))
        diff_rows.append({
            "i": rec["i"], "kind": rec["kind"],
            "name": rec.get("asset") or rec.get("style"),
            "level": rec["level"], "n_storeys": n,
            "old_origin": origin, "new_origin": new_storey,
            "old_frac": round(frac_now, 3), "new_frac": new_frac,
            "old_stem": old_stem, "new_stem": new_stem,
        })
    return out, diff_rows


def format_diff_table(diff_rows):
    hdr = "{0:>4} {1:<5} {2:<18} {3:<4} {4:>4} {5:>9} {6:>9}  {7} -> {8}"
    lines = [hdr.format("i", "kind", "name", "lvl", "n_st",
                        "old o/n", "new o/n", "old_stem", "new_stem")]
    for r in diff_rows:
        lines.append(hdr.format(
            r["i"], r["kind"], r["name"], r["level"], r["n_storeys"],
            "{0}({1:.2f})".format(r["old_origin"], r["old_frac"]),
            "{0}({1:.2f})".format(r["new_origin"], r["new_frac"]),
            r["old_stem"], r["new_stem"]))
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("manifest", help="input fire-city manifest json")
    ap.add_argument("--out", required=True,
                    help="candidate output path (never _plans/)")
    ap.add_argument("--threshold", type=float, default=TOP_HEAVY_THRESHOLD)
    args = ap.parse_args(argv)

    if os.path.abspath(os.path.dirname(args.out)) == os.path.abspath(
            os.path.join(_SCENE_GEN_DIR, "_plans")):
        raise SystemExit(
            "refusing to write into _plans/ -- pick a scratch path")

    with open(args.manifest) as fh:
        manifest = json.load(fh)

    new_manifest, diff_rows = lower_top_heavy(manifest,
                                              threshold=args.threshold)
    n_gac = sum(1 for r in diff_rows if r["kind"] == "gac")
    n_kit = sum(1 for r in diff_rows if r["kind"] == "kit")
    new_manifest["origin_lowering_patch"] = {
        "note": ("post-hoc origin/origin_frac rewrite via "
                "tools/fire_city_lower_origins.py -- no geometry/level/"
                "sides/seed/how touched, only `origin`/`origin_frac` on "
                "top-heavy (origin/n_storeys > {0}) records; see "
                "diff_rows".format(args.threshold)),
        "threshold": args.threshold, "low_band": list(LOW_BAND),
        "bias": BIAS, "n_changed": len(diff_rows),
        "n_gac_changed": n_gac, "n_kit_changed": n_kit,
        "n_extra_rebakes_beyond_gac": n_kit,
        "diff_rows": diff_rows,
    }

    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w") as fh:
        json.dump(new_manifest, fh, indent=2)

    print("read {0} records from {1}".format(
        len(manifest["records"]), args.manifest))
    print("{0} top-heavy record(s) rewritten ({1} gac, {2} kit)".format(
        len(diff_rows), n_gac, n_kit))
    print(format_diff_table(diff_rows))
    print()
    print("GAC changes: 0 extra re-bakes (16/16 already scheduled)")
    print("KIT changes: {0} extra re-bake(s) beyond the 16 GAC".format(n_kit))
    print("wrote candidate manifest -> {0}".format(args.out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
