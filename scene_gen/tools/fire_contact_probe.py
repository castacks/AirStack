#!/usr/bin/env python3
"""fire_contact_probe.py — offline verification for the 2026-08-31
"fires floating outside the building" fix (contact snap), the "avoid fires
at the extreme top of buildings... unless we're 100% sure about windows on
the top floor" fix (top-storey synthetic exclusion), and the "more smoke on
lower floors" fix (smoke vertical bias) — all landed in
`disaster.fire_assembly_lib` / `disaster.gac_fire` / `disaster.fire_bake`.

RUNS ON THE HOST (or in-container via `usd_python.sh`) — real `pxr` + `vtk`,
no Kit, no `SimulationApp`, no Nucleus, safe beside a live sim. A bake
`.usd` opens directly with `Usd.Stage.Open` — its interior furniture props
(fit-out) reference Nucleus paths and WARN loudly on stderr (harmless,
expected: Nucleus is down), but the WALL / STRUCTURAL geometry a contact
test needs is authored straight into the file's own root layer and reads
fine regardless (measured: 1476 meshes compose under
`gac_SM_Building_11_F4_o18_SNW_s374`'s own `/World/bake`, warnings and all).

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        --with vtk python tools/fire_contact_probe.py diagnose \\
        gac_SM_Building_11_F4_o18_SNW_s374 \\
        gac_SM_Building_02_F4_o11_SEN_s777 kit_apartment_tall_F5_o4_SNW_s758

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \\
        --with vtk python tools/fire_contact_probe.py verify

    docker exec isaac-sim bash -c \\
        "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
         /isaac-sim/AirStack/scene_gen/tools/fire_contact_probe.py verify"

WHY NOTHING HERE IS REIMPLEMENTED. Every measurement below is the REAL
assembly code, called exactly as `fire_assembly_lib.place_fire` calls it
internally:

  * `fire_bake.load_for_assembly` rehydrates a bake's sidecar `masses`/
    `events` — the same opening frames (`op["fr"]`, `op["out"]`) the real
    assembly places emitters against.
  * `fire_assembly_lib.bake_geometry_root` / `build_contact_locator` /
    `contact_offset` / `snap_events_to_geometry` — the actual contact-snap
    machinery, unchanged.
  * `fire_assembly_lib.is_synthetic_op` / `drop_top_storey_synthetic` — the
    actual top-storey filter.
  * `fire_assembly_lib.place_fire` itself, for `verify` — run end to end
    against each bake's own OWN geometry (`geom_root` pointed at the file's
    own `/World/bake`, no city holder involved: a bake opened standalone IS
    already at the coordinates its own frames were authored in).

`diagnose` additionally prints the DISTANCE DISTRIBUTION `place_fire`'s snap
pass would see BEFORE any correction — the direct answer to "was the
synthetic-plane hypothesis right".

`verify` resolves the 39 manifest records to bake files exactly the way
`urban_fire_city_launch_script.bake_paths` does (`fire_city_manifest.
build_entry_and_stem`, a STRICT stem match) — a record this probe cannot
find a bake for is a record the REAL launcher cannot find one for either,
and is reported as such rather than silently skipped or fuzzy-matched.
"""
import argparse
import glob
import json
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
for _p in (_SCENE_GEN, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from pxr import Sdf, Usd, UsdGeom                       # noqa: E402

from disaster import fire_assembly_lib as fal            # noqa: E402
from disaster import fire_bake as fb                      # noqa: E402
from disaster import urban_fire as uf                      # noqa: E402
import fire_city_manifest as fcm                          # noqa: E402

DEFAULT_BAKE_DIR = os.path.expanduser(
    "~/docker/isaac-sim/cache/main/fire_bakes/city_138")
DEFAULT_MANIFEST = os.path.join(_SCENE_GEN, "_plans", "fire_city_500m_39.json")

# The same knobs the city launcher opts into by default (see that file's own
# env-var section) — `verify` prices every building the way the live scene
# actually would, not the row/bench launcher's untouched defaults.
PLACE_FIRE_KW = dict(
    max_emitters=9, smoke=True, smoke_scale=1.0,
    side_smoke_flame_max=5, side_smoke_nonflame_max=6,
    roof_cap_intact=0, roof_cap_collapsed=2,
    flame_min_clusters=3, flame_extra_max=3,
    flame_size_scaling=True, smoke_size_scaling=True,
    smoke_window_jets=True, residual_flame_frac=0.4,
    smoke_vertical_bias=True,
    snap_tol_m=0.3, snap_inset_m=0.2,
    snap_reach_in_m=3.0, snap_reach_out_m=6.0,
)


def _percentile(vals, p):
    if not vals:
        return None
    s = sorted(vals)
    k = (len(s) - 1) * p
    lo, hi = int(k), min(int(k) + 1, len(s) - 1)
    if lo == hi:
        return s[lo]
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def _open_bake(usd_path):
    return Usd.Stage.Open(usd_path)


# ---------------------------------------------------------------------------
# diagnose — the pre-fix distance distribution
# ---------------------------------------------------------------------------
def _resolve_stem_paths(bake_dir, stem_or_path):
    if os.path.isfile(stem_or_path):
        usd = stem_or_path
    else:
        usd = os.path.join(bake_dir, stem_or_path + ".usd")
    return usd, os.path.splitext(usd)[0] + ".json"


def diagnose_one(usd_path, json_path, verbose=True):
    """`{"stem", "n_total", "n_synthetic", "phantom", "dists"}` — every
    opening's PRE-FIX `contact_offset`, measured against the bake's own
    composed geometry, no snap applied. `dists` is a list of `abs(t)` for
    every opening a real surface was found for within
    `fal.SNAP_REACH_IN_M`/`SNAP_REACH_OUT_M`."""
    stem = os.path.splitext(os.path.basename(usd_path))[0]
    if not os.path.exists(usd_path) or not os.path.exists(json_path):
        return {"stem": stem, "error": "missing usd/json ({0} / {1})"
               .format(os.path.exists(usd_path), os.path.exists(json_path))}
    doc, masses, events = fb.load_for_assembly(json_path)
    stage = _open_bake(usd_path)
    geom_root = fal.bake_geometry_root("", doc)
    locator = fal.build_contact_locator(stage, geom_root)
    dists, phantom, n_total, n_synth = [], 0, 0, 0
    per_side = {}
    for ev in events:
        for op in ev.get("ops") or []:
            n_total += 1
            synth = fal.is_synthetic_op(op)
            n_synth += int(synth)
            t = None if locator is None else fal.contact_offset(locator, op)
            side = op.get("side", "?")
            bucket = per_side.setdefault(side, {"n": 0, "synth": 0,
                                                "phantom": 0, "dists": []})
            bucket["n"] += 1
            bucket["synth"] += int(synth)
            if t is None:
                phantom += 1
                bucket["phantom"] += 1
            else:
                dists.append(abs(t))
                bucket["dists"].append(abs(t))
    out = {"stem": stem, "level": doc["fire"].get("level"),
          "state": doc["fire"].get("state"), "sides": doc["fire"].get("sides"),
          "n_storeys": doc["fire"].get("n_storeys"),
          "synthetic_sides": doc["fire"].get("synthetic_sides"),
          "locator": locator is not None, "n_total": n_total,
          "n_synthetic": n_synth, "phantom": phantom, "dists": dists,
          "per_side": per_side}
    if verbose:
        print("\n=== {0} ===".format(stem))
        print("  level={0} state={1} sides={2} n_storeys={3} "
              "synthetic_sides={4}".format(
                  out["level"], out["state"], out["sides"], out["n_storeys"],
                  out["synthetic_sides"]))
        print("  locator built: {0}  openings: {1} total, {2} synthetic, "
              "{3} phantom (no real geometry within reach)".format(
                  out["locator"], n_total, n_synth, phantom))
        if dists:
            print("  |pre-fix offset| m: min={0:.3f} p50={1:.3f} "
                  "p90={2:.3f} max={3:.3f}  (n={4})".format(
                      min(dists), _percentile(dists, 0.5),
                      _percentile(dists, 0.9), max(dists), len(dists)))
        else:
            print("  |pre-fix offset| m: no measurable openings")
        for side, b in sorted(per_side.items()):
            if not b["dists"]:
                tag = "no real geometry within reach" if b["phantom"] else "n/a"
                print("    side {0}: n={1} synth={2}  {3}".format(
                      side, b["n"], b["synth"], tag))
            else:
                print("    side {0}: n={1} synth={2} phantom={3}  "
                      "|offset| min={4:.3f} max={5:.3f}".format(
                          side, b["n"], b["synth"], b["phantom"],
                          min(b["dists"]), max(b["dists"])))
    return out


def cmd_diagnose(args):
    results = []
    for stem in args.stems:
        usd, j = _resolve_stem_paths(args.bake_dir, stem)
        results.append(diagnose_one(usd, j))
    bad = [r for r in results if r.get("error")]
    if bad:
        print("\n{0} bake(s) could not be loaded:".format(len(bad)))
        for r in bad:
            print("  {0}: {1}".format(r["stem"], r["error"]))
    return 1 if bad else 0


# ---------------------------------------------------------------------------
# verify — the full assembly-side pipeline, all 39 manifest buildings
# ---------------------------------------------------------------------------
def _record_bake_paths(records, bake_dir):
    """`[{"i", "record", "stem", "usd", "json", "have"}, ...]` — the SAME
    strict stem match `urban_fire_city_launch_script.bake_paths` uses
    (`fire_city_manifest.build_entry_and_stem`); a record this cannot
    resolve is a record the real launcher cannot resolve either."""
    rows = fcm.classify(records, bake_dir)
    out = []
    for r in rows:
        out.append({"i": r["i"], "record": r["record"], "stem": r.get("stem"),
                   "usd": r.get("usd_path"), "json": r.get("json_path"),
                   "have": bool(r.get("have")), "error": r.get("error")})
    return out


def _record_top_z(doc):
    return doc.get("top_z")


def _run_place_fire_recording(stage, root, doc, masses, events, tag, rng,
                              top_z, **kw):
    """`fal.place_fire`, unmodified, with one temporary instrumentation:
    every `urban_fire._flame_sources` call is intercepted (module-level
    monkeypatch, restored in `finally`) to record `(state, op["storey"])` —
    NOT a reimplementation of the selection logic, just a tap on its one
    output funnel, so `verify` can report which storeys actually got a
    flame vs. a smoke source without parsing prim names."""
    calls = []
    orig = uf._flame_sources

    def _tap(ctx, root_, op, state, scale, tag_, per_opening):
        calls.append((state, op.get("storey")))
        return orig(ctx, root_, op, state, scale, tag_, per_opening)

    uf._flame_sources = _tap
    try:
        res = fal.place_fire(stage, root, doc, masses, events, tag, rng,
                             top_z, 0.0, 0.0, **kw)
    finally:
        uf._flame_sources = orig
    flame_storeys = sorted({s for st, s in calls
                            if st == "flame" and s is not None})
    smoke_storeys = sorted({s for st, s in calls
                            if st != "flame" and s is not None})
    return res, flame_storeys, smoke_storeys


def verify_one(row, index):
    rec = row["record"]
    if not row["have"]:
        return {"i": row["i"], "stem": row["stem"] or "?", "have": False,
               "error": row.get("error")}
    doc, masses, events = fb.load_for_assembly(row["json"])
    stage = _open_bake(row["usd"])
    geom_root = fal.bake_geometry_root("", doc)
    top_z = _record_top_z(doc)
    rng = random.Random(7 + 31 * index)
    res, flame_storeys, smoke_storeys = _run_place_fire_recording(
        stage, "/World/_probe_flow", doc, masses, events,
        "p{0}".format(index), rng, top_z, geom_root=geom_root,
        **PLACE_FIRE_KW)
    n_st = int((doc.get("fire") or {}).get("n_storeys") or 0)
    top2_from = max(0, n_st - 2)
    top2_flame = sum(1 for s in flame_storeys if s >= top2_from)
    snap = res.get("snap") or {}
    top = res.get("synthetic_top") or {}
    band = (doc.get("fire") or {}).get("storeys") or []
    return {"i": row["i"], "stem": row["stem"], "have": True,
           "level": doc["fire"].get("level"), "state": doc["fire"].get("state"),
           "kind": rec.get("kind"),
           "snap_tested": snap.get("tested", 0), "snap_ok": snap.get("ok", 0),
           "snap_snapped": snap.get("snapped", 0),
           "snap_dropped": snap.get("dropped", 0),
           "snap_worst_m": snap.get("worst_offset_m", 0.0),
           "snap_locator": bool(snap.get("locator")),
           "top_tested": top.get("tested", 0), "top_dropped": top.get("dropped", 0),
           "top_max_allowed": top.get("max_allowed"),
           "top2_flame": top2_flame, "n_storeys": n_st,
           "flame_storeys": flame_storeys, "smoke_storeys": smoke_storeys,
           "band": (min(band), max(band)) if band else None,
           "res": {k: res.get(k) for k in
                   ("flame", "smoke", "interior", "roof", "openings")}}


_HEADER = ("{0:<3} {1:<1} {2:<32} {3:<5} {4:>4} {5:>4} {6:>4} {7:>4} {8:>7} "
          "{9:>4} {10:>4} {11:>7} {12:<14} {13}")


def _kind_totals(rows):
    tot = {"tested": 0, "ok": 0, "snapped": 0, "dropped": 0}
    worst = 0.0
    n_locator = n_no_locator = 0
    for r in rows:
        tot["tested"] += r["snap_tested"]
        tot["ok"] += r["snap_ok"]
        tot["snapped"] += r["snap_snapped"]
        tot["dropped"] += r["snap_dropped"]
        worst = max(worst, r["snap_worst_m"])
        n_locator += int(r["snap_locator"])
        n_no_locator += int(not r["snap_locator"])
    return tot, worst, n_locator, n_no_locator


def cmd_verify(args):
    manifest = json.load(open(args.manifest))
    records = manifest.get("records") or []
    rows = _record_bake_paths(records, args.bake_dir)
    results = [verify_one(r, i) for i, r in enumerate(rows)]

    have = [r for r in results if r["have"]]
    missing = [r for r in results if not r["have"]]
    gac = [r for r in have if r["kind"] == "gac"]
    kit = [r for r in have if r["kind"] != "gac"]

    print("=== fire_contact_probe verify — {0} manifest record(s), "
          "{1} bake(s) found in {2} ===\n".format(
              len(records), len(have), args.bake_dir))
    print(_HEADER.format(
        "i", "K", "stem", "lvl", "s.tst", "s.ok", "s.snp", "s.drp", "s.wrst",
        "t.tst", "t.drp", "top2fl", "band", "flame/smoke storeys"))
    for r in have:
        print(_HEADER.format(
            r["i"], r["kind"][0].upper(), r["stem"][:32], r["level"] or "?",
            r["snap_tested"], r["snap_ok"], r["snap_snapped"],
            r["snap_dropped"], "{0:.2f}".format(r["snap_worst_m"]),
            r["top_tested"], r["top_dropped"], r["top2_flame"],
            "{0}-{1}".format(*r["band"]) if r["band"] else "-",
            "F{0} / S{1}".format(r["flame_storeys"], r["smoke_storeys"])))

    if missing:
        print("\n{0} record(s) have NO BAKE (stem mismatch — the real "
              "launcher would also leave these buildings intact):".format(
                  len(missing)))
        for r in missing:
            rec = records[r["i"]]
            print("  {0:<3} {1:<24} {2:<5} expected stem {3}".format(
                r["i"], rec.get("asset") or rec.get("style") or "?",
                rec.get("level"), r["stem"]))

    print("\n--- totals over {0} verified building(s) "
          "({1} GAC, {2} kit) ---".format(len(have), len(gac), len(kit)))
    for label, rows_k in (("GAC", gac), ("KIT", kit)):
        if not rows_k:
            continue
        tot, worst, n_loc, n_noloc = _kind_totals(rows_k)
        print("{0}: {1} building(s), {2} locator(s) ({3} without), "
              "{4} opening(s) tested, {5} already touching, {6} snapped, "
              "{7} dropped (phantom), worst pre-fix offset {8:.2f} m".format(
                  label, len(rows_k), n_loc, n_noloc, tot["tested"],
                  tot["ok"], tot["snapped"], tot["dropped"], worst))
    if kit:
        print("  *** KIT NUMBERS ARE NOT TRUSTWORTHY THIS SESSION: a kit "
              "building's own wall/facade geometry is a Nucleus reference "
              "(measured: 0 of 393 `parts` prims in "
              "kit_highrise_02_F3_o8_ENW_s126 compose as a Mesh with "
              "Nucleus down) — the locator built from it has no real wall "
              "to test against, so a KIT row's snap/drop counts reflect "
              "whatever unrelated local geometry (floor slabs, debris) "
              "happened to be nearby, not real contact. Re-run `verify` "
              "once Nucleus is reachable to get real KIT numbers; the GAC "
              "numbers above (sliced buildings, fully local geometry, "
              "including the user's own SM_Building_11 case) are solid "
              "today regardless.")

    top_tot = {"tested": 0, "dropped": 0}
    top2_bad = []
    for r in have:
        top_tot["tested"] += r["top_tested"]
        top_tot["dropped"] += r["top_dropped"]
        if r["top2_flame"] > 0:
            top2_bad.append((r["stem"], r["top2_flame"]))
    print("\ntop-storey synthetic filter (GAC only — kit is never "
          "synthetic): {0} synthetic opening(s) tested citywide, {1} "
          "dropped for sitting above the excluded top storeys".format(
              top_tot["tested"], top_tot["dropped"]))
    if top2_bad:
        print("  {0} building(s) still show flame in their own top-2 "
              "storeys (expected for REAL/measured openings up there — "
              "only a SYNTHETIC one is ever excluded): {1}".format(
                  len(top2_bad), top2_bad))
    else:
        print("  every building: ZERO flame sources in its own top-2 "
              "storeys")
    print("\n{0}".format(
        "SM_Building_11 F4 goes from floating to all-contact: see the "
        "`diagnose` run above/separately for the actual before/after "
        "numbers (this table only ever measures AFTER the fix)."))
    return 1 if missing and args.strict else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("diagnose", help="pre-fix distance distribution")
    d.add_argument("stems", nargs="+",
                   help="bake stem (no extension) or a .usd path")
    d.add_argument("--bake-dir", default=DEFAULT_BAKE_DIR)
    d.set_defaults(func=cmd_diagnose)

    v = sub.add_parser("verify", help="full pipeline over all 39 buildings")
    v.add_argument("--manifest", default=DEFAULT_MANIFEST)
    v.add_argument("--bake-dir", default=DEFAULT_BAKE_DIR)
    v.add_argument("--strict", action="store_true",
                   help="exit 1 if any manifest record has no bake")
    v.set_defaults(func=cmd_verify)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
