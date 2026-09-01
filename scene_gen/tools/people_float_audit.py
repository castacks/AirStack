#!/usr/bin/env python3
"""
people_float_audit.py — did the SUPPORT HEIGHT this record claims actually
land there, and does the launcher's authoring path preserve it?

HOST-SIDE, no Isaac, no Nucleus, no `pxr`. Two separate questions, because
"floating" has two separate possible causes and conflating them wastes a
review:

1. **Is `z` in the record itself right, by the record's OWN class rule?**
   `fire_people.to_placements` documents `z` as "the support surface, not the
   prim z" (ground/kerb top, sill, storey floor, roof deck, debris top) —
   see `fire_people.to_placements.__doc__`. This tool re-derives that support
   height from the record's own fields (and, for the roof classes, from the
   CURRENT bake sidecar via `fire_people.deck_z` — the sidecar can have
   changed since the record was solved) and flags any record whose stored
   `z` disagrees by more than `--threshold` metres (default 0.3).

2. **Does the AUTHORING PATH add the right pose drop on top of that surface?**
   `to_placements(records, ctx=None)` — which is what
   `urban_fire_city_launch_script.place_people` actually calls, with NO
   `ctx` — runs `_placement_no_ctx`, not `people._human_placement`.
   `_human_placement` applies `people._seated_asset_dz`
   (`_MALE_SEATED_DZ_M = -0.15`, "the male rigs sit 0.15 m high") for
   `("seated_car", "seated_car_arms_down", "sit_edge")`; `_placement_no_ctx`
   USED TO have no equivalent call (found 2026-08-31: every MALE rig —
   `rp_eric`/`rp_manuel`/`rp_nathan`/`rp_dennis` — placed in `sit_edge`, the
   only `_SEAT_PLACED_POSES` member `fire_people` ever draws, was authored
   0.15 m above its kerb/sill in the live scene) and now does — the fix
   REUSES `people._seated_asset_dz` rather than copying it, so this part of
   the tool actually AUTHORS every seated-male record through
   `to_placements` (no `ctx=`, matching the live launcher exactly) and
   checks the correction really lands, rather than trusting a code read of
   whether the call is still there.

USAGE

    cd scene_gen && python3 tools/people_float_audit.py \\
        --manifest _plans/fire_people_final.json \\
        --dump _plans/fc_dump_500.json \\
        --sidecar-dir ~/docker/isaac-sim/cache/main/fire_bakes/city_138 \\
        --threshold 0.3

`--dump` is accepted for parity with `fire_people_rerun.sh` and printed in
the header; this tool does not need it for the z audit itself (every people
record already carries its own `building_cell`, which is how bake sidecars
are looked up — see `fire_people.load_sidecars`'s dual keying).
"""
import argparse
import json
import math
import os
import sys

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_TOOLS_DIR)
if _SCENE_GEN_DIR not in sys.path:
    sys.path.insert(0, _SCENE_GEN_DIR)

from disaster import fire_people as fp        # noqa: E402


STREET_CLASSES = fp.STREET_CLASSES                     # evacuee, onlooker, at_car
ROOF_CLASSES = ("roof", "roof_victim")
DEBRIS_CLASSES = ("casualty_apron", "roof_debris")

EXPECTED_Z_MODE = {
    "evacuee": "ground", "onlooker": "ground", "at_car": "ground",
    "window": "floor", "roof": "deck", "roof_victim": "deck",
    "casualty_apron": "debris", "roof_debris": "debris",
}

# `people._seated_asset_dz` / `people._MALE_RIGS`, transcribed for the static
# check (part 2) — this module has no import-time dependency on `people.py`
# and the tool wants to name the same rigs by the same basenames.
_MALE_RIGS = ("rp_eric", "rp_manuel", "rp_nathan", "rp_dennis")
_SEAT_PLACED_POSES = ("seated_car", "seated_car_arms_down", "sit_edge")
_MALE_SEATED_DZ_M = 0.15


def _records_of(doc):
    for key in ("people", "records"):
        v = doc.get(key)
        if isinstance(v, list):
            return v
    raise SystemExit("{0}: no 'people'/'records' list".format(doc))


def _fmt(v, nd=3):
    try:
        return round(float(v), nd)
    except (TypeError, ValueError):
        return v


def audit_ground(rec, cfg):
    """evacuee / onlooker / at_car — `_street_group`/`_pass_at_car`'s own
    rule: `kerb_z_m` on a sidewalk/paved surface, else 0.0."""
    surf = rec.get("surface")
    expected = float(cfg["kerb_z_m"]) if surf in ("sidewalk", "paved") else 0.0
    return expected, "surface={0!r} -> kerb_z_m={1} else 0.0".format(
        surf, cfg["kerb_z_m"])


def audit_window(rec, cfg):
    """window — `_pass_window`: `z = sill_z - _HIP_H * NOMINAL_HEIGHT_M`.

    Self-consistency against the record's OWN stored `sill_z` (there is no
    building geometry in a people record to re-derive `sill_z` itself from
    the sidecar — that needs the building's W/D/H/yaw, which only the FIRE
    manifest carries, not the people one). A second, cheap sanity bound:
    `sill_z - floor_z` should be a plausible sill height (0 - 4 m); a value
    outside that band means `storey_period`/`floor_z` disagreed with
    `sill_z` at solve time, which is a storey-math bug, not a support-height
    one, but this tool flags it as `note` rather than silently passing.
    """
    if rec.get("sill_z") is None:
        return None, "no sill_z on record — cannot audit"
    expected = float(rec["sill_z"]) - fp._HIP_H * fp.NOMINAL_HEIGHT_M
    note = "sill_z - _HIP_H*NOMINAL_HEIGHT_M ({0} - {1:.3f})".format(
        rec["sill_z"], fp._HIP_H * fp.NOMINAL_HEIGHT_M)
    floor_z = rec.get("floor_z")
    if floor_z is not None:
        sill_rel = float(rec["sill_z"]) - float(floor_z)
        if not (0.0 <= sill_rel <= 4.0):
            note += ("; ALSO sill_z-floor_z={0:.2f} m is outside the "
                     "plausible sill-height band [0, 4] — storey/period "
                     "arithmetic looks wrong upstream of this record"
                     .format(sill_rel))
    return expected, note


def audit_roof(rec, sidecars, cfg):
    """roof / roof_victim — re-derive `deck_z` from the CURRENT sidecar
    (`fire_people.deck_z`, the exact function the pass used) via the
    record's own `building_cell`, which is how `load_sidecars` keys a doc
    whenever `fire_bake` recorded one. A record whose stored `z` no longer
    matches what `deck_z()` returns today means the sidecar (or the code)
    changed under a manifest that was never re-solved — the exact staleness
    `_manifest_matches_dump` guards for the x/y geometry, extended here to
    z.

    LEVEL-QUALIFIED LOOKUP FIRST, exactly like `_Solver.__init__` (see
    `fire_people.load_sidecars`'s own account of the 2026-08-31 collision):
    a cell re-baked at more than one severity has TWO sidecar files sharing
    one `cell`/`tag`, and a bare `sidecars[cell]` lookup returns whichever
    one `os.listdir` happened to sort last — not necessarily the level this
    record's own `building_level` names. Falling back to the bare key only
    when there is no level-qualified entry keeps this tool honest about
    what the real solver actually saw."""
    cell = rec.get("building_cell")
    level = rec.get("building_level")
    doc = None
    if cell:
        doc = sidecars.get((cell, level)) or sidecars.get(cell)
    if doc is None:
        return None, "building_cell {0!r} has no sidecar on disk — cannot " \
            "re-derive deck_z; record's own deck_source={1!r}".format(
                cell, rec.get("deck_source"))
    fake = {"H": doc.get("top_z", 0.0)}
    z, source = fp.deck_z(fake, doc, cfg["parapet_est_m"])
    note = "fire_people.deck_z() against the on-disk sidecar -> {0:.3f} " \
        "({1}); record's own deck_source={2!r}".format(
            z, source, rec.get("deck_source"))
    if source != rec.get("deck_source"):
        note += " -- SOURCE DISAGREES with the record (sidecar changed " \
            "since this manifest was solved, or the manifest predates " \
            "fire_bake persisting deck_z)"
    return z, note


def audit_debris(rec, cfg):
    """casualty_apron / roof_debris — `_burial_record`:
    `apron_surface_z(apron_t, debris_depth_m)`. Self-consistency against the
    record's own stored `apron_t`/`debris_depth_m` (the windrow geometry
    itself needs the building's W/D/H/yaw, not carried on a people record)."""
    t, depth = rec.get("apron_t"), rec.get("debris_depth_m")
    if t is None or depth is None:
        return None, "no apron_t/debris_depth_m on record — cannot audit"
    expected = fp.apron_surface_z(t, depth)
    return expected, "apron_surface_z(t={0}, depth={1}) = " \
        "depth*(1-t)**1.3".format(t, depth)


def audit_record(rec, sidecars, cfg):
    """`(expected_z_or_None, note, mechanism)` for one record."""
    cls = rec.get("cls")
    if cls in STREET_CLASSES:
        exp, note = audit_ground(rec, cfg)
        mech = "ground_support"
    elif cls == "window":
        exp, note = audit_window(rec, cfg)
        mech = "sill_hip_formula"
    elif cls in ROOF_CLASSES:
        exp, note = audit_roof(rec, sidecars, cfg)
        mech = "deck_z_stale_or_missing_sidecar"
    elif cls in DEBRIS_CLASSES:
        exp, note = audit_debris(rec, cfg)
        mech = "apron_surface_profile"
    else:
        return None, "unknown class {0!r}".format(cls), "unknown_class"
    return exp, note, mech


def find_seated_male_gap(records):
    """Part 2 — every record the correction actually applies to, AUTHORED
    through the exact call the live launcher makes
    (`fp.to_placements([rec])`, no `ctx=`), so this checks the real output
    rather than trusting a code read of whether `_placement_no_ctx` still
    calls `people._seated_asset_dz`. Returns
    `[(rec, z_authored, z_without_correction, applied_m)]` — `applied_m` is
    0.0 (correctly nothing) whenever the rig is not one of `_MALE_RIGS`."""
    from disaster import people as ppl

    hits = []
    for rec in records:
        pose = str(rec.get("pose"))
        if pose not in _SEAT_PLACED_POSES:
            continue
        placements, skipped = fp.to_placements([rec])
        if not placements:
            continue        # unconvertible on this host path; not this check
        z_authored = float(placements[0]["z_m"])
        usd = os.path.basename(str(rec.get("usd", ""))).lower()
        is_male = usd.startswith(_MALE_RIGS)
        # What z WOULD be with no seated-asset correction at all — the pose
        # drop alone (`pose_z_offset`, unaffected by this fix).
        try:
            import scene_generator as sg
            z_no_fix = float(rec["z"]) + sg.pose_z_offset(
                rec["usd"], pose, fp.NOMINAL_HEIGHT_M)
        except Exception:
            z_no_fix = z_authored
        applied_m = round(z_no_fix - z_authored, 3)
        if is_male:
            hits.append((rec, z_authored, z_no_fix, applied_m))
    return hits


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--manifest", required=True,
                    help="the PEOPLE records JSON (fire_people_dry_run.py "
                         "output, e.g. _plans/fire_people_final.json)")
    ap.add_argument("--dump", default=None,
                    help="the FC placements dump (printed in the header "
                         "only; the z audit keys sidecars off each record's "
                         "own building_cell, not the dump)")
    ap.add_argument("--sidecar-dir", default=None,
                    help="dir of fire_bake sidecar .json files "
                         "(city_138/...); required for the roof/roof_victim "
                         "part of the audit")
    ap.add_argument("--threshold", type=float, default=0.3,
                    help="metres of |z - expected| to flag (default 0.3)")
    args = ap.parse_args(argv)

    with open(args.manifest) as fh:
        doc = json.load(fh)
    records = _records_of(doc)
    cfg = fp.resolve_cfg(None)
    sidecars = fp.load_sidecars(args.sidecar_dir) if args.sidecar_dir else {}

    print("=" * 78)
    print("people_float_audit: {0} record(s) from {1!r}".format(
        len(records), args.manifest))
    if args.dump:
        print("  dump: {0!r} (context only)".format(args.dump))
    print("  sidecars: {0} loaded from {1!r}".format(
        len(sidecars), args.sidecar_dir))
    print("  threshold: {0} m".format(args.threshold))
    print("=" * 78)

    # ---- part 1: per-record support-height audit -------------------------
    by_class = {}
    n_checked = 0
    n_unaudited = 0
    for rec in records:
        cls = rec.get("cls", "?")
        exp, note, mech = audit_record(rec, sidecars, cfg)
        by_class.setdefault(cls, {"flagged": [], "mechanisms": {},
                                  "n": 0, "unaudited": 0})
        by_class[cls]["n"] += 1
        if exp is None:
            by_class[cls]["unaudited"] += 1
            n_unaudited += 1
            continue
        n_checked += 1
        diff = float(rec.get("z", 0.0)) - float(exp)
        if abs(diff) > args.threshold:
            by_class[cls]["flagged"].append({
                "id": rec.get("id"), "z": _fmt(rec.get("z")),
                "expected": _fmt(exp), "diff_m": _fmt(diff),
                "building_i": rec.get("building_i"),
                "building_cell": rec.get("building_cell"),
                "z_mode": rec.get("z_mode"), "note": note,
            })
            by_class[cls]["mechanisms"][mech] = \
                by_class[cls]["mechanisms"].get(mech, 0) + 1
        # z_mode sanity, independent of the numeric compare
        want_mode = EXPECTED_Z_MODE.get(cls)
        if want_mode and rec.get("z_mode") != want_mode:
            by_class[cls]["mechanisms"]["z_mode_field_wrong"] = \
                by_class[cls]["mechanisms"].get("z_mode_field_wrong", 0) + 1

    print("\n--- PART 1: support-height audit "
          "({0} checked, {1} unaudited [no sidecar / no source field]) ---"
          .format(n_checked, n_unaudited))
    total_flagged = 0
    for cls in sorted(by_class):
        info = by_class[cls]
        flagged = info["flagged"]
        total_flagged += len(flagged)
        top_mech = (sorted(info["mechanisms"].items(),
                           key=lambda kv: -kv[1])[0][0]
                    if info["mechanisms"] else "-")
        print("  {0:<16} n={1:<4} unaudited={2:<4} flagged={3:<4} "
              "top_mechanism={4}".format(
                  cls, info["n"], info["unaudited"], len(flagged), top_mech))
        for f in flagged[:8]:
            print("      id={0} z={1} expected={2} diff={3:+.3f}m "
                  "building_i={4} z_mode={5}".format(
                      f["id"], f["z"], f["expected"], f["diff_m"],
                      f["building_i"], f["z_mode"]))
            print("        {0}".format(f["note"]))
        if len(flagged) > 8:
            print("      ... and {0} more".format(len(flagged) - 8))

    # ---- part 2: the male-seated authoring-path correction, VERIFIED ------
    hits = find_seated_male_gap(records)
    still_broken = [h for h in hits if abs(h[3] - _MALE_SEATED_DZ_M) > 1e-6]
    print("\n--- PART 2: seated-pose male-rig correction "
          "(authored through the real to_placements(recs), no ctx=) ---")
    print("  {0} MALE-rig record(s) in one of {1} — {2} correctly dropped "
          "{3:.2f} m below the pose-only z, {4} NOT (still {5:.2f} m HIGH "
          "if this list is nonzero)".format(
              len(hits), _SEAT_PLACED_POSES, len(hits) - len(still_broken),
              _MALE_SEATED_DZ_M, len(still_broken), _MALE_SEATED_DZ_M))
    for rec, z_authored, z_no_fix, applied_m in hits[:10]:
        print("    id={0} cls={1} usd={2} z_authored={3:.3f} "
              "z_without_correction={4:.3f} applied={5:.3f}m{6}".format(
                  rec.get("id"), rec.get("cls"),
                  os.path.basename(str(rec.get("usd"))), z_authored,
                  z_no_fix, applied_m,
                  "" if abs(applied_m - _MALE_SEATED_DZ_M) < 1e-6
                  else "  <-- NOT CORRECTED"))

    print("\n" + "=" * 78)
    print("SUMMARY: {0} support-height flag(s) over threshold, {1}/{2} "
          "male-rig seated record(s) still uncorrected".format(
              total_flagged, len(still_broken), len(hits)))
    print("=" * 78)
    return 1 if (total_flagged or still_broken) else 0


if __name__ == "__main__":
    raise SystemExit(main())
