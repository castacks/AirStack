#!/usr/bin/env python
"""wall_backing_probe -- dry-run demonstration of `fire_bake`'s new BACKING
TEST (`_wall_backing_contact`, `_WALL_DECAL_FAMILIES`) against an
ALREADY-BAKED evidence file, opened read-only.

    docker exec isaac-sim bash -c \
        "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
         /isaac-sim/AirStack/scene_gen/tools/wall_backing_probe.py \
         /isaac-sim/.cache/fire_bakes_dtc/gac_SM_Building_26_F4_s162.usd"

Never writes anything: it calls `fire_bake._judge_candidates` (the same
function `tools/airborne_probe.py --fix` would call before flipping
`active`) and only ever prints its judgement. Safe on the read-only
`.cache` evidence bakes.

WHY THIS EXISTS. `airborne_probe.py` already prints a general worst-by-gap
/ spot-check report; this one is narrower and answers the two specific
questions the backing-test fix needs answered against a REAL building
instead of a synthetic one: (1) of the known floating `spall`/`spallhalo`
prims from the original bug report (fire_dtc3 bench, building b5,
2026-08-30), does every one of them now get `deactivate=True`, all with
`backing_m=None` (no wall found within `fire_bake._BACKING_MAX_M`)? and
(2) do wall-backed stamps ELSEWHERE on the same building -- ones that never
came up in the bug report because they look fine -- correctly come back
`deactivate=False` with a small, sane `backing_m`?
"""
import os
import sys

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")

from disaster import fire_bake as fb                          # noqa: E402

# the 9 prims the original bug report measured as floating on
# gac_SM_Building_26_F4_s162.usd (fire_dtc3 bench, building b5, 2026-08-30)
KNOWN_FLOATERS = [
    "spall_g5_70", "spallhalo_g5_69", "spallhalo_g5_29", "spall_g5_30",
    "spallhalo_g5_47", "spall_g5_24", "spallhalo_g5_23", "spallhalo_g5_16",
    "spall_g5_17",
]


def main():
    ap_paths = sys.argv[1:]
    if not ap_paths:
        raise SystemExit("usage: wall_backing_probe.py <baked.usd> [...]")

    from pxr import Usd

    bad = 0
    for path in ap_paths:
        print("=" * 92)
        print("WALL BACKING PROBE  {0}".format(os.path.basename(path)))
        print("=" * 92)
        stage = Usd.Stage.Open(path)
        if stage is None:
            print("  *** could not open {0}".format(path))
            bad += 1
            continue

        info = fb._judge_candidates(stage, fb.BAKE_ROOT, gap_m=1.0,
                                    verbose=False)
        judged = info["judged"]
        decal = [j for j in judged if j["prefix"] in fb._WALL_DECAL_FAMILIES]
        flagged = [j for j in decal if j["deactivate"]]
        kept = [j for j in decal if not j["deactivate"]]
        by_leaf = {j["path"].rsplit("/", 1)[-1]: j for j in decal}

        print("  candidates judged   {0}".format(len(judged)))
        print("  decal-family (spall/spallhalo/crack) candidates  {0}".format(
            len(decal)))
        print("  -> now deactivated  {0}".format(len(flagged)))
        print("  -> kept (backed)    {0}".format(len(kept)))

        print("\n  KNOWN FLOATERS FROM THE BUG REPORT ({0})".format(
            len(KNOWN_FLOATERS)))
        n_ok = 0
        for leaf in KNOWN_FLOATERS:
            j = by_leaf.get(leaf)
            if j is None:
                print("    {0:<24} NOT FOUND on this bake".format(leaf))
                continue
            ok = j["deactivate"] and j["backing_m"] is None
            n_ok += 1 if ok else 0
            print("    {0:<24} deactivate={1!s:<5} backing_m={2}  "
                  "gap={3:.2f}  {4}".format(
                      leaf, j["deactivate"], j["backing_m"], j["gap"],
                      "OK" if ok else "*** UNEXPECTED ***"))
        print("  {0}/{1} known floaters now correctly deactivated".format(
            n_ok, len(KNOWN_FLOATERS)))
        if n_ok != len(KNOWN_FLOATERS):
            bad += 1

        # `backing_m` is None for a candidate kept via the EARLIER flush
        # vertical seat (`gap <= _SEAT_TOL_M`) rather than the backing
        # test -- it never ran `_wall_backing_contact` at all (the "not
        # contact and pfx in _WALL_DECAL_FAMILIES" guard). Both are
        # legitimate "kept"; only the backed ones are the new test's own
        # evidence, so report them separately.
        backed = [j for j in kept if j["backing_m"] is not None]
        flush = [j for j in kept if j["backing_m"] is None]
        print("\n  SAMPLE OF KEPT, WALL-BACKED DECAL STAMPS ELSEWHERE ON "
              "THE BUILDING ({0} of {1} kept; {2} more kept via the "
              "pre-existing flush-seat check, backing test never ran)"
              .format(len(backed), len(kept), len(flush)))
        for j in sorted(backed, key=lambda j: j["backing_m"])[:8]:
            print("    {0:<40} backing_m={1:.4f}  bottom_z={2:.2f}".format(
                j["path"].rsplit("/", 1)[-1], j["backing_m"], j["bottom_z"]))
        if not backed:
            print("    (none -- no decal-family stamp on this bake was kept "
                  "by the backing test itself; unexpected)")
            bad += 1

    print("\n{0}/{1} file(s) clean".format(len(ap_paths) - bad,
                                           len(ap_paths)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
