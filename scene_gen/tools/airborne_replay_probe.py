#!/usr/bin/env python
"""airborne_replay_probe -- replay `deactivate_airborne`'s ORIGINAL
single-pass moment on a REAL bake, in memory only, to prove the iterative
fix actually closes the gap it was written for.

THE SHAPE OF THE BUG (measured on `gac_SM_Building_26_F4_s162.usd`,
2026-08-31): the live bake's ONE call to the old single-pass
`deactivate_airborne` judged 76 candidates unsupported and switched them
off in a single batch -- but 14 `spall`/`spallhalo` stamps had been found
"seated" on ANOTHER member of that SAME batch (an `sbar` a few centimetres
away, itself unsupported) before the batch was applied, so they survived
into the export. `tools/cascade_check_probe.py` found the smoking gun:
those 14 stamps' nearest INACTIVE candidate on the current file (already
switched off, since the export reflects the bake's final state) sits inside
`fire_bake._BACKING_MAX_M` of them.

This script REVERSES that one fact -- reactivates every candidate mesh
that is currently inactive under `fb.BAKE_ROOT`, i.e. rolls the stage back
to exactly the state `deactivate_airborne` saw the FIRST time it was ever
called on this building -- then calls the CURRENT (iterative)
`fire_bake.deactivate_airborne` on it and checks that every one of the 14
originally-surviving stamps is deactivated this time, in addition to the
76 the old single pass already caught. Never calls `stage.Save()` /
`GetRootLayer().Save()` -- the reactivation and the new judgement live only
in this process's memory, so the file on disk is untouched no matter what
this script decides.

    docker exec isaac-sim bash -c \
        "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
         /isaac-sim/AirStack/scene_gen/tools/airborne_replay_probe.py \
         /isaac-sim/.cache/fire_bakes_dtc/gac_SM_Building_26_F4_s162.usd"
"""
import sys

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")

from disaster import fire_bake as fb                            # noqa: E402

KNOWN_FLOATERS = [
    "spall_g5_115", "spallhalo_g5_114", "spall_g5_97", "spall_g5_70",
    "spallhalo_g5_69", "spallhalo_g5_29", "spall_g5_30", "spallhalo_g5_47",
    "spall_g5_24", "spallhalo_g5_23", "spall_g5_40", "spallhalo_g5_39",
    "spallhalo_g5_16", "spall_g5_17",
]


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else (
        "/isaac-sim/.cache/fire_bakes_dtc/gac_SM_Building_26_F4_s162.usd")

    from pxr import Sdf, Usd, UsdGeom

    stage = Usd.Stage.Open(path)
    root = stage.GetPrimAtPath(Sdf.Path(fb.BAKE_ROOT))

    reactivated = 0
    for p in Usd.PrimRange(root, Usd.PrimAllPrimsPredicate):
        if not p.IsA(UsdGeom.Mesh) or p.IsActive():
            continue
        if fb._match_prefix(p.GetPath().name) is None:
            continue                    # only candidates, not stray inactives
        p.SetActive(True)
        reactivated += 1
    print("[replay] reactivated {0} candidate mesh(es) -- this is now the "
          "PRE-JUDGE state the real bake's settle handed to "
          "`deactivate_airborne` the first time".format(reactivated))

    before = {name: stage.GetPrimAtPath(
        Sdf.Path("{0}/{1}".format(fb.BAKE_ROOT + "/g5", name))).IsActive()
        for name in KNOWN_FLOATERS}

    n = fb.deactivate_airborne(stage, fb.BAKE_ROOT, gap_m=1.0, verbose=True)

    print("\n[replay] {0} total candidate(s) deactivated this run "
          "(old single-pass bake got 76)".format(n))
    print("\n[replay] the 14 originally-surviving floaters, now:")
    ok = 0
    for name in KNOWN_FLOATERS:
        pth = "{0}/{1}".format(fb.BAKE_ROOT + "/g5", name)
        prim = stage.GetPrimAtPath(Sdf.Path(pth))
        active = prim.IsActive() if prim and prim.IsValid() else None
        good = active is False
        ok += 1 if good else 0
        print("    {0:<20} was_active_before={1!s:<5} active_now={2!s:<5} "
              "{3}".format(name, before[name], active,
                           "OK" if good else "*** STILL ACTIVE ***"))
    print("\n[replay] {0}/{1} originally-surviving floaters now correctly "
          "deactivated. Nothing was saved to disk.".format(
              ok, len(KNOWN_FLOATERS)))
    return 0 if ok == len(KNOWN_FLOATERS) else 1


if __name__ == "__main__":
    sys.exit(main())
