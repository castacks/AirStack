#!/usr/bin/env python
"""Dry-run (by default) the real-geometry airborne-debris check on baked
fire USDs and report what it found, without touching the file.

    docker exec isaac-sim bash -c \
        "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
         /isaac-sim/AirStack/scene_gen/tools/airborne_probe.py \
         /isaac-sim/.cache/fire_bakes/*.usd"

    # actually save the deactivations back to the file(s):
    ... airborne_probe.py --fix /isaac-sim/.cache/fire_bakes/gac_SM_Building_09_F6_s131.usd

WHY THIS EXISTS. `disaster.fire_bake.deactivate_airborne` (rewritten
2026-08-30 off real geometry -- see its docstring and `_judge_candidates`'s)
runs inside the live bake, once, with no way to inspect what it judged short
of the one-line summary it prints. This probe calls the same
`_judge_candidates` the real function calls, on an ALREADY-BAKED file opened
cold, and prints the full judgement: how many meshes/triangles went into the
locator, how long the locator took to build, every candidate's gap and
contact state, the worst offenders by gap, and -- so a fix can be trusted
before it is applied -- a spot check of named prims on both sides of the
line (flagged and not) showing exactly what each ray found.

`--fix` is the only thing that WRITES: without it, `_judge_candidates` never
calls `SetActive` at all (it only judges), so opening a file with this tool
and not passing `--fix` cannot change it on disk no matter how many times it
is run. `--fix` deactivates the flagged prims on the in-memory stage (same
as `deactivate_airborne` would) and saves the root layer.

Standalone `pxr` + `vtk`, no Kit/SimulationApp -- safe beside a live sim.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")

from disaster import fire_bake as fb                          # noqa: E402


def _fmt_path(p, width=64):
    p = str(p)
    if len(p) <= width:
        return p
    return "..." + p[-(width - 3):]


def _support_label(j):
    if j["support_path"]:
        return j["support_path"].rsplit("/", 1)[-1]
    return "ground" if j["support_z"] == 0.0 else "nothing"


def probe_one(path, fix=False, n_worst=12, n_spot=10, gap_m=1.0):
    from pxr import Sdf

    t0 = time.time()
    print("=" * 92)
    print("AIRBORNE PROBE  {0}".format(os.path.basename(path)))
    print("=" * 92)

    stage_open_t0 = time.time()
    from pxr import Usd
    stage = Usd.Stage.Open(path)
    if stage is None:
        print("  *** could not open {0}".format(path))
        return 1
    open_s = time.time() - stage_open_t0

    info = fb._judge_candidates(stage, fb.BAKE_ROOT, gap_m=gap_m,
                                verbose=False)
    judged = info["judged"]
    flagged = [j for j in judged if j["deactivate"]]
    kept = [j for j in judged if not j["deactivate"]]

    print("  stage open        {0:.2f} s".format(open_s))
    print("  meshes            {0}".format(info["n_meshes"]))
    print("  triangles         {0}".format(info["n_triangles"]))
    print("  load+triangulate  {0:.2f} s".format(info["load_s"]))
    print("  locator build     {0:.3f} s  (vtkStaticCellLocator)".format(
        info["build_s"]))
    print("  ray sweep         {0:.2f} s".format(info["ray_s"]))
    print("  candidates judged {0}".format(len(judged)))
    print("  flagged airborne  {0}".format(len(flagged)))

    if flagged:
        worst = sorted(flagged, key=lambda j: -j["gap"])[:n_worst]
        print("\n  WORST {0} BY GAP".format(len(worst)))
        print("  {0:<64} {1:>9} {2:>9} {3}".format(
            "path", "bottom_z", "gap", "contact"))
        for j in worst:
            print("  {0:<64} {1:>9.3f} {2:>9.3f} {3}".format(
                _fmt_path(j["path"]), j["bottom_z"], j["gap"], j["contact"]))

    # by-prefix summary, same bucketing `deactivate_airborne` prints
    by_kind = {}
    for j in flagged:
        by_kind[j["prefix"]] = by_kind.get(j["prefix"], 0) + 1
    if by_kind:
        print("\n  BY PREFIX  " + ", ".join(
            "{0} {1}".format(v, k) for k, v in
            sorted(by_kind.items(), key=lambda kv: -kv[1])))

    # spot check: N flagged + N unflagged, showing exactly what the rays found
    print("\n  SPOT CHECK -- {0} flagged".format(min(n_spot, len(flagged))))
    for j in sorted(flagged, key=lambda j: -j["gap"])[:n_spot]:
        print("    {0:<60} bottom_z {1:>8.3f}  support_z {2:>8.3f}  "
              "hit={3:<28} gap {4:>7.3f}  contact={5}".format(
                  _fmt_path(j["path"], 60), j["bottom_z"], j["support_z"],
                  _support_label(j), j["gap"], j["contact"]))
    print("\n  SPOT CHECK -- {0} UN-flagged (debris, kept)".format(
        min(n_spot, len(kept))))
    for j in sorted(kept, key=lambda j: j["gap"])[:n_spot]:
        print("    {0:<60} bottom_z {1:>8.3f}  support_z {2:>8.3f}  "
              "hit={3:<28} gap {4:>7.3f}  contact={5}".format(
                  _fmt_path(j["path"], 60), j["bottom_z"], j["support_z"],
                  _support_label(j), j["gap"], j["contact"]))

    if fix and flagged:
        for j in flagged:
            prim = stage.GetPrimAtPath(Sdf.Path(j["path"]))
            if prim and prim.IsValid():
                prim.SetActive(False)
        stage.GetRootLayer().Save()
        print("\n  --fix: saved {0} deactivation(s) to {1}".format(
            len(flagged), path))
    elif fix:
        print("\n  --fix: nothing to save (0 flagged)")
    else:
        print("\n  dry run -- nothing written ({0} would be deactivated with "
              "--fix)".format(len(flagged)))

    print("\n  total time        {0:.2f} s".format(time.time() - t0))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("usd", nargs="+", help="one or more baked .usd files "
                    "(a directory is expanded to every *.usd in it)")
    ap.add_argument("--fix", action="store_true",
                    help="save the deactivations back to each file "
                         "(default: dry run, nothing written)")
    ap.add_argument("--gap", type=float, default=1.0, dest="gap_m",
                    help="the gap_m deactivate_airborne would use (default "
                         "1.0 m, matches the real bake)")
    ap.add_argument("-n", "--worst", type=int, default=12,
                    help="how many worst-by-gap flagged prims to list")
    ap.add_argument("--spot", type=int, default=10,
                    help="how many flagged + how many kept prims to "
                         "spot-check in detail")
    args = ap.parse_args()

    import glob
    paths = []
    for item in args.usd:
        if os.path.isdir(item):
            paths += sorted(glob.glob(os.path.join(item, "*.usd")))
        else:
            paths.append(item)
    if not paths:
        raise SystemExit("no .usd file matched")

    bad = 0
    t_all = time.time()
    for p in paths:
        try:
            bad += probe_one(p, fix=args.fix, n_worst=args.worst,
                             n_spot=args.spot, gap_m=args.gap_m)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("*** {0}: {1}".format(p, exc))
            bad += 1
    print("\n{0}/{1} file(s) probed cleanly, total {2:.2f} s".format(
        len(paths) - bad, len(paths), time.time() - t_all))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
