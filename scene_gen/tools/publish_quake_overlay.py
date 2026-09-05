#!/usr/bin/env python3
"""Materialize an approved symlink overlay as a complete Nucleus library.

Unchanged archetypes are copied Nucleus-to-Nucleus from the canonical tree;
only overlay targets whose byte size differs are uploaded from this checkout.
The resulting directory contains real files (never symlinks or local paths),
so another device can build/freeze a quake cell from Nucleus alone.
"""

import argparse
import os
import sys
import time

import omni.client


DEFAULT_CANONICAL = (
    "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
    "scene_gen/assets/archetype")
DEFAULT_DEST = DEFAULT_CANONICAL + "_r15"
DEFAULT_OVERLAY = (
    "/isaac-sim/AirStack/scene_gen/assets/eq_review_overlay_r15")


def _ok(result):
    return result == omni.client.Result.OK


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--overlay", default=DEFAULT_OVERLAY)
    ap.add_argument("--canonical", default=DEFAULT_CANONICAL)
    ap.add_argument("--dest", default=DEFAULT_DEST)
    ap.add_argument("--names-out", default="")
    args = ap.parse_args(argv)

    if not os.path.isdir(args.overlay):
        raise SystemExit("overlay does not exist: " + args.overlay)
    names = sorted(n for n in os.listdir(args.overlay)
                   if os.path.isfile(os.path.join(args.overlay, n)))
    if "archetypes.json" not in names or not any(n.endswith(".usd") for n in names):
        raise SystemExit("overlay is not a complete archetype library")

    t0 = time.time()
    result = omni.client.copy(
        args.canonical.rstrip("/"), args.dest.rstrip("/"),
        behavior=omni.client.CopyBehavior.OVERWRITE)
    if not _ok(result):
        raise RuntimeError("Nucleus canonical tree copy failed: {0}".format(result))
    print("[quake_overlay] canonical tree copied server-side -> {0}".format(
        args.dest), flush=True)

    uploaded = matched = 0
    for i, name in enumerate(names, 1):
        local = os.path.realpath(os.path.join(args.overlay, name))
        if not os.path.isfile(local):
            raise RuntimeError("overlay target missing: " + local)
        remote = args.dest.rstrip("/") + "/" + name
        stat_result, entry = omni.client.stat(remote)
        if (_ok(stat_result) and int(entry.size) == int(os.path.getsize(local))):
            matched += 1
        else:
            result = omni.client.copy(
                local, remote, behavior=omni.client.CopyBehavior.OVERWRITE)
            if not _ok(result):
                raise RuntimeError("upload failed for {0}: {1}".format(
                    name, result))
            uploaded += 1
        if i % 25 == 0 or i == len(names):
            print("[quake_overlay] {0}/{1}: matched={2}, uploaded={3}".format(
                i, len(names), matched, uploaded), flush=True)

    failures = []
    for name in names:
        local = os.path.realpath(os.path.join(args.overlay, name))
        result, entry = omni.client.stat(args.dest.rstrip("/") + "/" + name)
        if not _ok(result) or int(entry.size) != int(os.path.getsize(local)):
            failures.append(name)
    if failures:
        raise RuntimeError("remote overlay size gate failed: " +
                           ", ".join(failures[:20]))
    if args.names_out:
        with open(args.names_out, "w") as fh:
            for name in names:
                if name.endswith(".usd"):
                    fh.write(name + "\n")
    print("[quake_overlay] COMPLETE: {0} files verified in {1:.1f}s "
          "({2} uploaded, {3} server-copy matches)".format(
              len(names), time.time() - t0, uploaded, matched), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
