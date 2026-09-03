#!/usr/bin/env python
"""Synchronise the geometry-only sliced-kit cache with Nucleus.

Run with Isaac's Python because it supplies ``omni.client``.  The remote
manifest stores USD basenames; pull rewrites them to this checkout's local
cache path and merges by ``(asset, signature)``.
"""

import argparse
import json
import os
import tempfile
import time
import sys

import omni.client

SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if SCENE_GEN not in sys.path:
    sys.path.insert(0, SCENE_GEN)

from detail import kit_bake as kb


DEFAULT_DST = (
    "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
    "scene_gen/cache/sliced_kits/v1")


def _ok(result):
    return result == omni.client.Result.OK


def _copy(src, dst):
    return _ok(omni.client.copy(
        src, dst, behavior=omni.client.CopyBehavior.OVERWRITE))


def push(dst):
    records = kb.read_manifest()
    current = kb.fingerprint()
    records = [r for r in records if r.get("fingerprint") == current
               and os.path.isfile(r.get("usd") or "")]
    portable = []
    omni.client.create_folder(dst)
    ok = skip = fail = 0
    result, entries = omni.client.list(dst)
    sizes = ({e.relative_path: int(getattr(e, "size", 0) or 0) for e in entries}
             if _ok(result) else {})
    for r in records:
        src = r["usd"]
        name = os.path.basename(src)
        if sizes.get(name) == os.path.getsize(src):
            skip += 1
        elif _copy(src, dst + "/" + name):
            ok += 1
        else:
            fail += 1
        q = dict(r)
        q["usd"] = name
        portable.append(q)
    fd, tmp = tempfile.mkstemp(prefix="sliced_kits_", suffix=".json")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(portable, f, indent=1, sort_keys=True)
        if not _copy(tmp, dst + "/kits.json"):
            fail += 1
    finally:
        os.unlink(tmp)
    print("sliced-kit push: %d uploaded, %d already current, %d failed; "
          "%d manifest records -> %s" % (ok, skip, fail, len(portable), dst))
    return 1 if fail else 0


def pull(dst):
    os.makedirs(kb.KIT_DIR, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix="sliced_kits_remote_", suffix=".json")
    os.close(fd)
    try:
        if not _copy(dst + "/kits.json", tmp):
            print("sliced-kit pull: no remote manifest at %s" % dst)
            return 0
        with open(tmp) as f:
            remote = json.load(f)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    current = kb.fingerprint()
    fresh = []
    ok = skip = fail = 0
    for r in remote:
        if r.get("fingerprint") != current:
            continue
        name = os.path.basename(r.get("usd") or "")
        if not name:
            continue
        local = os.path.join(kb.KIT_DIR, name)
        if os.path.isfile(local):
            skip += 1
        elif _copy(dst + "/" + name, local):
            ok += 1
        else:
            fail += 1
            continue
        q = dict(r)
        q["usd"] = os.path.abspath(local)
        fresh.append(q)
    if fresh:
        kb.merge_manifest(fresh)
    print("sliced-kit pull: %d downloaded, %d already local, %d failed; "
          "%d current-vintage records" % (ok, skip, fail, len(fresh)))
    return 1 if fail else 0


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--push", action="store_true")
    g.add_argument("--pull", action="store_true")
    ap.add_argument("--dst", default=os.environ.get("SLICED_KIT_NUCLEUS", DEFAULT_DST))
    args = ap.parse_args()
    t0 = time.time()
    rc = push(args.dst) if args.push else pull(args.dst)
    print("sync elapsed %.1f s" % (time.time() - t0))
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
