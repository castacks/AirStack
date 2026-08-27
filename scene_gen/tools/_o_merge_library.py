#!/usr/bin/env python
"""_o_merge_library.py — merge a WHOLE baked archetype library, offline.

    scene_gen/tools/_t_pxr.sh scene_gen/tools/_o_merge_library.py \
        SRC=<arch dir> DST=<arch dir> [SHARD=0/4] [ONLY=bld_a,bld_b]

Bare pxr — no Kit, no GPU, no PhysX — so this runs beside a sim without
taking a bench slot, and it takes MINUTES where a re-bake takes hours. A
baked archetype is exactly what `bake.export_object` consumes, so re-exporting
one with `merge="on"` gives a merged twin of the same geometry: the ONLY
difference between the two libraries is the packing, which is what makes a
city A/B honest.

`archetypes.json` is copied with every `usd` path re-anchored to DST, so the
city can be pointed at the new directory with nothing but `ARCH_DIR=`.

`SHARD=i/n` processes only files whose index mod n == i, so several of these
can run at once (they are CPU-bound and single-threaded).
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..")))

from pxr import Usd                                          # noqa: E402

from disaster import bake                                    # noqa: E402


def main():
    src = os.environ["SRC"].rstrip("/")
    dst = os.environ["DST"].rstrip("/")
    only = set(q.strip() for q in os.environ.get("ONLY", "").split(",") if q.strip())
    sh = os.environ.get("SHARD", "").strip()
    si, sn = (0, 1)
    if sh:
        si, sn = [int(q) for q in sh.replace("/", " ").split()]
    os.makedirs(dst, exist_ok=True)
    names = sorted(q for q in os.listdir(src)
                   if q.startswith("bld_") and q.endswith(".usd"))
    if only:
        names = [q for q in names if q[:-4] in only or q in only]
    todo = [q for i, q in enumerate(names) if i % sn == si]
    t0 = time.time()
    tot_a = tot_b = 0.0
    done = 0
    for name in todo:
        sp = os.path.join(src, name)
        dp = os.path.join(dst, name)
        try:
            st = Usd.Stage.Open(sp)
            if st is None:
                print("[omerge] SKIP unreadable {0}".format(name))
                continue
            if os.path.exists(dp):
                os.remove(dp)
            info = {}
            t1 = time.time()
            ok = bake.export_object(st, None, ["/Baked"], dp, merge="on",
                                    stats_out=info)
            if not ok:
                print("[omerge] EMPTY {0}".format(name))
                continue
            a = os.path.getsize(sp) / 1e6
            b = os.path.getsize(dp) / 1e6
            tot_a += a
            tot_b += b
            done += 1
            print("[omerge] {0:<34} {1:7.2f} -> {2:7.2f} MB   {3:6d} src meshes"
                  " -> {4:4d} merged + {5:3d} kept, {6:3d} mats, {7:6d} prims"
                  "   {8:.1f} s".format(
                      name, a, b, info.get("src_meshes", 0),
                      info.get("merged_prims", 0), info.get("kept_src", 0),
                      info.get("materials", 0), info.get("out_prims", 0),
                      time.time() - t1), flush=True)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[omerge] FAILED {0}: {1}".format(name, exc), flush=True)

    # the manifest, re-anchored (only the shard that owns index 0 writes it)
    if si == 0:
        mp = os.path.join(src, "archetypes.json")
        if os.path.exists(mp):
            recs = json.load(open(mp))
            for r in recs:
                u = r.get("usd")
                if u:
                    r["usd"] = os.path.join(dst, os.path.basename(u))
            with open(os.path.join(dst, "archetypes.json"), "w") as fh:
                json.dump(recs, fh, indent=1)
            print("[omerge] manifest -> {0}/archetypes.json ({1} records)".format(
                dst, len(recs)))
    print("[omerge] shard {0}/{1}: {2} file(s), {3:.0f} -> {4:.0f} MB "
          "({5:.2f}x) in {6:.0f} s".format(
              si, sn, done, tot_a, tot_b,
              (tot_b / tot_a) if tot_a else 0.0, time.time() - t0))
    print("MERGE LIBRARY DONE")


main()
