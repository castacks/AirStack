#!/usr/bin/env python
"""nucleus_extents.py — measure the bounding box of every asset in an index.

    OMNI_USER=... OMNI_PASS=... OMNI_KIT_ACCEPT_EULA=YES \
    AirStack/.venv/bin/python scene_gen/tools/nucleus_extents.py \
        --index scene_gen/_plans/sei_coa_usable.tsv \
        --root omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/ \
        --out scene_gen/_plans/sei_coa_extents.tsv

Sizes come out in METRES. That conversion is the whole reason this cannot be a
one-liner: `UsdGeom.GetStageMetersPerUnit` differs per asset — the AEC packs
are authored in centimetres, the Megascans wrappers in metres — so a raw bbox
compared across packs is off by 100x on half of them, silently.

PAYLOADS ARE NOT LOADED. `Usd.Stage.Open(..., load=LoadNone)` plus
`BBoxCache(useExtentsHint=True)` reads the `extentsHint` the exporter already
authored on the default prim, which is why a 150 MB building can be measured
without pulling 150 MB over the network. Only when that comes back empty does
this fall back to composing the stage, and only for files under `--load-max-mb`
— above that the fallback would cost minutes per asset for a number that is
nice to have.

`method` in the output says which path produced each row, so a reader can tell
a cheap authored hint from an expensive computed bound from a miss.
"""

import argparse
import os
import queue
import threading
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--index", required=True)
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--load-max-mb", type=float, default=40.0)
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True})
    from pxr import Usd, UsdGeom

    root = args.root if args.root.endswith("/") else args.root + "/"
    todo = []
    with open(args.index) as f:
        next(f)
        for line in f:
            c = line.rstrip("\n").split("\t")
            path, ext, size = c[0], c[2], int(c[3])
            if ext in ("mdl", "hdr"):        # no geometry to measure
                continue
            todo.append((path, size))
    print(f"[extents] {len(todo)} assets to measure", flush=True)

    q = queue.Queue()
    for t in todo:
        q.put(t)
    out_rows, lock, done = [], threading.Lock(), [0]

    def measure(path, size):
        url = root + path
        try:
            stage = Usd.Stage.Open(url, load=Usd.Stage.LoadNone)
        except Exception as exc:
            return (path, "", "", "", "open-failed", str(exc)[:120])
        if stage is None:
            return (path, "", "", "", "open-failed", "None")
        mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
        prim = stage.GetDefaultPrim() or stage.GetPseudoRoot()
        method = "extents-hint"
        try:
            bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                   [UsdGeom.Tokens.default_],
                                   useExtentsHint=True)
            rng = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            if rng.IsEmpty() and size <= args.load_max_mb * 1e6:
                stage.Load()
                bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                       [UsdGeom.Tokens.default_])
                rng = bc.ComputeWorldBound(prim).ComputeAlignedRange()
                method = "composed"
            if rng.IsEmpty():
                return (path, "", "", "", "no-extent", "")
            mn, mx = rng.GetMin(), rng.GetMax()
            d = [(mx[i] - mn[i]) * mpu for i in range(3)]
            return (path, f"{d[0]:.3f}", f"{d[1]:.3f}", f"{d[2]:.3f}", method, "")
        except Exception as exc:
            return (path, "", "", "", "error", str(exc)[:120])

    def work():
        while True:
            try:
                path, size = q.get(timeout=3.0)
            except queue.Empty:
                return
            try:
                row = measure(path, size)
            except Exception as exc:
                row = (path, "", "", "", "error", str(exc)[:120])
            with lock:
                out_rows.append(row)
                done[0] += 1
                if done[0] % 250 == 0:
                    print(f"[extents] {done[0]}/{len(todo)}", flush=True)
            q.task_done()

    t0 = time.time()
    ts = [threading.Thread(target=work, daemon=True) for _ in range(args.workers)]
    for t in ts:
        t.start()
    for t in ts:
        t.join()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    with open(args.out, "w") as f:
        f.write("path\tsize_x_m\tsize_y_m\tsize_z_m\tmethod\terror\n")
        for r in sorted(out_rows):
            f.write("\t".join(r) + "\n")
    ok = sum(1 for r in out_rows if r[1])
    print(f"[extents] {ok}/{len(out_rows)} measured in {time.time() - t0:.0f}s "
          f"-> {args.out}", flush=True)
    app.close()


if __name__ == "__main__":
    main()
