#!/usr/bin/env python
"""nucleus_index.py — index a Nucleus tree: every file, its type and its size.

    OMNI_USER='$omni-api-token' OMNI_PASS=<jwt> \
    AirStack/.venv/bin/python scene_gen/tools/nucleus_index.py \
        --root omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/ \
        --out scene_gen/_plans/sei_coa_index.tsv

Runs on the HOST. `usd-core` alone cannot open an `omniverse://` URL at all —
there is no resolver for the scheme — but the repo's venv carries a full Isaac
Sim pip install, so `omni.client` is importable once `SimulationApp` has
started. That boot is the fixed ~90 s cost; everything after it is network.

WHY IT IS THREADED. `omni.client.list` is one HTTP round trip per DIRECTORY,
and an asset tree is mostly directories — serial, a few thousand of them at
~60 ms each is half an hour of waiting on latency with the link idle. A pool of
workers pulling from a queue turns that into minutes. The client is used only
through `list`, which is read-only and thread-safe.

MEASURED on `Projects/SEI-COA`: 365,304 files in 2,603 directories, 207 GB,
433 s wall including a ~13 s Kit boot — about 6 dirs/s of round trips against
32 workers. The rows are sorted and written at the END, so an interrupted run
leaves nothing; that is the obvious thing to improve if a tree ever takes long
enough to want resuming.
"""

import argparse
import os
import queue
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--workers", type=int, default=24)
    ap.add_argument("--max-depth", type=int, default=64)
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True})
    import omni.client

    root = args.root if args.root.endswith("/") else args.root + "/"
    q = queue.Queue()
    q.put((root, 0))
    rows, errors = [], []
    lock = threading.Lock()
    seen = set()
    n_dirs = [0]

    def work():
        while True:
            try:
                url, depth = q.get(timeout=2.0)
            except queue.Empty:
                return
            try:
                if depth > args.max_depth:
                    continue
                with lock:
                    if url in seen:
                        continue
                    seen.add(url)
                try:
                    res, entries = omni.client.list(url)
                except Exception as exc:
                    with lock:
                        errors.append(f"{url}\tEXC {exc}")
                    continue
                if res != omni.client.Result.OK:
                    with lock:
                        errors.append(f"{url}\t{res}")
                    continue
                local = []
                for e in entries:
                    name = e.relative_path
                    is_dir = bool(e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN)
                    if is_dir:
                        q.put((f"{url}{name}/", depth + 1))
                    else:
                        ext = os.path.splitext(name)[1].lower().lstrip(".") or "(none)"
                        local.append((f"{url}{name}"[len(root):], name, ext,
                                      int(getattr(e, "size", 0) or 0)))
                with lock:
                    rows.extend(local)
                    n_dirs[0] += 1
                    if n_dirs[0] % 100 == 0:
                        print(f"[nucleus_index] {n_dirs[0]} dirs, "
                              f"{len(rows)} files", flush=True)
            finally:
                q.task_done()

    t0 = time.time()
    threads = [threading.Thread(target=work, daemon=True)
               for _ in range(args.workers)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    rows.sort(key=lambda r: -r[3])
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    with open(args.out, "w") as f:
        f.write("path\tname\text\tsize_bytes\n")
        for path, name, ext, size in rows:
            f.write(f"{path}\t{name}\t{ext}\t{size}\n")
    if errors:
        with open(args.out + ".errors", "w") as f:
            f.write("\n".join(errors) + "\n")
    print(f"[nucleus_index] {len(rows)} files in {n_dirs[0]} dirs, "
          f"{sum(r[3] for r in rows) / 1e9:.2f} GB, "
          f"{len(errors)} errors, {time.time() - t0:.0f}s -> {args.out}",
          flush=True)
    app.close()


if __name__ == "__main__":
    main()
