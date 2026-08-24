#!/usr/bin/env python3
"""categorize_assets.py — what kind of mesh is each asset in a pack?

    cd AirStack
    uv run python scene_gen/tools/categorize_assets.py \
        --asset-pack suburban_nucleus --category buildings.intact --rubble

    uv run python scene_gen/tools/categorize_assets.py \
        --assets-file scene_gen/tools/assets.txt --csv /tmp/report.csv

Answers three questions per asset, the ones `disaster.solids` needs answered
before it can fracture anything — see `disaster/survey.py` for how each is
measured:

    CLOSED       already bounds a volume (watertight / hole-filled /
                 multi-solid), or is it an open shell that needs `thicken()`?
    COMPONENTS   one connected piece, or many separate ones?
    HOLLOW       for whatever IS closed: a thin shell, or real interior
                 volume?

No Isaac Sim needed for `objaverse://` / `airstack://` / plain-path assets —
classification only reads geometry, which usd-core does on the host. Work runs
in a process pool since it is pure CPU. `omniverse://` assets are the
exception (only Kit resolves that scheme) and fall back to a single Kit boot
at the end, the way an offline batch tool has to.
"""

from __future__ import annotations

import argparse
import csv as csv_mod
import multiprocessing as mp
import os
import sys
import time

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import asset_pack as asset_packs                                  # noqa: E402


def _classify_into_queue(queue, idx, entry, hollow_ratio):
    """`_classify`, but posts its result to a Queue instead of returning it.

    The entry point for a per-task `Process` rather than a `Pool` task — see
    `_run_with_timeouts` for why a Pool cannot do this job.
    """
    queue.put(_classify((idx, entry, hollow_ratio)))


def _run_with_timeouts(jobs_by_idx, hollow_ratio, jobs, timeout):
    """Classify every entry in parallel; a hung one is killed, not waited on.

    `multiprocessing.Pool` cannot do this: its workers are long-lived and
    reused, and there is no API to kill only the one task that is stuck
    without tearing down every other in-flight task with it. So this runs
    each classification in its OWN `Process`, bounded to `jobs` concurrent at
    a time by hand, and `.terminate()`s (then `.kill()`s, if that alone does
    not take) anything still running past `timeout` — which is precisely the
    "if any take too long, skip them and mark as such" contract. The one
    realistic way an asset hangs this long is an uncached `objaverse://` uid
    triggering a download-and-Blender-convert that stalls.
    """
    ctx = mp.get_context("spawn")
    pending = list(jobs_by_idx.items())
    running = {}
    results = {}

    def launch(idx, entry):
        q = ctx.Queue()
        proc = ctx.Process(target=_classify_into_queue,
                           args=(q, idx, entry, hollow_ratio))
        proc.start()
        running[idx] = (proc, q, time.time())

    while pending or running:
        while pending and len(running) < jobs:
            idx, entry = pending.pop(0)
            launch(idx, entry)
        time.sleep(0.2)
        for idx in list(running):
            proc, q, t0 = running[idx]
            if not proc.is_alive():
                try:
                    results[idx] = q.get_nowait()
                except Exception:
                    results[idx] = {"idx": idx,
                                    "asset": jobs_by_idx[idx]["asset"],
                                    "error": "worker exited with no result",
                                    "timed_out": False, "seconds": time.time() - t0}
                proc.join()
                del running[idx]
                print(f"[survey]   [{idx:3d}] "
                      f"{results[idx].get('error') or results[idx]['tier']}",
                      flush=True)
            elif time.time() - t0 > timeout:
                proc.terminate()
                proc.join(2)
                if proc.is_alive():
                    proc.kill()
                    proc.join()
                results[idx] = {"idx": idx, "asset": jobs_by_idx[idx]["asset"],
                                "error": None, "timed_out": True,
                                "seconds": timeout}
                print(f"[survey]   [{idx:3d}] TIMED OUT after {timeout:.0f}s",
                      flush=True)
                del running[idx]
    return results


def _classify(job):
    """Load and classify one asset. Runs in a worker process.

    Resolution happens here, not in the parent, for the reason documented in
    `asset_pack`: importing `pxr` before Kit boots stops Kit
    prepending its own USD build.
    """
    idx, entry, hollow_ratio = job
    t0 = time.time()
    try:
        from pxr import Usd, UsdGeom

        from disaster.source import load_source, resolve_asset
        from disaster.survey import classify_mesh

        asset = resolve_asset(entry["asset"], entry["target_size"])
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, "/World")
        src = load_source(
            stage, "/World/_source", asset,
            0.0 if entry["scale"] else entry["target_size"],
            entry["up_axis"], quiet=True, scale=entry["scale"])
        report = classify_mesh(src.vertices, src.faces,
                               hollow_ratio=hollow_ratio)
    except Exception as exc:
        return {"idx": idx, "asset": entry["asset"], "error":
                f"{type(exc).__name__}: {exc}", "seconds": time.time() - t0}
    return {"idx": idx, "asset": entry["asset"], "error": None,
            "timed_out": False, "seconds": time.time() - t0, **report}


def _classify_in_kit(idx, entry, hollow_ratio, timeout):
    """Best-effort per-asset timeout for a call that has to share Kit's
    already-running process.

    `omniverse://` resolution needs Kit's own `omni.client` resolver, so
    unlike every other asset here it cannot run in its own killable
    subprocess (see `_run_with_timeouts`) — there is exactly one Kit process
    for the whole batch, and terminating it to escape one stuck asset would
    also lose every Kit-only asset still queued behind it.

    SIGALRM is the fallback: it raises inside whatever Python code is running
    when the timer fires, which is enough for a genuinely slow-but-eventually-
    -returning network call (the realistic case — an unmirrored Nucleus path
    that stalls trying to reach the real server). It is NOT a hard kill: a
    call stuck entirely inside a C extension that never checks for a pending
    signal would not be interrupted by this. That gap is exactly why Kit-only
    assets are worth keeping visually separate in the report rather than
    silently trusted the same as everything else.
    """
    import signal

    def handler(signum, frame):
        raise TimeoutError(f"exceeded {timeout:.0f}s inside Kit")

    old = signal.signal(signal.SIGALRM, handler)
    signal.alarm(max(int(timeout), 1))
    try:
        return _classify((idx, entry, hollow_ratio))
    except TimeoutError:
        return {"idx": idx, "asset": entry["asset"], "error": None,
                "timed_out": True, "seconds": timeout}
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old)


def _finish(results, entries, args):
    """Assemble rows, print the report, write the CSV, and pick an exit code.

    Pulled out to a function specifically so it can be called BEFORE
    `SimulationApp.close()` in the Kit-only path: that close call can
    terminate the process outright rather than returning to the caller, which
    silently ate the entire report the first time this ran with Kit-only
    assets in the batch — the run looked identical from the outside (clean
    Kit shutdown log, exit code 0) except that no CSV ever appeared.
    """
    rows = []
    for k in sorted(results):
        row = dict(results[k])
        row["source"] = entries[k].get("source", set())
        rows.append(row)
    _report(rows, args.hollow_ratio)
    if args.csv:
        _write_csv(rows, args.csv)
        print(f"[survey] wrote {args.csv}", flush=True)
    if args.md:
        _write_markdown(rows, args.md, args.hollow_ratio)
        print(f"[survey] wrote {args.md}", flush=True)
    return 0 if not any(r["error"] for r in rows) else 2


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--asset-pack", nargs="+", default=[],
                   help="one or more asset packs: names from "
                        "config/asset_packs/ or paths to those YAMLs")
    p.add_argument("--category", nargs="+", default=["buildings"],
                   help="dotted keys under the set's `usds:` to survey "
                        "(default: buildings)")
    p.add_argument("--assets", nargs="*", default=[],
                   help="asset strings, as the asset packs spell them")
    p.add_argument("--assets-file",
                   help="file with one asset per line; '#' starts a comment, "
                        "whole-line or trailing")
    p.add_argument("--limit", type=int, default=0, help="0 = no limit")
    p.add_argument("--target-size", type=float, default=8.0,
                   help="only used for entries with no size of their own")
    p.add_argument("--hollow-ratio", type=float, default=0.05,
                   help="wall-thickness / span below which a closed mesh "
                        "counts as hollow")
    p.add_argument("--jobs", type=int, default=0,
                   help="worker processes (0 = one per core)")
    p.add_argument("--timeout", type=float, default=90.0,
                   help="seconds before an asset is killed and marked "
                        "timed-out, rather than let one asset stall the run")
    p.add_argument("--csv", default="", help="also write a CSV report here")
    p.add_argument("--md", default="",
                   help="also write a readable Markdown index here")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    entries = [{"asset": a, "scale": 0.0, "target_size": args.target_size,
                "up_axis": "z", "source": {"cli"}} for a in args.assets]
    if args.assets_file:
        entries += [{"asset": a, "scale": 0.0, "target_size": args.target_size,
                    "up_axis": "z", "source": {os.path.basename(args.assets_file)}}
                   for a in asset_packs.read_assets_file(args.assets_file)]
    if args.asset_pack:
        by_path = {}
        for set_name in args.asset_pack:
            for e in asset_packs.run_isolated(
                    asset_packs.read_asset_pack,
                    (set_name, args.category, args.target_size)):
                prior = by_path.get(e["asset"])
                # An asset can be named by more than one pack (an urban
                # set extending a base one, say); keep the first entry's
                # resolved size/scale, and just record every source it was
                # found under, so the index says where it came from
                # without listing the same building twice.
                if prior:
                    prior["source"].add(f"{set_name}:{e['category']}")
                else:
                    e["source"] = {f"{set_name}:{e['category']}"}
                    by_path[e["asset"]] = e
        entries += list(by_path.values())
    if args.limit:
        entries = entries[:args.limit]
    if not entries:
        print("[survey] no assets given", flush=True)
        return 1

    parallel_idx, kit_idx = asset_packs.split_kit_only(
        entries, key=lambda e: e["asset"])
    jobs = args.jobs or min(mp.cpu_count(), max(len(parallel_idx), 1))
    print(f"[survey] {len(entries)} assets | {jobs} workers | "
          f"timeout {args.timeout:.0f}s", flush=True)

    results = {}
    if parallel_idx:
        results.update(_run_with_timeouts(
            {i: entries[i] for i in parallel_idx}, args.hollow_ratio, jobs,
            args.timeout))

    if not kit_idx:
        return _finish(results, entries, args)

    print(f"[survey] {len(kit_idx)} omniverse:// asset(s) need Kit — "
          f"booting Isaac Sim once for those", flush=True)
    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True})
    for i, e in enumerate(entries):
        if i in kit_idx and i not in results:
            res = _classify_in_kit(i, e, args.hollow_ratio, args.timeout)
            results[i] = res
            tag = ("TIMED OUT" if res.get("timed_out")
                  else res["error"] or res["tier"])
            print(f"[survey]   [{i:3d}] {tag}", flush=True)
    code = _finish(results, entries, args)   # BEFORE close() — see _finish
    app.close()
    return code


def _stem(asset):
    return asset.rstrip("/").rsplit("/", 1)[-1][:40]


def _report(rows, hollow_ratio):
    # A timed-out row has `error=None` (it never got far enough to raise one)
    # and none of the classification fields, so it is its own bucket rather
    # than falling into "ok" and KeyError-ing on `r["tier"]`.
    timed_out = [r for r in rows if r.get("timed_out")]
    ok = [r for r in rows if not r["error"] and not r.get("timed_out")]
    bad = [r for r in rows if r["error"]]

    print(f"\n{'asset':42s} {'tier':13s} {'parts':>6s} {'closed':>7s} "
          f"{'interior':>9s} {'span':>6s}")
    print("-" * 90)
    for r in sorted(ok, key=lambda r: (r["tier"], -r["span"])):
        parts = f"{r['n_closed']}/{r['n_components']}"
        closed_pct = f"{100 * r['closed_area_frac']:.0f}%"
        if r["hollow"] is None:
            interior = "n/a"
        else:
            interior = "hollow" if r["hollow"] else "filled"
        print(f"{_stem(r['asset']):42s} {r['tier']:13s} {parts:>6s} "
              f"{closed_pct:>7s} {interior:>9s} {r['span']:5.1f}m")
    for r in timed_out:
        print(f"{_stem(r['asset']):42s} TIMED OUT after {r['seconds']:.0f}s")
    for r in bad:
        print(f"{_stem(r['asset']):42s} ERROR  {r['error']}")

    print(f"\n{len(ok)} classified, {len(bad)} failed, "
          f"{len(timed_out)} timed out")
    if not ok:
        return

    from collections import Counter
    tiers = Counter(r["tier"] for r in ok)
    print("by tier:      " + ", ".join(f"{k}={v}" for k, v in tiers.most_common()))

    multi = sum(1 for r in ok if r["n_components"] > 1)
    print(f"by components: single={len(ok) - multi}, multi={multi}")

    tested = [r for r in ok if r["hollow"] is not None]
    hollow = sum(1 for r in tested if r["hollow"])
    untested = len(ok) - len(tested)
    print(f"by interior:   hollow={hollow}, filled={len(tested) - hollow}, "
          f"n/a (nothing closed)={untested}  [threshold {hollow_ratio}]")


def _write_csv(rows, path):
    fields = ["asset", "source", "error", "timed_out", "tier",
              "n_components", "n_closed", "closed_area_frac",
              "wall_thickness_ratio", "hollow", "span", "area", "faces",
              "seconds"]
    with open(path, "w", newline="") as fh:
        w = csv_mod.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            row = dict(r)
            row["source"] = ", ".join(sorted(row.get("source", ()))) \
                if isinstance(row.get("source"), set) else row.get("source", "")
            w.writerow(row)


def _write_markdown(rows, path, hollow_ratio):
    """A readable companion to the CSV: grouped by tier, one table each."""
    from collections import Counter, defaultdict

    ok = [r for r in rows if not r["error"] and not r.get("timed_out")]
    timed_out = [r for r in rows if r.get("timed_out")]
    bad = [r for r in rows if r["error"]]
    tiers = Counter(r["tier"] for r in ok)

    by_tier = defaultdict(list)
    for r in ok:
        by_tier[r["tier"]].append(r)

    lines = ["# Building asset index",
            "",
            f"Generated by `categorize_assets.py` — see `disaster/survey.py` "
            f"for how CLOSED / COMPONENTS / HOLLOW are measured "
            f"(hollow threshold: wall thickness < {hollow_ratio:.0%} of span).",
            "",
            f"**{len(ok)} classified, {len(bad)} unavailable, "
            f"{len(timed_out)} timed out** out of {len(rows)} total.",
            "",
            "| tier | count | meaning |",
            "|---|---|---|",
            f"| `watertight` | {tiers.get('watertight', 0)} | already one closed "
            "surface — cuts directly, no changes |",
            f"| `hole-filled` | {tiers.get('hole-filled', 0)} | small gaps only — "
            "closes with `fill_holes`, outer surface untouched |",
            f"| `multi-solid` | {tiers.get('multi-solid', 0)} | several separate "
            "closed parts — cut as-is, no fusing needed |",
            f"| `open-shell` | {tiers.get('open-shell', 0)} | genuinely open — "
            "needs `solids.thicken()` before it can be cut |",
            ""]

    tier_order = ["watertight", "hole-filled", "multi-solid", "open-shell"]
    for tier in tier_order:
        group = sorted(by_tier.get(tier, ()), key=lambda r: -r["span"])
        if not group:
            continue
        lines += [f"## {tier} ({len(group)})", "",
                  "| asset | source | components | closed | interior | span |",
                  "|---|---|---|---|---|---|"]
        for r in group:
            src = ", ".join(sorted(r.get("source", ()))) \
                if isinstance(r.get("source"), (set, list)) else r.get("source", "")
            interior = "n/a" if r["hollow"] is None \
                else ("hollow" if r["hollow"] else "filled")
            lines.append(
                f"| {_stem(r['asset'])} | {src} | {r['n_closed']}/{r['n_components']} "
                f"| {100 * r['closed_area_frac']:.0f}% | {interior} | {r['span']:.1f}m |")
        lines.append("")

    if timed_out:
        lines += [f"## Timed out ({len(timed_out)})", "",
                  "Killed after exceeding the run's `--timeout`; not classified.",
                  "", "| asset | source | after |", "|---|---|---|"]
        for r in timed_out:
            src = ", ".join(sorted(r.get("source", ()))) \
                if isinstance(r.get("source"), (set, list)) else r.get("source", "")
            lines.append(f"| {_stem(r['asset'])} | {src} | {r['seconds']:.0f}s |")
        lines.append("")

    if bad:
        lines += [f"## Unavailable ({len(bad)})", "",
                  "Referenced by a pack but the geometry could not be loaded — "
                  "usually a Nucleus path with no local mirror.", "",
                  "| asset | source | error |", "|---|---|---|"]
        for r in bad:
            src = ", ".join(sorted(r.get("source", ()))) \
                if isinstance(r.get("source"), (set, list)) else r.get("source", "")
            lines.append(f"| {_stem(r['asset'])} | {src} | {r['error']} |")
        lines.append("")

    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())


