#!/usr/bin/env python3
"""dataset_upload.py — push `final_disaster_dataset/` cells to Nucleus.

WHERE THIS GOES, AND HOW IT WAS FOUND
--------------------------------------
`omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/
final_disaster_dataset` — LISTED LIVE (2026-09-01) via bare `omni.client`
inside the running `isaac-sim` container, no `SimulationApp`:

    Projects/SEI-COA/final_disaster_dataset/
      Fire/
        Suburban/
          level_1/{1,2,3,4,5}/   GT_hints.json  GT_people.json
                                 build_stats.json  fire_suburban_lvl1_<k>.usd
          level_2/  level_3/
      Tornado/
        Suburban/

This confirms `freeze-dataset-state`'s account (uploaded 2026-08-29) still
holds and gives the exact directory/file shape to mirror: the contract's
`<Disaster>/<Locale>/level_<n>/<k>/` layout, one `.usd` +
`GT_hints.json` + `GT_people.json` + `build_stats.json` per cell.
**`snaps/` is NOT on Nucleus** (571 MB of human-review captures nothing at
run time reads — matching `freeze-dataset-state`'s own note) and neither is
`Materials/` yet (the collect step is off; nothing to localise). There is no
`Fire/Urban` yet — this tool is what creates it, mirroring the exact same
contract, one cell at a time as urban-fire cells are frozen.

Same discovery script, generalised: `freeze-dataset-state`'s own inline
`omni.client` uploader (a `docker exec ... <<PY` heredoc) proved the pattern
this file packages as a real, dry-runnable, verifiable tool — same
`stat`-by-size dedup, same `CopyBehavior.OVERWRITE` (a re-upload without it
returns `ERROR_ALREADY_EXISTS` and writes nothing), same `snaps/` prune.

RUNS UNDER KIT'S PYTHON, INSIDE THE isaac-sim CONTAINER (or an OSMO pod's
`isaac-sim`/`isaac-sim-livestream` container) — this is a bare-`omni.client`
tool, no `SimulationApp`, no GPU, safe to run beside a live sim (see
run-isaac-sim-launcher's copy recipe, section 6):

    docker exec isaac-sim bash -c '
      EXT=/isaac-sim/kit/extscore/omni.client.lib
      LIB=$(ls -d /isaac-sim/extscache/omni.client-*/bin | head -1)
      LD_LIBRARY_PATH="/isaac-sim/kit:$LIB:$LD_LIBRARY_PATH" \\
      PYTHONPATH="$EXT" /isaac-sim/kit/python/bin/python3 -u \\
      /isaac-sim/AirStack/scene_gen/tools/dataset_upload.py \\
      --only Fire/Urban --dry-run'

    # for real, once the dry run looks right:
    docker exec isaac-sim bash -c '... dataset_upload.py --only Fire/Urban'

USAGE
-----
    dataset_upload.py [--local DIR] [--remote URL] [--only SUBPATH]
                       [--include-snaps] [--include-tmp] [--no-verify]
                       [--dry-run]

    --local        local dataset root. Default `/isaac-sim/final_disaster_
                   dataset` (the in-container bind mount every compose block
                   uses; pass the host path instead if run outside a
                   container).
    --remote       Nucleus root. Default the URL discovered above.
    --only         restrict to one subtree, e.g. `Fire/Urban` or
                   `Fire/Urban/level_1/1` — walks only that slice, so a
                   fresh cell does not re-stat the whole multi-hundred-cell
                   dataset (the size-skip dedup makes a full re-walk SAFE,
                   just slower).
    --include-snaps  also upload `snaps/` (571 MB/cell at the suburban
                   scale — off by default, matching the shipped precedent;
                   the LOCAL puller (`dataset_pull.sh`) is where the user
                   wants full copies including snaps, not Nucleus).
    --include-tmp  also upload any leftover `_freeze_tmp/` (a failed/
                   interrupted freeze's scratch dir — never wanted; off by
                   default and normally empty, kept as an explicit opt-in
                   rather than silently skipped forever).
    --no-verify    skip the post-upload listing comparison.
    --dry-run      stat only; prints what WOULD move without copying.
"""
import argparse
import asyncio
import os
import sys

DEFAULT_LOCAL = "/isaac-sim/final_disaster_dataset"
DEFAULT_REMOTE = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/"
                  "Projects/SEI-COA/final_disaster_dataset")

#: directories that never travel to Nucleus. `snaps/` and `_freeze_tmp/` are
#: opt-in (see the module docstring); anything starting with `.` (stray
#: editor/OS files) is always skipped.
_ALWAYS_PRUNE = ()


def _iter_local(local_root, only, include_snaps, include_tmp):
    """`(rel_path, abs_path, size)` for every file under `local_root`
    (optionally restricted to the `only` subtree), snaps/_freeze_tmp pruned
    unless opted in."""
    root = os.path.join(local_root, only) if only else local_root
    root = os.path.normpath(root)
    if not os.path.isdir(root):
        raise SystemExit("dataset_upload: no such local directory: "
                         "{0}".format(root))
    prune = set()
    if not include_snaps:
        prune.add("snaps")
    if not include_tmp:
        prune.add("_freeze_tmp")
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = sorted(d for d in dirnames
                             if d not in prune and not d.startswith("."))
        rel_dir = os.path.relpath(dirpath, local_root)
        for name in sorted(filenames):
            if name.startswith("."):
                continue
            abspath = os.path.join(dirpath, name)
            rel = name if rel_dir == "." else os.path.join(rel_dir, name)
            yield rel.replace(os.sep, "/"), abspath, os.path.getsize(abspath)


def _remote_join(remote_root, rel):
    return remote_root.rstrip("/") + "/" + rel


async def _bounded_async(items, workers, operation):
    semaphore = asyncio.Semaphore(workers)

    async def one(item):
        async with semaphore:
            return await operation(item)

    return await asyncio.gather(*(one(item) for item in items))


def upload(local_root, remote_root, only, include_snaps, include_tmp,
          dry_run, verbose=True, workers=1):
    """CALLER'S JOB to bracket this with `omni.client.initialize()`/
    `shutdown()` — see `_client_session` and the module docstring's "one
    `initialize()`/`shutdown()` per process" note. This function no longer
    calls either itself."""
    import omni.client as c
    have = need = copied = failed = 0
    errors = []
    planned = []
    files = list(_iter_local(local_root, only, include_snaps, include_tmp))
    remote_sizes = (dict(_iter_remote(remote_root, None))
                    if workers > 1 else None)
    for rel, abspath, size in files:
        dst = _remote_join(remote_root, rel)
        if workers > 1:
            matches = remote_sizes.get(rel) == size
        else:
            r, entry = c.stat(dst)
            matches = r == c.Result.OK and entry.size == size
        if matches:
            have += 1
            continue
        need += 1
        planned.append((rel, abspath, dst, size))
    if verbose:
        print("[dataset_upload] {0}: {1} already on Nucleus (same "
              "size), {2} to copy{3}".format(
                  local_root, have, need,
                  "  [DRY RUN]" if dry_run else ""))
    if not dry_run and workers > 1:
        async def _copy(item):
            return await c.copy_async(item[1], item[2],
                                      behavior=c.CopyBehavior.OVERWRITE)
        copy_results = asyncio.run(_bounded_async(planned, workers, _copy))
    else:
        copy_results = [None] * len(planned)
    for index, (rel, abspath, dst, size) in enumerate(planned):
        if dry_run:
            print("  WOULD COPY  {0}  ({1:.1f} MB)".format(
                rel, size / 1e6))
            continue
        r = (copy_results[index] if workers > 1 else
             c.copy(abspath, dst, behavior=c.CopyBehavior.OVERWRITE))
        if str(r) == "Result.OK":
            copied += 1
            if verbose and workers <= 1:
                print("  copied  {0}  ({1:.1f} MB)".format(
                    rel, size / 1e6))
            elif verbose and copied % 250 == 0:
                print("  copied  {0}/{1} file(s)".format(
                    copied, len(planned)))
        else:
            failed += 1
            errors.append((rel, str(r)))
            print("  *** FAILED  {0}  [{1}]".format(rel, r))
    print("[dataset_upload] {0} copied, {1} failed, {2} already "
          "present".format(copied, failed, have))
    return {"have": have, "need": need, "copied": copied,
           "failed": failed, "errors": errors, "planned": planned,
           "dry_run": dry_run}


def verify(local_root, remote_root, only, include_snaps, include_tmp,
          verbose=True, workers=1):
    """Re-list every planned file on Nucleus and confirm size matches the
    local copy — the "verify listing after" step the task asks for. Returns
    the list of mismatches (empty = clean). Same calling convention as
    `upload`: no `initialize()`/`shutdown()` of its own."""
    import omni.client as c
    bad = []
    n = 0
    files = list(_iter_local(local_root, only, include_snaps, include_tmp))
    remote_sizes = (dict(_iter_remote(remote_root, None))
                    if workers > 1 else None)
    for rel, abspath, size in files:
        n += 1
        if workers > 1:
            remote_size = remote_sizes.get(rel)
            if remote_size is None:
                bad.append((rel, "NOT FOUND on Nucleus"))
                continue
        else:
            r, entry = c.stat(_remote_join(remote_root, rel))
            if r != c.Result.OK:
                bad.append((rel, "NOT FOUND on Nucleus ({0})".format(r)))
                continue
            remote_size = entry.size
        if remote_size != size:
            bad.append((rel, "size mismatch: local {0} vs remote {1}"
                       .format(size, remote_size)))
    if verbose:
        print("[dataset_upload] verify: {0}/{1} file(s) confirmed on "
              "Nucleus with a matching size".format(n - len(bad), n))
        for rel, why in bad[:20]:
            print("  MISMATCH  {0}  -- {1}".format(rel, why))
    return bad


def native_tree_upload(local_root, remote_root, dry_run):
    """Copy one complete directory through omni.client's native tree path."""
    import omni.client as c
    if dry_run:
        print("[dataset_upload] WOULD NATIVE-COPY {0} -> {1}".format(
            local_root, remote_root))
        return True
    result = c.copy(local_root, remote_root,
                    behavior=c.CopyBehavior.OVERWRITE)
    print("[dataset_upload] native tree copy: {0}".format(result))
    return result == c.Result.OK


# ---------------------------------------------------------------------------
# the reverse direction — Nucleus -> local. `dataset_pull.sh --from-nucleus`
# shells out to this (`--download`, run inside a container that can reach
# Nucleus) rather than duplicating the omni.client plumbing in a second
# place; `dataset_pull.sh`'s OTHER mode (tar over an OSMO ssh tunnel) never
# touches this file at all — that mode does not go through Nucleus.
# ---------------------------------------------------------------------------
def _iter_remote(remote_root, only):
    """`(rel_path, size)` for every FILE under `remote_root` (optionally
    restricted to `only`), walked with plain `omni.client.list` recursion —
    there is no Nucleus equivalent of `os.walk`, so this does it by hand,
    one directory at a time, breadth need not be bounded since a dataset
    cell count is in the hundreds, not millions."""
    import omni.client as c
    base = _remote_join(remote_root, only) if only else remote_root
    base = base.rstrip("/")
    stack = [""]
    while stack:
        rel_dir = stack.pop()
        path = base if not rel_dir else base + "/" + rel_dir
        r, entries = c.list(path)
        if r != c.Result.OK:
            print("  *** could not list {0} [{1}]".format(path, r))
            continue
        for e in entries:
            rel = e.relative_path if not rel_dir else rel_dir + "/" + e.relative_path
            if e.flags & c.ItemFlags.CAN_HAVE_CHILDREN:
                stack.append(rel)
            else:
                full_rel = ((only.rstrip("/") + "/" + rel) if only else rel)
                yield full_rel, int(e.size)


def download(local_root, remote_root, only, dry_run, verbose=True):
    """Pull every file under `remote_root` (optionally `only`) into
    `local_root`, same size-skip dedup as `upload`. `snaps/` is included —
    Nucleus never carries it (see the module docstring), so this is a no-op
    for that directory today; kept unconditional (unlike `upload`'s
    `--include-snaps` gate) because there is nothing on the remote side to
    exclude. Same calling convention as `upload`/`verify`: no
    `initialize()`/`shutdown()` of its own."""
    import omni.client as c
    have = need = copied = failed = 0
    for rel, size in _iter_remote(remote_root, only):
        dst = os.path.join(local_root, rel.replace("/", os.sep))
        if os.path.isfile(dst) and os.path.getsize(dst) == size:
            have += 1
            continue
        need += 1
        if dry_run:
            print("  WOULD PULL  {0}  ({1:.1f} MB)".format(
                rel, size / 1e6))
            continue
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        src = _remote_join(remote_root, rel)
        r = c.copy(src, dst, behavior=c.CopyBehavior.OVERWRITE)
        if str(r) == "Result.OK":
            copied += 1
            if verbose:
                print("  pulled  {0}  ({1:.1f} MB)".format(
                    rel, size / 1e6))
        else:
            failed += 1
            print("  *** FAILED  {0}  [{1}]".format(rel, r))
    print("[dataset_upload] download: {0} already local (same size), "
          "{1} pulled, {2} failed{3}".format(
              have, copied, failed, "  [DRY RUN]" if dry_run else ""))
    return {"have": have, "need": need, "copied": copied,
           "failed": failed, "dry_run": dry_run}


class _client_session(object):
    """ONE `omni.client.initialize()`/`shutdown()` bracket for the whole
    process. MEASURED (2026-09-01, against the live Nucleus host): calling
    `shutdown()` and then `initialize()` again in the SAME process — which
    `upload()` immediately followed by `verify()` used to do, each with its
    own bracket — hangs forever on the second `initialize()` (or somewhere
    very close to it: the process produces no further output, uses ~0% CPU,
    and does not respond to SIGTERM, only SIGKILL). `omni.client` is not
    documented as safe to re-initialize after a shutdown in the same
    process, and empirically it is not. Every entry point below now opens
    exactly one bracket per run, even when it calls more than one of
    `upload`/`verify`/`download`."""

    def __enter__(self):
        import omni.client as c
        c.initialize()
        return self

    def __exit__(self, *exc):
        import omni.client as c
        c.shutdown()
        return False


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--local", default=DEFAULT_LOCAL)
    ap.add_argument("--remote", default=DEFAULT_REMOTE)
    ap.add_argument("--only", default=None,
                    help="restrict to one subtree, e.g. Fire/Urban")
    ap.add_argument("--include-snaps", action="store_true")
    ap.add_argument("--include-tmp", action="store_true")
    ap.add_argument("--no-verify", action="store_true")
    ap.add_argument("--workers", type=int,
                    default=int(os.environ.get("DATASET_UPLOAD_WORKERS", "1")),
                    help="bounded concurrent Nucleus requests (default 1)")
    ap.add_argument("--native-tree-copy", action="store_true",
                    help="use one recursive omni.client copy, then verify")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--download", action="store_true",
                    help="reverse direction: Nucleus -> --local (the "
                         "`dataset_pull.sh --from-nucleus` backend)")
    args = ap.parse_args(argv)
    if args.workers < 1:
        ap.error("--workers must be at least 1")

    print("[dataset_upload] local  {0}".format(args.local))
    print("[dataset_upload] remote {0}".format(args.remote))
    if args.only:
        print("[dataset_upload] only   {0}".format(args.only))

    # ONE initialize()/shutdown() bracket for everything this run does — see
    # `_client_session`'s own docstring for why a second bracket in the same
    # process hangs.
    with _client_session():
        if args.download:
            result = download(args.local, args.remote, args.only,
                              args.dry_run)
            return 1 if result["failed"] else 0

        if args.native_tree_copy:
            if args.only:
                ap.error("--native-tree-copy does not support --only")
            if not args.include_snaps or args.include_tmp:
                ap.error("--native-tree-copy requires --include-snaps and "
                         "cannot include temporary scratch")
            if not native_tree_upload(args.local, args.remote, args.dry_run):
                return 1
            if args.dry_run or args.no_verify:
                return 0
            bad = verify(args.local, args.remote, None, True, False,
                         workers=max(args.workers, 2))
            return 1 if bad else 0

        result = upload(args.local, args.remote, args.only,
                        args.include_snaps, args.include_tmp, args.dry_run,
                        workers=args.workers)
        if result["failed"]:
            return 1
        if args.dry_run or args.no_verify:
            return 0
        bad = verify(args.local, args.remote, args.only, args.include_snaps,
                    args.include_tmp, workers=args.workers)
        return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
