#!/usr/bin/env python3
"""edit_env.py — surgical, auditable edits to the repo's `.env`.

Written for the LIVE RAVEN test (`scripts/raven_live/flight_checklist.md`).
Nothing here starts, stops or talks to a container; it only rewrites one
text file, always prints a unified diff of what it did, and refuses to write
if the file changed on disk since the caller last looked at it.

WHY THE md5 GUARD. Other agent sessions edit and commit this repo
concurrently (AGENTS.md / the user's own standing note). `.env` is the one
file every one of them wants, and a lost update there does not fail loudly —
it flies the wrong scene. So: read the md5, decide what to set, then pass
that md5 back with `--expect-md5`. If anything moved in between, this
refuses and prints both md5s instead of clobbering.

USAGE (always from the repo root)

    # what is it right now
    scripts/raven_live/edit_env.py --print-md5

    # set keys (snapshot is automatic unless --no-snapshot)
    scripts/raven_live/edit_env.py --expect-md5 <md5> \
        --set NUM_ROBOTS=1 --set 'SPAWN_CONFIGS=[{"x_m": 1.0, ...}]'

    # see what WOULD change, touch nothing
    scripts/raven_live/edit_env.py --dry-run --set NUM_ROBOTS=2

    # put it back
    scripts/raven_live/edit_env.py --restore scripts/raven_live/env_snapshots/env_20260902_014500.bak

RULES

  * A key that already exists UNCOMMENTED is edited IN PLACE, keeping its
    line position and any trailing `# comment` on that line.
  * A key that does not exist (or exists only COMMENTED OUT) is APPENDED
    inside a marked block at the end of the file:

        # >>> raven_live managed block >>>
        KEY="value"
        # <<< raven_live managed block <<<

    Docker Compose parses `.env` top-to-bottom and the LAST definition of a
    key wins, so an appended key also beats any commented-out template
    earlier in the file — which is exactly what the RAVEN block in `.env`
    (`#RAYFRONTS_MODE="shared"` and friends) needs.
  * Quoting: `"..."` normally; `'...'` when the value contains a double
    quote and no single quote (this is how `.env` already writes
    SPAWN_CONFIGS). A value containing BOTH is refused rather than guessed
    at — `.env` is not a shell and has no escape that both docker compose
    and `set -a; . .env` agree on.
  * `--set KEY=` (empty value) writes `KEY=""`. That is NOT the same as
    unsetting: several compose entries in this repo pass BARE NAMES
    (`- RAYFRONTS_MODE`) precisely because an empty string beats a
    `$(env VAR default)` launch default. Use `--unset KEY` to comment a key
    out instead.
"""

import argparse
import difflib
import hashlib
import os
import re
import shutil
import sys
from datetime import datetime

BEGIN = "# >>> raven_live managed block >>>"
END = "# <<< raven_live managed block <<<"

_KEY_RE = re.compile(r"^(\s*)([A-Za-z_][A-Za-z0-9_]*)\s*=(.*)$")


def md5_of(path):
    h = hashlib.md5()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def quote(value):
    """`.env` quoting for one value. Raises on the ambiguous case."""
    if '"' in value and "'" in value:
        raise SystemExit(
            "edit_env: value contains BOTH a single and a double quote, which "
            "`.env` cannot express unambiguously for docker compose and for "
            "`set -a; . .env` at the same time:\n  {0!r}".format(value))
    if '"' in value:
        return "'" + value + "'"
    return '"' + value + '"'


def split_trailing_comment(rest):
    """`'\"x\"   # why' -> ('\"x\"', '   # why')`.

    Only splits on a `#` that is OUTSIDE quotes, so a `#` inside a JSON value
    (a colour, a fragment) is left alone.
    """
    q = None
    for i, ch in enumerate(rest):
        if q:
            if ch == q:
                q = None
        elif ch in "\"'":
            q = ch
        elif ch == "#":
            val = rest[:i]
            stripped = val.rstrip()
            # Keep the ORIGINAL run of spaces before the `#` — dropping it
            # produces `NUM_ROBOTS="1"# Number of...`, which parses but makes
            # the diff look like the comment moved.
            gap = val[len(stripped):] or "  "
            return stripped, gap + rest[i:]
    return rest.rstrip(), ""


def parse_sets(pairs):
    out = []
    for p in pairs:
        if "=" not in p:
            raise SystemExit("edit_env: --set needs KEY=VALUE, got {0!r}".format(p))
        k, v = p.split("=", 1)
        k = k.strip()
        if not re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", k):
            raise SystemExit("edit_env: {0!r} is not a valid env key".format(k))
        out.append((k, v))
    return out


def apply_sets(lines, sets, unsets):
    """Return the new list of lines. `lines` keep their trailing newlines."""
    lines = list(lines)

    # --- comment out anything in --unset -----------------------------------
    for key in unsets:
        for i, ln in enumerate(lines):
            m = _KEY_RE.match(ln.rstrip("\n"))
            if m and m.group(2) == key:
                lines[i] = "#" + ln

    # --- in-place updates ---------------------------------------------------
    pending = []
    for key, value in sets:
        hits = [i for i, ln in enumerate(lines)
                if (lambda m: m and m.group(2) == key)(_KEY_RE.match(ln.rstrip("\n")))]
        # Ignore hits inside the managed block for the "does it already exist"
        # question — we rewrite that block wholesale below.
        blk = block_span(lines)
        if blk:
            hits = [i for i in hits if not (blk[0] <= i < blk[1])]
        if not hits:
            pending.append((key, value))
            continue
        if len(hits) > 1:
            print("edit_env: WARNING: {0} is defined {1} times outside the "
                  "managed block; updating the LAST one (line {2}) because "
                  "that is the one docker compose uses"
                  .format(key, len(hits), hits[-1] + 1), file=sys.stderr)
        i = hits[-1]
        m = _KEY_RE.match(lines[i].rstrip("\n"))
        _, comment = split_trailing_comment(m.group(3))
        lines[i] = "{0}{1}={2}{3}\n".format(m.group(1), key, quote(value), comment)

    # --- the managed block --------------------------------------------------
    blk = block_span(lines)
    existing = {}
    if blk:
        for ln in lines[blk[0] + 1:blk[1] - 1]:
            m = _KEY_RE.match(ln.rstrip("\n"))
            if m:
                existing[m.group(2)] = ln
        # A key we just updated in place must not linger in the block too.
        for key, _v in sets:
            existing.pop(key, None)
        for key in unsets:
            existing.pop(key, None)

    for key, value in pending:
        existing[key] = "{0}={1}\n".format(key, quote(value))

    if existing:
        body = ([BEGIN + "\n",
                 "# Written by scripts/raven_live/edit_env.py — safe to delete "
                 "or restore from a snapshot.\n"]
                + [existing[k] for k in sorted(existing)]
                + [END + "\n"])
        if blk:
            lines[blk[0]:blk[1]] = body
        else:
            if lines and not lines[-1].endswith("\n"):
                lines[-1] += "\n"
            lines += ["\n"] + body
    elif blk:
        lines[blk[0]:blk[1]] = []

    return lines


def block_span(lines):
    """`(start_idx, end_idx_exclusive)` of the managed block, or None."""
    try:
        s = next(i for i, ln in enumerate(lines) if ln.rstrip("\n") == BEGIN)
        e = next(i for i, ln in enumerate(lines) if ln.rstrip("\n") == END)
    except StopIteration:
        return None
    if e < s:
        raise SystemExit("edit_env: managed block markers are out of order in "
                         "the env file — fix it by hand")
    return (s, e + 1)


def show_diff(before, after, path):
    diff = list(difflib.unified_diff(before, after,
                                     fromfile=path + " (before)",
                                     tofile=path + " (after)", n=2))
    if not diff:
        print("edit_env: no change")
        return False
    sys.stdout.writelines(diff)
    if diff and not diff[-1].endswith("\n"):
        print()
    return True


def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.normpath(os.path.join(here, "..", ".."))

    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--env-file", default=os.path.join(repo, ".env"))
    ap.add_argument("--set", dest="sets", action="append", default=[],
                    metavar="KEY=VALUE",
                    help="set a key (repeatable)")
    ap.add_argument("--unset", dest="unsets", action="append", default=[],
                    metavar="KEY",
                    help="comment a key out (repeatable)")
    ap.add_argument("--expect-md5", default=None,
                    help="refuse to write unless the file's md5 matches this "
                         "(concurrent-session guard)")
    ap.add_argument("--restore", default=None, metavar="SNAPSHOT",
                    help="copy SNAPSHOT back over the env file")
    ap.add_argument("--snapshot-dir",
                    default=os.path.join(here, "env_snapshots"))
    ap.add_argument("--no-snapshot", action="store_true",
                    help="do not write a pre-edit snapshot")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the diff and the resulting md5; write nothing")
    ap.add_argument("--print-md5", action="store_true",
                    help="print the env file's md5 and exit 0")
    args = ap.parse_args(argv)

    path = os.path.abspath(args.env_file)
    if not os.path.isfile(path):
        raise SystemExit("edit_env: no such file: {0}".format(path))

    cur = md5_of(path)
    if args.print_md5:
        print(cur)
        return 0

    if args.expect_md5 and args.expect_md5.strip().lower() != cur:
        raise SystemExit(
            "edit_env: REFUSING — {0} changed on disk since you read it.\n"
            "  expected md5 {1}\n"
            "  actual   md5 {2}\n"
            "Another session (or a hand edit) touched it. Re-read the file, "
            "redo your decision, and pass the NEW md5."
            .format(path, args.expect_md5.strip().lower(), cur))

    with open(path, "r", encoding="utf-8") as fh:
        before = fh.readlines()

    if args.restore:
        snap = os.path.abspath(args.restore)
        if not os.path.isfile(snap):
            raise SystemExit("edit_env: no such snapshot: {0}".format(snap))
        with open(snap, "r", encoding="utf-8") as fh:
            after = fh.readlines()
        if args.sets or args.unsets:
            raise SystemExit("edit_env: --restore and --set/--unset are "
                             "mutually exclusive; do one, then the other")
    else:
        if not args.sets and not args.unsets:
            raise SystemExit("edit_env: nothing to do — pass --set/--unset, "
                             "--restore or --print-md5")
        after = apply_sets(before, parse_sets(args.sets), args.unsets)

    changed = show_diff(before, after, path)
    if args.dry_run:
        print("edit_env: --dry-run, nothing written (current md5 {0})".format(cur))
        return 0
    if not changed:
        return 0

    if not args.no_snapshot:
        os.makedirs(args.snapshot_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        snap = os.path.join(args.snapshot_dir, "env_{0}.bak".format(stamp))
        shutil.copy2(path, snap)
        print("edit_env: snapshot -> {0}".format(snap))

    tmp = path + ".raven_live.tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.writelines(after)
    os.replace(tmp, path)
    print("edit_env: wrote {0}  (new md5 {1})".format(path, md5_of(path)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
