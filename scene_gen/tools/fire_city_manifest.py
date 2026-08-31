#!/usr/bin/env python3
"""fire_city_manifest — classify a CITY fire manifest's records as
HAVE / STALE / NEED against a bake output directory, and print the
`fire_bake.sh` entry string + cache stem for each one.

    python3 fire_city_manifest.py <manifest.json> [--out-dir DIR] [--verify]

`scene_gen/tools/fire_city_bake.sh` (`urban_fire_city_plan.md` sec 3, work
item #6) is the intended caller: it reads this tool's plain-text output to
decide which records to skip and which to bake, using the exact same
`IFS=':' read` idiom `fire_bake.sh` already uses to turn one entry string
into `KIND NAME LEVEL ORIGIN SIDES SEED`.

PURE PYTHON, NO `pxr` AT IMPORT TIME OR IN THE DEFAULT PATH. The only
sibling module this imports is `disaster.fire_bake`, whose own top-level
imports are `hashlib, json, math, os` — safe on a bare host `python3`, no
Isaac Sim, no Nucleus, no venv. `--verify` is the one path that needs `pxr`
(`fire_bake.verify_export` opens the USD), and it NEVER runs in this
process: it shells out to `docker exec $CONTAINER ... usd_python.sh
scene_gen/tests/test_fire_bake.py --verify <paths>` — the exact mechanism
`fire_bake.sh --verify-only` already uses — and falls back to "trust
existence" if that call cannot be made at all (container down, docker
missing), rather than ever failing the whole classification over it.

MANIFEST FORMAT. Either:
  * a bare JSON array of `urban_fire_city.damaged_manifest` records (see
    that module's docstring for the schema: each record has at least
    `kind` (`"gac"|"dtc"|"kit"`), `level`, `seed`, `origin`, `sides`, and
    `asset` (gac/dtc) or `style` (kit) — plus the static placement facts
    `cell`/`x`/`y`/`yaw_deg`/`z`/`typology`/`usd` this tool passes through
    under `city` for `--write-city-json`), or
  * a JSON OBJECT with that array under one of `manifest` / `records` /
    `buildings` / `entries`, and OPTIONALLY a top-level `"seed"` — the
    CITY/layout seed the whole manifest was solved at (NOT any one
    record's own per-building bake `seed`). `fire_city_bake.sh` needs this
    for its `FB_OUT=<base>/city_<seed>/` naming (plan sec 3); see
    `--print-seed`.

`entry_string()` below mirrors `disaster.urban_fire_city.entry_string`'s
own algorithm exactly (`kind:name:level:origin:sides:seed`, all six fields
always written so `fire_bake.parse_entry` never falls back to its own
default-seed formula) but is kept as a LOCAL, self-contained copy rather
than an import: `urban_fire_city.py` is still under active development
elsewhere in this repo as of 2026-08-30 (`urban_fire_spread`/preset/dry-run
work items of the same plan), and this tool's only hard dependency should
be the stable, already-shipped `fire_bake.py` half — not a module that can
change its import graph out from under a classification tool someone is
running to decide whether to spend GPU time.

STATUS
  HAVE    both `<stem>.usd` and `<stem>.json` exist under `--out-dir`.
  STALE   both exist but `--verify` was passed and the container-side
          `verify_export` came back NOT ok (or its sidecar could not even
          be read).
  NEED    either file is missing. With no `--verify`, existence is
          trusted outright — "otherwise trust existence" (this tool's own
          brief) — so a HAVE here is never independently re-opened.
  ERROR   the record itself could not be turned into an entry string
          (missing `kind`/`seed`/name, or an unknown `kind`) — reported
          with the reason, never silently skipped.

`--out-dir` must be a path THIS PROCESS can see directly — the container's
`$FB_OUT` if this happens to run inside the container, or its host-mounted
equivalent (`$HOME/docker/isaac-sim/cache/main/fire_bakes/...`, per
`simulation/isaac-sim/docker/docker-compose.yaml`'s
`.../cache/main:/isaac-sim/.cache` bind mount) if it runs on the host, the
same way `fire_bake.sh` itself keeps a HOSTLOGDIR/CLOGDIR pair for the
identical reason. This tool does no host/container translation itself —
`fire_city_bake.sh` does that once, up front, and hands this tool whichever
side of the mount it is actually running on.
"""

import argparse
import json
import os
import re
import subprocess
import sys

# ---------------------------------------------------------------------------
# scene_gen on sys.path -- see urban_fire_city.py / kit_substitute.py's
# identical helper. This file lives at scene_gen/tools/, so its grandparent
# is scene_gen itself.
# ---------------------------------------------------------------------------
_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _ensure_scene_gen_on_path():
    if _SCENE_GEN not in sys.path:
        sys.path.insert(0, _SCENE_GEN)


_ensure_scene_gen_on_path()
from disaster import fire_bake as fb  # noqa: E402  (needs sys.path fixed up first)

#: the static placement facts a city-cell record carries, per
#: `urban_fire_city.burnable()`'s own record shape -- exactly what
#: `urban_fire_city_plan.md` sec 3's sidecar `extra={"city": {...}}` wants,
#: plus `orig_usd` (the intact city asset this bake replaces).
_CITY_KEYS = ("cell", "x", "y", "yaw_deg", "z", "typology")


def _safe(s):
    """Filesystem-safe token -- same rule `fire_bake._safe` uses, kept as a
    local copy since that one is a private module symbol."""
    return "".join(c if (c.isalnum() or c in "._-") else "_" for c in str(s))


# ---------------------------------------------------------------------------
# The manifest
# ---------------------------------------------------------------------------
def load_manifest(path):
    """`(top_seed, [record, ...])`. Raises `ValueError` on a shape this
    tool does not recognise -- never silently treats the wrong thing as an
    empty manifest."""
    with open(path) as fh:
        data = json.load(fh)
    if isinstance(data, list):
        return None, data
    if isinstance(data, dict):
        top_seed = data.get("seed")
        for key in ("manifest", "records", "buildings", "entries"):
            val = data.get(key)
            if isinstance(val, list):
                return top_seed, val
        raise ValueError(
            "manifest object at {0!r} has no list under manifest/records/"
            "buildings/entries -- top-level keys: {1}".format(
                path, sorted(data.keys())))
    raise ValueError(
        "manifest JSON at {0!r} must be a list or an object, got {1}"
        .format(path, type(data).__name__))


def resolve_city_seed(manifest_path, top_seed):
    """The manifest's top-level `"seed"` if it has one, else the trailing
    digits of its own filename (`fire_city_1013.json` -> `"1013"`), else the
    sanitised filename stem itself -- `fire_city_bake.sh` always gets SOME
    non-empty, filesystem-safe token to build `city_<seed>/` from."""
    if top_seed is not None and str(top_seed).strip():
        return _safe(top_seed)
    stem = os.path.splitext(os.path.basename(manifest_path))[0]
    m = re.search(r"(\d+)$", stem)
    return m.group(1) if m else _safe(stem)


# ---------------------------------------------------------------------------
# One record -> an entry string -> a cache stem
# ---------------------------------------------------------------------------
def entry_string(record):
    """`kind:name:level:origin:sides:seed` for one manifest record --
    mirrors `disaster.urban_fire_city.entry_string` exactly (see the module
    docstring for why this is a local copy, not an import). Raises
    `ValueError` with a human reason on anything that cannot be turned into
    a valid `fire_bake.parse_entry` text."""
    kind = str(record.get("kind") or "").lower()
    if kind not in fb.KINDS:
        raise ValueError("unknown or missing kind {0!r} (expected one of "
                          "{1})".format(record.get("kind"), "/".join(fb.KINDS)))
    name = record.get("asset") if kind in fb.SLICED_KINDS else record.get("style")
    if not name:
        raise ValueError("no {0} name on a {1!r} record"
                          .format("asset" if kind in fb.SLICED_KINDS else "style",
                                  kind))
    seed = record.get("seed")
    if seed is None:
        raise ValueError("record has no seed")
    level = record.get("level") or "F3"
    origin = record.get("origin")
    sides = record.get("sides") or ()
    return "{0}:{1}:{2}:{3}:{4}:{5}".format(
        kind, name, level, "" if origin is None else int(origin),
        ",".join(str(s) for s in sides), int(seed))


def build_entry_and_stem(record, index):
    """`(entry_string, stem)` for one record, via the exact same
    `fire_bake.parse_entry` / `fire_bake.out_stem` a real bake uses -- so a
    stem printed here is the stem `fire_city_bake.sh` will actually look
    for and, if it bakes, actually produce."""
    text = entry_string(record)
    entry = fb.parse_entry(text, index=index, base_seed=7)
    return text, fb.out_stem(entry)


def city_fields(record):
    doc = {k: record.get(k) for k in _CITY_KEYS}
    doc["orig_usd"] = record.get("usd")
    return doc


# ---------------------------------------------------------------------------
# Classification
# ---------------------------------------------------------------------------
def classify(records, out_dir):
    """`[{"i", "record", "entry", "stem", "usd_path", "json_path", "have",
    "status", "error"}, ...]`, one per record, IN MANIFEST ORDER. `status`
    is `"HAVE"` or `"NEED"` here (never `"STALE"` -- that is applied
    afterwards, only under `--verify`) and `"ERROR"` for a record
    `entry_string` refused."""
    out = []
    for i, rec in enumerate(records or []):
        row = {"i": i, "record": rec}
        try:
            text, stem = build_entry_and_stem(rec, i)
        except Exception as exc:
            row.update(entry=None, stem=None, usd_path=None, json_path=None,
                       have=False, status="ERROR", error=str(exc))
            out.append(row)
            continue
        usd_path = os.path.join(out_dir, stem + ".usd")
        json_path = os.path.join(out_dir, stem + ".json")
        have = os.path.exists(usd_path) and os.path.exists(json_path)
        row.update(entry=text, stem=stem, usd_path=usd_path,
                   json_path=json_path, have=have,
                   status=("HAVE" if have else "NEED"), error=None)
        out.append(row)
    return out


def verify_stems(usd_paths, container, repo, timeout_s=900):
    """`{usd_path: ok_bool}` by running `verify_export` on every path in
    `usd_paths` INSIDE `container`, through ITS OWN `usd_python.sh`
    (`scene_gen/tests/test_fire_bake.py --verify <comma list>`) -- the
    exact call `fire_bake.sh --verify-only` makes, batched into one
    `docker exec` rather than one per stem. Returns `None` (not a dict) if
    the docker call itself could not even be attempted -- the caller's cue
    to fall back to "trust existence" rather than mark everything STALE."""
    if not usd_paths:
        return {}
    spec = ",".join(usd_paths)
    inner = ("{0}/scene_gen/tools/usd_python.sh "
             "{0}/scene_gen/tests/test_fire_bake.py --verify {1}"
             .format(repo, spec))
    cmd = ["docker", "exec", container, "bash", "-c", inner]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout_s)
    except Exception as exc:
        print("[fire_city_manifest] WARNING: could not run verify_export via "
              "docker exec {0} ({1}) -- trusting existence for {2} "
              "bake(s)".format(container, exc, len(usd_paths)), file=sys.stderr)
        return None
    ok = {}
    pat = re.compile(r"^BAKE VERIFY\s+(OK|\*\*\* PROBLEM \*\*\*)\s+(.+?)\s*$")
    for line in (proc.stdout or "").splitlines():
        m = pat.match(line)
        if m:
            ok[m.group(2)] = (m.group(1) == "OK")
    missing = [p for p in usd_paths if p not in ok]
    if missing:
        print("[fire_city_manifest] WARNING: verify_export reported nothing "
              "for {0}/{1} path(s) (container problem, or `test_fire_bake.py`"
              " changed its output) -- trusting existence for them"
              .format(len(missing), len(usd_paths)), file=sys.stderr)
        for p in missing:
            ok[p] = True
    if proc.returncode not in (0, 1):
        # 0 == all clean, 1 == some PROBLEM -- both are a completed run.
        # Anything else (127, a signal, ...) means the harness itself did
        # not run cleanly; still trust whatever it DID manage to report,
        # per-path, above.
        print("[fire_city_manifest] NOTE: verify_export harness exited {0} "
              "(0=clean, 1=some PROBLEM expected; other codes suggest the "
              "harness itself did not finish normally)".format(proc.returncode),
              file=sys.stderr)
    return ok


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _print_rows(rows, verify_ok):
    counts = {"HAVE": 0, "STALE": 0, "NEED": 0, "ERROR": 0}
    for row in rows:
        status = row["status"]
        if status == "HAVE" and verify_ok is not None:
            ok = verify_ok.get(row["usd_path"], True)
            if not ok:
                status = "STALE"
        counts[status] = counts.get(status, 0) + 1
        if status == "ERROR":
            print("{0}\t{1}\t{2}\t{3}".format(
                "-", "-", status, row.get("error")))
        else:
            print("{0}\t{1}\t{2}".format(row["entry"], row["stem"], status))
    total = len(rows)
    print("SUMMARY total={0} have={1} stale={2} need={3} error={4}".format(
        total, counts["HAVE"], counts["STALE"], counts["NEED"], counts["ERROR"]))
    return counts


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Classify a fire-city manifest's records as HAVE / "
                    "STALE / NEED against a bake output directory.")
    ap.add_argument("manifest", help="path to the manifest .json")
    ap.add_argument("--out-dir", default=os.environ.get("FB_OUT")
                    or fb.DEFAULT_OUT_DIR,
                    help="directory to check for <stem>.usd/.json "
                        "(default: $FB_OUT or {0})".format(fb.DEFAULT_OUT_DIR))
    ap.add_argument("--verify", action="store_true",
                    help="re-open every HAVE bake cold, inside "
                        "--container, and demote a failing one to STALE")
    ap.add_argument("--container", default=os.environ.get("CONTAINER", "isaac-sim"))
    ap.add_argument("--repo", default=os.environ.get("REPO", "/isaac-sim/AirStack"))
    ap.add_argument("--print-seed", action="store_true",
                    help="print the resolved city seed only, then exit")
    ap.add_argument("--write-city-json", metavar="DIR",
                    help="write <DIR>/<stem>.city.json for every record "
                        "(the sidecar `extra={\"city\": ...}` payload the "
                        "city bake driver points FB_CITY_JSON at), then exit")
    args = ap.parse_args(argv)

    try:
        top_seed, records = load_manifest(args.manifest)
    except Exception as exc:
        print("fire_city_manifest: {0}".format(exc), file=sys.stderr)
        return 2

    if args.print_seed:
        print(resolve_city_seed(args.manifest, top_seed))
        return 0

    rows = classify(records, args.out_dir)

    if args.write_city_json:
        os.makedirs(args.write_city_json, exist_ok=True)
        n = 0
        for row in rows:
            if row["status"] == "ERROR":
                print("fire_city_manifest: skipping record {0} for "
                      "--write-city-json: {1}".format(row["i"], row["error"]),
                      file=sys.stderr)
                continue
            out_path = os.path.join(args.write_city_json, row["stem"] + ".city.json")
            with open(out_path, "w") as fh:
                json.dump(city_fields(row["record"]), fh, indent=1, sort_keys=True)
            print(out_path)
            n += 1
        print("fire_city_manifest: wrote {0}/{1} city sidecar file(s) -> {2}"
              .format(n, len(rows), args.write_city_json), file=sys.stderr)
        return 0

    verify_ok = None
    if args.verify:
        have_usd = [r["usd_path"] for r in rows if r["status"] == "HAVE"]
        verify_ok = verify_stems(have_usd, args.container, args.repo)

    _print_rows(rows, verify_ok)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
