#!/usr/bin/env python3
"""Pre-flight: cache the Objaverse assets a scene needs, before ``airstack up``.

Asset sets reference Objaverse objects by uid (``objaverse://<uid>``). Those are
**not** in the repo — they're downloaded from Objaverse and converted to USD
into a local cache at ``scene_gen/assets/objaverse/<uid>/``. Run this once on
the host and every later ``airstack up`` just reads the cache.

It has to run here rather than inside the sim container: the container has no
``uv``, ``objaverse`` or Blender, and Isaac Sim's own converter costs ~80 s of
app startup per invocation and emits Y-up centimetre USDs that would need
renormalizing. The repo is bind-mounted, so the container sees whatever this
writes.

Usage
-----
    # everything every asset set references, skipping what's already cached
    python3 scene_gen/prepare_assets.py

    # just what one scene needs (preset, compiled config, or bare name)
    python3 scene_gen/prepare_assets.py suburban
    python3 scene_gen/prepare_assets.py config/presets/tornado.yaml

    # or name an asset set directly
    python3 scene_gen/prepare_assets.py --asset-set urban

    # see what would happen, download nothing
    python3 scene_gen/prepare_assets.py --list

    # re-convert even if cached (e.g. after changing target-size-m)
    python3 scene_gen/prepare_assets.py suburban --force

Run it with the repo's venv, which has ``objaverse``/``trimesh``::

    .venv/bin/python scene_gen/prepare_assets.py

Caching is keyed on uid *and* the requested ``target-size-m``, so re-running is
cheap and safe: already-correct assets are skipped, and only a changed target
size forces a re-convert.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

_SCENE_GEN_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCENE_GEN_DIR)

import objaverse_assets as oa

CONFIG_DIR = os.path.join(_SCENE_GEN_DIR, "config")
# Where a bare name is looked up, in the same order compile_disaster uses.
CONFIG_SEARCH = [
    os.path.join(CONFIG_DIR, "presets"),
    os.path.join(CONFIG_DIR, "low_level", "compiled"),
    os.path.join(CONFIG_DIR, "low_level"),
    os.path.join(CONFIG_DIR, "asset_sets"),
]


ASSET_SETS_DIR = os.path.join(CONFIG_DIR, "asset_sets")


def _resolve(name: str, search: list, what: str = "config") -> str:
    """Find a config by path or bare name within *search*.

    The scope matters: an asset set and a preset can share a name (``suburban``
    is both), so resolving a set has to look only in ``asset_sets/`` rather
    than falling through the generic order and picking up the preset.
    """
    if os.path.isfile(name):
        return name
    stem = os.path.basename(name)
    for d in search:
        for cand in (os.path.join(d, stem),
                     os.path.join(d, stem + ".yaml"),
                     os.path.join(d, stem + ".yml")):
            if os.path.isfile(cand):
                return cand
    available = ["    " + os.path.relpath(f, _SCENE_GEN_DIR)
                 for d in search
                 for f in sorted(glob.glob(os.path.join(d, "*.y*ml")))]
    raise SystemExit(f"{what} not found: {name!r}\n  available:\n"
                     + "\n".join(available))


def _with_extends(path: str) -> list:
    """*path* plus every asset set it inherits from, base last.

    Asset sets chain: `suburban` extends `suburban_nucleus` extends `shared`, and
    the pools live all the way down — the 15 bungalows are in `suburban.yaml`,
    not in `suburban.yaml`. Scanning only the named file reported "all 2
    assets already cached" for a scene that actually wants 24, and the sim then
    rendered every house as a placeholder prism. `scene_generator` resolves the
    chain (it prints `suburban.yaml <- suburban_nucleus.yaml <- shared.yaml`); this
    has to do the same, in plain YAML, because it must run without `pxr`.
    """
    import yaml
    out, seen = [], set()
    stack = [path]
    while stack:
        p = stack.pop(0)
        if p in seen:
            continue
        seen.add(p)
        out.append(p)
        try:
            with open(p) as f:
                doc = yaml.safe_load(f) or {}
        except OSError:
            continue
        base = doc.get("extends")
        for name in ([base] if isinstance(base, str) else (base or [])):
            try:
                stack.append(_resolve(str(name), [ASSET_SETS_DIR], "asset set"))
            except SystemExit:
                raise
            except Exception:
                pass
    return out


def yaml_files_for(config: str = None, asset_set: str = None) -> list:
    """The YAML files to scan for ``objaverse://`` references.

    Deliberately resolved with plain YAML rather than through
    ``scene_generator.resolve_asset_set``: that would pull in ``pxr``, which
    the host venv doesn't have — and this script's whole job is to run on the
    host before the sim ever starts.
    """
    if asset_set:
        return _with_extends(_resolve(asset_set, [ASSET_SETS_DIR], "asset set"))

    if not config:                       # default: every asset set
        return sorted(glob.glob(os.path.join(ASSET_SETS_DIR, "*.yaml"))
                      + glob.glob(os.path.join(ASSET_SETS_DIR, "*.yml")))

    import yaml

    path = _resolve(config, CONFIG_SEARCH, "config")
    with open(path) as f:
        doc = yaml.safe_load(f) or {}

    # Scan the config itself (it may inline `usds`, or override a set's), plus
    # the asset set it resolves to. An explicit set wins — high-level specs
    # write `asset-set`, low-level configs `asset_set`; otherwise the spec's
    # `locale` picks one, via the same mapping the compiler uses so this can't
    # drift from what the scene will actually load.
    files = [path]
    name = doc.get("asset_set") or doc.get("asset-set")
    if not name and doc.get("locale"):
        from compile_locale import default_asset_set
        name = default_asset_set(doc["locale"])
    if name:
        files.extend(_with_extends(
            _resolve(str(name), [ASSET_SETS_DIR], "asset set")))
    return list(dict.fromkeys(files))    # de-dup, preserve order


def report(wanted: dict) -> int:
    """Print cache status per uid. Returns the number not yet cached."""
    manifest = oa.load_manifest()["assets"]
    missing = 0
    print(f"{'uid':34s} {'target':>11s}  {'state':<12s} title")
    for uid, spec in sorted(wanted.items()):
        size, fit = spec["target_size_m"], spec["fit"]
        cached = oa.is_cached(uid, size, fit)
        missing += not cached
        state = "cached" if cached else (
            "stale" if os.path.exists(oa.cache_path(uid)) else "not cached")
        title = manifest.get(uid, {}).get("title", "")
        tgt = f"{size:g} m {fit[:4]}" if size else "-"
        print(f"{uid:34s} {tgt:>11s}  {state:<12s} {title[:38]}")
    return missing


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Download + convert the Objaverse assets a scene needs, "
                    "into the local cache used by `airstack up`.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="With no config, processes every asset set and caches whatever "
               "is missing.")
    ap.add_argument("config", nargs="?",
                    help="scene config (preset, compiled, or bare name) whose "
                         "assets to prepare; default: all asset sets")
    ap.add_argument("--asset-set", help="name an asset set directly instead")
    ap.add_argument("--list", action="store_true",
                    help="show what is cached / missing and exit; downloads nothing")
    ap.add_argument("--force", action="store_true",
                    help="re-download and re-convert even if already cached")
    args = ap.parse_args()

    files = yaml_files_for(args.config, args.asset_set)
    wanted = oa.scan_asset_sets(files)

    scope = args.asset_set or args.config or "all asset sets"
    rel = ", ".join(os.path.relpath(f, _SCENE_GEN_DIR) for f in files)
    print(f"[prepare_assets] {scope} -> {rel}")

    if not wanted:
        print("[prepare_assets] no Objaverse assets referenced — nothing to do.")
        return 0

    if args.list:
        missing = report(wanted)
        print(f"\n[prepare_assets] {len(wanted) - missing}/{len(wanted)} cached"
              + (f", {missing} to fetch — re-run without --list" if missing else
                 " — ready for `airstack up`"))
        return 0

    todo = [u for u, spec in wanted.items()
            if args.force or not oa.is_cached(u, spec["target_size_m"], spec["fit"])]
    if not todo:
        print(f"[prepare_assets] all {len(wanted)} assets already cached — "
              "ready for `airstack up`.")
        return 0

    print(f"[prepare_assets] {len(todo)} of {len(wanted)} assets need fetching "
          "(downloading + converting to USD; this takes a moment each)…")

    ok, failed = [], {}
    catalog = oa._catalog_if_cached()
    for uid in sorted(todo):
        try:
            oa.ensure(uid, target_size_m=wanted[uid]["target_size_m"],
                      fit=wanted[uid]["fit"], force=args.force, catalog=catalog)
            ok.append(uid)
        except Exception as exc:   # one bad asset must not stop the batch
            failed[uid] = f"{type(exc).__name__}: {exc}"
            print(f"[prepare_assets] ERROR {uid}: {exc}", file=sys.stderr)

    print(f"\n[prepare_assets] {len(ok)} prepared, {len(failed)} failed, "
          f"{len(wanted) - len(todo)} already cached.")
    if failed:
        print("[prepare_assets] Uncached assets render as placeholder prisms; "
              "the generator lists them at startup.", file=sys.stderr)
        return 1
    print("[prepare_assets] Ready — `airstack up` will use the cache.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
