"""asset_sets — read a `config/asset_sets/*.yaml` pack into plain asset entries.

Wraps the scene generator's own `resolve_asset_set` / `_parse_usd_entry`, so
`extends:`, `asset_root`, `asset_scale`, per-entry `scale`, `axis-up`,
`target-size-m` and `tags` all mean here exactly what they mean to the
generator — there is one reader for the format, not a second interpretation
of it grown in a tool.

MUST be called from a process that has not yet booted Isaac Sim (a
`multiprocessing` worker, or the main process before `SimulationApp`).
`scene_generator` imports `pxr` at module scope, and pulling usd-core into a
process before Kit starts stops Kit prepending its own USD build — Kit then
dies at startup with "extension class wrapper for base class UsdTyped has not
been created yet". Both callers here already run this off the main thread of
a Kit process for that reason.
"""

from __future__ import annotations

import multiprocessing as mp


def walk_entries(node):
    """Every USD entry under a `usds:` node, however deeply it is nested."""
    if isinstance(node, dict):
        for child in node.values():
            yield from walk_entries(child)
    elif isinstance(node, (list, tuple)):
        for item in node:
            if isinstance(item, dict) and "usd" not in item:
                yield from walk_entries(list(item.values()))
            else:
                yield item


def read_asset_set(name, categories, target_size=0.0):
    """`[{asset, scale, target_size, up_axis, tags, category}, ...]`.

    One dict per USD entry found under the requested dotted `categories`
    (e.g. ``"buildings.intact"``), de-duplicated by resolved path. `tags` is
    the entry's own tag set, passed through unfiltered — a caller that cares
    about a specific tag (a structural-material tag, say) filters it there;
    this reader has no opinion about what a tag means.
    """
    from scene_generator import _parse_usd_entry, resolve_asset_set

    cfg = resolve_asset_set({"asset_set": name})
    root = str(cfg.get("asset_root", "") or "")
    default_scale = float(cfg.get("asset_scale", 1.0) or 1.0)
    usds = cfg.get("usds", {}) or {}

    entries, seen = [], set()
    for dotted in categories:
        node = usds
        for part in dotted.split("."):
            node = (node or {}).get(part) if isinstance(node, dict) else None
        if node is None:
            print(f"[asset_sets] '{name}' has no category '{dotted}'",
                  flush=True)
            continue
        for entry in walk_entries(node):
            path, scale, axis, _yaw, tags = _parse_usd_entry(
                entry, default_scale, root)
            if path in seen:
                continue
            seen.add(path)
            size = float(entry.get("target-size-m", 0.0)) \
                if isinstance(entry, dict) else 0.0
            entries.append({
                "asset": path,
                # An explicit per-entry scale is how the packs state size;
                # only fall back to normalising when the entry gives a target
                # size, or the caller asked for one.
                "scale": 0.0 if size else float(scale),
                "target_size": size or (target_size if size else 0.0),
                "up_axis": "y" if str(axis).upper() == "Y" else "z",
                "tags": set(tags),
                "category": dotted,
            })
    return entries


def read_assets_file(path):
    """One asset per line; `#` starts a comment, whole-line or trailing."""
    assets = []
    with open(path) as fh:
        for line in fh:
            entry = line.split("#", 1)[0].strip()
            if entry:
                assets.append(entry)
    return assets


def split_kit_only(items, key=lambda x: x):
    """Partition *items* into (worker-safe indices, Kit-only indices).

    `omniverse://` assets need Kit's own `omni.client` resolver and cannot run
    in a worker process the way everything else can — see this module's
    docstring. Returns two index SETS rather than filtered lists, so a caller
    tests membership in O(1) rather than re-scanning the kit-only sublist for
    every item — that O(n*k) scan, done independently by both callers below,
    is what this replaces.
    """
    kit_idx, other_idx = set(), set()
    for i, item in enumerate(items):
        if str(key(item)).strip().startswith("omniverse://"):
            kit_idx.add(i)
        else:
            other_idx.add(i)
    return other_idx, kit_idx


def run_isolated(fn, args):
    """Call ``fn(*args)`` in a disposable spawned process, before Kit can boot.

    Both `batch_damage.py` and `categorize_assets.py` need to resolve an asset
    set before deciding whether to start Isaac Sim, and both used to hand-roll
    the same "throwaway single-worker pool" dance to do it — this is that
    dance, written once. It matters because `read_asset_set` reaches into
    `scene_generator`, which imports `pxr` at module scope, and pulling
    usd-core into THIS process before `SimulationApp` starts would stop Kit
    prepending its own USD build.
    """
    ctx = mp.get_context("spawn")
    with ctx.Pool(1) as pool:
        return pool.apply(fn, args)
