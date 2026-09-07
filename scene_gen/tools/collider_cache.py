#!/usr/bin/env python3
"""collider_cache — collider-ready variants of an asset, so it can be INSTANCED.

    <kit python> scene_gen/tools/collider_cache.py --out assets/_collider_cache \
        <usd> [<usd> ...]

WHY THIS EXISTS. `apply_placements` does not instance by default because
`scene_prep.add_colliders` walks `GetChildren()` applying `UsdPhysics.CollisionAPI`
to every gprim -- and an instanceable prim has no traversable children, so an
instanced building silently gets no collider and the drone flies through it.
`scene_generator.apply_placements` names the fix in its own docstring:

    pre-author `CollisionAPI` into cached per-asset variants so prototypes
    arrive collider-ready, then instance those

which is exactly what Stage A already does for archetypes (`bake.export_object`
authors the API on every merged mesh, and `add_colliders` skips a prim that has
it). This does the same for ordinary pack assets.

WHAT IT WRITES. Not a copy -- an OVERLAY. The cached layer references the source
at its default prim and adds one `over` per gprim carrying the API, so the cache
is kilobytes regardless of the asset, and re-running it after the source changes
costs nothing. Geometry, materials and bbox are the source's, untouched, which
is what lets the placement keep using the ORIGINAL path for its resolver lookup
and swap only the reference.
"""
from __future__ import annotations
import argparse, hashlib, os, sys


def cache_name(usd: str) -> str:
    """Stable filename for *usd*. Content-independent on purpose: the overlay
    holds no geometry, so it only has to track the PATH."""
    h = hashlib.sha1(str(usd).encode("utf-8")).hexdigest()[:16]
    stem = os.path.splitext(os.path.basename(str(usd).rstrip("/")))[0]
    stem = "".join(c if c.isalnum() or c in "-_" else "_" for c in stem)[:48]
    return f"{stem}_{h}.usda"


def cache_path(usd: str, cache_dir: str) -> str:
    return os.path.join(cache_dir, cache_name(usd))


def build(usd: str, cache_dir: str, force: bool = False) -> str:
    """Write the collider-ready overlay for *usd*. Returns its path, or "" if
    the source could not be opened or holds no gprim."""
    from pxr import Usd, UsdGeom, UsdPhysics

    out = cache_path(usd, cache_dir)
    if os.path.isfile(out) and not force:
        return out
    try:
        src = Usd.Stage.Open(usd)
    except Exception as exc:                                     # noqa: BLE001
        print(f"[collider_cache] cannot open {usd}: "
              f"{type(exc).__name__}: {exc}")
        return ""
    if src is None:
        return ""
    root = src.GetDefaultPrim()
    if not root or not root.IsValid():
        print(f"[collider_cache] {usd} has no default prim -- skipped")
        return ""

    rel = []
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Gprim):
            r = prim.GetPath().MakeRelativePath(root.GetPath()).pathString
            rel.append(r)
    if not rel:
        print(f"[collider_cache] {usd} has no gprim -- skipped")
        return ""

    os.makedirs(cache_dir, exist_ok=True)
    if os.path.exists(out):
        os.remove(out)
    cs = Usd.Stage.CreateNew(out)
    holder = cs.DefinePrim("/Asset")
    if not holder.GetReferences().AddReference(usd):
        print(f"[collider_cache] failed to reference {usd}")
        return ""
    cs.SetDefaultPrim(holder)
    n = 0
    for r in rel:
        # "." is the root prim itself (a Mesh-rooted asset).
        path = holder.GetPath() if r == "." else holder.GetPath().AppendPath(r)
        over = cs.OverridePrim(path)
        if over and over.IsValid():
            UsdPhysics.CollisionAPI.Apply(over)
            n += 1
    cs.GetRootLayer().Save()
    print(f"[collider_cache] {os.path.basename(out)}  {n} gprim(s)  <- {usd}")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("usds", nargs="*")
    ap.add_argument("--out", default="assets/_collider_cache")
    ap.add_argument("--from-json", default="",
                    help="a {category: [[usd, count], ...]} map to read paths "
                         "from, as `collider_cache` is usually fed the whole "
                         "scene at once")
    ap.add_argument("--categories", default="",
                    help="comma-separated categories to take from --from-json")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args(argv)

    usds = list(a.usds)
    if a.from_json:
        import json
        doc = json.load(open(a.from_json))
        want = {c for c in a.categories.split(",") if c} or set(doc)
        for cat, rows in doc.items():
            if cat in want:
                usds += [u for u, _n in rows]
    seen, ordered = set(), []
    for u in usds:
        if u and u not in seen:
            seen.add(u); ordered.append(u)
    if not ordered:
        print("nothing to do"); return 1

    ok = 0
    for u in ordered:
        if build(u, a.out, a.force):
            ok += 1
    print(f"[collider_cache] {ok}/{len(ordered)} cached -> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
