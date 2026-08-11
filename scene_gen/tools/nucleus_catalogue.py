#!/usr/bin/env python
"""
nucleus_catalogue.py — measure Nucleus USDs and dump scene structure.

Companion to nucleus_browse.py. Boots a headless SimulationApp (the only way to
get `pxr`/`omni.client` in the isaac-sim container), then for each URL in the
JOBS file:

    measure <url>   bbox in metres, mpu, up-axis, Mesh point count
    tree    <url>   prim tree to TREE_DEPTH, payloads unloaded — for telling a
                    monolithic scene from one with addressable building prims
    walk    <url>   recursive directory listing

Reads a JSON job list from $JOBS (default /tmp/jobs.json), writes TSV/text to
$OUT (default /tmp/catalogue.txt). stdout is swallowed by Kit, so cat the file.
"""

from isaacsim import SimulationApp

app = SimulationApp(launch_config={"headless": True})

import json
import os

import carb
import omni.client
from pxr import Usd, UsdGeom

OUT = os.environ.get("OUT", "/tmp/catalogue.txt")
JOBS = os.environ.get("JOBS", "/tmp/jobs.json")
TREE_DEPTH = int(os.environ.get("TREE_DEPTH", "3"))
WALK_DEPTH = int(os.environ.get("WALK_DEPTH", "3"))
SKIP_DIRS = ("textures", "texture", "materials", "maps", ".thumbs", "sourceimages")


def measure(url):
    st = Usd.Stage.Open(url)
    if st is None:
        return None
    mpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    up = str(UsdGeom.GetStageUpAxis(st) or "Z")
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    dp = st.GetDefaultPrim()
    root = dp if (dp and dp.IsValid()) else st.GetPseudoRoot()
    rng = cache.ComputeWorldBound(root).ComputeAlignedRange()
    if rng.IsEmpty():
        size, base = (0.0, 0.0, 0.0), 0.0
    else:
        mn, mx = rng.GetMin(), rng.GetMax()
        size = tuple((mx[i] - mn[i]) * mpu for i in range(3))
        base = mn[2] * mpu
    pts = nmesh = 0
    for prim in st.Traverse():
        if prim.GetTypeName() == "Mesh":
            a = prim.GetAttribute("points").Get()
            if a:
                pts += len(a)
                nmesh += 1
    return dict(mpu=mpu, up=up, size=size, base=base, pts=pts, meshes=nmesh,
                default_prim=(dp.GetName() if dp and dp.IsValid() else ""))


def measure_prim(url, path):
    """Measure one prim inside a scene — for scenes whose buildings are
    addressable sub-prims rather than separate files."""
    st = Usd.Stage.Open(url)
    if st is None:
        return None
    prim = st.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None
    mpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        size, base = (0.0, 0.0, 0.0), 0.0
    else:
        mn, mx = rng.GetMin(), rng.GetMax()
        size = tuple((mx[i] - mn[i]) * mpu for i in range(3))
        base = mn[2] * mpu
    pts = nmesh = 0
    for p in Usd.PrimRange(prim):
        if p.GetTypeName() == "Mesh":
            a = p.GetAttribute("points").Get()
            if a:
                pts += len(a)
                nmesh += 1
    return dict(mpu=mpu, up="Z", size=size, base=base, pts=pts, meshes=nmesh,
                default_prim=path)


def textures(url, out):
    """Every texture asset path an asset binds — the only reliable way to tell a
    green tree from an autumn one without rendering it."""
    st = Usd.Stage.Open(url)
    if st is None:
        out.write(f"!! tex open failed: {url}\n")
        return
    seen = set()
    for prim in st.Traverse():
        for attr in prim.GetAttributes():
            if attr.GetTypeName().type.typeName != "SdfAssetPath":
                continue
            v = attr.Get()
            if v is None:
                continue
            p = getattr(v, "path", "") or getattr(v, "resolvedPath", "")
            if p:
                seen.add(os.path.basename(p))
    for s in sorted(seen):
        out.write(f"  tex {s}\n")


def tree(url, out):
    st = Usd.Stage.Open(url, load=Usd.Stage.LoadNone)
    if st is None:
        out.write(f"!! tree open failed: {url}\n")
        return
    out.write(f"  mpu={UsdGeom.GetStageMetersPerUnit(st)} "
              f"up={UsdGeom.GetStageUpAxis(st)}\n")
    for prim in st.Traverse():
        path = prim.GetPath()
        depth = path.pathElementCount
        if depth > TREE_DEPTH:
            continue
        refs = ""
        try:
            items = prim.GetMetadata("references")
            if items and items.prependedItems:
                refs = " -> " + os.path.basename(items.prependedItems[0].assetPath)
        except Exception:
            pass
        pay = " [payload]" if prim.HasAuthoredPayloads() else ""
        out.write(f"  {'  ' * (depth - 1)}{path.name or '/'}"
                  f" <{prim.GetTypeName()}>{pay}{refs}\n")


def walk(url, depth, out, seen):
    if depth > WALK_DEPTH or url in seen:
        return
    seen.add(url)
    res, entries = omni.client.list(url)
    if res != omni.client.Result.OK:
        out.write(f"!! {url} -> {res}\n")
        return
    dirs = []
    for e in entries:
        if e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN:
            dirs.append(e.relative_path)
            out.write(f"D {url}{e.relative_path}/\n")
        else:
            out.write(f"F {url}{e.relative_path}\t{getattr(e, 'size', 0)}\n")
    for d in dirs:
        if d.lower() not in SKIP_DIRS:
            walk(f"{url}{d}/", depth + 1, out, seen)


def main():
    root = os.environ.get("OMNI_SERVER", "").strip().strip('"').rstrip("/")
    carb.settings.get_settings().set("/persistent/isaac/asset_root/default", root)
    with open(JOBS) as fh:
        jobs = json.load(fh)
    with open(OUT, "w") as out:
        for kind, url, *rest in jobs:
            if kind in ("measure", "prim"):
                try:
                    r = (measure(url) if kind == "measure"
                         else measure_prim(url, rest[0]))
                except Exception as exc:
                    out.write(f"ERR\t{url}\t{type(exc).__name__}: {exc}\n")
                    continue
                if r is None:
                    out.write(f"ERR\t{url}\topen failed\n")
                    continue
                x, y, z = r["size"]
                out.write(f"M\t{url}\t{r['mpu']:.4g}\t{r['up']}\t{x:.2f}\t"
                          f"{y:.2f}\t{z:.2f}\t{r['base']:.3f}\t{r['pts']}\t"
                          f"{r['meshes']}\t{r['default_prim']}\n")
            elif kind == "tex":
                out.write(f"\n===== TEX {url} =====\n")
                try:
                    textures(url, out)
                except Exception as exc:
                    out.write(f"!! {type(exc).__name__}: {exc}\n")
            elif kind == "tree":
                out.write(f"\n===== TREE {url} =====\n")
                try:
                    tree(url, out)
                except Exception as exc:
                    out.write(f"!! {type(exc).__name__}: {exc}\n")
            elif kind == "walk":
                out.write(f"\n===== WALK {url} =====\n")
                walk(url, 0, out, set())
            out.flush()


main()
app.close()
