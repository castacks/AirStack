#!/usr/bin/env python
"""
measure_assets.py — compact size/cost table for a set of USD assets.

Written for vetting the NVIDIA AEC packs before wiring them into an asset set:
their entries need a real `scale` (the packs are not all authored in meters) and
`fallback_sizes` needs real footprints, while the point counts say whether an
archviz asset is affordable at city density.

Reads local files with plain pxr — no SimulationApp, no Nucleus. Run inside the
isaac-sim container, which is the easy way to get pxr on the path:

    docker run --rm -v <repo>:/isaac-sim/AirStack --entrypoint bash <image> -c \\
      'cd /isaac-sim && PYTHONPATH="$ISAAC_SIM_PYTHONPATH" ./python.sh \\
       /isaac-sim/AirStack/scene_gen/tools/measure_assets.py <dir-or-file> ...'

Columns:
    mpu       metersPerUnit as authored (1.0 = meters, 0.01 = centimeters)
    up        stage up-axis ("Y" needs axis-up: "Y" in the asset-set entry)
    x/y/z     bounding box size, converted to METERS
    base      bbox min-z in meters — the generator's ground offset
    pts       total Mesh points; the poly-budget proxy
    bnd       meshes with a material bound / total meshes (GeomSubset-aware)
    mdl       bound materials whose MDL module RESOLVES / bound materials

`bnd` and `mdl` fail differently and the difference is the point. `bnd < meshes`
is the old check: geometry with no material at all. `mdl < bound` is geometry
that passes every binding check and still renders unshaded, because the material
it binds names an `info:mdl:sourceAsset` that is not on disk — a content-server
path that was never mirrored, or a module the pack simply does not ship.

That second case is why this column exists. Both Hibiscus and Douglas_Fir were
curated into suburban_v2 on a clean "8/8 bound" and both had a mesh bound to a
missing MDL; a binding-existence check cannot see it, because the binding is
real and it is the far end of it that is missing. Anything listed under the
UNRESOLVED footer renders untextured no matter what the bind count says.
"""

import os
import sys

from pxr import Usd, UsdGeom, UsdShade


def _iter_usds(paths):
    for p in paths:
        if os.path.isdir(p):
            for root, _dirs, files in os.walk(p):
                for f in sorted(files):
                    if f.endswith((".usd", ".usda", ".usdc", ".usdz")):
                        yield os.path.join(root, f)
        else:
            yield p


def _bound_material(prim):
    """The material bound to *prim*, or None. Never raises."""
    try:
        mat, _rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    except Exception:
        return None
    return mat if (mat and mat.GetPrim().IsValid()) else None


def _mdl_status(mat, cache):
    """``("ok" | "broken" | "nonmdl", missing_path)`` for a bound material.

    "broken" means a shader in the material declares an MDL source asset that
    does NOT resolve — the material is bound, and shades nothing. "nonmdl" is a
    material with no MDL shader at all (a UsdPreviewSurface-only material, say);
    it is not counted against the asset, since nothing here can judge it.

    Cached on the material's prim path: one asset binds a handful of materials
    from dozens of GeomSubsets, and every check is a resolver round trip.
    """
    key = mat.GetPrim().GetPath()
    if key not in cache:
        status, missing = "nonmdl", ""
        for p in Usd.PrimRange(mat.GetPrim()):
            shd = UsdShade.Shader(p)
            if not shd:
                continue
            src = shd.GetSourceAsset("mdl")
            if not src:
                continue
            if src.resolvedPath:
                if status == "nonmdl":
                    status = "ok"
            else:
                # One dead shader is enough: that subset renders unshaded
                # whatever the rest of the network does.
                status, missing = "broken", src.path
                break
        cache[key] = (status, missing)
    return cache[key]


def _materials(stage):
    """Binding audit for a stage.

    Returns ``(n_meshes, n_bound, n_mats, n_resolve, broken)`` where *broken* is
    ``[(material name, unresolved mdl path)]``.

    GeomSubset-aware for the same reason material_binding.py is: UE and
    Omniverse exports routinely leave the Mesh unbound and bind each
    ``Section0..N`` subset instead, so asking only the Mesh reports a fully
    textured asset as unbound. A mesh counts as bound only when every one of its
    subsets resolves — but a PARTIALLY bound mesh still has its materials
    checked, since the half that binds can perfectly well bind something dead.
    """
    cache, broken = {}, {}
    n_meshes = n_bound = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Mesh":
            continue
        n_meshes += 1
        subs = [c for c in prim.GetChildren()
                if c.GetTypeName() == "GeomSubset"]
        direct = _bound_material(prim)
        if direct:
            found, full = [direct], True
        elif subs:
            found = [m for m in (_bound_material(s) for s in subs) if m]
            full = len(found) == len(subs)
        else:
            found, full = [], False
        if full and found:
            n_bound += 1
        for m in found:
            status, missing = _mdl_status(m, cache)
            if status == "broken":
                broken[m.GetPrim().GetName()] = missing

    judged = [v for v in cache.values() if v[0] != "nonmdl"]
    n_resolve = sum(1 for v in judged if v[0] == "ok")
    return n_meshes, n_bound, len(judged), n_resolve, sorted(broken.items())


def measure(url):
    stage = Usd.Stage.Open(url)
    if stage is None:
        return None
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    up = UsdGeom.GetStageUpAxis(stage) or "Z"

    # Default purpose only — excludes proxy/guide geometry, which would
    # otherwise inflate both the bbox and the point count.
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    dp = stage.GetDefaultPrim()
    root = dp if (dp and dp.IsValid()) else stage.GetPseudoRoot()
    rng = cache.ComputeWorldBound(root).ComputeAlignedRange()

    if rng.IsEmpty():
        size = (0.0, 0.0, 0.0)
        base = 0.0
    else:
        mn, mx = rng.GetMin(), rng.GetMax()
        size = tuple((mx[i] - mn[i]) * mpu for i in range(3))
        base = mn[2] * mpu

    pts = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() == "Mesh":
            a = prim.GetAttribute("points").Get()
            if a:
                pts += len(a)

    n_meshes, n_bound, n_mats, n_resolve, broken = _materials(stage)

    return dict(url=url, mpu=mpu, up=str(up), size=size, base=base, pts=pts,
                meshes=n_meshes, bound=n_bound, mats=n_mats,
                resolve=n_resolve, broken=broken)


def main(argv):
    if not argv:
        print(__doc__)
        return 1

    rows = []
    for url in _iter_usds(argv):
        try:
            r = measure(url)
        except Exception as exc:                       # keep going on one bad asset
            print(f"  !! {url}: {type(exc).__name__}: {exc}")
            continue
        if r is None:
            print(f"  !! {url}: failed to open")
            continue
        rows.append(r)

    if not rows:
        print("no assets measured")
        return 1

    common = os.path.commonpath([r["url"] for r in rows]) if len(rows) > 1 else ""
    print(f"{'asset':<52} {'mpu':>6} {'up':>3} "
          f"{'x_m':>8} {'y_m':>8} {'z_m':>8} {'base_m':>8} {'pts':>10} "
          f"{'bnd':>9} {'mdl':>9}")
    print("-" * 132)
    for r in sorted(rows, key=lambda r: -r["pts"]):
        name = os.path.relpath(r["url"], common) if common else r["url"]
        x, y, z = r["size"]
        # A trailing ! marks the row whose materials do not all resolve, so the
        # defect is visible in the table and not only in the footer.
        bnd = f"{r['bound']}/{r['meshes']}"
        mdl = f"{r['resolve']}/{r['mats']}" + ("!" if r["broken"] else "")
        print(f"{name[:52]:<52} {r['mpu']:>6.3f} {r['up']:>3} "
              f"{x:>8.2f} {y:>8.2f} {z:>8.2f} {r['base']:>8.3f} {r['pts']:>10,} "
              f"{bnd:>9} {mdl:>9}")

    total = sum(r["pts"] for r in rows)
    print("-" * 132)
    print(f"{len(rows)} assets, {total:,} points total, "
          f"{total // max(1, len(rows)):,} mean")
    # A city places hundreds of props; anything in the millions is a problem at
    # density regardless of how good it looks in isolation.
    heavy = [r for r in rows if r["pts"] > 200_000]
    if heavy:
        print(f"\n{len(heavy)} asset(s) over 200k points — check these at density:")
        for r in sorted(heavy, key=lambda r: -r["pts"]):
            print(f"  {r['pts']:>10,}  {os.path.relpath(r['url'], common)}")

    # Reported separately from unbound geometry and last, so it is the thing
    # left on screen: these assets pass a bind count and still render unshaded.
    bad = [r for r in rows if r["broken"]]
    if bad:
        n = sum(len(r["broken"]) for r in bad)
        print(f"\n{n} material(s) in {len(bad)} asset(s) bind an MDL that does "
              f"NOT resolve — this geometry renders UNSHADED:")
        for r in sorted(bad, key=lambda r: r["url"]):
            print(f"  {os.path.relpath(r['url'], common) if common else r['url']}")
            for mat, missing in r["broken"]:
                print(f"      {mat}  ->  {missing or '<no mdl source asset>'}")

    unbound = [r for r in rows if r["meshes"] and r["bound"] < r["meshes"]]
    if unbound:
        print(f"\n{len(unbound)} asset(s) with meshes bound to NO material:")
        for r in sorted(unbound, key=lambda r: r["bound"] - r["meshes"]):
            rel = os.path.relpath(r["url"], common) if common else r["url"]
            print(f"  {r['meshes'] - r['bound']:>4} of {r['meshes']:<4} meshes  {rel}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
