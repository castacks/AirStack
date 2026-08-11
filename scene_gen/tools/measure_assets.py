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
"""

import os
import sys

from pxr import Usd, UsdGeom


def _iter_usds(paths):
    for p in paths:
        if os.path.isdir(p):
            for root, _dirs, files in os.walk(p):
                for f in sorted(files):
                    if f.endswith((".usd", ".usda", ".usdc", ".usdz")):
                        yield os.path.join(root, f)
        else:
            yield p


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

    return dict(url=url, mpu=mpu, up=str(up), size=size, base=base, pts=pts)


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
          f"{'x_m':>8} {'y_m':>8} {'z_m':>8} {'base_m':>8} {'pts':>10}")
    print("-" * 112)
    for r in sorted(rows, key=lambda r: -r["pts"]):
        name = os.path.relpath(r["url"], common) if common else r["url"]
        x, y, z = r["size"]
        print(f"{name[:52]:<52} {r['mpu']:>6.3f} {r['up']:>3} "
              f"{x:>8.2f} {y:>8.2f} {z:>8.2f} {r['base']:>8.3f} {r['pts']:>10,}")

    total = sum(r["pts"] for r in rows)
    print("-" * 112)
    print(f"{len(rows)} assets, {total:,} points total, "
          f"{total // max(1, len(rows)):,} mean")
    # A city places hundreds of props; anything in the millions is a problem at
    # density regardless of how good it looks in isolation.
    heavy = [r for r in rows if r["pts"] > 200_000]
    if heavy:
        print(f"\n{len(heavy)} asset(s) over 200k points — check these at density:")
        for r in sorted(heavy, key=lambda r: -r["pts"]):
            print(f"  {r['pts']:>10,}  {os.path.relpath(r['url'], common)}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
