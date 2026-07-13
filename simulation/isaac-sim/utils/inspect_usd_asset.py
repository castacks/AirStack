#!/usr/bin/env python
"""
inspect_usd_asset.py — dump the composed prim tree + composition arcs of USD
asset files, to debug assets that reference in with no visible geometry.

Usage (inside the isaac-sim container, no SimulationApp needed):
    /isaac-sim/python.sh inspect_usd_asset.py <usd_url> [<usd_url> ...]
"""

import sys

# A headless SimulationApp is the one-stop way to get pxr, the omniverse://
# resolver, and cached Nucleus auth on python.sh's path.
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402


def _arcs(prim):
    out = []
    try:
        q = Usd.PrimCompositionQuery(prim)
        for arc in q.GetCompositionArcs():
            t = str(arc.GetArcType()).split(".")[-1]
            layer = arc.GetTargetLayer()
            node = arc.GetTargetPrimPath()
            ident = layer.identifier if layer else "?"
            if t != "PcpArcTypeRoot":
                out.append(f"{t} -> {ident} @{node}")
    except Exception as e:
        out.append(f"<arc query failed: {e}>")
    return out


def _dump(stage, prim, depth=0):
    pad = "  " * depth
    img = UsdGeom.Imageable(prim)
    bits = [prim.GetTypeName() or "<no type>"]
    if img:
        bits.append(f"vis={img.ComputeVisibility()}")
        bits.append(f"purpose={img.ComputePurpose()}")
    if not prim.IsActive():
        bits.append("INACTIVE")
    if prim.IsInstance():
        bits.append("instance")
    if prim.HasAuthoredPayloads():
        bits.append("has-payload")
    if prim.HasAuthoredReferences():
        bits.append("has-reference")
    if prim.GetTypeName() == "Mesh":
        pts = prim.GetAttribute("points").Get()
        bits.append(f"points={len(pts) if pts else 0}")
    print(f"{pad}{prim.GetPath()}  [{', '.join(bits)}]")
    for a in _arcs(prim):
        print(f"{pad}    arc: {a}")
    for child in prim.GetChildren():
        _dump(stage, child, depth + 1)


def main():
    for url in sys.argv[1:]:
        print("=" * 100)
        print(f"ASSET: {url}")
        print("=" * 100)
        for load_mode, label in ((Usd.Stage.LoadAll, "LoadAll"),
                                 (Usd.Stage.LoadNone, "LoadNone")):
            try:
                stage = Usd.Stage.Open(url, load_mode)
            except Exception as e:
                print(f"  [{label}] OPEN FAILED: {e}")
                continue
            dp = stage.GetDefaultPrim()
            print(f"--- {label}: defaultPrim = "
                  f"{dp.GetPath() if dp and dp.IsValid() else 'NONE'}")
            n_mesh = len([p for p in stage.Traverse()
                          if p.GetTypeName() == "Mesh"])
            print(f"    composed Mesh prims: {n_mesh}")
            if label == "LoadAll":
                for root in stage.GetPseudoRoot().GetChildren():
                    _dump(stage, root, 1)
        print()


if __name__ == "__main__":
    main()
    simulation_app.close()
