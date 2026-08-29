#!/usr/bin/env python
"""Is a whole-asset building actually a BAG OF PIECES underneath?

`burn_building` needs a building decomposed into addressable parts, because
its recipes work by taking parts away. `burn_monolith` exists only because the
whole-asset packs were assumed to be single un-openable meshes. That
assumption has never been measured per pack — and the user's claim is that
`ModernCityEnvironment` (whole) and `ModernCityEnvironment01` (kit) are two
exports of the SAME scene, in which case the merged one may still carry its
pieces and could take the full ladder.

Prints, per asset: mesh count, how many distinct prim names, the size
distribution of the meshes, and a sample of names. A bag of pieces looks like
"many small meshes, piece-shaped names"; a monolith looks like "1-3 meshes,
one of them the whole building".

Bare pxr, no Kit: safe beside a running sim.
    docker exec isaac-sim bash scene_gen/tools/usd_python.sh \
        scene_gen/tools/pack_structure_probe.py
"""
import os
import sys

import numpy as np
from pxr import Usd, UsdGeom

SEI = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
# `urban.yaml`'s entries are relative to the asset set's `asset_root`, which is
# Library/Stages — NOT Projects/SEI-COA. Getting that wrong is why the first
# run of this probe reported "Failed to open layer" for every Muyang asset.
LIB = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/"
TARGETS = [
    ("MCE whole MBuilding01", LIB + "Muyang/ModernCityEnvironment/Collected_Building01/SM_MERGED_BP_MBuilding01.usd"),
    ("MCE whole MBuilding02", LIB + "Muyang/ModernCityEnvironment/Collected_Building02/SM_MERGED_BP_MBuilding02.usd"),
    ("MCE whole MBuilding05", LIB + "Muyang/ModernCityEnvironment/Collected_Building05/SM_MERGED_BP_MBuilding05.usd"),
    ("MuyangDT BG_Building_A", LIB + "Muyang/DownTown/Assets/BG_Building_A.usd"),
    ("Dmytro  Building_TypeA_A", LIB + "Dmytro/Assets/Game/FactoryDistrict/Meshes/Building_TypeA_A.usd"),
    ("GAC     SM_Building_01", SEI + "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/SM_Building_01.usd"),
]

EXTRA = os.environ.get("PROBE_EXTRA", "")
if EXTRA:
    for i, u in enumerate(EXTRA.split(",")):
        TARGETS.append(("extra%d" % i, u.strip()))


def report(label, url):
    st = Usd.Stage.Open(url)
    if st is None:
        print("%-26s OPEN FAILED" % label)
        return
    mpu = UsdGeom.GetStageMetersPerUnit(st)
    xc = UsdGeom.XformCache()
    rows = []
    for prim in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        if pts is None or not len(pts):
            continue
        M = np.asarray(xc.GetLocalToWorldTransform(prim), dtype=float)
        P = np.asarray(pts, dtype=float)
        P = (np.c_[P, np.ones(len(P))] @ M)[:, :3] * mpu
        sz = P.max(axis=0) - P.min(axis=0)
        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
        rows.append((prim.GetName(), len(pts), sz, len(subs)))
    if not rows:
        print("%-26s no meshes" % label)
        return
    diag = np.array([max(r[2]) for r in rows])
    big = sum(1 for d in diag if d > 15.0)
    small = sum(1 for d in diag if d <= 8.0)
    names = [r[0] for r in rows]
    print("%-26s mpu=%-6.3g meshes=%-5d subsets=%-4d distinct names=%d"
          % (label, mpu, len(rows), sum(r[3] for r in rows), len(set(names))))
    print("%-26s mesh longest-edge: min %.2f  median %.2f  max %.2f m"
          % ("", diag.min(), float(np.median(diag)), diag.max()))
    print("%-26s %d mesh(es) > 15 m (building-scale), %d <= 8 m (piece-scale)"
          % ("", big, small))
    print("%-26s names: %s" % ("", ", ".join(sorted(set(names))[:6])))
    verdict = ("BAG OF PIECES — burn_building may be reachable"
               if small >= 8 and small > big else
               "MONOLITH — one/few building-scale meshes")
    print("%-26s -> %s\n" % ("", verdict))


for label, url in TARGETS:
    try:
        report(label, url)
    except Exception as exc:
        print("%-26s FAILED %s\n" % (label, exc))
