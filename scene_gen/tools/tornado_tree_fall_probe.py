#!/usr/bin/env python
"""tornado_tree_fall_probe -- what a windthrown STREET TREE actually looks
like after `vegetation.tip_tree`, measured on a bare Usd stage.

    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \\
        bash scene_gen/tools/usd_python.sh \\
        scene_gen/tools/tornado_tree_fall_probe.py"

For every candidate species it prints, per lean angle: the world min-z of
the whole tree, the min-z of the TRUNK-only geometry, and the horizontal
distance from the trunk base to the crown's bbox centre (the "reads fallen
from 60-90 m" metric -- once that offset exceeds the crown radius, the
canopy no longer sits over its own stump).
"""
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from pxr import Gf, Sdf, Usd, UsdGeom                          # noqa: E402

from disaster import vegetation as veg                          # noqa: E402

ROOT = "/isaac-sim/AirStack/scene_gen/assets/aec"
SPECIES = [
    ("Shumard_Oak", ROOT + "/tower/Assets/Vegetation/Shumard_Oak/"
     "Shumard_Oak.usd"),
    ("Black_Oak", ROOT + "/tower/Assets/Vegetation/Black_Oak/Black_Oak.usd"),
    ("Douglas_Fir", ROOT + "/brownstone/Assets/Vegetation/Trees/"
     "Douglas_Fir.usd"),
]
SCALE = 0.01
LEANS = [0.0, 30.0, 46.0, 62.0, 70.0, 77.0, 82.0, 88.0]


def describe(stage, path):
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    rows = []
    for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies(
            Usd.PrimIsActive & Usd.PrimIsDefined)):
        if not p.IsA(UsdGeom.Gprim) and not p.IsA(UsdGeom.PointInstancer):
            continue
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_,
                                UsdGeom.Tokens.render])
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        n = 0
        if p.IsA(UsdGeom.PointBased):
            pts = UsdGeom.PointBased(p).GetPointsAttr().Get()
            n = len(pts) if pts else 0
        rows.append((p.GetPath().pathString, p.GetTypeName(), n,
                     None if r.IsEmpty() else
                     (r.GetMin()[2], r.GetMax()[2],
                      max(r.GetSize()[0], r.GetSize()[1]))))
    return rows


def measure(stage, path):
    """(min_z, crown_dx, crown_r) over every gprim under *path*."""
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    cx = 0.5 * (mn[0] + mx[0])
    cy = 0.5 * (mn[1] + mx[1])
    return (float(mn[2]), math.hypot(cx, cy),
            0.5 * max(float(mx[0] - mn[0]), float(mx[1] - mn[1])))


def main():
    for name, usd in SPECIES:
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        path = "/World/tree"
        UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        prim = stage.DefinePrim(path)
        if not prim.GetReferences().AddReference(usd):
            print("!! could not reference {0}".format(usd))
            continue
        prim.Load()
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
        xf.AddRotateZOp().Set(0.0)
        xf.AddScaleOp().Set(Gf.Vec3f(SCALE, SCALE, SCALE))
        base = measure(stage, path)
        print("")
        print("=" * 74)
        print("{0}  standing: min_z={1:.2f}  crown_r={2:.2f}".format(
            name, base[0], base[2]))
        for row in describe(stage, path):
            pth, ty, n, bb = row
            print("    {0:<52} {1:<14} pts={2}".format(
                pth[-52:], str(ty), n))
            if bb:
                print("        z {0:.2f}..{1:.2f}  span {2:.2f}".format(
                    bb[0], bb[1], bb[2]))
        print("  lean   min_z   crown_off   off/r    lift_to_seat")
        for lean in LEANS:
            veg.tip_tree(stage, path, lean, azimuth_deg=0.0, lift_m=0.0,
                         seat_band=None)
            m = measure(stage, path)
            print("  {0:5.1f} {1:8.2f} {2:10.2f} {3:8.2f} {4:12.2f}".format(
                lean, m[0], m[1], m[1] / max(1e-6, base[2]), -m[0]))
        # what tip_tree's own bisection settles on with the shipped band
        for band, lmin in ((( -1.1, -0.15), 46.0), ((-2.5, -0.15), 62.0),
                           ((-4.0, -0.15), 70.0)):
            got = veg.tip_tree(stage, path, 82.0, azimuth_deg=0.0,
                               lift_m=0.16, seat_band=band, lean_min_deg=lmin)
            m = measure(stage, path)
            print("  band={0} lean_min={1}: settled lean {2:.1f}, "
                  "min_z {3:.2f}, crown_off {4:.2f}".format(
                      band, lmin, got, m[0], m[1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
