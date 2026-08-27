"""Structural pre-screen for `glass_separable` — does this car have glass as
its own Mesh, bound to a fractional-opacity material?

NOT a substitute for `car_occupants_launch_script.py`; it only says whether
there is anything for `strip_glass` to remove, and whether removing it would
also take the roof (the Nissan failure).
"""
import sys
from pxr import Usd, UsdGeom, UsdShade

GLASSY = ("glass", "window", "windscreen", "windshield", "glazing")


def opacity_of(mat):
    lo = 1.0
    for sh in Usd.PrimRange(mat.GetPrim()):
        s = UsdShade.Shader(sh)
        if not s:
            continue
        for n in ("opacity", "opacity_constant"):
            a = s.GetInput(n)
            v = a.Get() if a else None
            if isinstance(v, float):
                lo = min(lo, v)
    return lo


for url in sys.argv[1:]:
    st = Usd.Stage.Open(url)
    name = url.rsplit("/", 1)[-1]
    if st is None:
        print(f"{name}\tOPEN-FAILED"); continue
    meshes, glass, tot_pts, glass_pts = 0, [], 0, 0
    for p in st.Traverse():
        if not p.IsA(UsdGeom.Mesh):
            continue
        meshes += 1
        pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
        n = len(pts) if pts else 0
        tot_pts += n
        nm = p.GetName().lower()
        is_g = any(g in nm for g in GLASSY)
        try:
            mat, _ = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()
        except Exception:
            mat = None
        mo = opacity_of(mat) if mat else 1.0
        mn = mat.GetPrim().GetName().lower() if mat else ""
        if is_g or mo < 1.0 or any(g in mn for g in GLASSY):
            glass.append((p.GetName(), mn, round(mo, 2), n))
            glass_pts += n
    frac = (100.0 * glass_pts / tot_pts) if tot_pts else 0.0
    verdict = ("no separable glass" if not glass else
               ("SEPARABLE" if frac < 25 else f"RISKY ({frac:.0f}% of points)"))
    print(f"{name}\t{meshes} meshes\t{verdict}")
    for g in glass[:5]:
        print(f"      mesh={g[0]!r} mat={g[1]!r} opacity={g[2]} pts={g[3]}")
