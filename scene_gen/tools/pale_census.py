#!/usr/bin/env python
"""pale_census — how much of a burnt sliced building still renders BRIGHT.

    usd_python.sh pale_census.py <bake.usd> <sidecar.json>

The acceptance criterion of the fire_dtc3 row-6 review, made countable: "the
main thing to fix was the weird pattern of unscorched prims" (user, on
`3_SM_Building_19_F3_obl.png` — a periodic checkerboard of pale pier caps
across a soot-black facade).

Per GeomSubset of every sliced piece it measures the mean luminance of the
map THAT SUBSET ACTUALLY SAMPLES — the bound material's base colour over the
subset's own wrapped UV rectangle, or its constant colour sRGB-encoded — and
counts the ones over 0.5. Never the whole atlas: a correctly sooted subset
can occupy 7% of a pale tile, and the atlas mean says nothing about what
renders (2026-08-31 finding). Split by whether the subset is inside the
fire's own z band and on a venting elevation, because a fire has to keep
reading directional and a module the plume never reached is allowed to stay
as pale as its asset shipped it.

Read-only: opens the bake USD, writes nothing.
"""
import json
import math
import sys

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade                            # noqa: E402
from PIL import Image                                             # noqa: E402
from disaster import quake_flow as qf, soot_bake as sbk           # noqa: E402
from disaster import soot_plume as spl, urban_fire as uf          # noqa: E402

PALE = float(__import__("os").environ.get("PALE_MAX", "0.5"))  # env knob, fire_dtc3 row-6 review
stage = Usd.Stage.Open(sys.argv[1])
car = json.load(open(sys.argv[2]))
m_ = car["masses"][car["fire"]["mass"]]
lv = list(m_["levels"]) + [m_["top"]]
f_ = car["fire"]
z_lo = lv[int(f_["origin"])]
z_hi = lv[min(int(f_["top"]) + 1, len(lv) - 1)]
hot = set(f_["sides"])
_IMG = {}


def tex_mean(tex, u0, u1, v0, v1):
    im = _IMG.get(tex)
    if im is None:
        try:
            Image.MAX_IMAGE_PIXELS = None
            im = np.asarray(Image.open(str(tex).replace("@", "")).convert("RGB"),
                            dtype=np.float32) / 255.0
        except Exception:
            im = False
        _IMG[tex] = im
    if im is False:
        return None
    h, w = im.shape[0], im.shape[1]
    c0, c1 = int(max(0, (u0 % 1.0) * w)), int(min(w, np.ceil(min(u1, 1.0) * w)))
    r0, r1 = int(max(0, (1.0 - min(v1, 1.0)) * h)), \
        int(min(h, np.ceil((1.0 - max(v0, 0.0)) * h)))
    if c1 <= c0 or r1 <= r0:
        c0, c1, r0, r1 = 0, w, 0, h
    return float(im[r0:r1, c0:c1].mean())


def srgb(x):
    x = max(0.0, min(1.0, float(x)))
    return 12.92 * x if x <= 0.0031308 else 1.055 * x ** (1 / 2.4) - 0.055


xfc = UsdGeom.XformCache()
rows = []
for pr in stage.Traverse():
    pp = pr.GetPath().pathString
    if "/pieces/" not in pp or not pr.IsA(UsdGeom.Mesh):
        continue
    arrays = uf._mesh_arrays(pr)
    if arrays is None:
        continue
    Mg = xfc.GetLocalToWorldTransform(pr)
    M = np.array([[float(Mg[r][c]) for c in range(4)] for r in range(4)])
    pw = arrays["points"].astype(np.float64) @ M[:3, :3] + M[3, :3]
    subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(pr)))
    for sub in (subs or [None]):
        t = sub.GetPrim() if sub is not None else pr
        ids = ([int(k) for k in (sub.GetIndicesAttr().Get() or [])]
               if sub is not None else None)
        tri, _f, slot = sbk.triangles(arrays["counts"], arrays["indices"], ids)
        if tri.shape[0] == 0:
            continue
        P = pw[np.unique(tri.reshape(-1))]
        z = P[:, 2]
        ang = math.radians(-float(m_["yaw"]))
        dx, dy = float(P[:, 0].mean()) - m_["cx"], float(P[:, 1].mean()) - m_["cy"]
        side = qf._side_of(m_, dx * math.cos(ang) - dy * math.sin(ang),
                           dx * math.sin(ang) + dy * math.cos(ang))
        mm = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
        mp = mm.GetPrim() if mm else None
        _s, _i, tex = spl.find_basecolor(mp) if mp else (None, None, None)
        if tex:
            uv = sbk._corner_uv(tri, slot, arrays["uv"], arrays["interp"],
                                arrays["uv_indices"]).reshape(-1, 2)
            lum = tex_mean(tex, float(uv[:, 0].min()), float(uv[:, 0].max()),
                           float(uv[:, 1].min()), float(uv[:, 1].max()))
        else:
            rgb = uf._flat_diffuse(mp) if mp else None
            lum = srgb(sum(rgb) / 3.0) if rgb else None
        if lum is None:
            continue
        e0, e1, e2 = pw[tri[:, 0]], pw[tri[:, 1]], pw[tri[:, 2]]
        area = float(0.5 * np.linalg.norm(np.cross(e1 - e0, e2 - e0),
                                          axis=1).sum())
        rows.append({"area": area,
                     "piece": pp.rsplit("/", 1)[-1], "sub": str(t.GetName()),
                     "side": side, "z0": float(z.min()), "z1": float(z.max()),
                     "lum": lum, "hot": side in hot,
                     "band": not (z.max() < z_lo or z.min() > z_hi),
                     "mat": (mp.GetName() if mp else "?")})


def tally(sel, label):
    pl = [r for r in sel if r["lum"] > PALE]
    ta = sum(r["area"] for r in sel) or 1.0
    pa = sum(r["area"] for r in pl)
    print("   %-44s %4d subset(s), %4d PALE (%.1f%%); pale AREA %8.1f of "
          "%9.1f m2 (%.1f%%)"
          % (label, len(sel), len(pl), 100.0 * len(pl) / max(1, len(sel)),
             pa, ta, 100.0 * pa / ta))
    return pl


print("PALE CENSUS  %s" % sys.argv[1].rsplit("/", 1)[-1])
print("   burning side(s) %s, fire band z %.1f..%.1f m of a %.1f m building; "
      "PALE = map mean over the subset's own UV rect > %.2f"
      % ("/".join(sorted(hot)), z_lo, z_hi, float(m_["top"]), PALE))
tally(rows, "WHOLE BUILDING")
inzone = [r for r in rows if r["band"] and r["hot"]]
pl = tally(inzone, "  IN THE BURN ZONE (band AND venting side)")
tally([r for r in rows if not (r["band"] and r["hot"])],
      "  outside it (may stay pale: fire is directional)")
for r in sorted(rows, key=lambda x: -(x["area"] if x["lum"] > PALE else 0))[:8]:
    if r["lum"] <= PALE:
        continue
    print("      PALE %-22s %-9s side=%s hot=%-5s band=%-5s z %6.1f..%6.1f "
          "lum %.3f area %7.1f m2  mat %s"
          % (r["piece"], r["sub"], r["side"], r["hot"], r["band"], r["z0"],
             r["z1"], r["lum"], r["area"], r["mat"][:30]))
