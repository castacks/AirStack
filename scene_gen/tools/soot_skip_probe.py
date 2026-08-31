#!/usr/bin/env python
"""soot_skip_probe — WHY a sliced piece's subset stayed CLEAN while its
neighbours on the same piece carry a `sootbake_*.png`.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/soot_skip_probe.py \\
        SM_Building_26 F4 162 g5 'corner_NE_0_10_0070'"

        <name> <level> <seed> [tag] [piece-regex ...]

Runs the SAME chain `fire_bake` bakes (`gac_fire.burn_gac` on a bare in-memory
stage, `random.Random(seed)` / `np.random.default_rng(seed)`, the cell named
`/World/bake/<tag>` so the printed prim paths are the bake's own), but with
`urban_fire`'s soot pass instrumented:

  * `soot_plume.piece_crop` is wrapped, and the ELEMENT is recovered from the
    caller's frame, so the `_r_soot_overlay` prefilter ("does any soot reach
    this module's rectangle?") is recorded per module rather than inferred;
  * `ctx["soot_stats"]` is a dict subclass whose `__setitem__` reads
    `_bind_soot`'s own locals (`sub_name`, `t`, `tex`, `fsel`, `a`), so every
    `inward` / `clean` / `skipped_notex` / `no_uv` / `unreadable` increment is
    filed against the exact piece + subset + material that caused it.

Nothing is written and no behaviour changes: the wrappers only observe.
Ends with a `piece_soot_probe`-style dump of the bound material of every
subset of the matched pieces, so the skip reason and the resulting look are
side by side. (fire_dtc3 review, 2026-08-30: "everything around it is burnt
except this ... repeating rectangular pattern".)
"""
import random
import math
import os
import re
import sys
import time
import traceback
from collections import Counter

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade                            # noqa: E402
from disaster import fracture, gac_fire as gf, urban_fire as uf   # noqa: E402
from disaster import quake_flow as qf                             # noqa: E402
from disaster import soot_bake as sb, soot_bake as sbk           # noqa: E402
from disaster import soot_plume as spl                            # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "SM_Building_26"
level = sys.argv[2] if len(sys.argv) > 2 else "F4"
seed = int(sys.argv[3]) if len(sys.argv) > 3 else 162
tag = sys.argv[4] if len(sys.argv) > 4 else "g5"
rx = [re.compile(p) for p in sys.argv[5:]] or [re.compile(r"^$")]

CROPS = []          # (element name, piece path, side, role, mean, max)
SKIPS = []          # (kind, piece path, subset, material, extra)


def _frame_el(depth=2):
    """The `e` (and its piece path) of the `_r_soot_overlay` frame above."""
    try:
        loc = sys._getframe(depth).f_locals
    except ValueError:
        return None
    return loc.get("e")


_orig_crop = spl.piece_crop


def _crop(sk, side, u0, u1, za, zb):
    out = _orig_crop(sk, side, u0, u1, za, zb)
    e = _frame_el(2)
    if e is not None:
        a = out[..., 3]
        CROPS.append((str(e.get("name")), str((e.get("p") or {}).get("prim_path")),
                      str(e.get("side")), str(e.get("role")),
                      float(a.mean()), float(a.max())))
    return out


spl.piece_crop = _crop

_KEYS = ("inward", "clean", "skipped_notex", "no_uv", "unreadable",
         "flat_tone", "flat_material")


class _Trace(dict):
    def __setitem__(self, k, v):
        if k in _KEYS:
            loc = sys._getframe(1).f_locals
            t, e = loc.get("t"), loc.get("e")
            fsel, a = loc.get("fsel"), loc.get("a")
            mat = None
            bp = loc.get("bprim")
            if bp is not None:
                try:
                    mat = bp.GetName()
                except Exception:
                    mat = None
            extra = []
            if fsel is not None and getattr(fsel, "size", 0):
                extra.append("faces_out=%d/%d" % (int(np.count_nonzero(fsel)),
                                                  int(fsel.size)))
            if a is not None and getattr(a, "size", 0):
                extra.append("skin_a max=%.3f mean=%.3f"
                             % (float(a.max()), float(a.mean())))
            tex = loc.get("tex")
            if tex:
                extra.append("tex=" + str(tex).rsplit("/", 1)[-1][:40])
            SKIPS.append((k,
                          str((e.get("p") or {}).get("prim_path")) if e else "?",
                          str(loc.get("sub_name") or ""),
                          mat or "?", " ".join(extra)))
        dict.__setitem__(self, k, v)


BAKES = []          # (piece path, subset, alpha mean/max, coverage, result)

_orig_bake = sb.bake_module


def _bake(sk, side, m, xform, pos, mask, base_rgb, px=None, desat=None,
          sampler=None):
    """Record what the composite was actually GIVEN — the sampled skin alpha
    — alongside the piece/subset the caller is on. A subset can be correctly
    bound to a `sootbake_*.png` copy and still render clean if the alpha it
    sampled was ~0; that is invisible in the skip tallies, which only count
    subsets that never reached here."""
    kw = {} if px is None else {"px": px}
    out = _orig_bake(sk, side, m, xform, pos, mask, base_rgb,
                     desat=desat, sampler=sampler, **kw)
    try:
        loc = sys._getframe(1).f_locals
        e = loc.get("e")
        world = np.asarray(pos)[mask] @ np.asarray(xform)[:3, :3] \
            + np.asarray(xform)[3, :3]
        a = (sampler or sb.sample_skin)(sk, side, m, world)[..., 3]
        BAKES.append((str((e.get("p") or {}).get("prim_path")) if e else "?",
                      str(loc.get("sub_name") or ""), str(side),
                      float(a.mean()), float(a.max()),
                      float(np.asarray(mask).mean()), float(out.mean())))
    except Exception:
        pass
    return out


sb.bake_module = _bake

_orig_bind = uf._bind_soot


COV = {}


def _bind(ctx, e, sk):
    COV[str((e.get("p") or {}).get("prim_path"))] = float(e.get("_soot_cover", 0.0))
    st = ctx.get("soot_stats")
    if not isinstance(st, _Trace):
        d = {"unreadable": 0, "flat_material": 0, "skipped_notex": 0,
             "flat_tone": 0, "no_uv": 0, "clean": 0, "inward": 0}
        d.update(st or {})
        ctx["soot_stats"] = _Trace(d)
    return _orig_bind(ctx, e, sk)


uf._bind_soot = _bind

t0 = time.time()
fracture.ensure_vtk(verbose=False)
st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(st.GetPrimAtPath("/World"))
UsdGeom.Scope.Define(st, "/World/bake")
cell = "/World/bake/" + tag
UsdGeom.Xform.Define(st, cell)
mats = uf.materials(st, "/World/bake")
try:
    ctx = gf.burn_gac(st, cell, name, level, random.Random(seed),
                      np.random.default_rng(seed), mats, tag,
                      flow_root=None, mat_cache={}, ssf=1.0, verbose=True)
except Exception:
    traceback.print_exc()
    sys.exit(1)
print("=" * 78)
for n in ctx["notes"]:
    if n.startswith(("smoke:", "gac:", "char:")):
        print("   note:", n)

print("-" * 78)
print("A. _r_soot_overlay CROP PREFILTER (mean < 0.01 and max < 0.15 -> module "
      "never reaches _bind_soot)")
drop = [c for c in CROPS if c[4] < 0.01 and c[5] < 0.15]
print("   %d module crop(s) taken, %d dropped by the prefilter" % (len(CROPS), len(drop)))
byrole = Counter((c[3], "drop") for c in drop)
byrole.update(Counter((c[3], "keep") for c in CROPS if c not in drop))
for k in sorted(byrole):
    print("      %-22s %s  %d" % (k[0], k[1], byrole[k]))

print("-" * 78)
print("B. _bind_soot PER-SUBSET SKIPS by reason")
print("   " + ", ".join("%s %d" % (k, sum(1 for s in SKIPS if s[0] == k))
                        for k in _KEYS))
mat_by_reason = {}
for k, path, sub, mat, extra in SKIPS:
    mat_by_reason.setdefault(k, Counter())[mat] += 1
for k in sorted(mat_by_reason):
    print("   %s:" % k)
    for mat, c in mat_by_reason[k].most_common(12):
        print("      %-46s %d" % (mat, c))

print("-" * 78)
print("B2. WHAT THE COMPOSITE WAS GIVEN — sampled skin alpha per baked subset")
if BAKES:
    am = np.array([b[3] for b in BAKES])
    print("   %d bake(s): alpha mean over subsets p10 %.3f p50 %.3f p90 %.3f; "
          "%d (%.0f%%) deposited alpha mean < 0.05 (a copy that renders CLEAN)"
          % (len(BAKES), np.percentile(am, 10), np.percentile(am, 50),
             np.percentile(am, 90), int((am < 0.05).sum()),
             100.0 * (am < 0.05).mean()))

print("-" * 78)
print("C. THE NAMED PIECES — crop, skip records, and final bound material")
for pat in rx:
    for c in CROPS:
        if pat.search(c[1] or ""):
            print("   crop  %-58s %s side=%s role=%s mean=%.4f max=%.4f"
                  % (c[1].rsplit("/", 1)[-1], c[0], c[2], c[3], c[4], c[5]))
    for b in BAKES:
        if pat.search(b[0] or ""):
            print("   BAKE  %-34s sub=%-10s side=%s alpha mean=%.3f max=%.3f "
                  "covered=%.2f out_mean=%.3f"
                  % (b[0].rsplit("/", 1)[-1], b[1], b[2], b[3], b[4], b[5],
                     b[6]))
    for s in SKIPS:
        if pat.search(s[1] or ""):
            print("   SKIP  %-10s %-34s sub=%-10s mat=%-34s %s"
                  % (s[0], s[1].rsplit("/", 1)[-1], s[2], s[3], s[4]))


def sinfo(mat):
    if not mat:
        return "(none)"
    for c in Usd.PrimRange(mat.GetPrim()):
        sh = UsdShade.Shader(c)
        if sh and sh.GetIdAttr().Get() == "UsdPreviewSurface":
            bits = []
            for nm in ("diffuseColor", "roughness", "metallic", "opacity"):
                i = sh.GetInput(nm)
                if not i:
                    continue
                if i.HasConnectedSource():
                    ts = UsdShade.Shader(i.GetConnectedSource()[0].GetPrim())
                    f = ts.GetInput("file")
                    bits.append("%s=tex:%s" % (nm, str(f.Get() if f else None)
                                               .rsplit("/", 1)[-1][:56]))
                else:
                    bits.append("%s=%s" % (nm, i.Get()))
            return "%s  [%s]" % (mat.GetPrim().GetName(), " ".join(bits))
    return mat.GetPrim().GetName() + " (no preview surface)"


for pr in st.Traverse():
    p = pr.GetPath().pathString
    if not any(r.search(p) for r in rx) or pr.GetTypeName() != "Mesh":
        continue
    subs = [c for c in pr.GetChildren() if c.GetTypeName() == "GeomSubset"]
    print("   " + p)
    for c in (subs or [pr]):
        m = UsdShade.MaterialBindingAPI(c).ComputeBoundMaterial()[0]
        print("      sub %-22s -> %s" % (c.GetName(), sinfo(m)))

# ---------------------------------------------------------------------------
# D. THE PALE CENSUS — what still renders BRIGHT inside the burn zone
# ---------------------------------------------------------------------------
# "The main thing to fix was the weird pattern of unscorched prims" (user, on
# `~/fire_previews/fire_dtc3/3_SM_Building_19_F3_obl.png`: a periodic
# checkerboard of pale pier caps / spandrel end-bands across a soot-black
# facade). A subset is counted here when it sits on a BURNING side, inside
# the fire's own z band, and the map it actually samples still reads bright.
# Luminance is measured over the subset's OWN wrapped UV rectangle, never the
# whole atlas -- a correctly sooted subset can sit in 7% of a pale tile and
# the atlas mean says nothing about what renders (2026-08-31 finding).
from PIL import Image                                              # noqa: E402

_IMG = {}


def _tex_mean(tex, u0, u1, v0, v1):
    im = _IMG.get(tex)
    if im is None:
        try:
            Image.MAX_IMAGE_PIXELS = None
            a = np.asarray(Image.open(str(tex).replace("@", "")).convert("RGB"),
                           dtype=np.float32) / 255.0
        except Exception:
            a = False
        _IMG[tex] = a
        im = a
    if im is False:
        return None
    h, w = im.shape[0], im.shape[1]
    c0, c1 = int(max(0, (u0 % 1.0) * w)), int(min(w, np.ceil(min(u1, 1.0) * w)))
    r0, r1 = int(max(0, (1.0 - min(v1, 1.0)) * h)), \
        int(min(h, np.ceil((1.0 - max(v0, 0.0)) * h)))
    if c1 <= c0 or r1 <= r0:
        c0, c1, r0, r1 = 0, w, 0, h
    return float(im[r0:r1, c0:c1].mean())


def _srgb(x):
    x = max(0.0, min(1.0, float(x)))
    return 12.92 * x if x <= 0.0031308 else 1.055 * x ** (1 / 2.4) - 0.055


f_ = ctx["fire"]
m_ = ctx["info"]["masses"][f_["mass"]]
lv = list(m_["levels"]) + [m_["top"]]
z_lo = lv[int(f_["origin"])]
z_hi = lv[min(int(f_["top"]) + 1, len(lv) - 1)]
hot = set(f_["sides"])
bucket = {}
for k, path, sub, mat, extra in SKIPS:
    bucket[(path, sub)] = k
for b in BAKES:
    bucket.setdefault((b[0], b[1]), "bake(a=%.3f)" % b[3])
xfc2 = UsdGeom.XformCache()
rows = []
for pr in st.Traverse():
    pp = pr.GetPath().pathString
    if "/pieces/" not in pp or not pr.IsA(UsdGeom.Mesh):
        continue
    arrays = uf._mesh_arrays(pr)
    if arrays is None:
        continue
    Mg = xfc2.GetLocalToWorldTransform(pr)
    M2 = np.array([[float(Mg[r][c]) for c in range(4)] for r in range(4)])
    pw = arrays["points"].astype(np.float64) @ M2[:3, :3] + M2[3, :3]
    subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(pr)))
    for sub in (subs or [None]):
        t2 = sub.GetPrim() if sub is not None else pr
        ids = ([int(k) for k in (sub.GetIndicesAttr().Get() or [])]
               if sub is not None else None)
        tri, _f2, slot = sbk.triangles(arrays["counts"], arrays["indices"], ids)
        if tri.shape[0] == 0:
            continue
        vs = np.unique(tri.reshape(-1))
        P = pw[vs]
        zc = P[:, 2]
        cx_, cy_ = float(P[:, 0].mean()), float(P[:, 1].mean())
        ang = math.radians(-float(m_["yaw"]))
        dx_, dy_ = cx_ - float(m_["cx"]), cy_ - float(m_["cy"])
        side_ = qf._side_of(m_, dx_ * math.cos(ang) - dy_ * math.sin(ang),
                            dx_ * math.sin(ang) + dy_ * math.cos(ang))
        mm = UsdShade.MaterialBindingAPI(t2).ComputeBoundMaterial()[0]
        mprim = mm.GetPrim() if mm else None
        _s2, _i2, tex2 = spl.find_basecolor(mprim) if mprim else (None, None, None)
        if tex2:
            uvs = sbk._corner_uv(tri, slot, arrays["uv"], arrays["interp"],
                                 arrays["uv_indices"]).reshape(-1, 2)
            lum = _tex_mean(tex2, float(uvs[:, 0].min()), float(uvs[:, 0].max()),
                            float(uvs[:, 1].min()), float(uvs[:, 1].max()))
        else:
            rgb = uf._flat_diffuse(mprim) if mprim else None
            lum = (_srgb(sum(rgb) / 3.0) if rgb else None)
        if lum is None:
            continue
        key = (pp, str(t2.GetName()))
        e0, e1, e2 = pw[tri[:, 0]], pw[tri[:, 1]], pw[tri[:, 2]]
        area = float(0.5 * np.linalg.norm(np.cross(e1 - e0, e2 - e0),
                                          axis=1).sum())
        rows.append({"area": area,
                     "piece": pp, "sub": str(t2.GetName()), "side": side_,
                     "z0": float(zc.min()), "z1": float(zc.max()), "lum": lum,
                     "band": not (zc.max() < z_lo or zc.min() > z_hi),
                     "hot": side_ in hot,
                     "cov": COV.get(pp, 0.0),
                     "bucket": bucket.get(key, "(not visited)")})
PALE = float(os.environ.get("PALE_MAX", "0.5"))
print("-" * 78)
print("D. PALE CENSUS — what still renders BRIGHT  (burning side(s) %s, "
      "fire band z %.1f..%.1f m of a %.1f m building)"
      % ("/".join(sorted(hot)), z_lo, z_hi, float(m_["top"])))
print("   luminance is the mean of the map over the subset's OWN wrapped UV "
      "rect; PALE = > %.2f" % PALE)


def _tally(sel, label):
    n = len(sel)
    pl = [r for r in sel if r["lum"] > PALE]
    ta = sum(r["area"] for r in sel) or 1.0
    pa = sum(r["area"] for r in pl)
    print("   %-46s %4d subset(s), %4d PALE (%.1f%%); pale AREA %8.1f of "
          "%8.1f m2 (%.1f%%)"
          % (label, n, len(pl), 100.0 * len(pl) / max(1, n), pa, ta,
             100.0 * pa / ta))
    return pl


_tally(rows, "WHOLE BUILDING")
_tally([r for r in rows if r["band"] and r["hot"]],
       "  IN THE BURN ZONE (band AND venting side)")
_tally([r for r in rows if r["hot"] and not r["band"] and r["z0"] >= z_lo],
       "  venting side, ABOVE the band")
_tally([r for r in rows if not r["hot"]], "  COLD sides (may stay pale)")
pl_soaked = _tally([r for r in rows if r["cov"] >= 0.35],
                   "  on a SOAKED module (cov >= 0.35)")
_tally([r for r in rows if r["cov"] < 0.35],
       "  on a dry module (cov < 0.35) — must stay pale")
bb = {}
for r in pl_soaked:
    bb[r["bucket"]] = bb.get(r["bucket"], 0) + 1
print("   the PALE-on-a-SOAKED-module population, by bucket:")
for k in sorted(bb, key=lambda x: -bb[x]):
    print("      %-24s %4d" % (k, bb[k]))
pl_dry = [r for r in rows if r["cov"] < 0.35 and r["lum"] > PALE]
print("   the PALE-on-a-DRY-module population (left alone by design), "
      "biggest first:")
for r in sorted(pl_dry, key=lambda x: -x["area"])[:10]:
    print("      pale %-24s %-10s side=%s hot=%-5s band=%-5s z %6.1f..%6.1f "
          "lum %.3f cov %.2f area %7.1f m2 <- %s"
          % (r["piece"].rsplit("/", 1)[-1], r["sub"], r["side"], r["hot"],
             r["band"], r["z0"], r["z1"], r["lum"], r["cov"], r["area"],
             r["bucket"]))
for r in pl_soaked[:8]:
    print("      PALE %-24s %-10s side=%s z %6.1f..%6.1f lum %.3f cov %.2f "
          "<- %s" % (r["piece"].rsplit("/", 1)[-1], r["sub"], r["side"],
                     r["z0"], r["z1"], r["lum"], r["cov"], r["bucket"]))
tone = [r for r in rows if r["sub"] and r["bucket"] == "flat_tone"]
print("   flat-tone binds: %d total, %d outside the fire band, %d on a cold "
      "side, %d on a dry module (cov < 0.35)"
      % (len(tone), sum(1 for r in tone if not r["band"]),
         sum(1 for r in tone if not r["hot"]),
         sum(1 for r in tone if r["cov"] < 0.35)))
# ---------------------------------------------------------------------------
# E. HOW HIGH DOES THE SKIN ITSELF CLIMB, per elevation
# ---------------------------------------------------------------------------
# A face that is correctly SELECTED still bakes clean if the skin carries no
# alpha at its height. This samples the building's own skin on a vertical
# line 0.1 m proud of each elevation, so "the plume never got there" and
# "the face was never selected" cannot be confused.
try:
    _sk = uf._soot_skin(ctx, 1.0)
    if _sk is not None:
        import numpy as _np
        zs = _np.linspace(float(m_["z0"]) + 1.0, float(m_["top"]) - 0.5, 12)
        print("-" * 78)
        print("E. SKIN ALPHA vs HEIGHT (fire band z %.1f..%.1f)" % (z_lo, z_hi))
        for sd in ("S", "E", "N", "W"):
            ox3, oy3 = qf._outward(m_, sd)
            pts = _np.stack([
                _np.full_like(zs, float(m_["cx"]) + ox3 * (
                    (m_["W"] if sd in ("E", "W") else m_["D"]) / 2.0 + 0.1)),
                _np.full_like(zs, float(m_["cy"]) + oy3 * (
                    (m_["W"] if sd in ("E", "W") else m_["D"]) / 2.0 + 0.1)),
                zs], axis=1)
            al = sbk.sample_skin(_sk, sd, m_, pts)[:, 3]
            print("   %s%s  %s" % (sd, "*" if sd in hot else " ",
                                   " ".join("%.0fm:%.2f" % (z, a)
                                            for z, a in zip(zs, al))))
        print("   (* = venting elevation)")
except Exception as _exc:
    print("E. skin probe unavailable:", _exc)
print("WALL_S %.0f" % (time.time() - t0))
