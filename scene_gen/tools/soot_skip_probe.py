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
import re
import sys
import time
import traceback
from collections import Counter

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade                            # noqa: E402
from disaster import fracture, gac_fire as gf, urban_fire as uf   # noqa: E402
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


_orig_bind = uf._bind_soot


def _bind(ctx, e, sk):
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
print("C. THE NAMED PIECES — crop, skip records, and final bound material")
for pat in rx:
    for c in CROPS:
        if pat.search(c[1] or ""):
            print("   crop  %-58s %s side=%s role=%s mean=%.4f max=%.4f"
                  % (c[1].rsplit("/", 1)[-1], c[0], c[2], c[3], c[4], c[5]))
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
print("WALL_S %.0f" % (time.time() - t0))
