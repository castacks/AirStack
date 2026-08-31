#!/usr/bin/env python
"""frag_facade_probe — DOES A TEAR'S FRAGMENT READ AS AN EXTENSION OF ITS WALL?

The offline answer to the second half of the fire_dtc3 review (2026-08-30):

    "the ragged break pieces ... are a completely diff texture/color from the
     wall they extend ... make them look like extensions of the wall it's
     attached to"

`quake_flow._break_split` writes a tear's fragments under
`<parent>/brk_<tag>_<piece>/frag_NNN`. Each carries TWO bindings: one on the
prim (the façade half — `static_mat`, i.e. `_clad_material`'s TRIPLANAR, or
`_mat_fn`'s invented core when the piece had no readable texture) and one on
the `core` GeomSubset (`_t_core_bind`, the faces the fracture invented).
`fire_collapse._refire` then writes the fire palette over one or both.

This prints, per fragment: the PRIM-level bound material + its diffuse source,
the `core` subset's, and whether the mesh carries `primvars:st` at all — a
Voronoi fragment does NOT (`fracture.prim_to_mesh` reads only points / counts /
indices and `_write_mesh` writes no UVs), so any UV-mapped atlas bound to it
samples ONE texel and a triplanar projection of a UNIQUE-unwrap atlas smears
unrelated regions of it across the break. Both are the review's "completely
diff texture/color".

    usd_python.sh frag_facade_probe.py <bake.usd> [<regex> ...]

With no regex every `brk_*` fragment in the stage is reported.
"""
import re
import sys

from pxr import Usd, UsdGeom, UsdShade

st = Usd.Stage.Open(sys.argv[1])
rx = [re.compile(p) for p in sys.argv[2:]] or [re.compile(r"/brk_")]


def diffuse(mpath):
    """(material name, diffuse source) for a material by PATH.

    By path, never by handle: a `UsdShade.Material` handed out by
    `ComputeBoundMaterial` inside a `Traverse` expires the moment the
    traversal moves on, and reading an input off it throws
    UsdExpiredPrimAccessError.
    """
    if not mpath:
        return "(none)", "(none)"
    mp = st.GetPrimAtPath(mpath)
    if not mp or not mp.IsValid():
        return "(expired)", "(none)"
    for c in mp.GetChildren():
        sh = UsdShade.Shader(c)
        if not sh:
            continue
        for name in ("diffuseColor", "diffuse_color_constant", "base_color",
                     "diffuse_tint"):
            i = sh.GetInput(name)
            if i is None:
                continue
            try:
                if i.HasConnectedSource():
                    src = i.GetConnectedSource()[0].GetPrim()
                    f = UsdShade.Shader(src).GetInput("file")
                    v = f.Get() if f else None
                    return mp.GetName(), "tex:" + str(v).rsplit("/", 1)[-1].rstrip("@")
                v = i.Get()
            except Exception:
                continue
            if v is not None:
                return mp.GetName(), "rgb:" + str(v)
    return mp.GetName(), "(no diffuse input)"


def bound_path(prim):
    try:
        m = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    except Exception:
        return None
    return m.GetPrim().GetPath().pathString if (m and m.GetPrim().IsValid()) else None


def has_uv(prim):
    for p in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
        if p.GetPrimvarName().startswith(("st", "uv", "UV")):
            v = p.Get()
            return "st[%d]" % (len(v) if v is not None else 0)
    return "NO-UV"


rows = []
for pr in st.Traverse():
    p = pr.GetPath().pathString
    if not any(r.search(p) for r in rx) or pr.GetTypeName() != "Mesh":
        continue
    rows.append((p, has_uv(pr), bound_path(pr),
                 [(c.GetName(), bound_path(c)) for c in pr.GetChildren()
                  if c.GetTypeName() == "GeomSubset"]))

buckets = {}
for p, uv, mp, subs in rows:
    nm, df = diffuse(mp)
    print("%s  %s" % (p, uv))
    print("    prim -> %-30s %s" % (nm, df))
    for sn, sp in subs:
        snm, sdf = diffuse(sp)
        print("    sub %-8s -> %-30s %s" % (sn, snm, sdf))
    key = nm.rstrip("0123456789_")
    buckets[key] = buckets.get(key, 0) + 1
print("[frag_facade_probe] %d mesh(es); prim-level material buckets: %s" % (
    len(rows), ", ".join("%s=%d" % kv for kv in sorted(buckets.items())) or "(none)"))
