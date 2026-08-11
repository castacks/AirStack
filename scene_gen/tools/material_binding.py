#!/usr/bin/env python
"""
material_binding.py — does the geometry actually USE a material?

Written after a catalogue error: leaf albedo textures were sampled off disk to
classify trees as green/autumn, and the recommendation that followed was wrong,
because several of those meshes bind NO material at all. They render untextured
white however green the .png is. Sampling a texture file says nothing about
whether any mesh references it.

So the check is binding-first:

    meshes        Mesh prims in the asset
    bound         meshes (or their GeomSubsets) with a resolved bound material
    surf          bound materials whose surface output is actually connected
    tex           distinct texture files reachable from the bound shader network

`bound < meshes` means part of the asset renders untextured. `surf < bound`
means a material is bound but drives nothing — same visual result.

Two traps this handles that a naive ComputeBoundMaterial(prim) walks into:

* **GeomSubset bindings.** UE/Omniverse exports frequently leave the Mesh itself
  unbound and bind each `Section0..N` GeomSubset instead. Asking only the Mesh
  reports "unbound" for a fully textured asset.
* **Material purpose.** A binding can exist for `full` (MDL) or `preview`
  (UsdPreviewSurface) only. allPurpose alone misses those, which is exactly the
  "colour lives inside an MDL" case. Each purpose is reported separately.

Runs anywhere pxr imports. For local files that is a plain `usd-core` venv; for
`omniverse://` URLs it must be the isaac-sim container behind SimulationApp,
because only Kit resolves Nucleus paths:

    docker exec isaac-sim bash -c 'cd /isaac-sim && \
      PYTHONPATH="$ISAAC_SIM_PYTHONPATH" ./python.sh /tmp/mb.py --stdin-json'
"""

import json
import os
import sys

_NEEDS_KIT = any("omniverse://" in a for a in sys.argv[1:])
if os.environ.get("MB_SIMAPP") == "1":
    from isaacsim import SimulationApp

    _app = SimulationApp(launch_config={"headless": True})
    import carb

    _root = os.environ.get("OMNI_SERVER", "").strip().strip('"').rstrip("/")
    carb.settings.get_settings().set("/persistent/isaac/asset_root/default", _root)
else:
    _app = None

from pxr import Sdf, Usd, UsdGeom, UsdShade  # noqa: E402

PURPOSES = [
    ("all", UsdShade.Tokens.allPurpose),
    ("full", UsdShade.Tokens.full),
    ("prev", UsdShade.Tokens.preview),
]


def _textures_of(material):
    """Texture asset paths reachable from a material's shader network."""
    out = set()
    if not material:
        return out
    prim = material.GetPrim()
    if not prim.IsValid():
        return out
    for p in Usd.PrimRange(prim):
        for attr in p.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            v = attr.Get()
            if v is None:
                continue
            path = getattr(v, "path", "") or getattr(v, "resolvedPath", "")
            if path:
                out.add(os.path.basename(path))
    return out


def _bound(prim, purpose):
    """(material, bound?) for a prim, or None if nothing resolves."""
    try:
        mat, _rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(purpose)
    except Exception:
        return None
    return mat if (mat and mat.GetPrim().IsValid()) else None


def _surface_connected(mat):
    """A bound material that drives no surface renders the same as no material."""
    if not mat:
        return False
    for name in ("surface", "displacement", "volume"):
        for ctx in (None, "mdl", "glslfx"):
            out = (mat.GetOutput(name) if ctx is None
                   else mat.GetOutput(f"{ctx}:{name}"))
            if out and out.HasConnectedSource():
                return True
    src = mat.ComputeSurfaceSource()
    if isinstance(src, tuple):
        src = src[0]
    return bool(src and src.GetPrim().IsValid())


def audit(url):
    stage = Usd.Stage.Open(url)
    if stage is None:
        return {"url": url, "error": "open failed"}

    meshes = [p for p in stage.Traverse() if p.GetTypeName() == "Mesh"]
    row = {"url": url, "meshes": len(meshes), "pts": 0, "tex": set(),
           "faces": 0, "faces_unbound": 0}
    for m in meshes:
        a = m.GetAttribute("points").Get()
        if a:
            row["pts"] += len(a)
        # Face share is the honest severity measure. "1 of 1 meshes unbound" on a
        # single-mesh asset with 32 GeomSubsets reads as total loss when it may
        # be 3% of the surface — or 33%, which is the difference between a
        # cosmetic nit and an unusable asset.
        counts = m.GetAttribute("faceVertexCounts").Get()
        total = len(counts) if counts else 0
        subs = [c for c in m.GetChildren() if c.GetTypeName() == "GeomSubset"]
        if not subs:
            row["faces"] += total
            if not _bound(m, UsdShade.Tokens.allPurpose):
                row["faces_unbound"] += total
            continue
        direct = _bound(m, UsdShade.Tokens.allPurpose)
        for s in subs:
            idx = s.GetAttribute("indices").Get()
            nf = len(idx) if idx else 0
            row["faces"] += nf
            if not (_bound(s, UsdShade.Tokens.allPurpose) or direct):
                row["faces_unbound"] += nf

    for label, purpose in PURPOSES:
        n_bound = n_surf = n_partial = 0
        for m in meshes:
            # A Mesh can be unbound while its GeomSubsets carry the bindings —
            # count the Mesh as bound only if EVERY subset resolves, and record
            # the in-between case separately rather than silently rounding it
            # down to "unbound": a mesh with 3 of 5 subsets bound renders part
            # textured and part white, which is its own kind of wrong.
            subsets = [c for c in m.GetChildren()
                       if c.GetTypeName() == "GeomSubset"]
            mats = []
            direct = _bound(m, purpose)
            if direct:
                mats = [direct]
            elif subsets:
                sub = [s for s in (_bound(s, purpose) for s in subsets) if s]
                if len(sub) == len(subsets):
                    mats = sub
                elif sub:
                    n_partial += 1
            if not mats:
                continue
            n_bound += 1
            if any(_surface_connected(mm) for mm in mats):
                n_surf += 1
            for mm in mats:
                row["tex"] |= _textures_of(mm)
        row[f"bound_{label}"] = n_bound
        row[f"surf_{label}"] = n_surf
        row[f"partial_{label}"] = n_partial

    row["tex"] = sorted(row["tex"])
    return row


def main(argv):
    if argv and argv[0] == "--stdin-json":
        urls = json.load(sys.stdin)
    elif argv and argv[0] == "--json":
        urls = json.load(open(argv[1]))
    else:
        urls = argv
    if not urls:
        print(__doc__)
        return 1

    out_path = os.environ.get("MB_OUT")
    fh = open(out_path, "w") if out_path else sys.stdout

    hdr = (f"{'asset':<62} {'meshes':>6} {'bnd/all':>8} {'bnd/full':>8} "
           f"{'bnd/prev':>8} {'surf':>5} {'pts':>10}  verdict")
    print(hdr, file=fh)
    print("-" * 130, file=fh)
    rows = []
    # Flush every row: the isaac-sim container has died mid-audit and taken an
    # unflushed 218-asset result with it. Point MB_OUT at the bind-mounted repo
    # so partial output survives the container, not just the process.
    for u in urls:
        fh.flush()
        try:
            r = audit(u)
        except Exception as exc:
            print(f"{os.path.basename(u)[:62]:<62}  !! {type(exc).__name__}: {exc}",
                  file=fh)
            continue
        if "error" in r:
            print(f"{os.path.basename(u)[:62]:<62}  !! {r['error']}", file=fh)
            continue
        rows.append(r)
        n = r["meshes"]
        best = max(r["bound_all"], r["bound_full"], r["bound_prev"])
        surf = max(r["surf_all"], r["surf_full"], r["surf_prev"])
        part = max(r["partial_all"], r["partial_full"], r["partial_prev"])
        pct = (100.0 * r["faces_unbound"] / r["faces"]) if r["faces"] else 0.0
        if n == 0:
            verdict = "EMPTY (no meshes)"
        elif r["faces_unbound"] == 0 and best == n:
            verdict = "ok"
        elif pct >= 99.9:
            verdict = "*** UNBOUND — RENDERS UNTEXTURED ***"
        elif r["faces_unbound"]:
            verdict = f"*** {pct:.1f}% OF FACES UNBOUND — renders white ***"
            if part:
                verdict += f" ({part} mesh(es) part-bound by subset)"
        elif surf < best:
            verdict = f"*** BOUND BUT NO SURFACE on {best - surf} ***"
        else:
            verdict = "ok"
        print(f"{os.path.basename(u)[:62]:<62} {n:>6} {r['bound_all']:>8} "
              f"{r['bound_full']:>8} {r['bound_prev']:>8} {surf:>5} "
              f"{r['pts']:>10,}  {verdict}", file=fh)

    print("\n--- textures reachable from bound materials ---", file=fh)
    for r in rows:
        print(f"{os.path.basename(r['url'])}: "
              f"{', '.join(r['tex']) if r['tex'] else '(none)'}", file=fh)
    if out_path:
        fh.close()
    return 0


if __name__ == "__main__":
    rc = main(sys.argv[1:])
    if _app is not None:
        _app.close()
    sys.exit(rc)
