"""aec_mdl_probe — bare-USD inspection of an already-baked AEC brownstone
fire bake: material class histogram (MDL vs UsdPreviewSurface), which
subsets are on ShellFallbackLooks (and why: what their source `mats[]`
material actually is), and a fit-out-vs-shell bbox comparison.

Read-only. No Kit, no GPU. Run via usd_python.sh:
  docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \
    ./scene_gen/tools/usd_python.sh scene_gen/tools/aec_mdl_probe.py <bake.usd>"
"""
import sys
from collections import Counter


def main(path):
    from pxr import Usd, UsdGeom, UsdShade

    stage = Usd.Stage.Open(path)
    if stage is None:
        print("FAILED to open", path)
        return

    shader_id_hist = Counter()
    mat_class_hist = Counter()          # per MATERIAL prim: "mdl" / "preview" / "both" / "neither"
    shellfallback_pieces = []
    all_pieces_mesh = 0
    mdl_pieces = 0
    preview_pieces = 0

    def classify_material(mat_prim):
        has_mdl, has_prev = False, False
        for p in Usd.PrimRange(mat_prim):
            sh = UsdShade.Shader(p)
            if not sh:
                continue
            sid = sh.GetIdAttr().Get()
            shader_id_hist[str(sid)] += 1
            if sid == "UsdPreviewSurface":
                has_prev = True
            impl = sh.GetImplementationSourceAttr().Get() if sh.GetImplementationSourceAttr() else None
            src_asset = None
            try:
                src_asset = sh.GetSourceAsset("mdl")
            except Exception:
                pass
            if src_asset or (sid and ".mdl" in str(sid).lower()):
                has_mdl = True
        if has_mdl and has_prev:
            return "both"
        if has_mdl:
            return "mdl"
        if has_prev:
            return "preview"
        return "neither"

    # 1. material class histogram across every bound Material prim under /World
    root = stage.GetPseudoRoot()
    seen_mats = set()
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdShade.Material):
            if prim.GetPath() in seen_mats:
                continue
            seen_mats.add(prim.GetPath())
            cls = classify_material(prim)
            mat_class_hist[cls] += 1

    # 2. per-mesh: what class of material does it resolve to (direct or via subset)?
    shellfb_count = 0
    total_meshes = 0
    mesh_mat_class_hist = Counter()
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        total_meshes += 1
        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
        targets = ([s.GetPrim() for s in subs] or [prim])
        for t in targets:
            mb = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
            if not mb or not mb.GetPrim().IsValid():
                mesh_mat_class_hist["unbound"] += 1
                continue
            mpath = str(mb.GetPrim().GetPath())
            if "ShellFallbackLooks" in mpath:
                shellfb_count += 1
                if len(shellfallback_pieces) < 30:
                    shellfallback_pieces.append((str(prim.GetPath()), mpath))
                mesh_mat_class_hist["shellfallback"] += 1
            else:
                cls = classify_material(mb.GetPrim())
                mesh_mat_class_hist[cls] += 1

    print("=== FILE:", path)
    print("--- material PRIM class histogram (unique materials) ---")
    for k, v in mat_class_hist.most_common():
        print("  {0:12s} {1}".format(k, v))
    print("--- shader id histogram (all shader prims under any material) ---")
    for k, v in shader_id_hist.most_common(20):
        print("  {0:30s} {1}".format(k, v))
    print("--- per-mesh/subset bound-material class histogram ---")
    print("  total mesh prims:", total_meshes)
    for k, v in mesh_mat_class_hist.most_common():
        print("  {0:14s} {1}".format(k, v))
    print("--- sample ShellFallbackLooks bindings (mesh/subset path -> material) ---")
    for p, mp in shellfallback_pieces:
        print("  ", p, "->", mp)

    # 3. wall_x_* pieces: are they ALL ShellFallback, or partially?
    x_pieces = Counter()
    x_shellfb = Counter()
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        name = prim.GetName()
        if "_x_" not in name and not name.startswith("wall_x"):
            continue
        x_pieces["total"] += 1
        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
        targets = ([s.GetPrim() for s in subs] or [prim])
        any_fallback = False
        any_real = False
        for t in targets:
            mb = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
            mpath = str(mb.GetPrim().GetPath()) if mb and mb.GetPrim().IsValid() else ""
            if "ShellFallbackLooks" in mpath:
                any_fallback = True
            elif mpath:
                any_real = True
        if any_fallback and not any_real:
            x_shellfb["all_fallback"] += 1
        elif any_fallback and any_real:
            x_shellfb["mixed"] += 1
        elif any_real:
            x_shellfb["all_real"] += 1
        else:
            x_shellfb["unbound"] += 1
    print("--- '_x' (region-cut merged) pieces ---")
    print("  total:", x_pieces["total"])
    for k, v in x_shellfb.most_common():
        print("  {0:14s} {1}".format(k, v))

    # 4. fit-out vs shell bbox
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    def bbox_of(paths_prefixes):
        import numpy as np
        lo = np.full(3, np.inf)
        hi = np.full(3, -np.inf)
        n = 0
        for prim in Usd.PrimRange(root):
            name = prim.GetName()
            pp = str(prim.GetPath())
            if not any(pref in pp for pref in paths_prefixes):
                continue
            if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Xform)):
                continue
            r = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            mn, mx = r.GetMin(), r.GetMax()
            lo = np.minimum(lo, [mn[0], mn[1], mn[2]])
            hi = np.maximum(hi, [mx[0], mx[1], mx[2]])
            n += 1
        return lo, hi, n

    shell_lo, shell_hi, n_shell = bbox_of(("/pieces/wall", "/pieces/corner",
                                           "/pieces/pier", "/pieces/roof",
                                           "/pieces/parapet"))
    fit_lo, fit_hi, n_fit = bbox_of(("slab_", "col_", "part_", "prop_", "/fit"))
    print("--- shell vs fit-out bbox ---")
    print("  shell  n={0} lo={1} hi={2}".format(n_shell, shell_lo, shell_hi))
    print("  fitout n={0} lo={1} hi={2}".format(n_fit, fit_lo, fit_hi))
    if n_shell and n_fit:
        over_lo = fit_lo - shell_lo   # negative = fit-out pokes OUT below/left
        over_hi = fit_hi - shell_hi   # positive = fit-out pokes OUT above/right
        print("  fit lo - shell lo (negative = overshoot out):", over_lo)
        print("  fit hi - shell hi (positive = overshoot out):", over_hi)


if __name__ == "__main__":
    for p in sys.argv[1:]:
        main(p)
