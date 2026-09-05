"""Cold-stage material completeness checks used by dataset exports.

An asset path can resolve while a mesh still renders white: its binding may
target a prim that vanished during flatten, a typeless placeholder, or no
material at all. This checks the composed objects the renderer sees, including
instance proxies and material-binding subsets.
"""
from collections import Counter


def _display_color(prim):
    from pxr import UsdGeom
    try:
        value = UsdGeom.PrimvarsAPI(prim).FindPrimvarWithInheritance(
            "displayColor")
        return bool(value and value.HasValue() and value.Get())
    except Exception:  # pragma: no cover - malformed third-party prim
        return False


def _authored_targets(prim):
    out = []
    for rel in prim.GetRelationships():
        if rel.GetName().startswith("material:binding"):
            out.extend(str(path) for path in rel.GetTargets())
    return out


def audit(stage, max_examples=25):
    """Return a bounded report of visible mesh material failures.

    Display-colour-only meshes are accepted: procedural road markings and
    similar geometry deliberately use that representation. Dangling/typeless
    targets, surface-less materials, and visible faces with neither a material
    nor display colour are hard failures.
    """
    from pxr import Usd, UsdGeom, UsdShade

    counts = Counter()
    examples = {
        "unbound_uncolored": [], "dangling_targets": [],
        "typeless_targets": [], "surface_less_materials": [],
        "unassigned_faces": [],
    }
    checked_materials = set()

    def example(key, value):
        if len(examples[key]) < int(max_examples):
            examples[key].append(value)

    def check_target(prim, role):
        counts["targets"] += 1
        # Procedural colour-only geometry normally has no binding at all.
        # Avoid an expensive inherited-binding walk (and a USD warning) for
        # that intentional representation. A BROKEN authored binding still
        # goes through the full check even when displayColor is present.
        authored = _authored_targets(prim)
        if not authored and _display_color(prim):
            counts["display_color_only"] += 1
            return True
        try:
            material = UsdShade.MaterialBindingAPI(
                prim).ComputeBoundMaterial()[0]
        except Exception as exc:  # pragma: no cover
            counts["binding_errors"] += 1
            example("unbound_uncolored", {
                "prim": str(prim.GetPath()), "role": role,
                "error": str(exc)})
            return False

        if not (material and material.GetPrim().IsValid()):
            targets = authored
            dangling, typeless = [], []
            for target in targets:
                target_prim = stage.GetPrimAtPath(target)
                if not (target_prim and target_prim.IsValid()):
                    dangling.append(target)
                elif not target_prim.IsA(UsdShade.Material):
                    typeless.append({"path": target,
                                     "type": str(target_prim.GetTypeName())})
            if typeless:
                counts["typeless_targets"] += 1
                example("typeless_targets", {
                    "prim": str(prim.GetPath()), "role": role,
                    "targets": typeless})
            elif dangling:
                counts["dangling_targets"] += 1
                example("dangling_targets", {
                    "prim": str(prim.GetPath()), "role": role,
                    "targets": dangling})
            elif _display_color(prim):
                counts["display_color_only"] += 1
                return True
            else:
                counts["unbound_uncolored"] += 1
                example("unbound_uncolored", {
                    "prim": str(prim.GetPath()), "role": role})
            return False

        material_prim = material.GetPrim()
        if not material_prim.IsA(UsdShade.Material):
            counts["typeless_targets"] += 1
            example("typeless_targets", {
                "prim": str(prim.GetPath()), "role": role,
                "targets": [{"path": str(material_prim.GetPath()),
                             "type": str(material_prim.GetTypeName())}]})
            return False

        counts["bound_targets"] += 1
        material_path = str(material_prim.GetPath())
        if material_path not in checked_materials:
            checked_materials.add(material_path)
            outputs = material.GetSurfaceOutputs()
            if not outputs or not any(output.HasConnectedSource()
                                      for output in outputs):
                counts["surface_less_materials"] += 1
                example("surface_less_materials", material_path)
        return True

    seen_proxy_meshes = set()
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if prim.IsInstanceProxy():
            prototype_prim = prim.GetPrimInPrototype()
            key = str(prototype_prim.GetPath()) if prototype_prim else ""
            if key in seen_proxy_meshes:
                counts["duplicate_instance_proxy_meshes"] += 1
                continue
            seen_proxy_meshes.add(key)
        imageable = UsdGeom.Imageable(prim)
        try:
            if imageable.ComputeVisibility() == UsdGeom.Tokens.invisible:
                counts["invisible_meshes"] += 1
                continue
            if imageable.ComputePurpose() in (UsdGeom.Tokens.guide,
                                              UsdGeom.Tokens.proxy):
                counts["nonrender_meshes"] += 1
                continue
        except Exception:  # pragma: no cover
            pass
        counts["visible_meshes"] += 1

        subsets = [subset for subset in
                   UsdGeom.Subset.GetAllGeomSubsets(imageable)
                   if subset.GetFamilyNameAttr().Get() == "materialBind"]
        assigned_count = 0
        for subset in subsets:
            indices = subset.GetIndicesAttr().Get() or []
            if not indices:
                continue
            assigned_count += len(indices)
            check_target(subset.GetPrim(), "subset")

        face_count = len(UsdGeom.Mesh(
            prim).GetFaceVertexCountsAttr().Get() or [])
        # `materialBind` families are non-overlapping/partitioned by USD
        # convention. Summing cardinalities avoids allocating a Python set of
        # every face index on multi-million-triangle frozen scenes.
        unassigned = max(0, face_count - assigned_count) if subsets else 0
        if unassigned:
            counts["unassigned_faces"] += unassigned
            counts["meshes_with_unassigned_faces"] += 1
            example("unassigned_faces", {
                "prim": str(prim.GetPath()), "faces": unassigned,
                "total_faces": face_count})
        if not subsets or unassigned:
            check_target(prim, "mesh" if not subsets else
                         "unassigned_faces")

    hard = (counts["binding_errors"] + counts["unbound_uncolored"] +
            counts["dangling_targets"] + counts["typeless_targets"] +
            counts["surface_less_materials"])
    return {
        "ok": hard == 0,
        "counts": dict(counts),
        "unique_materials": len(checked_materials),
        "examples": examples,
    }
