#!/usr/bin/env python3
"""
inject_skinned_patch_auto.py

Workflow:
  Input 1: original clean character USD.
  Input 2: Blender-exported USD containing a hand-modeled patch mesh.
  Output: original character USD + injected skinned patch.

Compared with inject_skinned_patch.py, this version can automatically detect
which original skinned mesh the patch is closest to. You can still override
the detected mesh with --target-mesh-path or --target-mesh-keyword.

Typical use:
  python inject_skinned_patch_auto.py \
    --original F_Business_02.usd \
    --patched F_Business_02_patched_skinned.usd \
    --output F_Business_02_with_skinned_patch.usd \
    --patch-name BackPatch

Optional override:
  python inject_skinned_patch_auto.py \
    --original F_Business_02.usd \
    --patched F_Business_02_patched_skinned.usd \
    --output F_Business_02_with_skinned_patch.usd \
    --patch-name BackPatch \
    --target-mesh-keyword vest
"""

from __future__ import annotations

import argparse
import heapq
import math
import sys
from typing import Dict, List, Optional, Sequence, Tuple

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, UsdSkel, Vt


def die(msg: str) -> None:
    print(f"[ERROR] {msg}", file=sys.stderr)
    sys.exit(1)


def is_mesh(prim: Usd.Prim) -> bool:
    return bool(prim and prim.IsValid() and prim.IsA(UsdGeom.Mesh))


def local_to_world(prim: Usd.Prim) -> Gf.Matrix4d:
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    return cache.GetLocalToWorldTransform(prim)


def transform_point(mat: Gf.Matrix4d, p: Gf.Vec3f) -> Gf.Vec3f:
    q = mat.Transform(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
    return Gf.Vec3f(float(q[0]), float(q[1]), float(q[2]))


def dist2(a: Gf.Vec3f, b: Gf.Vec3f) -> float:
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = float(a[2]) - float(b[2])
    return dx * dx + dy * dy + dz * dz


def find_mesh(
    stage: Usd.Stage,
    *,
    exact_path: Optional[str] = None,
    name: Optional[str] = None,
    keyword: Optional[str] = None,
    label: str = "mesh",
) -> Usd.Prim:
    if exact_path:
        prim = stage.GetPrimAtPath(exact_path)
        if not is_mesh(prim):
            die(f"{label} path is not a UsdGeom.Mesh: {exact_path}")
        return prim

    matches: List[Usd.Prim] = []
    for prim in stage.Traverse():
        if not is_mesh(prim):
            continue
        path = str(prim.GetPath())
        leaf = prim.GetName()
        if name and leaf == name:
            matches.append(prim)
        elif keyword and keyword.lower() in path.lower():
            matches.append(prim)

    if not matches:
        desc = f"path={exact_path!r}, name={name!r}, keyword={keyword!r}"
        die(f"Could not find {label}: {desc}")

    if len(matches) > 1:
        print(f"[WARN] Multiple candidates for {label}; using the first one:")
        for m in matches[:20]:
            print(f"  - {m.GetPath()}")
        if len(matches) > 20:
            print(f"  ... {len(matches) - 20} more")

    return matches[0]


def all_meshes(stage: Usd.Stage) -> List[Usd.Prim]:
    return [prim for prim in stage.Traverse() if is_mesh(prim)]


def find_source_patch_mesh(
    stage: Usd.Stage,
    *,
    exact_path: Optional[str] = None,
    name: Optional[str] = None,
    keyword: Optional[str] = None,
) -> Usd.Prim:
    """
    Find the patch mesh in the Blender-exported USD.

    This is intentionally more permissive than find_mesh():
      1. exact --patch-path still wins;
      2. exact mesh leaf name / keyword works;
      3. if --patch-name refers to an Xform, use the first Mesh under it;
      4. if nothing matches but the patched USD contains exactly one Mesh,
         assume that Mesh is the patch. This handles Chinese Blender names like
         /root/平面/平面.
    """
    if exact_path:
        prim = stage.GetPrimAtPath(exact_path)
        if not is_mesh(prim):
            die(f"--patch-path is not a UsdGeom.Mesh: {exact_path}")
        return prim

    meshes = all_meshes(stage)

    matches: List[Usd.Prim] = []
    for prim in meshes:
        path = str(prim.GetPath())
        leaf = prim.GetName()
        if name and leaf == name:
            matches.append(prim)
        elif keyword and keyword.lower() in path.lower():
            matches.append(prim)

    if matches:
        if len(matches) > 1:
            print("[WARN] Multiple source patch mesh candidates; using the first one:")
            for m in matches[:20]:
                print(f"  - {m.GetPath()}")
        return matches[0]

    # Blender often exports ObjectName/ObjectName where ObjectName is an Xform
    # and the actual Mesh is a child with a different or localized name.
    if name:
        for prim in stage.Traverse():
            if prim.GetName() == name:
                child_meshes = [p for p in Usd.PrimRange(prim) if is_mesh(p)]
                if child_meshes:
                    if len(child_meshes) > 1:
                        print(f"[WARN] Multiple meshes under /{name}; using the first one:")
                        for m in child_meshes[:20]:
                            print(f"  - {m.GetPath()}")
                    return child_meshes[0]

    if len(meshes) == 1:
        print(f"[INFO] No patch name match, but patched USD has exactly one Mesh. Using it as patch: {meshes[0].GetPath()}")
        return meshes[0]

    print("[ERROR] Could not identify source patch mesh automatically.")
    print("[ERROR] Meshes in patched USD:")
    for prim in meshes[:200]:
        print(f"  - {prim.GetPath()}")
    if len(meshes) > 200:
        print(f"  ... {len(meshes) - 200} more")
    die("Please pass --patch-path with one of the mesh paths above.")


def has_vertex_skinning(prim: Usd.Prim) -> bool:
    if not is_mesh(prim):
        return False

    binding = UsdSkel.BindingAPI(prim)
    if not binding.GetSkeletonRel().GetTargets():
        return False

    ji_attr = prim.GetAttribute("primvars:skel:jointIndices")
    jw_attr = prim.GetAttribute("primvars:skel:jointWeights")
    if not ji_attr or not ji_attr.HasAuthoredValue():
        return False
    if not jw_attr or not jw_attr.HasAuthoredValue():
        return False

    ji_pv = UsdGeom.Primvar(ji_attr)
    jw_pv = UsdGeom.Primvar(jw_attr)
    return (
        ji_pv.GetInterpolation() == UsdGeom.Tokens.vertex
        and jw_pv.GetInterpolation() == UsdGeom.Tokens.vertex
        and ji_pv.GetElementSize() == jw_pv.GetElementSize()
    )


def skinned_mesh_candidates(stage: Usd.Stage, exclude_keywords: Sequence[str]) -> List[Usd.Prim]:
    out: List[Usd.Prim] = []
    for prim in stage.Traverse():
        if not has_vertex_skinning(prim):
            continue

        path = str(prim.GetPath()).lower()
        if any(k.lower() in path for k in exclude_keywords):
            continue

        points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if not points:
            continue

        out.append(prim)

    return out


def mesh_points_world(prim: Usd.Prim) -> List[Gf.Vec3f]:
    mat = local_to_world(prim)
    pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
    return [transform_point(mat, p) for p in pts]


def patch_points_world(src_patch_prim: Usd.Prim, bake_src_geom_bind: bool) -> List[Gf.Vec3f]:
    mesh = UsdGeom.Mesh(src_patch_prim)
    pts = mesh.GetPointsAttr().Get()
    if not pts:
        die(f"Patch mesh has no points: {src_patch_prim.GetPath()}")

    l2w = local_to_world(src_patch_prim)

    gbt = None
    if bake_src_geom_bind:
        gbt_attr = src_patch_prim.GetAttribute("primvars:skel:geomBindTransform")
        if gbt_attr and gbt_attr.HasAuthoredValue():
            gbt = Gf.Matrix4d(gbt_attr.Get())

    out: List[Gf.Vec3f] = []
    for p in pts:
        q = p
        if gbt is not None:
            q = transform_point(gbt, q)
        q = transform_point(l2w, q)
        out.append(q)
    return out


def mean_nearest_distance(patch_pts: Sequence[Gf.Vec3f], mesh_pts: Sequence[Gf.Vec3f]) -> float:
    if not patch_pts or not mesh_pts:
        return float("inf")

    total = 0.0
    for p in patch_pts:
        d = min(dist2(p, q) for q in mesh_pts)
        total += math.sqrt(d)
    return total / len(patch_pts)


def auto_detect_target_mesh(
    *,
    original_stage: Usd.Stage,
    src_patch_prim: Usd.Prim,
    bake_src_geom_bind: bool,
    exclude_keywords: Sequence[str],
    top_n: int,
) -> Usd.Prim:
    candidates = skinned_mesh_candidates(original_stage, exclude_keywords)
    if not candidates:
        die("No skinned mesh candidates found in original USD")

    p_world = patch_points_world(src_patch_prim, bake_src_geom_bind)

    scored: List[Tuple[float, Usd.Prim]] = []
    for prim in candidates:
        m_world = mesh_points_world(prim)
        score = mean_nearest_distance(p_world, m_world)
        scored.append((score, prim))

    scored.sort(key=lambda x: x[0])

    print("[INFO] Auto target-mesh detection results:")
    for score, prim in scored[:max(1, top_n)]:
        print(f"  mean nearest distance = {score:.6f}  mesh = {prim.GetPath()}")

    best_score, best_prim = scored[0]
    print(f"[INFO] Selected target mesh: {best_prim.GetPath()}  score={best_score:.6f}")
    return best_prim


def get_skeleton_target(mesh_prim: Usd.Prim) -> Sdf.Path:
    binding = UsdSkel.BindingAPI(mesh_prim)
    targets = binding.GetSkeletonRel().GetTargets()
    if not targets:
        die(f"Target mesh has no skeleton binding: {mesh_prim.GetPath()}")
    return targets[0]


def copy_non_skel_primvars(src_prim: Usd.Prim, dst_prim: Usd.Prim) -> None:
    src_api = UsdGeom.PrimvarsAPI(src_prim)
    dst_api = UsdGeom.PrimvarsAPI(dst_prim)

    for pv in src_api.GetPrimvars():
        primvar_name = pv.GetPrimvarName()
        base_name = pv.GetBaseName()

        # Copy UVs and other useful primvars, but skip skinning and display color/opacity.
        # Display color/opacity often causes the patch to inherit a confusing Blender/vest look.
        if "skel" in primvar_name:
            continue
        if base_name in {"displayColor", "displayOpacity"}:
            continue

        value = pv.Get()
        if value is None:
            continue

        new_pv = dst_api.CreatePrimvar(
            base_name,
            pv.GetTypeName(),
            pv.GetInterpolation(),
            pv.GetElementSize(),
        )
        new_pv.Set(value)

        indices = pv.GetIndices()
        if indices:
            new_pv.SetIndices(indices)


def copy_patch_geometry(
    *,
    src_patch_prim: Usd.Prim,
    original_stage: Usd.Stage,
    dst_parent_prim: Usd.Prim,
    dst_patch_path: Sdf.Path,
    bake_src_geom_bind: bool,
    copy_normals: bool,
    flip_winding: bool,
) -> Usd.Prim:
    src_mesh = UsdGeom.Mesh(src_patch_prim)
    src_points = src_mesh.GetPointsAttr().Get()
    if not src_points:
        die(f"Patch mesh has no points: {src_patch_prim.GetPath()}")

    src_l2w = local_to_world(src_patch_prim)
    dst_parent_w2l = local_to_world(dst_parent_prim).GetInverse()

    gbt = None
    if bake_src_geom_bind:
        gbt_attr = src_patch_prim.GetAttribute("primvars:skel:geomBindTransform")
        if gbt_attr and gbt_attr.HasAuthoredValue():
            gbt = Gf.Matrix4d(gbt_attr.Get())
            print("[INFO] Baking source primvars:skel:geomBindTransform into patch points")

    dst_points: List[Gf.Vec3f] = []
    for p in src_points:
        q = p
        if gbt is not None:
            q = transform_point(gbt, q)
        q = transform_point(src_l2w, q)
        q = transform_point(dst_parent_w2l, q)
        dst_points.append(q)

    original_stage.RemovePrim(dst_patch_path)
    dst_prim = original_stage.DefinePrim(dst_patch_path, "Mesh")
    dst_mesh = UsdGeom.Mesh(dst_prim)

    dst_mesh.CreatePointsAttr().Set(Vt.Vec3fArray(dst_points))

    face_counts = list(src_mesh.GetFaceVertexCountsAttr().Get())
    face_indices = list(src_mesh.GetFaceVertexIndicesAttr().Get())

    if flip_winding:
        flipped_indices: List[int] = []
        cursor = 0
        for n in face_counts:
            face = face_indices[cursor:cursor + n]
            flipped_indices.extend(reversed(face))
            cursor += n
        face_indices = flipped_indices
        print("[INFO] Flipped patch face winding")

    dst_mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    dst_mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    dst_mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    # A thin patch plane can face inward depending on Blender export orientation.
    # Making it double-sided avoids dark/invisible back-face rendering artifacts.
    dst_mesh.CreateDoubleSidedAttr().Set(True)

    if copy_normals:
        normals_attr = src_mesh.GetNormalsAttr()
        if normals_attr and normals_attr.HasAuthoredValue():
            # Warning: these normals are copied verbatim from the Blender USD.
            # They may be in the source mesh's local space, so use this only if
            # you know the normals are already correct in the destination space.
            dst_mesh.CreateNormalsAttr().Set(normals_attr.Get())
            interp = src_mesh.GetNormalsInterpolation()
            if interp:
                dst_mesh.SetNormalsInterpolation(interp)
            print("[INFO] Copied authored normals from source patch")
    else:
        # Match the earlier hand-generated working USD: do not author normals.
        # Let Hydra/RTX recompute them from the final topology/points.
        dst_prim.RemoveProperty("normals")
        print("[INFO] Dropped authored patch normals; renderer will compute normals")

    for attr_name in ["orientation", "doubleSided"]:
        attr = src_patch_prim.GetAttribute(attr_name)
        if attr and attr.HasAuthoredValue():
            dst_prim.CreateAttribute(attr_name, attr.GetTypeName()).Set(attr.Get())

    copy_non_skel_primvars(src_patch_prim, dst_prim)
    return dst_prim


def read_vertex_skinning(mesh_prim: Usd.Prim) -> Tuple[List[int], List[float], int]:
    ji_attr = mesh_prim.GetAttribute("primvars:skel:jointIndices")
    jw_attr = mesh_prim.GetAttribute("primvars:skel:jointWeights")

    if not ji_attr or not ji_attr.HasAuthoredValue():
        die(f"Missing primvars:skel:jointIndices on {mesh_prim.GetPath()}")
    if not jw_attr or not jw_attr.HasAuthoredValue():
        die(f"Missing primvars:skel:jointWeights on {mesh_prim.GetPath()}")

    ji_pv = UsdGeom.Primvar(ji_attr)
    jw_pv = UsdGeom.Primvar(jw_attr)

    if ji_pv.GetInterpolation() != UsdGeom.Tokens.vertex:
        die(f"Expected vertex jointIndices on {mesh_prim.GetPath()}, got {ji_pv.GetInterpolation()}")
    if jw_pv.GetInterpolation() != UsdGeom.Tokens.vertex:
        die(f"Expected vertex jointWeights on {mesh_prim.GetPath()}, got {jw_pv.GetInterpolation()}")

    elem = ji_pv.GetElementSize()
    if elem != jw_pv.GetElementSize():
        die("jointIndices and jointWeights have different elementSize")

    ji = list(ji_attr.Get())
    jw = list(jw_attr.Get())

    if len(ji) != len(jw):
        die("jointIndices and jointWeights lengths differ")

    return ji, jw, elem


def points_in_parent_space(mesh_prim: Usd.Prim, parent_prim: Usd.Prim) -> List[Gf.Vec3f]:
    points = list(UsdGeom.Mesh(mesh_prim).GetPointsAttr().Get())
    mesh_l2w = local_to_world(mesh_prim)
    parent_w2l = local_to_world(parent_prim).GetInverse()
    return [transform_point(parent_w2l, transform_point(mesh_l2w, p)) for p in points]


def offset_patch_away_from_target(
    *,
    patch_prim: Usd.Prim,
    target_mesh_prim: Usd.Prim,
    offset_distance: float,
    offset_mode: str,
) -> None:
    """
    Push the patch slightly away from the target clothing mesh to reduce z-fighting.

    offset_mode:
      - uniform: compute one averaged outward direction and translate the whole patch.
                 This preserves the original flat shape and straight borders.
      - per_vertex: old behavior; each vertex moves away from its nearest target vertex.
                    This can make borders wavy.
    """
    if offset_distance <= 0.0:
        return

    if offset_mode not in {"uniform", "per_vertex"}:
        die(f"Unknown --offset-mode {offset_mode!r}. Expected 'uniform' or 'per_vertex'.")

    patch_parent = patch_prim.GetParent()
    patch_mesh = UsdGeom.Mesh(patch_prim)
    patch_points = list(patch_mesh.GetPointsAttr().Get())
    target_points = points_in_parent_space(target_mesh_prim, patch_parent)

    if not patch_points or not target_points:
        return

    directions: List[Gf.Vec3d] = []
    for p in patch_points:
        nearest = min(target_points, key=lambda q: dist2(p, q))
        direction = Gf.Vec3d(
            float(p[0]) - float(nearest[0]),
            float(p[1]) - float(nearest[1]),
            float(p[2]) - float(nearest[2]),
        )
        if direction.GetLength() > 1e-8:
            direction.Normalize()
            directions.append(direction)

    if not directions:
        # Fallback direction: from target centroid to patch centroid.
        pc = Gf.Vec3d(0.0, 0.0, 0.0)
        tc = Gf.Vec3d(0.0, 0.0, 0.0)
        for p in patch_points:
            pc += Gf.Vec3d(float(p[0]), float(p[1]), float(p[2]))
        for q in target_points:
            tc += Gf.Vec3d(float(q[0]), float(q[1]), float(q[2]))
        pc /= len(patch_points)
        tc /= len(target_points)
        fallback = pc - tc
        if fallback.GetLength() < 1e-8:
            fallback = Gf.Vec3d(0.0, 1.0, 0.0)
        fallback.Normalize()
        directions = [fallback]

    avg_dir = Gf.Vec3d(0.0, 0.0, 0.0)
    for d in directions:
        avg_dir += d
    if avg_dir.GetLength() < 1e-8:
        avg_dir = directions[0]
    else:
        avg_dir.Normalize()

    new_points: List[Gf.Vec3f] = []

    if offset_mode == "uniform":
        # Translate all vertices by the same vector, preserving flatness and borders.
        for p in patch_points:
            p2 = Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])) + avg_dir * offset_distance
            new_points.append(Gf.Vec3f(float(p2[0]), float(p2[1]), float(p2[2])))
    else:
        # Old mode. This conforms more locally but can wrinkle the border.
        for p in patch_points:
            nearest = min(target_points, key=lambda q: dist2(p, q))
            direction = Gf.Vec3d(
                float(p[0]) - float(nearest[0]),
                float(p[1]) - float(nearest[1]),
                float(p[2]) - float(nearest[2]),
            )
            if direction.GetLength() < 1e-8:
                direction = Gf.Vec3d(avg_dir)
            else:
                direction.Normalize()

            p2 = Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])) + direction * offset_distance
            new_points.append(Gf.Vec3f(float(p2[0]), float(p2[1]), float(p2[2])))

    patch_mesh.GetPointsAttr().Set(Vt.Vec3fArray(new_points))
    print(f"[INFO] Offset patch away from target by {offset_distance:.6f} scene units using {offset_mode} mode")


def transfer_skinning_from_target(
    *,
    patch_prim: Usd.Prim,
    target_mesh_prim: Usd.Prim,
    skeleton_path: Sdf.Path,
    k_nearest: int,
    max_influences: Optional[int],
    skin_mode: str,
) -> None:
    patch_parent = patch_prim.GetParent()
    patch_points = points_in_parent_space(patch_prim, patch_parent)
    target_points = points_in_parent_space(target_mesh_prim, patch_parent)

    target_ji, target_jw, target_elem = read_vertex_skinning(target_mesh_prim)

    if len(target_ji) != len(target_points) * target_elem:
        die(
            f"Skinning size mismatch on target mesh: {len(target_ji)} values for "
            f"{len(target_points)} points and elementSize={target_elem}"
        )

    out_elem = max_influences or target_elem

    # First compute a raw KNN skin-weight dictionary for each patch vertex.
    per_vertex_accum: List[Dict[int, float]] = []

    for p in patch_points:
        nearest = heapq.nsmallest(
            max(1, min(k_nearest, len(target_points))),
            ((dist2(p, q), i) for i, q in enumerate(target_points)),
        )

        accum: Dict[int, float] = {}
        total_neighbor_weight = 0.0

        for d2, vertex_idx in nearest:
            # Inverse squared distance. Clamp avoids exploding for nearly identical points.
            nw = 1.0 / max(d2, 1e-10)
            total_neighbor_weight += nw

            base = vertex_idx * target_elem
            for j in range(target_elem):
                joint = int(target_ji[base + j])
                weight = float(target_jw[base + j])
                if weight <= 0.0:
                    continue
                accum[joint] = accum.get(joint, 0.0) + nw * weight

        if total_neighbor_weight > 0.0:
            accum = {joint: w / total_neighbor_weight for joint, w in accum.items()}

        per_vertex_accum.append(accum)

    if skin_mode == "average":
        # Use one averaged skin-weight vector for every patch vertex.
        # This preserves the patch as a smooth/rigid-ish sheet: straight borders stay straight.
        avg: Dict[int, float] = {}
        for accum in per_vertex_accum:
            for joint, w in accum.items():
                avg[joint] = avg.get(joint, 0.0) + w
        if avg:
            inv_n = 1.0 / len(per_vertex_accum)
            avg = {joint: w * inv_n for joint, w in avg.items()}
        per_vertex_accum = [dict(avg) for _ in per_vertex_accum]
        print("[INFO] Using averaged patch skin weights; border should stay smoother/straighter")
    elif skin_mode != "knn":
        die(f"Unknown --skin-mode {skin_mode!r}. Expected 'knn' or 'average'.")

    out_ji: List[int] = []
    out_jw: List[float] = []

    for accum in per_vertex_accum:
        if not accum:
            pairs = [(0, 1.0)]
        else:
            pairs = list(accum.items())
            pairs.sort(key=lambda x: x[1], reverse=True)
            pairs = pairs[:out_elem]
            s = sum(w for _, w in pairs)
            pairs = [(joint, w / s) for joint, w in pairs] if s > 0 else [(pairs[0][0], 1.0)]

        while len(pairs) < out_elem:
            pairs.append((0, 0.0))

        out_ji.extend([joint for joint, _ in pairs])
        out_jw.extend([weight for _, weight in pairs])

    binding = UsdSkel.BindingAPI.Apply(patch_prim)
    binding.CreateSkeletonRel().SetTargets([skeleton_path])

    ji_pv = binding.CreateJointIndicesPrimvar(False, out_elem)
    ji_pv.SetInterpolation(UsdGeom.Tokens.vertex)
    ji_pv.Set(Vt.IntArray(out_ji))

    jw_pv = binding.CreateJointWeightsPrimvar(False, out_elem)
    jw_pv.SetInterpolation(UsdGeom.Tokens.vertex)
    jw_pv.Set(Vt.FloatArray(out_jw))

    # Do not keep Blender-authored geomBindTransform in the final output.
    gbt_attr = patch_prim.GetAttribute("primvars:skel:geomBindTransform")
    if gbt_attr and gbt_attr.HasAuthoredValue():
        gbt_attr.Clear()


def bind_simple_material(stage: Usd.Stage, patch_prim: Usd.Prim, color: Tuple[float, float, float]) -> None:
    parent = patch_prim.GetParent()
    looks_path = parent.GetPath().AppendChild("Looks")
    stage.DefinePrim(looks_path, "Scope")

    mat_path = looks_path.AppendChild("BackPatch_Material")
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, mat_path.AppendChild("PreviewSurface"))

    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    UsdShade.MaterialBindingAPI.Apply(patch_prim)
    UsdShade.MaterialBindingAPI(patch_prim).Bind(material, UsdShade.Tokens.strongerThanDescendants)

    mesh = UsdGeom.Mesh(patch_prim)
    mesh.CreateDisplayOpacityPrimvar(UsdGeom.Tokens.constant, 1).Set(Vt.FloatArray([1.0]))
    mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant, 1).Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))


def parse_rgb(s: str) -> Tuple[float, float, float]:
    vals = tuple(float(x.strip()) for x in s.split(","))
    if len(vals) != 3:
        die("--material-color must be three comma-separated floats, e.g. 1,0,0")
    return vals  # type: ignore[return-value]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--original", required=True, help="Original clean character USD")
    ap.add_argument("--patched", required=True, help="Blender-exported USD that contains the patch mesh")
    ap.add_argument("--output", required=True, help="Output USD")

    ap.add_argument("--patch-path", default=None, help="Exact patch mesh path in patched USD")
    ap.add_argument("--patch-name", default="BackPatch", help="Patch mesh prim name in patched USD")
    ap.add_argument("--patch-keyword", default=None, help="Keyword to find patch mesh if name/path is not enough")
    ap.add_argument("--list-patched-meshes", action="store_true", help="List mesh paths in --patched and exit")

    ap.add_argument("--target-mesh-path", default=None, help="Exact original clothing mesh path to transfer weights from")
    ap.add_argument(
        "--target-mesh-keyword",
        default=None,
        help="Keyword for original clothing mesh, e.g. vest or shirt. If omitted, auto-detection is used.",
    )

    ap.add_argument("--dst-parent-path", default=None, help="Destination parent in original USD. Default: target mesh parent")
    ap.add_argument("--dst-patch-name", default="BackPatch")

    ap.add_argument("--k-nearest", type=int, default=4)
    ap.add_argument("--max-influences", type=int, default=None)
    ap.add_argument(
        "--skin-mode",
        choices=["knn", "average"],
        default="knn",
        help=(
            "knn: per-vertex weights copied from nearby clothing vertices; conforms more but can make borders wavy. "
            "average: one averaged weight vector for the whole patch; keeps patch border smoother/straighter."
        ),
    )
    ap.add_argument(
        "--offset-distance",
        type=float,
        default=0.0,
        help="Push patch slightly away from the detected target mesh, e.g. 0.003 or 0.005, to reduce z-fighting/occlusion.",
    )
    ap.add_argument(
        "--offset-mode",
        choices=["uniform", "per_vertex"],
        default="uniform",
        help=(
            "How to apply --offset-distance. uniform translates the whole patch and preserves shape; "
            "per_vertex uses the old nearest-vertex outward push and may make borders wavy."
        ),
    )
    ap.add_argument(
        "--no-force-double-sided",
        action="store_true",
        help="Do not force the patch mesh to be double-sided.",
    )
    ap.add_argument(
        "--copy-normals",
        action="store_true",
        help=(
            "Copy authored normals from the Blender patch. Default is false because "
            "Blender/USD normals can be in the wrong space after reparenting and cause "
            "dark shadow-like shading artifacts."
        ),
    )
    ap.add_argument(
        "--flip-winding",
        action="store_true",
        help="Reverse face vertex order for the injected patch if its front/back orientation is wrong.",
    )
    ap.add_argument("--no-bake-src-geom-bind", action="store_true", help="Do not bake source skel:geomBindTransform")
    ap.add_argument(
        "--exclude-candidate-keywords",
        default="patch,eye,eyelash,teeth,tongue,hair",
        help="Comma-separated keywords to exclude during auto target detection",
    )
    ap.add_argument("--print-top", type=int, default=8, help="How many auto-detection candidates to print")

    ap.add_argument("--material", action="store_true", help="Bind a simple opaque preview material")
    ap.add_argument("--material-color", default="0.45,0.05,0.55", help="RGB floats, e.g. 1,0,0")

    args = ap.parse_args()

    original_stage = Usd.Stage.Open(args.original)
    if not original_stage:
        die(f"Could not open original USD: {args.original}")

    patched_stage = Usd.Stage.Open(args.patched)
    if not patched_stage:
        die(f"Could not open patched USD: {args.patched}")

    if args.list_patched_meshes:
        print("=== Mesh prims in patched USD ===")
        for prim in all_meshes(patched_stage):
            print(prim.GetPath())
        return

    src_patch_prim = find_source_patch_mesh(
        patched_stage,
        exact_path=args.patch_path,
        name=args.patch_name,
        keyword=args.patch_keyword,
    )
    print(f"[INFO] Source patch: {src_patch_prim.GetPath()}")

    bake_src_geom_bind = not args.no_bake_src_geom_bind

    if args.target_mesh_path or args.target_mesh_keyword:
        target_mesh_prim = find_mesh(
            original_stage,
            exact_path=args.target_mesh_path,
            keyword=args.target_mesh_keyword,
            label="target clothing mesh",
        )
        print(f"[INFO] Target mesh override: {target_mesh_prim.GetPath()}")
    else:
        exclude_keywords = [x.strip() for x in args.exclude_candidate_keywords.split(",") if x.strip()]
        target_mesh_prim = auto_detect_target_mesh(
            original_stage=original_stage,
            src_patch_prim=src_patch_prim,
            bake_src_geom_bind=bake_src_geom_bind,
            exclude_keywords=exclude_keywords,
            top_n=args.print_top,
        )

    skeleton_path = get_skeleton_target(target_mesh_prim)
    print(f"[INFO] Skeleton: {skeleton_path}")

    if args.dst_parent_path:
        dst_parent_prim = original_stage.GetPrimAtPath(args.dst_parent_path)
        if not dst_parent_prim or not dst_parent_prim.IsValid():
            die(f"Destination parent does not exist: {args.dst_parent_path}")
    else:
        dst_parent_prim = target_mesh_prim.GetParent()

    dst_patch_path = dst_parent_prim.GetPath().AppendChild(args.dst_patch_name)
    print(f"[INFO] Destination patch: {dst_patch_path}")

    dst_patch_prim = copy_patch_geometry(
        src_patch_prim=src_patch_prim,
        original_stage=original_stage,
        dst_parent_prim=dst_parent_prim,
        dst_patch_path=dst_patch_path,
        bake_src_geom_bind=bake_src_geom_bind,
        copy_normals=args.copy_normals,
        flip_winding=args.flip_winding,
    )

    if args.no_force_double_sided:
        UsdGeom.Mesh(dst_patch_prim).CreateDoubleSidedAttr().Set(False)

    offset_patch_away_from_target(
        patch_prim=dst_patch_prim,
        target_mesh_prim=target_mesh_prim,
        offset_distance=args.offset_distance,
        offset_mode=args.offset_mode,
    )

    transfer_skinning_from_target(
        patch_prim=dst_patch_prim,
        target_mesh_prim=target_mesh_prim,
        skeleton_path=skeleton_path,
        k_nearest=args.k_nearest,
        max_influences=args.max_influences,
        skin_mode=args.skin_mode,
    )

    if args.material:
        bind_simple_material(original_stage, dst_patch_prim, parse_rgb(args.material_color))

    original_stage.GetRootLayer().Export(args.output)
    print(f"[OK] Wrote {args.output}")
    print("[OK] Original body/skeleton/skinning were preserved; only the patch was injected.")


if __name__ == "__main__":
    main()
