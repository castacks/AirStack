#!/usr/bin/env python3
"""
inject_skinned_patch_stable.py

Stable version for the workflow:

  original USD  +  Blender-exported patched USD
        -> output USD that keeps original body/skeleton/skinning,
           injects only the modeled patch mesh,
           gives the patch smooth averaged skin weights,
           drops Blender-authored normals to avoid RTX shading artifacts,
           and optionally pushes the patch outward with a uniform offset.

Recommended command:

  /isaac-sim/python.sh inject_skinned_patch_stable.py \
    --original ../assets/characters/F_Business_02/F_Business_02.usd \
    --patched ../assets/characters/F_Business_02/F_Business_02_patched.usd \
    --output ../assets/characters/F_Business_02/F_Business_02_with_skinned_patch.usd \
    --patch-path "/root/Plane/Plane"

This script requires the pxr USD Python module, so run it with Isaac/Omniverse Python.
"""

from __future__ import annotations

import argparse
import heapq
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel, Vt


def die(msg: str) -> None:
    print(f"[ERROR] {msg}", file=sys.stderr)
    raise SystemExit(1)


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


def all_meshes(stage: Usd.Stage) -> List[Usd.Prim]:
    return [prim for prim in stage.Traverse() if is_mesh(prim)]


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


def find_mesh(
    stage: Usd.Stage,
    *,
    exact_path: Optional[str] = None,
    keyword: Optional[str] = None,
    label: str = "mesh",
) -> Usd.Prim:
    if exact_path:
        prim = stage.GetPrimAtPath(exact_path)
        if not is_mesh(prim):
            die(f"{label} path is not a UsdGeom.Mesh: {exact_path}")
        return prim

    if not keyword:
        die(f"No path or keyword provided for {label}")

    matches = []
    for prim in stage.Traverse():
        if is_mesh(prim) and keyword.lower() in str(prim.GetPath()).lower():
            matches.append(prim)

    if not matches:
        die(f"Could not find {label} with keyword={keyword!r}")

    if len(matches) > 1:
        print(f"[WARN] Multiple candidates for {label}; using first:")
        for m in matches[:20]:
            print(f"  - {m.GetPath()}")

    return matches[0]


def find_source_patch_mesh(
    patched_stage: Usd.Stage,
    *,
    patch_path: Optional[str],
    patch_keyword: Optional[str],
) -> Usd.Prim:
    """
    Prefer explicit --patch-path. Otherwise:
      1. keyword match, if provided;
      2. exactly one non-skinned mesh in the patched USD;
      3. common Blender patch names such as Plane / 平面 / BackPatch / Patch;
      4. print all mesh paths and ask for --patch-path.
    """
    if patch_path:
        prim = patched_stage.GetPrimAtPath(patch_path)
        if not is_mesh(prim):
            die(f"--patch-path is not a UsdGeom.Mesh: {patch_path}")
        return prim

    meshes = all_meshes(patched_stage)
    if not meshes:
        die("No Mesh prim found in patched USD")

    if patch_keyword:
        matches = [m for m in meshes if patch_keyword.lower() in str(m.GetPath()).lower()]
        if matches:
            if len(matches) > 1:
                print("[WARN] Multiple patch keyword matches; using first:")
                for m in matches[:20]:
                    print(f"  - {m.GetPath()}")
            return matches[0]

    # In Blender-exported files that contain the full character, body/clothing meshes
    # are usually skinned, while the newly modeled patch Plane is often not skinned.
    non_skinned = [m for m in meshes if not has_vertex_skinning(m)]
    if len(non_skinned) == 1:
        print(f"[INFO] Auto-selected the only non-skinned mesh as patch: {non_skinned[0].GetPath()}")
        return non_skinned[0]

    common_names = ("BackPatch", "Patch", "Plane", "平面")
    for name in common_names:
        matches = [m for m in meshes if m.GetName() == name or name.lower() in str(m.GetPath()).lower()]
        if len(matches) == 1:
            print(f"[INFO] Auto-selected patch by common name {name!r}: {matches[0].GetPath()}")
            return matches[0]

    if len(meshes) == 1:
        print(f"[INFO] Auto-selected the only mesh as patch: {meshes[0].GetPath()}")
        return meshes[0]

    print("[ERROR] Could not determine patch mesh automatically.")
    print("[ERROR] Meshes in patched USD:")
    for m in meshes:
        flag = "skinned" if has_vertex_skinning(m) else "non-skinned"
        print(f"  - {m.GetPath()}  [{flag}]")
    die("Pass --patch-path with the patch mesh path, e.g. --patch-path '/root/Plane/Plane'")


def mesh_points_world(prim: Usd.Prim) -> List[Gf.Vec3f]:
    mat = local_to_world(prim)
    pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
    return [transform_point(mat, p) for p in pts]


def patch_points_world(src_patch_prim: Usd.Prim) -> List[Gf.Vec3f]:
    # Stable behavior: do NOT bake source geomBindTransform.
    # This matched the successful version for Blender-modeled Plane patches.
    return mesh_points_world(src_patch_prim)


def mean_nearest_distance(a_pts: Sequence[Gf.Vec3f], b_pts: Sequence[Gf.Vec3f]) -> float:
    if not a_pts or not b_pts:
        return float("inf")
    total = 0.0
    for p in a_pts:
        total += math.sqrt(min(dist2(p, q) for q in b_pts))
    return total / len(a_pts)


def skinned_mesh_candidates(stage: Usd.Stage, exclude_keywords: Sequence[str]) -> List[Usd.Prim]:
    out = []
    for prim in stage.Traverse():
        if not has_vertex_skinning(prim):
            continue
        path = str(prim.GetPath()).lower()
        if any(k.lower() in path for k in exclude_keywords):
            continue
        if UsdGeom.Mesh(prim).GetPointsAttr().Get():
            out.append(prim)
    return out


def auto_detect_target_mesh(
    original_stage: Usd.Stage,
    src_patch_prim: Usd.Prim,
    *,
    exclude_keywords: Sequence[str],
    print_top: int,
) -> Usd.Prim:
    candidates = skinned_mesh_candidates(original_stage, exclude_keywords)
    if not candidates:
        die("No skinned mesh candidates found in original USD")

    p_world = patch_points_world(src_patch_prim)
    scored = []
    for c in candidates:
        scored.append((mean_nearest_distance(p_world, mesh_points_world(c)), c))
    scored.sort(key=lambda x: x[0])

    print("[INFO] Auto target mesh detection:")
    for score, prim in scored[:max(1, print_top)]:
        print(f"  mean nearest distance = {score:.6f}  mesh = {prim.GetPath()}")

    print(f"[INFO] Selected target mesh: {scored[0][1].GetPath()}")
    return scored[0][1]


def get_skeleton_target(mesh_prim: Usd.Prim) -> Sdf.Path:
    targets = UsdSkel.BindingAPI(mesh_prim).GetSkeletonRel().GetTargets()
    if not targets:
        die(f"Target mesh has no skeleton binding: {mesh_prim.GetPath()}")
    return targets[0]


def copy_non_skel_non_display_primvars(src_prim: Usd.Prim, dst_prim: Usd.Prim) -> None:
    src_api = UsdGeom.PrimvarsAPI(src_prim)
    dst_api = UsdGeom.PrimvarsAPI(dst_prim)

    for pv in src_api.GetPrimvars():
        primvar_name = pv.GetPrimvarName()
        base_name = pv.GetBaseName()

        # Keep useful things like UVs, but skip:
        #   - skel primvars: we rewrite skinning
        #   - displayColor/displayOpacity: can cause confusing gray/transparent appearance
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
        if indices is not None and len(indices) > 0:
            new_pv.SetIndices(indices)


def copy_patch_geometry(
    *,
    src_patch_prim: Usd.Prim,
    original_stage: Usd.Stage,
    dst_parent_prim: Usd.Prim,
    dst_patch_path: Sdf.Path,
    flip_winding: bool,
) -> Usd.Prim:
    src_mesh = UsdGeom.Mesh(src_patch_prim)
    src_points = src_mesh.GetPointsAttr().Get()
    if not src_points:
        die(f"Patch mesh has no points: {src_patch_prim.GetPath()}")

    # Stable behavior:
    #   local source points -> source world -> destination parent local.
    # No geomBindTransform bake.
    src_l2w = local_to_world(src_patch_prim)
    dst_parent_w2l = local_to_world(dst_parent_prim).GetInverse()

    dst_points: List[Gf.Vec3f] = []
    for p in src_points:
        q = transform_point(src_l2w, p)
        q = transform_point(dst_parent_w2l, q)
        dst_points.append(q)

    original_stage.RemovePrim(dst_patch_path)
    dst_prim = original_stage.DefinePrim(dst_patch_path, "Mesh")
    dst_mesh = UsdGeom.Mesh(dst_prim)

    face_counts = list(src_mesh.GetFaceVertexCountsAttr().Get())
    face_indices = list(src_mesh.GetFaceVertexIndicesAttr().Get())
    if flip_winding:
        flipped = []
        cursor = 0
        for n in face_counts:
            flipped.extend(reversed(face_indices[cursor:cursor + n]))
            cursor += n
        face_indices = flipped
        print("[INFO] Flipped patch face winding")

    dst_mesh.CreatePointsAttr().Set(Vt.Vec3fArray(dst_points))
    dst_mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    dst_mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    dst_mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    dst_mesh.CreateDoubleSidedAttr().Set(True)

    # Important: do NOT copy authored normals from Blender.
    # Let RTX/Hydra compute normals from the final points/topology to avoid
    # the shadow-like artifacts we observed.
    dst_prim.RemoveProperty("normals")

    copy_non_skel_non_display_primvars(src_patch_prim, dst_prim)
    return dst_prim


def points_in_parent_space(mesh_prim: Usd.Prim, parent_prim: Usd.Prim) -> List[Gf.Vec3f]:
    points = list(UsdGeom.Mesh(mesh_prim).GetPointsAttr().Get())
    mesh_l2w = local_to_world(mesh_prim)
    parent_w2l = local_to_world(parent_prim).GetInverse()
    return [transform_point(parent_w2l, transform_point(mesh_l2w, p)) for p in points]


def offset_patch_uniform(
    *,
    patch_prim: Usd.Prim,
    target_mesh_prim: Usd.Prim,
    offset_distance: float,
) -> None:
    if offset_distance <= 0.0:
        return

    parent = patch_prim.GetParent()
    patch_mesh = UsdGeom.Mesh(patch_prim)
    patch_points = list(patch_mesh.GetPointsAttr().Get())
    target_points = points_in_parent_space(target_mesh_prim, parent)

    if not patch_points or not target_points:
        return

    directions: List[Gf.Vec3d] = []
    for p in patch_points:
        nearest = min(target_points, key=lambda q: dist2(p, q))
        d = Gf.Vec3d(
            float(p[0]) - float(nearest[0]),
            float(p[1]) - float(nearest[1]),
            float(p[2]) - float(nearest[2]),
        )
        if d.GetLength() > 1e-8:
            d.Normalize()
            directions.append(d)

    if not directions:
        return

    avg = Gf.Vec3d(0.0, 0.0, 0.0)
    for d in directions:
        avg += d
    if avg.GetLength() < 1e-8:
        avg = directions[0]
    else:
        avg.Normalize()

    new_points = []
    for p in patch_points:
        q = Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])) + avg * offset_distance
        new_points.append(Gf.Vec3f(float(q[0]), float(q[1]), float(q[2])))

    patch_mesh.GetPointsAttr().Set(Vt.Vec3fArray(new_points))
    print(f"[INFO] Uniformly offset patch by {offset_distance:.6f} scene units")


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
        die("jointIndices and jointWeights element sizes differ")

    ji = list(ji_attr.Get())
    jw = list(jw_attr.Get())
    if len(ji) != len(jw):
        die("jointIndices and jointWeights lengths differ")
    return ji, jw, elem


def transfer_averaged_skinning(
    *,
    patch_prim: Usd.Prim,
    target_mesh_prim: Usd.Prim,
    skeleton_path: Sdf.Path,
    k_nearest: int,
    max_influences: Optional[int],
) -> None:
    parent = patch_prim.GetParent()
    patch_points = points_in_parent_space(patch_prim, parent)
    target_points = points_in_parent_space(target_mesh_prim, parent)

    target_ji, target_jw, target_elem = read_vertex_skinning(target_mesh_prim)
    if len(target_ji) != len(target_points) * target_elem:
        die("Target mesh skinning size does not match target point count")

    out_elem = max_influences or target_elem

    # Compute raw KNN weights for each patch vertex, then average all patch vertices
    # into one shared weight vector. This keeps the patch border smooth/rigid-ish.
    total_accum: Dict[int, float] = {}

    for p in patch_points:
        nearest = heapq.nsmallest(
            max(1, min(k_nearest, len(target_points))),
            ((dist2(p, q), i) for i, q in enumerate(target_points)),
        )

        v_accum: Dict[int, float] = {}
        neighbor_total = 0.0

        for d2, vertex_idx in nearest:
            nw = 1.0 / max(d2, 1e-10)
            neighbor_total += nw

            base = vertex_idx * target_elem
            for j in range(target_elem):
                joint = int(target_ji[base + j])
                weight = float(target_jw[base + j])
                if weight <= 0.0:
                    continue
                v_accum[joint] = v_accum.get(joint, 0.0) + nw * weight

        if neighbor_total > 0:
            for joint, w in v_accum.items():
                total_accum[joint] = total_accum.get(joint, 0.0) + (w / neighbor_total)

    if not total_accum:
        die("Could not compute patch skin weights")

    inv_n = 1.0 / max(1, len(patch_points))
    avg_pairs = [(joint, w * inv_n) for joint, w in total_accum.items()]
    avg_pairs.sort(key=lambda x: x[1], reverse=True)
    avg_pairs = avg_pairs[:out_elem]
    s = sum(w for _, w in avg_pairs)
    avg_pairs = [(joint, w / s) for joint, w in avg_pairs]

    while len(avg_pairs) < out_elem:
        avg_pairs.append((0, 0.0))

    out_ji = []
    out_jw = []
    for _ in patch_points:
        out_ji.extend([j for j, _ in avg_pairs])
        out_jw.extend([w for _, w in avg_pairs])

    binding = UsdSkel.BindingAPI.Apply(patch_prim)
    binding.CreateSkeletonRel().SetTargets([skeleton_path])

    ji_pv = binding.CreateJointIndicesPrimvar(False, out_elem)
    ji_pv.SetInterpolation(UsdGeom.Tokens.vertex)
    ji_pv.Set(Vt.IntArray(out_ji))

    jw_pv = binding.CreateJointWeightsPrimvar(False, out_elem)
    jw_pv.SetInterpolation(UsdGeom.Tokens.vertex)
    jw_pv.Set(Vt.FloatArray(out_jw))

    # Stable behavior: final patch has no geomBindTransform.
    gbt_attr = patch_prim.GetAttribute("primvars:skel:geomBindTransform")
    if gbt_attr and gbt_attr.HasAuthoredValue():
        gbt_attr.Clear()

    print("[INFO] Applied averaged vertex skin weights to patch")


def inject_patch(
    *,
    original_path: str,
    patched_path: str,
    output_path: str,
    patch_path: Optional[str] = None,
    patch_keyword: Optional[str] = None,
    target_mesh_path: Optional[str] = None,
    target_mesh_keyword: Optional[str] = None,
    dst_patch_name: str = "BackPatch",
    offset_distance: float = 0.003,
    k_nearest: int = 4,
    max_influences: Optional[int] = None,
    flip_winding: bool = False,
    print_top: int = 8,
) -> None:
    original_stage = Usd.Stage.Open(original_path)
    if not original_stage:
        die(f"Could not open original USD: {original_path}")

    patched_stage = Usd.Stage.Open(patched_path)
    if not patched_stage:
        die(f"Could not open patched USD: {patched_path}")

    src_patch_prim = find_source_patch_mesh(
        patched_stage,
        patch_path=patch_path,
        patch_keyword=patch_keyword,
    )
    print(f"[INFO] Source patch: {src_patch_prim.GetPath()}")

    if target_mesh_path or target_mesh_keyword:
        target_mesh_prim = find_mesh(
            original_stage,
            exact_path=target_mesh_path,
            keyword=target_mesh_keyword,
            label="target skinned mesh",
        )
        if not has_vertex_skinning(target_mesh_prim):
            die(f"Target mesh is not a skinned mesh with vertex weights: {target_mesh_prim.GetPath()}")
        print(f"[INFO] Target mesh override: {target_mesh_prim.GetPath()}")
    else:
        target_mesh_prim = auto_detect_target_mesh(
            original_stage,
            src_patch_prim,
            exclude_keywords=("patch", "eye", "eyelash", "teeth", "tongue", "hair"),
            print_top=print_top,
        )

    skeleton_path = get_skeleton_target(target_mesh_prim)
    print(f"[INFO] Skeleton: {skeleton_path}")

    dst_parent = target_mesh_prim.GetParent()
    dst_patch_path = dst_parent.GetPath().AppendChild(dst_patch_name)
    print(f"[INFO] Destination patch: {dst_patch_path}")

    dst_patch_prim = copy_patch_geometry(
        src_patch_prim=src_patch_prim,
        original_stage=original_stage,
        dst_parent_prim=dst_parent,
        dst_patch_path=dst_patch_path,
        flip_winding=flip_winding,
    )

    offset_patch_uniform(
        patch_prim=dst_patch_prim,
        target_mesh_prim=target_mesh_prim,
        offset_distance=offset_distance,
    )

    transfer_averaged_skinning(
        patch_prim=dst_patch_prim,
        target_mesh_prim=target_mesh_prim,
        skeleton_path=skeleton_path,
        k_nearest=k_nearest,
        max_influences=max_influences,
    )

    original_stage.GetRootLayer().Export(output_path)
    print(f"[OK] Wrote {output_path}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--original", required=True, help="Original clean character USD")
    ap.add_argument("--patched", required=True, help="Blender-exported USD containing the modeled patch")
    ap.add_argument("--output", required=True, help="Output USD")

    ap.add_argument("--patch-path", default=None, help="Exact patch mesh path in patched USD, e.g. /root/Plane/Plane")
    ap.add_argument("--patch-keyword", default=None, help="Keyword for patch mesh path if --patch-path is not set")

    ap.add_argument("--target-mesh-path", default=None, help="Optional exact target skinned mesh path in original USD")
    ap.add_argument("--target-mesh-keyword", default=None, help="Optional target skinned mesh keyword, e.g. vest or shirt")

    ap.add_argument("--dst-patch-name", default="BackPatch")
    ap.add_argument("--offset-distance", type=float, default=0.003, help="Uniform outward offset. Use 0 to disable.")
    ap.add_argument("--k-nearest", type=int, default=4)
    ap.add_argument("--max-influences", type=int, default=None)
    ap.add_argument("--flip-winding", action="store_true", help="Flip patch face winding if the front/back side is wrong")
    ap.add_argument("--print-top", type=int, default=8, help="Number of auto target candidates to print")

    ap.add_argument("--list-patched-meshes", action="store_true", help="List Mesh prims in --patched and exit")

    args = ap.parse_args()

    if args.list_patched_meshes:
        stage = Usd.Stage.Open(args.patched)
        if not stage:
            die(f"Could not open patched USD: {args.patched}")
        print("=== Mesh prims in patched USD ===")
        for m in all_meshes(stage):
            flag = "skinned" if has_vertex_skinning(m) else "non-skinned"
            print(f"{m.GetPath()}  [{flag}]")
        return

    inject_patch(
        original_path=args.original,
        patched_path=args.patched,
        output_path=args.output,
        patch_path=args.patch_path,
        patch_keyword=args.patch_keyword,
        target_mesh_path=args.target_mesh_path,
        target_mesh_keyword=args.target_mesh_keyword,
        dst_patch_name=args.dst_patch_name,
        offset_distance=args.offset_distance,
        k_nearest=args.k_nearest,
        max_influences=args.max_influences,
        flip_winding=args.flip_winding,
        print_top=args.print_top,
    )


if __name__ == "__main__":
    main()
