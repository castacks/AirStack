"""Vendored ORIGINAL RAVEN behaviour maths, for the parity test.

Copied from `RayFronts_raven/rayfronts/behaviors/{frontier,ray,voxel}_behavior.py`
with only the ROS publishing and printing removed; every arithmetic line is the
OG line, torch and all. This file is test-only reference material — nothing in
`raven_nav/` imports it.

Known OG defect kept as written: `og_ray_groups` applies the forward-filter
mask to `xy_dirs_np_normed` alone (OG ray_behavior.py:85) and then indexes the
UNFILTERED `orig_world` / `dir_world` with the filtered group indices
(OG:116-117). The parity test therefore drives it with inputs where every ray
passes the forward filter, so the two implementations are comparable.
"""
import numpy as np
import torch
from sklearn.cluster import DBSCAN


# ── frontier_behavior.py ────────────────────────────────────────────────────
def og_frontier_viewpoints(frontiers_rdf: torch.Tensor) -> torch.Tensor:
    """OG frontier_behavior.py:25-49."""
    transformed_frontiers = torch.stack([
        frontiers_rdf[:, 2], -frontiers_rdf[:, 0], -frontiers_rdf[:, 1]], dim=1)
    transformed_frontiers = transformed_frontiers[transformed_frontiers[:, 2] > 1.5]
    frontiers_cpu = transformed_frontiers.detach().cpu().numpy()
    clustering = DBSCAN(eps=2.7, min_samples=3).fit(frontiers_cpu)
    labels = clustering.labels_
    unique_labels = [l for l in set(labels) if l != -1]
    viewpoints = []
    for l in unique_labels:
        cluster_pts = frontiers_cpu[labels == l]
        centroid = cluster_pts.mean(axis=0)
        centroid_torch = torch.from_numpy(centroid).to(
            transformed_frontiers.device, dtype=transformed_frontiers.dtype)
        if centroid_torch[2] > 2.0:
            viewpoints.append(centroid_torch)
    if not viewpoints:
        return torch.zeros((0, 3), dtype=frontiers_rdf.dtype)
    return torch.stack(viewpoints)


def og_frontier_scores(viewpoints: torch.Tensor, cur_pose_np: np.ndarray,
                       target_waypoint) -> torch.Tensor:
    """OG frontier_behavior.py:54-67."""
    robot_pos_torch = torch.tensor(cur_pose_np, dtype=viewpoints.dtype,
                                   device=viewpoints.device)
    distances = torch.norm(viewpoints - robot_pos_torch, dim=1)
    if target_waypoint is not None:
        target_waypoint_tensor = torch.tensor(
            target_waypoint, device=viewpoints.device, dtype=viewpoints.dtype)
        cur_motion_vec = target_waypoint_tensor - robot_pos_torch
        cur_motion_vec = cur_motion_vec / (torch.norm(cur_motion_vec) + 1e-6)
        candidate_vecs = viewpoints - robot_pos_torch
        candidate_vecs = candidate_vecs / (
            torch.norm(candidate_vecs, dim=1, keepdim=True) + 1e-6)
        cos_sim = torch.matmul(candidate_vecs, cur_motion_vec)
        momentum_weight = 5.0
        return distances + momentum_weight * (1.0 - cos_sim)
    return distances


def og_frontier_waypoints(best_cent_np: np.ndarray, cur_pose_np: np.ndarray):
    """OG frontier_behavior.py:80-84."""
    target_waypoint = best_cent_np
    dir = target_waypoint - cur_pose_np
    dir = dir / np.linalg.norm(target_waypoint - cur_pose_np)
    return target_waypoint, target_waypoint + 2.0 * dir


# ── ray_behavior.py ─────────────────────────────────────────────────────────
def og_ray_groups(orig_world: torch.Tensor, dir_world: torch.Tensor,
                  cur_pose_np: np.ndarray):
    """OG ray_behavior.py:69-137. Returns (angle_groups, scored_groups)."""
    xy_dirs = dir_world[:, :2]
    xy_dirs_np = xy_dirs.cpu().numpy()
    xy_dirs_np_normed = xy_dirs_np / np.linalg.norm(xy_dirs_np, axis=1,
                                                    keepdims=True)
    cur_xy = cur_pose_np[:2]
    orig_xy = orig_world[:, :2]
    dir_xy = xy_dirs_np_normed
    ray_target_xy = orig_xy.cpu().numpy() + dir_xy
    to_ray_target = ray_target_xy - cur_xy
    dot = np.einsum('ij,ij->i', dir_xy, to_ray_target)
    valid_mask = dot > 0
    xy_dirs_np_normed = xy_dirs_np_normed[valid_mask]

    angle_groups = []
    angle_threshold_cos = np.cos(np.deg2rad(45))
    for i, xy_dir in enumerate(xy_dirs_np_normed):
        assigned = False
        for group in angle_groups:
            d = np.dot(xy_dir, group['centroid'])
            if d >= angle_threshold_cos:
                group['indices'].append(i)
                group['rays'].append(xy_dir)
                group['centroid'] = np.mean(group['rays'], axis=0)
                group['centroid'] /= np.linalg.norm(group['centroid'])
                assigned = True
                break
        if not assigned:
            angle_groups.append({'centroid': xy_dir, 'rays': [xy_dir],
                                 'indices': [i]})
    MIN_RAYS_PER_GROUP = 1
    angle_groups = [g for g in angle_groups if len(g['rays']) >= MIN_RAYS_PER_GROUP]

    group_averages = []
    for group in angle_groups:
        group_idx = group['indices']
        group_origins = orig_world[group_idx]
        group_directions = dir_world[group_idx]
        avg_origin = group_origins.mean(dim=0)
        avg_direction = group_directions.mean(dim=0)
        avg_direction = avg_direction / avg_direction.norm()
        density = len(group['rays'])
        group_averages.append((avg_origin, avg_direction, density))

    k = 5.0
    scored_groups = sorted(
        group_averages,
        key=lambda g: np.linalg.norm(g[0].cpu().numpy() - cur_pose_np) - k * g[2])
    return angle_groups, scored_groups


def og_ray_waypoints(best_origin_np, best_direction_np):
    """OG ray_behavior.py:139-163."""
    magnitude = 6.0
    origin = best_origin_np
    direction = best_direction_np / np.linalg.norm(best_direction_np)
    return origin + direction * magnitude, origin + direction * magnitude * 2


# ── voxel_behavior.py ───────────────────────────────────────────────────────
def og_voxel_clusters(filtered_vox_rdf: torch.Tensor, vox_size: float = 0.5,
                      min_voxels: int = 30):
    """OG voxel_behavior.py:58-94 — returns [cx,cy,cz,sx,sy,sz] rows in FLU."""
    import scipy.ndimage
    filtered_vox = filtered_vox_rdf.round(decimals=3)
    out = []
    if filtered_vox.numel() == 0:
        return out
    min_coords = filtered_vox.min(dim=0)[0]
    norm_coords = ((filtered_vox - min_coords) / vox_size).long()
    max_coords = norm_coords.max(dim=0)[0] + 1
    occupancy = np.zeros(tuple(max_coords.tolist()), dtype=np.uint8)
    for x, y, z in norm_coords:
        occupancy[x, y, z] = 1
    structure = np.ones((3, 3, 3), dtype=np.uint8)
    labeled, num_components = scipy.ndimage.label(occupancy, structure=structure)
    label_ids = torch.tensor([labeled[x, y, z] for x, y, z in norm_coords])
    norm_np = norm_coords.cpu().numpy()
    for label_val in range(1, num_components + 1):
        idx = (label_ids == label_val).nonzero(as_tuple=True)[0]
        if len(idx) < min_voxels:
            continue
        coords = norm_np[idx]
        min_voxel = coords.min(axis=0)
        max_voxel = coords.max(axis=0)
        min_world = min_voxel * vox_size + min_coords.cpu().numpy()
        max_world = (max_voxel + 1) * vox_size + min_coords.cpu().numpy()
        center = (min_world + max_world) / 2
        size = max_world - min_world
        out.append([center[2], -center[0], -center[1],
                    size[2], size[0], size[1]])
    return out


def og_cuboid_distance(center_a, size_a, center_b, size_b):
    """OG voxel_behavior.py:239-245."""
    half_a = size_a / 2.0
    half_b = size_b / 2.0
    dx = max(abs(center_a[0] - center_b[0]) - (half_a[0] + half_b[0]), 0)
    dy = max(abs(center_a[1] - center_b[1]) - (half_a[1] + half_b[1]), 0)
    dz = max(abs(center_a[2] - center_b[2]) - (half_a[2] + half_b[2]), 0)
    return np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)


def og_voxel_standoff(center, sizes, cur_pose_np):
    """OG voxel_behavior.py:136-159 — returns the standoff point or None."""
    center = np.asarray(center, dtype=float)
    sizes = np.asarray(sizes, dtype=float)
    half_sizes = sizes / 2.0
    dir = center - cur_pose_np
    dir_norm = dir / np.linalg.norm(dir)
    ray_origin_local = cur_pose_np - center
    tmin, tmax = -np.inf, np.inf
    for axis in range(3):
        if dir_norm[axis] != 0:
            t1 = (-half_sizes[axis] - ray_origin_local[axis]) / dir_norm[axis]
            t2 = (half_sizes[axis] - ray_origin_local[axis]) / dir_norm[axis]
            tmin = max(tmin, min(t1, t2))
            tmax = min(tmax, max(t1, t2))
        else:
            if abs(ray_origin_local[axis]) > half_sizes[axis]:
                continue
    if tmax < max(tmin, 0):
        return None
    t_hit = tmin if tmin > 0 else tmax
    surface_point = cur_pose_np + dir_norm * t_hit
    return surface_point - dir_norm * 1.0


# ── lvlm_behavior.py ────────────────────────────────────────────────────────
def og_set_guiding_objects(objects_string: str):
    """OG lvlm_behavior.py:19-35."""
    raw_objects = [o.strip().lower() for o in objects_string.split(",")]
    cleaned_objects = []
    seen = set()
    for obj in raw_objects:
        if obj.startswith("a "):
            obj = obj[2:]
        elif obj.startswith("an "):
            obj = obj[3:]
        elif obj.startswith("the "):
            obj = obj[4:]
        obj = obj.rstrip('.,').strip()
        if obj not in seen and obj != "":
            cleaned_objects.append(obj)
            seen.add(obj)
    return cleaned_objects


def og_lvlm_waypoints(fo: torch.Tensor, fd: torch.Tensor):
    """OG lvlm_behavior.py:97-110, on already-FLU tensors."""
    mean_origin = torch.mean(fo, dim=0)
    mean_direction = torch.mean(fd, dim=0)
    origin = mean_origin.cpu().numpy()
    direction = mean_direction.cpu().numpy()
    magnitude = 5.0
    unit_dir = direction / np.linalg.norm(direction)
    return origin, origin + unit_dir * magnitude
