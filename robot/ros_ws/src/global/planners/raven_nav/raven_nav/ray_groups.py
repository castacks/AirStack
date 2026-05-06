"""Canonical filtered-and-grouped ray representation.

Filtering pipeline (applied once per tick by raven_nav_node):
  1. score > threshold for at least one target label (softmax-based scores
     sum to 1, so this also excludes background-dominant rays)
  2. altitude in [min, max]
  3. argmax-within-targets label assignment
  4. greedy clustering: same-label rays within angle_thresh on XY direction

The forward-only "behind-ray" filter is intentionally NOT here — bidding needs
every direction, regardless of whether the ray points away from the bidder.
Apply it consumer-side only when picking a waypoint (see ray_behavior).

Each resulting group carries enough derived state to feed both the bid auction
and ray-mode waypoint generation without recomputing.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class RayGroup:
    label: str
    ray_origins: np.ndarray              # (N, 3)
    ray_dirs:    np.ndarray              # (N, 3) normalized
    ray_scores:  np.ndarray              # (N,)   score for the assigned label

    avg_origin:        np.ndarray = field(init=False)  # (3,)
    avg_dir:           np.ndarray = field(init=False)  # (3,) normalized
    num_rays:          int        = field(init=False)
    avg_score:         float      = field(init=False)
    max_score:         float      = field(init=False)
    min_dist_to_robot: float      = field(init=False)
    avg_dist_to_robot: float      = field(init=False)

    def _finalize(self, robot_pos: np.ndarray) -> None:
        self.num_rays = len(self.ray_origins)
        self.avg_origin = self.ray_origins.mean(axis=0)
        d = self.ray_dirs.mean(axis=0)
        self.avg_dir = d / (np.linalg.norm(d) + 1e-6)
        self.avg_score = float(self.ray_scores.mean())
        self.max_score = float(self.ray_scores.max())
        dists = np.linalg.norm(self.ray_origins - robot_pos[None, :], axis=1)
        self.min_dist_to_robot = float(dists.min())
        self.avg_dist_to_robot = float(dists.mean())


def compute_ray_groups(
    ray_origins: Optional[np.ndarray],
    ray_dirs:    Optional[np.ndarray],
    ray_scores:  Optional[np.ndarray],
    query_labels: list,
    target_objects: list,
    score_threshold: float,
    cur_pose: np.ndarray,
    min_altitude: float = 1.5,
    max_altitude: float = 100.0,
    angle_thresh_deg: float = 45.0,
) -> List[RayGroup]:
    if (ray_origins is None or ray_scores is None
            or len(ray_origins) == 0 or not target_objects):
        return []
    label_indices = [query_labels.index(t) for t in target_objects
                     if t in query_labels]
    if not label_indices:
        return []

    # 1. score threshold on target columns
    relevant = ray_scores[:, label_indices]
    keep = np.where((relevant > score_threshold).any(axis=1))[0]
    if keep.size == 0:
        return []

    # 2. altitude
    z = ray_origins[keep, 2]
    keep = keep[(z >= min_altitude) & (z <= max_altitude)]
    if keep.size == 0:
        return []

    # 3. per-ray argmax label
    rel_keep = relevant[keep]
    best_cols = np.argmax(rel_keep, axis=1)
    per_label = [target_objects[c] for c in best_cols]
    per_score = rel_keep[np.arange(len(keep)), best_cols]

    # 4. greedy per-label clustering on XY direction
    dirs_keep = ray_dirs[keep]
    xy_dirs = dirs_keep[:, :2]
    norms = np.linalg.norm(xy_dirs, axis=1, keepdims=True)
    xy_norm = xy_dirs / (norms + 1e-6)
    angle_cos = np.cos(np.deg2rad(angle_thresh_deg))
    buckets = []
    for i in range(len(keep)):
        xy_dir = xy_norm[i]
        label = per_label[i]
        placed = False
        for b in buckets:
            if b['label'] != label:
                continue
            if np.dot(xy_dir, b['centroid']) >= angle_cos:
                b['indices'].append(i)
                b['rays'].append(xy_dir)
                c = np.mean(b['rays'], axis=0)
                b['centroid'] = c / (np.linalg.norm(c) + 1e-6)
                placed = True
                break
        if not placed:
            buckets.append({
                'label': label, 'indices': [i],
                'rays': [xy_dir], 'centroid': xy_dir.copy(),
            })

    keep_origins = ray_origins[keep]
    keep_dirs = ray_dirs[keep]
    groups: List[RayGroup] = []
    for b in buckets:
        idx = b['indices']
        g = RayGroup(
            label=b['label'],
            ray_origins=keep_origins[idx],
            ray_dirs=keep_dirs[idx],
            ray_scores=per_score[idx],
        )
        g._finalize(cur_pose)
        groups.append(g)
    return groups
