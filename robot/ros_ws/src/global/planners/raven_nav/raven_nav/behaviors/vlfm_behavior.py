"""VLFM baseline behavior (single-robot).

Port of RayFronts' VLFM (rayfronts/behaviors/{vlfm_behavior,voxel_behavior}.py)
adapted to raven's decoupled setup: instead of aligning raw ray/voxel features to
language through the encoder, it consumes the per-query similarity RayFronts
already publishes (rays_sim/all, voxels_sim/all), which the node stores as
_ray_scores / _vox_scores. Two greedy phases:

  * go-to-object: confident target voxels -> cluster -> fly to nearest unvisited.
  * explore:      else fly toward the highest-similarity ray frontier (argmax).

No thresholding of the explore argmax, no ray grouping, no peer coordination or
geo-priors (those are RAVEN, not the baseline).
"""

import numpy as np
import scipy.ndimage
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from raven_nav.behaviors.frontier_behavior import _points_in_polygon


class VLFMBehavior:
    MAGNITUDE_M = 6.0        # look-ahead past the selected ray frontier
    BLEND_ALPHA = 0.8        # intermediate pose fraction toward the goal
    STANDOFF_M = 3.0         # xy standoff from a target voxel cluster
    VISIT_M = 3.0            # cluster center within this -> mark visited
    UNLOCK_M = 4.0           # within this of the look-ahead -> release lock
    NEAR_VISITED_M = 10.0    # cluster center within this of a visited one -> skip
    VOX_SIZE_M = 0.5

    def __init__(self, get_clock, min_altitude=1.5, max_altitude=100.0,
                 voxel_score_threshold=0.9, voxel_min_cluster_size=30):
        self.get_clock = get_clock
        self.name = 'VLFM-based'
        self.min_altitude = float(min_altitude)
        self.max_altitude = float(max_altitude)
        self.voxel_score_threshold = float(voxel_score_threshold)
        self.voxel_min_cluster_size = int(voxel_min_cluster_size)
        self.visited_clusters = []   # list of center (3,) np arrays

    def condition_check(self):
        return True

    def execute(self, ray_origins, ray_scores, ray_dirs, vox_xyz, vox_scores,
                query_labels, target_objects, cur_pose_np, waypoint_locked,
                target_waypoint, target_waypoint2, publisher_dict,
                search_area_xy=None):
        if cur_pose_np is None or not target_objects or not query_labels:
            return waypoint_locked, target_waypoint, target_waypoint2
        cols = [query_labels.index(t) for t in target_objects
                if t in query_labels]
        if not cols:
            return waypoint_locked, target_waypoint, target_waypoint2

        engaged = self._go_to_object(vox_xyz, vox_scores, cols, cur_pose_np,
                                     publisher_dict, search_area_xy)
        if engaged is not None:
            wp1, wp2 = engaged
            return True, wp1, wp2

        return self._explore(ray_origins, ray_scores, ray_dirs, cols,
                             cur_pose_np, waypoint_locked, target_waypoint,
                             target_waypoint2, publisher_dict, search_area_xy)

    # ── go-to-object ─────────────────────────────────────────────────────────

    def _go_to_object(self, vox_xyz, vox_scores, cols, cur_pose_np,
                      publisher_dict, search_area_xy):
        if vox_xyz is None or vox_scores is None or vox_xyz.shape[0] == 0:
            return None
        relevant = vox_scores[:, cols]
        hit = vox_xyz[(relevant > self.voxel_score_threshold).any(axis=1)]
        if hit.shape[0] > 0 and search_area_xy is not None:
            hit = hit[_points_in_polygon(hit[:, :2], search_area_xy)]
        if hit.shape[0] == 0:
            return None

        clusters = [(c, s) for (c, s) in self._cluster(hit)
                    if not self._is_near_visited(c)]
        if not clusters:
            return None
        clusters.sort(key=lambda cs: float(np.linalg.norm(cur_pose_np - cs[0])))
        center, size = clusters[0]

        if float(np.linalg.norm(cur_pose_np - center)) < self.VISIT_M:
            self.visited_clusters.append(center)

        approach = self._approach_point(center, size, cur_pose_np)
        self._publish_path([self._blend(cur_pose_np, approach), approach],
                           publisher_dict)
        return approach, approach

    def _cluster(self, pts):
        """Connected-component cluster FLU voxel centers into (center, size)."""
        p = np.round(pts, 3)
        min_c = p.min(axis=0)
        norm = np.floor((p - min_c) / self.VOX_SIZE_M).astype(int)
        dims = tuple((norm.max(axis=0) + 1).tolist())
        occ = np.zeros(dims, dtype=np.uint8)
        occ[norm[:, 0], norm[:, 1], norm[:, 2]] = 1
        labeled, n = scipy.ndimage.label(occ, structure=np.ones((3, 3, 3)))
        labels_at = labeled[norm[:, 0], norm[:, 1], norm[:, 2]]
        out = []
        for lv in range(1, n + 1):
            idx = np.where(labels_at == lv)[0]
            if idx.size < self.voxel_min_cluster_size:
                continue
            coords = norm[idx]
            min_w = coords.min(axis=0) * self.VOX_SIZE_M + min_c
            max_w = (coords.max(axis=0) + 1) * self.VOX_SIZE_M + min_c
            out.append(((min_w + max_w) / 2.0, max_w - min_w))
        return out

    def _is_near_visited(self, center):
        return any(float(np.linalg.norm(center - v)) < self.NEAR_VISITED_M
                   for v in self.visited_clusters)

    def _approach_point(self, center, size, cur_pose_np):
        """A point self.STANDOFF_M out from the cluster in XY, at flight altitude."""
        d = (cur_pose_np - center).astype(float)
        d[2] = 0.0
        n = float(np.linalg.norm(d))
        u = d / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])
        reach = float(max(size[0], size[1]) / 2.0) + self.STANDOFF_M
        xy = center[:2] + u[:2] * reach
        z = float(np.clip(cur_pose_np[2], self.min_altitude, self.max_altitude))
        return np.array([xy[0], xy[1], z])

    # ── explore ──────────────────────────────────────────────────────────────

    def _explore(self, ray_origins, ray_scores, ray_dirs, cols, cur_pose_np,
                 waypoint_locked, target_waypoint, target_waypoint2,
                 publisher_dict, search_area_xy):
        if ray_origins is None or ray_scores is None or ray_origins.shape[0] == 0:
            return waypoint_locked, target_waypoint, target_waypoint2
        z = ray_origins[:, 2]
        valid = (z >= self.min_altitude) & (z <= self.max_altitude)
        if search_area_xy is not None:
            valid &= _points_in_polygon(ray_origins[:, :2], search_area_xy)
        if not valid.any():
            return waypoint_locked, target_waypoint, target_waypoint2

        relevant = ray_scores[:, cols].astype(float).copy()
        relevant[~valid] = -np.inf
        ray_idx = int(np.argmax(relevant)) // relevant.shape[1]

        origin = ray_origins[ray_idx].astype(float)
        if ray_dirs is not None:
            direction = ray_dirs[ray_idx].astype(float)
        else:
            direction = origin - cur_pose_np
        n = float(np.linalg.norm(direction))
        direction = direction / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])

        wp1 = origin.copy()
        wp1[2] = float(np.clip(wp1[2], self.min_altitude, self.max_altitude))
        wp2 = wp1 + direction * self.MAGNITUDE_M
        wp2[2] = float(np.clip(wp2[2], self.min_altitude, self.max_altitude))

        self._publish_path([self._blend(cur_pose_np, wp1), wp1], publisher_dict)
        if float(np.linalg.norm(cur_pose_np - wp2)) < self.UNLOCK_M:
            waypoint_locked = False
        return waypoint_locked, wp1, wp2

    # ── helpers ────────────────────────────────────────────────────────────────

    def _blend(self, a, b):
        return np.asarray(a, dtype=float) * (1 - self.BLEND_ALPHA) + \
            np.asarray(b, dtype=float) * self.BLEND_ALPHA

    def _publish_path(self, points, publisher_dict):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for pt in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.position.z = float(pt[2])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        publisher_dict['path'].publish(path)
