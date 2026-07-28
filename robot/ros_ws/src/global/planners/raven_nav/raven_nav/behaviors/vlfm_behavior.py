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
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from raven_nav.behaviors.frontier_behavior import (
    _points_in_polygon, _peer_penalty, _neighborhood_density,
    _cells_set_from_xys, NOVELTY_WEIGHT, NOVELTY_NEIGHBORHOOD_CELLS)


class VLFMBehavior:
    MAGNITUDE_M = 6.0        # look-ahead past the selected ray frontier
    BLEND_ALPHA = 0.8        # intermediate pose fraction toward the goal
    STANDOFF_M = 3.0         # xy standoff from a target voxel cluster
    VISIT_M = 3.0            # cluster center within this -> mark visited
    UNLOCK_M = 4.0           # within this of the look-ahead -> release lock
    NEAR_VISITED_M = 10.0    # cluster center within this of a visited one -> skip
    VOX_SIZE_M = 0.5
    VIZ_MAX_RAYS = 80        # cap candidate-ray arrows to avoid clutter
    VIZ_ARROW_M = 2.5        # arrow length (m)
    VIZ_LIFETIME_S = 2       # markers auto-expire so stale ones self-clear
    # Semantic reward scale (meters of travel a unit of sim is worth). Combined
    # with frontier-style costs (distance + peer repulsion + novelty), this makes
    # multi-robot VLFM spread out: flat sim -> coordinated frontier exploration;
    # peaked sim -> pursue the high-value ray. Tunable via the vlfm_value_weight
    # param — watch the per-component [vlfm] explore log to calibrate.
    VALUE_WEIGHT = 300.0

    def __init__(self, get_clock, min_altitude=1.5, max_altitude=100.0,
                 voxel_score_threshold=0.9, voxel_min_cluster_size=30,
                 value_weight=VALUE_WEIGHT):
        self.get_clock = get_clock
        self.name = 'VLFM-based'
        self.min_altitude = float(min_altitude)
        self.max_altitude = float(max_altitude)
        self.voxel_score_threshold = float(voxel_score_threshold)
        self.voxel_min_cluster_size = int(voxel_min_cluster_size)
        self.value_weight = float(value_weight)
        self.visited_clusters = []   # list of center (3,) np arrays

    def condition_check(self):
        return True

    def execute(self, ray_origins, ray_scores, ray_dirs, vox_xyz, vox_scores,
                query_labels, target_objects, cur_pose_np, waypoint_locked,
                target_waypoint, target_waypoint2, publisher_dict,
                search_area_xy=None, debug_logger=None,
                peer_state=None, my_id=0, peer_weights=None,
                completed_zones_xy=None, cell_size_m=0.5):
        if cur_pose_np is None or not target_objects or not query_labels:
            return waypoint_locked, target_waypoint, target_waypoint2
        cols = [query_labels.index(t) for t in target_objects
                if t in query_labels]
        if not cols:
            return waypoint_locked, target_waypoint, target_waypoint2

        engaged = self._go_to_object(vox_xyz, vox_scores, cols, cur_pose_np,
                                     publisher_dict, search_area_xy, debug_logger)
        if engaged is not None:
            wp1, wp2 = engaged
            return True, wp1, wp2

        return self._explore(ray_origins, ray_scores, ray_dirs, cols,
                             cur_pose_np, waypoint_locked, target_waypoint,
                             target_waypoint2, publisher_dict, search_area_xy,
                             debug_logger, peer_state, my_id, peer_weights,
                             completed_zones_xy, cell_size_m)

    # ── go-to-object ─────────────────────────────────────────────────────────

    def _go_to_object(self, vox_xyz, vox_scores, cols, cur_pose_np,
                      publisher_dict, search_area_xy, debug_logger=None):
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
        self._publish_target_marker(publisher_dict, approach, (0.1, 0.6, 1.0))
        self._publish_status(publisher_dict,
                             f'VLFM go-to-object -> '
                             f'({approach[0]:.0f},{approach[1]:.0f})')
        if debug_logger is not None:
            debug_logger.info(
                f'[vlfm] go-to-object: {len(clusters)} cluster(s), '
                f'nearest center=({center[0]:.1f},{center[1]:.1f},{center[2]:.1f}) '
                f'approach=({approach[0]:.1f},{approach[1]:.1f},{approach[2]:.1f})',
                throttle_duration_sec=2.0)
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
                 publisher_dict, search_area_xy, debug_logger=None,
                 peer_state=None, my_id=0, peer_weights=None,
                 completed_zones_xy=None, cell_size_m=0.5):
        if ray_origins is None or ray_scores is None or ray_origins.shape[0] == 0:
            if debug_logger is not None:
                debug_logger.warn('[vlfm] explore: no semantic rays yet — holding',
                                  throttle_duration_sec=2.0)
            return waypoint_locked, target_waypoint, target_waypoint2
        z = ray_origins[:, 2]
        valid = (z >= self.min_altitude) & (z <= self.max_altitude)
        if search_area_xy is not None:
            valid &= _points_in_polygon(ray_origins[:, :2], search_area_xy)
        if not valid.any():
            if debug_logger is not None:
                debug_logger.warn(
                    f'[vlfm] explore: 0/{ray_origins.shape[0]} rays pass '
                    f'altitude band [{self.min_altitude:.0f},{self.max_altitude:.0f}]'
                    f' + polygon — holding', throttle_duration_sec=2.0)
            return waypoint_locked, target_waypoint, target_waypoint2

        # Per-ray semantic reward (best target sim) blended with frontier-style
        # costs at the ray origin. cost is minimized: value pulls toward high-sim
        # rays, peer repulsion + novelty + distance spread robots and avoid
        # re-covered ground. Evaluated at the ray origin (the frontier point),
        # matching how FrontierBehavior scores viewpoint centroids.
        per_ray = ray_scores[:, cols].astype(float).max(axis=1)
        dist = np.linalg.norm(ray_origins - np.asarray(cur_pose_np, float), axis=1)
        peer_pen, _ = _peer_penalty(ray_origins, peer_state, my_id,
                                    peer_weight=peer_weights)
        novelty = np.zeros(ray_origins.shape[0], dtype=np.float64)
        cells = (_cells_set_from_xys(completed_zones_xy, cell_size_m)
                 if completed_zones_xy is not None else set())
        if cells:
            novelty = NOVELTY_WEIGHT * _neighborhood_density(
                ray_origins[:, :2], cells, cell_size_m, NOVELTY_NEIGHBORHOOD_CELLS)
        cost = dist + peer_pen + novelty - self.value_weight * per_ray
        cost[~valid] = np.inf
        ray_idx = int(np.argmin(cost))

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
        best = float(per_ray[ray_idx])
        self._publish_ray_viz(publisher_dict, ray_origins, ray_dirs, per_ray,
                              valid, ray_idx, wp1)
        self._publish_status(publisher_dict,
                             f'VLFM explore -> ({wp1[0]:.0f},{wp1[1]:.0f}) '
                             f'sim={best:.2f}')
        if debug_logger is not None:
            debug_logger.info(
                f'[vlfm] explore: {int(valid.sum())}/{ray_origins.shape[0]} rays | '
                f'sel sim={best:.2f} '
                f'val={-self.value_weight * best:.0f} dist={float(dist[ray_idx]):.0f} '
                f'peer={float(peer_pen[ray_idx]):.0f} nov={float(novelty[ray_idx]):.0f} '
                f'cost={float(cost[ray_idx]):.0f} -> '
                f'({wp1[0]:.0f},{wp1[1]:.0f},{wp1[2]:.0f})',
                throttle_duration_sec=2.0)
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

    # ── visualization (reuses the filtered_rays / current_target topics) ─────────

    def _publish_status(self, publisher_dict, text):
        pub = publisher_dict.get('current_target')
        if pub is not None:
            pub.publish(String(data=text))

    def _arrow(self, i, origin, tip, rgb, now, width=0.4):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = now
        m.ns = 'vlfm'
        m.id = i
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.points = [Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2])),
                    Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2]))]
        m.scale.x = width
        m.scale.y = width * 2.5
        m.scale.z = width * 1.5
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 0.6
        m.lifetime.sec = self.VIZ_LIFETIME_S
        return m

    def _sphere(self, i, pt, rgb, now, size=3.0):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = now
        m.ns = 'vlfm'
        m.id = i
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(pt[0])
        m.pose.position.y = float(pt[1])
        m.pose.position.z = float(pt[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = size
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 0.9
        m.lifetime.sec = self.VIZ_LIFETIME_S
        return m

    def _publish_ray_viz(self, publisher_dict, origins, dirs, per_ray, valid,
                         sel_idx, target_pt):
        """Candidate rays as arrows colored by similarity (red->green) + the
        selected target as a magenta sphere, on the filtered_rays topic."""
        pub = publisher_dict.get('filtered_rays')
        if pub is None:
            return
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()
        vidx = np.where(valid)[0]
        if vidx.size:
            order = np.argsort(-per_ray[vidx])[:self.VIZ_MAX_RAYS]
            vidx = vidx[order]
            lo = float(per_ray[valid].min())
            hi = float(per_ray[valid].max())
            span = (hi - lo) or 1.0
            for j, i in enumerate(vidx):
                t = (float(per_ray[i]) - lo) / span
                o = origins[i]
                tip = o + (dirs[i] if dirs is not None else 0.0) * self.VIZ_ARROW_M
                ma.markers.append(
                    self._arrow(j + 1, o, tip, (1.0 - t, t, 0.15), now))
        ma.markers.append(self._sphere(0, target_pt, (1.0, 0.0, 1.0), now))
        pub.publish(ma)

    def _publish_target_marker(self, publisher_dict, target_pt, rgb):
        pub = publisher_dict.get('filtered_rays')
        if pub is None:
            return
        ma = MarkerArray()
        ma.markers.append(
            self._sphere(0, target_pt, rgb, self.get_clock().now().to_msg()))
        pub.publish(ma)
