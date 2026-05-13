import torch
import numpy as np
from sklearn.cluster import DBSCAN
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def _points_in_polygon(pts_xy: np.ndarray, poly_xy: np.ndarray) -> np.ndarray:
    """Vectorized even-odd ray cast. pts_xy: (N,2); poly_xy: (M,2)."""
    if pts_xy.size == 0 or poly_xy.shape[0] < 3:
        return np.zeros(pts_xy.shape[0], dtype=bool)
    px, py = pts_xy[:, 0], pts_xy[:, 1]
    inside = np.zeros(pts_xy.shape[0], dtype=bool)
    M = poly_xy.shape[0]
    for i in range(M):
        x1, y1 = poly_xy[i]
        x2, y2 = poly_xy[(i + 1) % M]
        cond_y = (y1 > py) != (y2 > py)
        with np.errstate(divide='ignore', invalid='ignore'):
            x_cross = (x2 - x1) * (py - y1) / (y2 - y1 + 1e-12) + x1
        inside ^= cond_y & (px < x_cross)
    return inside


def _peer_penalty(viewpoints, peer_state, my_id,
                  repulsion_weight=30.0,
                  repulsion_scale=30.0,
                  tie_distance=5.0,
                  tie_surcharge_mul=2.0):
    """Returns (penalty(M,), breakdown). Lower penalty = better.

    Soft repulsion = weight * exp(-d / scale), applied around every peer
    waypoint. Higher-id robot also pays tie_surcharge for candidates within
    tie_distance of any peer waypoint — asymmetric so two robots eyeing the
    same frontier can't both defer (lower id wins).
    """
    M = viewpoints.shape[0]
    pen = np.zeros(M, dtype=np.float64)
    breakdown = []
    if peer_state is None or M == 0:
        return pen, breakdown
    for name, wp in peer_state.peer_waypoints.items():
        peer_id = peer_state.peer_ids.get(name)
        if peer_id is None:
            continue
        d = np.linalg.norm(viewpoints - wp[None, :], axis=1)
        repulsion = repulsion_weight * np.exp(-d / repulsion_scale)
        pen += repulsion
        surcharge = np.zeros(M, dtype=np.float64)
        applied_tb = my_id > peer_id
        if applied_tb:
            close = d < tie_distance
            surcharge[close] = repulsion_weight * tie_surcharge_mul
            pen += surcharge
        breakdown.append({
            'name': name, 'peer_id': peer_id,
            'repulsion': repulsion, 'surcharge': surcharge,
            'nearest_d': float(d.min()) if d.size else float('inf'),
            'applied_tiebreak': applied_tb,
        })
    return pen, breakdown


# Completed-frontier-zone coverage tracker. Each "zone" is an XY center —
# any frontier within ZONE_RADIUS_M of any zone is treated as resolved and
# dropped before clustering. The novelty cost in scoring also penalizes
# candidates near zones so drones drift toward fresh areas.
ZONE_RADIUS_M = 10.0     # frontiers within this of any zone are filtered out
NOVELTY_SCALE_M = 20.0   # exp(-d/scale) decay
NOVELTY_WEIGHT = 25.0    # weight on the closeness-to-explored penalty


def _nearest_zone_dist(pts_xy: np.ndarray, zones_xy: np.ndarray) -> np.ndarray:
    """For each point return distance to the nearest zone center. inf if no zones."""
    if pts_xy.size == 0:
        return np.zeros(0, dtype=np.float64)
    if zones_xy is None or zones_xy.size == 0:
        return np.full(pts_xy.shape[0], np.inf, dtype=np.float64)
    # Pairwise: (N, K) — manageable for small K and a few hundred N.
    diff = pts_xy[:, None, :] - zones_xy[None, :, :]
    d2 = np.sum(diff * diff, axis=2)
    return np.sqrt(d2.min(axis=1))


class FrontierBehavior:
    def __init__(self, get_clock, min_altitude=1.5, max_altitude=100.0):
        self.get_clock = get_clock
        self.name = 'Frontier-based'
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude

    def condition_check(self):
        return True

    def execute(self, frontiers_raw, cur_pose_np, waypoint_locked, target_waypoint,
                target_waypoint2, publisher_dict,
                peer_state=None, my_id=0, search_area_xy=None,
                debug_logger=None,
                committed_target_dir=None,
                committed_target_origin=None,
                completed_zones_xy=None):
        viewpoint_publisher = publisher_dict['viewpoint']
        raw_frontier_publisher = publisher_dict.get('raw_frontiers')
        path_publisher = publisher_dict['path']

        if frontiers_raw is None or len(frontiers_raw) == 0:
            return waypoint_locked, target_waypoint, target_waypoint2

        # RDF -> FLU.
        frontiers_flu = np.stack([
            frontiers_raw[:, 2],
            -frontiers_raw[:, 0],
            -frontiers_raw[:, 1],
        ], axis=1)

        # Publish OWN raw frontiers UNFILTERED. Each receiver applies its own
        # altitude/polygon filter. Never publish the merged set — that would
        # create a feedback loop where each tick rebroadcasts peer points.
        if raw_frontier_publisher is not None and frontiers_flu.shape[0] > 0:
            raw_frontier_publisher.publish(self._create_pointcloud2_msg(frontiers_flu))

        alt_mask = (frontiers_flu[:, 2] >= self.min_altitude) & (frontiers_flu[:, 2] <= self.max_altitude)
        own_frontiers = frontiers_flu[alt_mask]
        own_count = int(own_frontiers.shape[0])

        polygon_dropped_own = 0
        if search_area_xy is not None and search_area_xy.shape[0] >= 3:
            in_poly = _points_in_polygon(own_frontiers[:, :2], search_area_xy)
            polygon_dropped_own = int((~in_poly).sum())
            own_frontiers = own_frontiers[in_poly]

        # Coverage filter: drop frontiers near already-cleared zones.
        # Replaces "fly to every frontier" — visiting one frontier marks the
        # whole 10m neighborhood as resolved (own + gossiped peer zones).
        zone_dropped_own = 0
        if (completed_zones_xy is not None
                and isinstance(completed_zones_xy, np.ndarray)
                and completed_zones_xy.shape[0] > 0
                and own_frontiers.shape[0] > 0):
            d_to_zone = _nearest_zone_dist(
                own_frontiers[:, :2], completed_zones_xy)
            keep_mask = d_to_zone > ZONE_RADIUS_M
            zone_dropped_own = int((~keep_mask).sum())
            own_frontiers = own_frontiers[keep_mask]

        # Published viewpoints come from own-only clustering.
        own_viewpoints = self._cluster_to_viewpoints(own_frontiers)
        if own_viewpoints.shape[0] > 0:
            viewpoint_publisher.publish(self._create_pointcloud2_msg(own_viewpoints))

        # Candidate pool for picking = own + peer raw frontiers, re-clustered.
        peer_count_summary = {}
        peer_raw_summary = {}
        peer_z_range = {}
        candidate_input = own_frontiers
        if peer_state is not None and peer_state.peer_frontiers:
            peer_chunks = []
            for _name, pf in peer_state.peer_frontiers.items():
                if pf is None or pf.shape[0] == 0:
                    continue
                peer_raw_summary[_name] = int(pf.shape[0])
                peer_z_range[_name] = (float(pf[:, 2].min()), float(pf[:, 2].max()))
                z = pf[:, 2]
                pf_filt = pf[(z >= self.min_altitude) & (z <= self.max_altitude)]
                if search_area_xy is not None and search_area_xy.shape[0] >= 3 and pf_filt.shape[0] > 0:
                    pf_filt = pf_filt[_points_in_polygon(pf_filt[:, :2], search_area_xy)]
                peer_count_summary[_name] = int(pf_filt.shape[0])
                if pf_filt.shape[0] > 0:
                    # Same zone filter on peer frontiers — peers don't
                    # pre-apply it because they don't know our zone state.
                    if (completed_zones_xy is not None
                            and isinstance(completed_zones_xy, np.ndarray)
                            and completed_zones_xy.shape[0] > 0):
                        d_peer = _nearest_zone_dist(
                            pf_filt[:, :2], completed_zones_xy)
                        pf_filt = pf_filt[d_peer > ZONE_RADIUS_M]
                    if pf_filt.shape[0] > 0:
                        peer_chunks.append(pf_filt.astype(own_frontiers.dtype, copy=False))
            if peer_chunks:
                candidate_input = np.vstack([own_frontiers] + peer_chunks)

        merged_count = int(candidate_input.shape[0])

        if debug_logger is not None:
            if peer_count_summary:
                peer_parts = []
                for n, c in peer_count_summary.items():
                    raw = peer_raw_summary.get(n, c)
                    zmin, zmax = peer_z_range.get(n, (0.0, 0.0))
                    peer_parts.append(
                        f'{n}={raw}->{c} z=[{zmin:.1f},{zmax:.1f}]')
                peer_str = ', '.join(peer_parts)
            else:
                peer_str = 'none'
            zones_n = (int(completed_zones_xy.shape[0])
                       if isinstance(completed_zones_xy, np.ndarray)
                       else 0)
            debug_logger.info(
                f'[coord] frontiers: own={own_count} peer=[{peer_str}] '
                f'merged={merged_count} '
                f'(polygon_dropped_own={polygon_dropped_own} '
                f'zone_dropped_own={zone_dropped_own} zones={zones_n})',
                throttle_duration_sec=2.0)

        if candidate_input.shape[0] == 0:
            if debug_logger is not None:
                debug_logger.info('[coord] no frontiers after filtering — staying put',
                                  throttle_duration_sec=2.0)
            return waypoint_locked, target_waypoint, target_waypoint2

        viewpoints = self._cluster_to_viewpoints(candidate_input)
        if viewpoints.shape[0] == 0:
            return waypoint_locked, target_waypoint, target_waypoint2

        # Cluster centroids can drift slightly outside the polygon.
        if search_area_xy is not None and search_area_xy.shape[0] >= 3:
            cent_in = _points_in_polygon(viewpoints[:, :2], search_area_xy)
            viewpoints = viewpoints[cent_in]
            if viewpoints.shape[0] == 0:
                return waypoint_locked, target_waypoint, target_waypoint2

        robot_pos = cur_pose_np
        distances = np.linalg.norm(viewpoints - robot_pos, axis=1)

        if target_waypoint is not None:
            cur_motion_vec = target_waypoint - robot_pos
            cur_motion_vec = cur_motion_vec / (np.linalg.norm(cur_motion_vec) + 1e-6)
            candidate_vecs = viewpoints - robot_pos
            norms = np.linalg.norm(candidate_vecs, axis=1, keepdims=True)
            candidate_vecs = candidate_vecs / (norms + 1e-6)
            cos_sim = candidate_vecs @ cur_motion_vec
            momentum_weight = 5.0
            scores = distances + momentum_weight * (1.0 - cos_sim)
        else:
            scores = distances

        base_scores = scores.copy()
        peer_pen, peer_breakdown = _peer_penalty(viewpoints, peer_state, my_id)
        scores = scores + peer_pen

        # Novelty cost: penalize candidates near already-cleared zones so
        # drones drift to unexplored regions. exp(-d/scale) decay matches the
        # peer-repulsion shape, but it's keyed to the *coverage* state rather
        # than to peer waypoints.
        novelty_pen = np.zeros(viewpoints.shape[0], dtype=np.float64)
        if (completed_zones_xy is not None
                and isinstance(completed_zones_xy, np.ndarray)
                and completed_zones_xy.shape[0] > 0
                and viewpoints.shape[0] > 0):
            d_vp = _nearest_zone_dist(viewpoints[:, :2], completed_zones_xy)
            novelty_pen = NOVELTY_WEIGHT * np.exp(-d_vp / NOVELTY_SCALE_M)
            scores = scores + novelty_pen

        # Strong bias toward viewpoints aligned with the last-seen direction of
        # the currently committed ray-mode target. Activates when raven has
        # committed to a target but ray-mode lost sight (so we fell back to
        # frontiers) — keep wandering in the same direction we last saw it.
        committed_bias = np.zeros(viewpoints.shape[0], dtype=np.float64)
        committed_dir_used = None
        if committed_target_dir is not None:
            cd = np.asarray(committed_target_dir, dtype=np.float64)
            cd_xy = cd[:2]
            cd_norm = np.linalg.norm(cd_xy)
            if cd_norm > 1e-6:
                cd_xy = cd_xy / cd_norm
                cand_xy = (viewpoints[:, :2] - cur_pose_np[:2])
                cand_norms = np.linalg.norm(cand_xy, axis=1, keepdims=True)
                cand_unit = cand_xy / (cand_norms + 1e-6)
                cos_sim_committed = cand_unit @ cd_xy
                # Heavy weight (50): aligned candidates ~free, opposite ~+100.
                committed_weight = 50.0
                committed_bias = committed_weight * (1.0 - cos_sim_committed)
                scores = scores + committed_bias
                committed_dir_used = cd_xy

        top_n = 5
        num_candidates = min(top_n, viewpoints.shape[0])
        if num_candidates == 0:
            return waypoint_locked, target_waypoint, target_waypoint2

        top_indices = np.argsort(scores)[:num_candidates]
        best_idx = top_indices[np.random.randint(0, num_candidates)]
        best_cent = viewpoints[best_idx]

        # Compact summary: own frontier count, peer counts, zones, dropped.
        frontier_table_pub = publisher_dict.get('frontier_table')
        if frontier_table_pub is not None:
            peer_zones_pairs = []
            if peer_state is not None:
                for pname, pz in peer_state.peer_completed_zones.items():
                    n = int(pz.shape[0]) if pz is not None else 0
                    peer_zones_pairs.append((pname, n))
            total_zones = (int(completed_zones_xy.shape[0])
                           if isinstance(completed_zones_xy, np.ndarray)
                           else 0)
            own_zones_n = total_zones - sum(n for _, n in peer_zones_pairs)

            peer_f = (', '.join(f'{n}={c}' for n, c in peer_count_summary.items())
                      if peer_count_summary else 'none')
            peer_z = (', '.join(f'{n}={c}' for n, c in peer_zones_pairs
                                if c > 0)
                      or 'none')
            lines = [
                f'own={own_count} (kept={own_frontiers.shape[0]}, '
                f'polygon_dropped={polygon_dropped_own}, '
                f'zone_dropped={zone_dropped_own})',
                f'peers: {peer_f}',
                f'zones: total={total_zones} (own={own_zones_n}; peers: {peer_z})',
            ]
            from std_msgs.msg import String as _String
            frontier_table_pub.publish(_String(data='\n'.join(lines)))

        if debug_logger is not None:
            known_peers = list(peer_state.peer_waypoints.items()) if peer_state else []
            seen_peers = list(peer_state.peer_last_seen.keys()) if peer_state else []
            unknown_peers = [n for n in seen_peers if n not in dict(known_peers)]
            if known_peers:
                going_str = ', '.join(
                    f'{n}->({wp[0]:.1f},{wp[1]:.1f},{wp[2]:.1f})'
                    for n, wp in known_peers)
                debug_logger.info(
                    f'[coord] aware of peer waypoints: {going_str} — '
                    f'applying repulsion to candidates near them',
                    throttle_duration_sec=2.0)
            elif seen_peers:
                debug_logger.info(
                    f'[coord] peers seen ({", ".join(seen_peers)}) but no waypoints '
                    f'known yet — picking independently this tick',
                    throttle_duration_sec=2.0)
            else:
                debug_logger.info(
                    '[coord] no peers in range — picking independently',
                    throttle_duration_sec=2.0)
            if unknown_peers:
                debug_logger.info(
                    f'[coord] no waypoint yet for peers: {", ".join(unknown_peers)}',
                    throttle_duration_sec=2.0)

            if committed_dir_used is not None:
                debug_logger.info(
                    f'[coord] committed-target bias active: '
                    f'dir_xy=({committed_dir_used[0]:.2f},{committed_dir_used[1]:.2f}) '
                    f'(weight=50)',
                    throttle_duration_sec=2.0)
            lines = []
            for rank, idx in enumerate(top_indices):
                vp = viewpoints[idx]
                marker = '*' if idx == best_idx else ' '
                lines.append(
                    f'    {marker} #{rank}: ({vp[0]:.1f},{vp[1]:.1f},{vp[2]:.1f}) '
                    f'base={base_scores[idx]:.2f} peer_pen={peer_pen[idx]:.2f} '
                    f'cmt_bias={committed_bias[idx]:.2f} '
                    f'novelty={novelty_pen[idx]:.2f} '
                    f'total={scores[idx]:.2f}')
            header = (f'[coord] frontier pick (id={my_id}, '
                      f'{len(viewpoints)} candidates, peers_repelled='
                      f'{len(peer_breakdown)}):')
            debug_logger.info(header + '\n' + '\n'.join(lines),
                              throttle_duration_sec=2.0)
            for b in peer_breakdown:
                tb = ' [tiebreak applied]' if b['applied_tiebreak'] else ''
                debug_logger.info(
                    f'[coord]   peer {b["name"]} (id={b["peer_id"]}){tb}: '
                    f'nearest_d={b["nearest_d"]:.1f}m, '
                    f'on_winner: rep={b["repulsion"][best_idx]:.2f} '
                    f'sur={b["surcharge"][best_idx]:.2f}',
                    throttle_duration_sec=2.0)

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        if not waypoint_locked:
            target_waypoint = best_cent
            direction = target_waypoint - cur_pose_np
            dir_norm = np.linalg.norm(direction)
            if dir_norm > 1e-6:
                direction = direction / dir_norm
                target_waypoint2 = target_waypoint + 2.0 * direction
            else:
                target_waypoint2 = target_waypoint.copy()
            waypoint_locked = True

        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = float(target_waypoint[0])
        target_pose.pose.position.y = float(target_waypoint[1])
        target_pose.pose.position.z = float(target_waypoint[2])
        target_pose.pose.orientation.w = 1.0
        path.poses.append(target_pose)

        target_pose2 = PoseStamped()
        target_pose2.header.stamp = self.get_clock().now().to_msg()
        target_pose2.header.frame_id = 'map'
        target_pose2.pose.position.x = float(target_waypoint2[0])
        target_pose2.pose.position.y = float(target_waypoint2[1])
        target_pose2.pose.position.z = float(target_waypoint2[2])
        target_pose2.pose.orientation.w = 1.0
        path.poses.append(target_pose2)

        path_publisher.publish(path)

        if np.linalg.norm(cur_pose_np - target_waypoint) < 5.0:
            waypoint_locked = False

        return waypoint_locked, target_waypoint, target_waypoint2

    def _cluster_to_viewpoints(self, points: np.ndarray) -> np.ndarray:
        """DBSCAN(eps=2.7, min_samples=3) -> (M,3) centroids in altitude band."""
        if points.shape[0] == 0:
            return np.zeros((0, 3), dtype=points.dtype)
        labels = DBSCAN(eps=2.7, min_samples=3).fit(points).labels_
        out = []
        for l in (l for l in set(labels) if l != -1):
            c = points[labels == l].mean(axis=0)
            if self.min_altitude <= c[2] <= self.max_altitude:
                out.append(c)
        return np.stack(out) if out else np.zeros((0, 3), dtype=points.dtype)

    def _create_pointcloud2_msg(self, xyz):
        if isinstance(xyz, np.ndarray):
            pass
        else:
            xyz = np.array(xyz, dtype=np.float32)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]
        points = [[float(xyz[i, 0]), float(xyz[i, 1]), float(xyz[i, 2])]
                  for i in range(xyz.shape[0])]
        return point_cloud2.create_cloud(header, fields, points)
