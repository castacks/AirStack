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
                  repulsion_weight=50.0,
                  repulsion_scale=15.0,
                  tie_distance=10.0,
                  tie_surcharge_mul=2.0):
    """Returns (penalty(M,), breakdown). Lower penalty = better.

    Soft 2D-xy repulsion = weight * exp(-d / scale) around where each peer is
    (peer_positions) and is heading (peer_waypoints). Higher-id robot pays
    tie_surcharge near a peer waypoint so two robots can't both defer.
    """
    M = viewpoints.shape[0]
    pen = np.zeros(M, dtype=np.float64)
    breakdown = []
    if peer_state is None or M == 0:
        return pen, breakdown
    vp_xy = viewpoints[:, :2]
    for name, wp in peer_state.peer_waypoints.items():
        peer_id = peer_state.peer_ids.get(name)
        if peer_id is None:
            continue
        d = np.linalg.norm(vp_xy - np.asarray(wp, dtype=float)[None, :2], axis=1)
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
    # Repel around where peers currently are, too (not just their waypoints).
    for name, pos in getattr(peer_state, 'peer_positions', {}).items():
        if peer_state.peer_ids.get(name) is None:
            continue
        d = np.linalg.norm(vp_xy - np.asarray(pos, dtype=float)[None, :2], axis=1)
        pen += repulsion_weight * np.exp(-d / repulsion_scale)
    return pen, breakdown


NOVELTY_WEIGHT = 100.0
NOVELTY_NEIGHBORHOOD_CELLS = 5
BLACKLIST_RADIUS_M = 10.0


def _nearest_dist(pts_xy: np.ndarray, centers_xy: np.ndarray) -> np.ndarray:
    """For each point, distance to the nearest center. inf if no centers."""
    if pts_xy.size == 0:
        return np.zeros(0, dtype=np.float64)
    if centers_xy is None or centers_xy.size == 0:
        return np.full(pts_xy.shape[0], np.inf, dtype=np.float64)
    diff = pts_xy[:, None, :] - centers_xy[None, :, :]
    return np.sqrt(np.sum(diff * diff, axis=2).min(axis=1))


def _xy_to_cells(pts_xy: np.ndarray, cell_size_m: float) -> np.ndarray:
    """(N,2) float XY -> (N,2) int64 cell indices."""
    if pts_xy.size == 0:
        return np.zeros((0, 2), dtype=np.int64)
    return np.floor(pts_xy / cell_size_m).astype(np.int64)


def _cells_set_from_xys(zones_xy: np.ndarray, cell_size_m: float) -> set:
    """Quantize gossiped cell-center XYs into a set of (ix, iy) tuples."""
    if zones_xy is None or zones_xy.size == 0:
        return set()
    cells = _xy_to_cells(np.asarray(zones_xy, dtype=np.float64), cell_size_m)
    return set(map(tuple, cells.tolist()))


def _cells_observed_mask(pts_xy: np.ndarray, cells: set,
                         cell_size_m: float) -> np.ndarray:
    """True where the point's cell is in `cells`."""
    if pts_xy.size == 0 or not cells:
        return np.zeros(pts_xy.shape[0], dtype=bool)
    ix = np.floor(pts_xy[:, 0] / cell_size_m).astype(np.int64)
    iy = np.floor(pts_xy[:, 1] / cell_size_m).astype(np.int64)
    return np.fromiter(
        ((int(a), int(b)) in cells for a, b in zip(ix.tolist(), iy.tolist())),
        bool, count=pts_xy.shape[0])


def _neighborhood_density(pts_xy: np.ndarray, cells: set,
                          cell_size_m: float, k: int) -> np.ndarray:
    """Fraction of cells in a (2k+1)x(2k+1) window around each point that
    are in `cells`. 0 in fully novel areas, 1 fully observed."""
    n = pts_xy.shape[0]
    if n == 0 or not cells:
        return np.zeros(n, dtype=np.float64)
    cx = np.floor(pts_xy[:, 0] / cell_size_m).astype(np.int64)
    cy = np.floor(pts_xy[:, 1] / cell_size_m).astype(np.int64)
    side = 2 * k + 1
    total = float(side * side)
    out = np.zeros(n, dtype=np.float64)
    for i in range(n):
        c = 0
        x0, y0 = int(cx[i]), int(cy[i])
        for dx in range(-k, k + 1):
            for dy in range(-k, k + 1):
                if (x0 + dx, y0 + dy) in cells:
                    c += 1
        out[i] = c / total
    return out


class FrontierBehavior:
    # Match semantic_search_task's stuck heuristic so a frontier that triggers
    # an ExplorationTask restart there also counts as one strike here.
    STUCK_DISTANCE_M = 0.3
    STUCK_TIMEOUT_S = 5.0
    MAX_STRIKES = 3
    BLACKLIST_RADIUS_M = BLACKLIST_RADIUS_M

    MOMENTUM_WEIGHT = 20.0
    REVERSE_SURCHARGE = 40.0
    VELOCITY_EMA_ALPHA = 0.3
    MIN_HEADING_SPEED_M = 0.05
    UNLOCK_RADIUS_M = 2.0
    SWAP_IMPROVEMENT_FRAC = 0.35
    TIE_BREAK_FRAC = 0.05
    MIN_LOCK_DURATION_S = 6.0

    INFO_GAIN_WEIGHT = 8.0

    def __init__(self, get_clock, min_altitude=1.5, max_altitude=100.0):
        self.get_clock = get_clock
        self.name = 'Frontier-based'
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude

        # Stuck-detection state for the currently locked target. Reset whenever
        # the drone moves >STUCK_DISTANCE_M or the locked target changes.
        self._last_motion_xy = None
        self._last_motion_time_s = None
        self._tracked_target_xy = None

        self._strikes = []
        self._blacklist_xy = []

        self._prev_pose_xy = None
        self._prev_pose_time_s = None
        self._vel_xy = np.zeros(2, dtype=np.float64)

        self._lock_time_s = None

    def condition_check(self):
        return True

    def clear_blacklist(self) -> int:
        """Wipe strike book + blacklist. Returns count of blacklisted XYs dropped."""
        n = len(self._blacklist_xy)
        self._strikes = []
        self._blacklist_xy = []
        self._last_motion_xy = None
        self._last_motion_time_s = None
        self._tracked_target_xy = None
        return n

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _update_heading(self, cur_pose_np):
        cur_xy = np.asarray(cur_pose_np[:2], dtype=np.float64)
        now = self._now_s()
        if self._prev_pose_xy is None or self._prev_pose_time_s is None:
            self._prev_pose_xy = cur_xy.copy()
            self._prev_pose_time_s = now
            return
        dt = now - self._prev_pose_time_s
        if dt <= 1e-3:
            return
        inst_vel = (cur_xy - self._prev_pose_xy) / dt
        a = self.VELOCITY_EMA_ALPHA
        self._vel_xy = a * inst_vel + (1.0 - a) * self._vel_xy
        self._prev_pose_xy = cur_xy.copy()
        self._prev_pose_time_s = now

    def _score_point(self, pt, robot_pos, heading_xy, peer_state, my_id,
                     completed_cells, cell_size_m, committed_target_dir):
        pt = np.asarray(pt, dtype=np.float64).reshape(1, 3)
        d = float(np.linalg.norm(pt[0] - robot_pos))
        s = d - self.INFO_GAIN_WEIGHT * float(np.log1p(1.0))
        if heading_xy is not None:
            v = pt[0, :2] - robot_pos[:2]
            n = float(np.linalg.norm(v))
            if n > 1e-6:
                cs = float(v @ heading_xy) / n
                s += self.MOMENTUM_WEIGHT * (1.0 - cs)
                s += self.REVERSE_SURCHARGE * max(-cs, 0.0)
        pen, _ = _peer_penalty(pt, peer_state, my_id)
        s += float(pen[0])
        if completed_cells:
            density = _neighborhood_density(
                pt[:, :2], completed_cells, cell_size_m,
                NOVELTY_NEIGHBORHOOD_CELLS)
            s += float(NOVELTY_WEIGHT * density[0])
        if committed_target_dir is not None:
            cd = np.asarray(committed_target_dir, dtype=np.float64)[:2]
            cd_n = float(np.linalg.norm(cd))
            if cd_n > 1e-6:
                cd = cd / cd_n
                v = pt[0, :2] - robot_pos[:2]
                vn = float(np.linalg.norm(v))
                if vn > 1e-6:
                    s += 50.0 * (1.0 - float(v @ cd) / vn)
        return s

    def _heading_xy(self, cur_pose_np, target_waypoint):
        speed = float(np.linalg.norm(self._vel_xy))
        if speed >= self.MIN_HEADING_SPEED_M:
            return self._vel_xy / speed
        if target_waypoint is not None:
            v = np.asarray(target_waypoint[:2], dtype=np.float64) - np.asarray(
                cur_pose_np[:2], dtype=np.float64)
            n = float(np.linalg.norm(v))
            if n > 1e-6:
                return v / n
        return None

    def _blacklist_array(self) -> np.ndarray:
        if not self._blacklist_xy:
            return np.zeros((0, 2), dtype=np.float64)
        return np.stack(self._blacklist_xy)

    def _register_strike(self, target_xy: np.ndarray, debug_logger) -> None:
        """Increment strike count for target_xy; blacklist after MAX_STRIKES."""
        for s in self._strikes:
            if np.linalg.norm(target_xy - s['xy']) <= self.BLACKLIST_RADIUS_M:
                s['count'] += 1
                if debug_logger is not None:
                    debug_logger.warn(
                        f'[stuck] strike {s["count"]}/{self.MAX_STRIKES} at '
                        f'({s["xy"][0]:.1f},{s["xy"][1]:.1f})')
                if s['count'] >= self.MAX_STRIKES:
                    self._blacklist_xy.append(s['xy'].copy())
                    self._strikes = [x for x in self._strikes if x is not s]
                    if debug_logger is not None:
                        debug_logger.warn(
                            f'[stuck] blacklisting frontier near '
                            f'({s["xy"][0]:.1f},{s["xy"][1]:.1f}) — '
                            f'unreachable after {self.MAX_STRIKES} restarts')
                return
        self._strikes.append({'xy': target_xy.copy(), 'count': 1})
        if debug_logger is not None:
            debug_logger.warn(
                f'[stuck] strike 1/{self.MAX_STRIKES} at '
                f'({target_xy[0]:.1f},{target_xy[1]:.1f})')

    def _update_stuck(self, cur_pose_np, target_waypoint, waypoint_locked,
                      debug_logger) -> bool:
        """Return True if a strike just fired and the caller should force-unlock."""
        if (not waypoint_locked) or target_waypoint is None:
            self._last_motion_xy = None
            self._last_motion_time_s = None
            self._tracked_target_xy = None
            return False

        now = self._now_s()
        target_xy = np.asarray(target_waypoint[:2], dtype=np.float64)

        if (self._tracked_target_xy is None
                or np.linalg.norm(target_xy - self._tracked_target_xy)
                > self.BLACKLIST_RADIUS_M):
            self._tracked_target_xy = target_xy.copy()
            self._last_motion_xy = np.asarray(cur_pose_np[:2], dtype=np.float64).copy()
            self._last_motion_time_s = now
            return False

        if self._last_motion_xy is None or self._last_motion_time_s is None:
            self._last_motion_xy = np.asarray(cur_pose_np[:2], dtype=np.float64).copy()
            self._last_motion_time_s = now
            return False

        moved = float(np.linalg.norm(
            np.asarray(cur_pose_np[:2], dtype=np.float64) - self._last_motion_xy))
        if moved > self.STUCK_DISTANCE_M:
            self._last_motion_xy = np.asarray(cur_pose_np[:2], dtype=np.float64).copy()
            self._last_motion_time_s = now
            return False

        if now - self._last_motion_time_s > self.STUCK_TIMEOUT_S:
            self._register_strike(target_xy, debug_logger)
            # Reset so a single stuck episode counts once, not every tick.
            self._last_motion_xy = np.asarray(cur_pose_np[:2], dtype=np.float64).copy()
            self._last_motion_time_s = now
            self._tracked_target_xy = None
            return True

        return False

    def execute(self, frontiers_raw, cur_pose_np, waypoint_locked, target_waypoint,
                target_waypoint2, publisher_dict,
                peer_state=None, my_id=0, search_area_xy=None,
                debug_logger=None,
                committed_target_dir=None,
                committed_target_origin=None,
                completed_zones_xy=None,
                cell_size_m=0.5):
        completed_cells = _cells_set_from_xys(completed_zones_xy, cell_size_m)
        viewpoint_publisher = publisher_dict['viewpoint']
        raw_frontier_publisher = publisher_dict.get('raw_frontiers')
        kept_frontier_publisher = publisher_dict.get('kept_frontiers')
        path_publisher = publisher_dict['path']

        self._update_heading(cur_pose_np)

        if self._update_stuck(cur_pose_np, target_waypoint, waypoint_locked,
                              debug_logger):
            waypoint_locked = False
            self._lock_time_s = None

        if frontiers_raw is None or len(frontiers_raw) == 0:
            return waypoint_locked, target_waypoint, target_waypoint2

        # frontiers_raw: (N,6) RDF xyz + [empty, unobs, occ]. Swap xyz to FLU,
        # pass counts through unchanged.
        has_cnts = frontiers_raw.shape[1] >= 6
        xyz_flu = np.stack([
            frontiers_raw[:, 2],
            -frontiers_raw[:, 0],
            -frontiers_raw[:, 1],
        ], axis=1)
        if has_cnts:
            frontiers_flu = np.concatenate([xyz_flu, frontiers_raw[:, 3:6]], axis=1)
        else:
            frontiers_flu = xyz_flu

        # Gossip own frontiers as xyz-only — peers don't need counts.
        if raw_frontier_publisher is not None and frontiers_flu.shape[0] > 0:
            raw_frontier_publisher.publish(
                self._create_pointcloud2_msg(frontiers_flu[:, :3]))

        alt_mask = (frontiers_flu[:, 2] >= self.min_altitude) & (frontiers_flu[:, 2] <= self.max_altitude)
        own_frontiers = frontiers_flu[alt_mask]
        own_count = int(own_frontiers.shape[0])

        polygon_dropped_own = 0
        if search_area_xy is not None and search_area_xy.shape[0] >= 3:
            in_poly = _points_in_polygon(own_frontiers[:, :2], search_area_xy)
            polygon_dropped_own = int((~in_poly).sum())
            own_frontiers = own_frontiers[in_poly]

        # Drop own frontiers whose cell is already in the observed grid.
        # Rayfronts can keep emitting frontier voxels in regions we've
        # covered (slow retraction, occlusion-edge artifacts), so without
        # this filter the drone re-explores ground it's already seen.
        zone_dropped_own = 0
        if completed_cells and own_frontiers.shape[0] > 0:
            in_observed = _cells_observed_mask(
                own_frontiers[:, :2], completed_cells, cell_size_m)
            zone_dropped_own = int(in_observed.sum())
            own_frontiers = own_frontiers[~in_observed]

        blacklist_xy = self._blacklist_array()
        blacklist_dropped_own = 0
        if blacklist_xy.shape[0] > 0 and own_frontiers.shape[0] > 0:
            d_bl = _nearest_dist(own_frontiers[:, :2], blacklist_xy)
            keep_mask = d_bl > self.BLACKLIST_RADIUS_M
            blacklist_dropped_own = int((~keep_mask).sum())
            own_frontiers = own_frontiers[keep_mask]

        # Publish the kept set (post altitude+polygon+zone filter) so an
        # operator can watch it shrink in real time in Foxglove. Always
        # publish — even an empty cloud — so the topic shows "0 points"
        # rather than going stale when there's nothing left.
        if kept_frontier_publisher is not None:
            kept_frontier_publisher.publish(
                self._create_pointcloud2_msg(own_frontiers[:, :3]))

        own_viewpoints, _own_sizes = self._cluster_to_viewpoints(own_frontiers)
        if own_viewpoints.shape[0] > 0:
            viewpoint_publisher.publish(
                self._create_pointcloud2_msg(own_viewpoints[:, :3]))

        peer_count_summary = {}
        peer_raw_summary = {}
        peer_z_range = {}
        candidate_input = own_frontiers
        n_cols = own_frontiers.shape[1]
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
                    # Cell-membership filter against the merged observed cells.
                    if completed_cells:
                        in_observed = _cells_observed_mask(
                            pf_filt[:, :2], completed_cells, cell_size_m)
                        pf_filt = pf_filt[~in_observed]
                    if pf_filt.shape[0] > 0 and blacklist_xy.shape[0] > 0:
                        d_peer_bl = _nearest_dist(
                            pf_filt[:, :2], blacklist_xy)
                        pf_filt = pf_filt[d_peer_bl > self.BLACKLIST_RADIUS_M]
                    if pf_filt.shape[0] > 0:
                        # Pad peer xyz with zero counts to match own (N,6).
                        if n_cols > pf_filt.shape[1]:
                            pad = np.zeros(
                                (pf_filt.shape[0], n_cols - pf_filt.shape[1]),
                                dtype=pf_filt.dtype)
                            pf_filt = np.concatenate([pf_filt, pad], axis=1)
                        peer_chunks.append(
                            pf_filt.astype(own_frontiers.dtype, copy=False))
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
            zones_n = len(completed_cells)
            debug_logger.info(
                f'[coord] frontiers: own={own_count} peer=[{peer_str}] '
                f'merged={merged_count} '
                f'(polygon_dropped_own={polygon_dropped_own} '
                f'zone_dropped_own={zone_dropped_own} cells={zones_n} '
                f'blacklist_dropped_own={blacklist_dropped_own} '
                f'strikes={len(self._strikes)} '
                f'blacklisted={len(self._blacklist_xy)})',
                throttle_duration_sec=2.0)

        if candidate_input.shape[0] == 0:
            if debug_logger is not None:
                debug_logger.info('[coord] no frontiers after filtering — staying put',
                                  throttle_duration_sec=2.0)
            return waypoint_locked, target_waypoint, target_waypoint2

        viewpoints, cluster_sizes = self._cluster_to_viewpoints(candidate_input)
        if viewpoints.shape[0] == 0:
            return waypoint_locked, target_waypoint, target_waypoint2

        if search_area_xy is not None and search_area_xy.shape[0] >= 3:
            cent_in = _points_in_polygon(viewpoints[:, :2], search_area_xy)
            viewpoints = viewpoints[cent_in]
            cluster_sizes = cluster_sizes[cent_in]
            if viewpoints.shape[0] == 0:
                return waypoint_locked, target_waypoint, target_waypoint2

        if blacklist_xy.shape[0] > 0 and viewpoints.shape[0] > 0:
            d_cent = _nearest_dist(viewpoints[:, :2], blacklist_xy)
            bl_keep = d_cent > self.BLACKLIST_RADIUS_M
            viewpoints = viewpoints[bl_keep]
            cluster_sizes = cluster_sizes[bl_keep]
            if viewpoints.shape[0] == 0:
                return waypoint_locked, target_waypoint, target_waypoint2

        robot_pos = cur_pose_np
        distances = np.linalg.norm(viewpoints - robot_pos, axis=1)

        info_gain_term = self.INFO_GAIN_WEIGHT * np.log1p(
            cluster_sizes.astype(np.float64))

        heading_xy = self._heading_xy(cur_pose_np, target_waypoint)
        scores = distances.copy() - info_gain_term
        if heading_xy is not None:
            cand_xy = viewpoints[:, :2] - robot_pos[:2]
            cand_norms = np.linalg.norm(cand_xy, axis=1, keepdims=True)
            cand_unit = cand_xy / (cand_norms + 1e-6)
            cos_sim = cand_unit @ heading_xy
            scores = scores + self.MOMENTUM_WEIGHT * (1.0 - cos_sim)
            scores = scores + self.REVERSE_SURCHARGE * np.clip(-cos_sim, 0.0, 1.0)

        base_scores = scores.copy()
        peer_pen, peer_breakdown = _peer_penalty(viewpoints, peer_state, my_id)
        scores = scores + peer_pen

        # Novelty: fraction of cells in a (2k+1)^2 window around each viewpoint
        # that are already observed.
        novelty_pen = np.zeros(viewpoints.shape[0], dtype=np.float64)
        if completed_cells and viewpoints.shape[0] > 0:
            density = _neighborhood_density(
                viewpoints[:, :2], completed_cells, cell_size_m,
                NOVELTY_NEIGHBORHOOD_CELLS)
            novelty_pen = NOVELTY_WEIGHT * density
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
        best_score = float(scores[top_indices[0]])
        gap = self.TIE_BREAK_FRAC * (abs(best_score) + 1e-6)
        tied = [i for i in top_indices if abs(scores[i] - best_score) <= gap]
        best_idx = (tied[np.random.randint(0, len(tied))]
                    if len(tied) > 1 else int(top_indices[0]))
        best_cent = viewpoints[best_idx]

        # Compact summary: own frontier count, peer counts, zones, dropped.
        frontier_table_pub = publisher_dict.get('frontier_table')
        if frontier_table_pub is not None:
            peer_zones_pairs = []
            if peer_state is not None:
                for pname, pz in peer_state.peer_completed_zones.items():
                    n = int(pz.shape[0]) if pz is not None else 0
                    peer_zones_pairs.append((pname, n))
            total_zones = len(completed_cells)
            own_zones_n = total_zones - sum(n for _, n in peer_zones_pairs)

            peer_f = (', '.join(f'{n}={c}' for n, c in peer_count_summary.items())
                      if peer_count_summary else 'none')
            peer_z = (', '.join(f'{n}={c}' for n, c in peer_zones_pairs
                                if c > 0)
                      or 'none')
            strike_summary = (
                ', '.join(f'({s["xy"][0]:.0f},{s["xy"][1]:.0f})={s["count"]}'
                          for s in self._strikes)
                or 'none')
            lines = [
                f'own={own_count} (kept={own_frontiers.shape[0]}, '
                f'polygon_dropped={polygon_dropped_own}, '
                f'zone_dropped={zone_dropped_own}, '
                f'blacklist_dropped={blacklist_dropped_own})',
                f'peers: {peer_f}',
                f'zones: total={total_zones} (own={own_zones_n}; peers: {peer_z})',
                f'strikes: {strike_summary}  blacklisted={len(self._blacklist_xy)}',
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
                    f'info_gain={int(cluster_sizes[idx])} '
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

        swap = not waypoint_locked or target_waypoint is None
        now_s = self._now_s()
        locked_for_s = (now_s - self._lock_time_s
                        if self._lock_time_s is not None else float('inf'))

        if not swap:
            cur_score = self._score_point(
                target_waypoint, robot_pos, heading_xy, peer_state, my_id,
                completed_cells, cell_size_m, committed_target_dir)
            margin = self.SWAP_IMPROVEMENT_FRAC * (abs(cur_score) + 1e-6)
            beats = best_score < cur_score - margin
            if beats and locked_for_s >= self.MIN_LOCK_DURATION_S:
                swap = True
                if debug_logger is not None:
                    debug_logger.info(
                        f'[coord] swap: best={best_score:.2f} cur={cur_score:.2f} '
                        f'margin={margin:.2f} locked_for={locked_for_s:.1f}s',
                        throttle_duration_sec=2.0)
            elif beats and debug_logger is not None:
                debug_logger.info(
                    f'[coord] hold: best beats cur by {cur_score - best_score:.2f} '
                    f'but locked_for={locked_for_s:.1f}s < '
                    f'{self.MIN_LOCK_DURATION_S:.1f}s',
                    throttle_duration_sec=2.0)

        if swap:
            target_waypoint = best_cent
            direction = target_waypoint - cur_pose_np
            dir_norm = np.linalg.norm(direction)
            if dir_norm > 1e-6:
                direction = direction / dir_norm
                target_waypoint2 = target_waypoint + 2.0 * direction
            else:
                target_waypoint2 = target_waypoint.copy()
            waypoint_locked = True
            self._lock_time_s = now_s

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

        if np.linalg.norm(cur_pose_np - target_waypoint) < self.UNLOCK_RADIUS_M:
            waypoint_locked = False
            self._lock_time_s = None

        return waypoint_locked, target_waypoint, target_waypoint2

    def _cluster_to_viewpoints(self, points: np.ndarray):
        """DBSCAN over XYZ. Returns ((M,3) centroids, (M,) info_gain).
        info_gain = summed unobserved_cnt when col 4 is present, else cluster
        point count."""
        empty = (np.zeros((0, 3), dtype=points.dtype),
                 np.zeros((0,), dtype=np.float64))
        if points.shape[0] == 0:
            return empty
        xyz = points[:, :3]
        unobs = points[:, 4] if points.shape[1] >= 6 else None
        labels = DBSCAN(eps=2.7, min_samples=3).fit(xyz).labels_
        centroids = []
        gains = []
        for l in (l for l in set(labels) if l != -1):
            mask = labels == l
            c = xyz[mask].mean(axis=0)
            if self.min_altitude <= c[2] <= self.max_altitude:
                centroids.append(c)
                if unobs is not None:
                    g = float(unobs[mask].sum())
                    if g <= 0.0:
                        g = float(mask.sum())
                    gains.append(g)
                else:
                    gains.append(float(mask.sum()))
        if not centroids:
            return empty
        return np.stack(centroids), np.asarray(gains, dtype=np.float64)

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
