"""Co-NavGPT baseline behavior — the "VLM-Assign" arm.

Co-NavGPT (Yu et al., arXiv:2310.07937) shows a VLM the team's shared map with
numbered frontier regions and numbered robots and asks it, once per round for
the WHOLE TEAM, which robot should take which region. This is that method
adapted per COA-docs/multi_robot_baselines.md sec.9: multi-target, no
terminate-on-first-find, non-co-located starts, time-integrated scoring. The
published method is single-target with co-located starts, so this is the
"VLM-Assign" arm and not a reproduction.

Only frontier *selection* is replaced. Extraction, clustering, the polygon /
altitude / observed-cell filters, the unreachable-frontier blacklist and the
stuck watchdog are all FrontierBehavior's, inherited unchanged, so a CoNavGPT
drone flies exactly like a frontier drone once a region has been chosen.

Region numbering is deliberately robot-independent. The VLM answers with bare
region ids and the assignment carries no coordinates, so a non-leader has to be
able to turn id 3 into the same physical place the leader offered as id 3.
Frontiers are gossiped, and every ranking term (info gain, range from the team
centroid, bearing from it) is invariant under the translation that separates
two local frames — so every robot that has heard the same gossip numbers the
same list, which is the assumption raven's consensus assigner already runs on.
An assigner that chooses to echo the request's `regions` back is honoured
first and skips the question entirely.

A robot with no fresh valid assignment (out of comms, leader dead, VLM refused)
flies to its nearest viable frontier and marks the round `fallback`. That
degradation is a measured result, not a bug.
"""

import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from raven_nav.behaviors.frontier_behavior import (
    FrontierBehavior, _cells_observed_mask, _cells_set_from_xys,
    _nearest_dist, _points_in_polygon)


class CoNavGPTBehavior(FrontierBehavior):
    # A region survives while some current candidate viewpoint is this close to
    # the point the round assigned. Cluster centroids drift as the map fills,
    # so region identity cannot be an equality test. Doubles as the "is this a
    # different pursuit" radius for the intent log.
    REGION_MATCH_M = 12.0

    def __init__(self, get_clock, min_altitude=1.5, max_altitude=100.0,
                 leader_id=1, max_regions=12, round_period_s=30.0,
                 assignment_ttl_s=90.0, replan_on_reach_m=8.0):
        super().__init__(get_clock, min_altitude=min_altitude,
                         max_altitude=max_altitude)
        self.name = 'CoNavGPT-based'
        self.leader_id = int(leader_id)
        self.max_regions = max(1, int(max_regions))
        self.round_period_s = float(round_period_s)
        self.assignment_ttl_s = float(assignment_ttl_s)
        self.replan_on_reach_m = float(replan_on_reach_m)

        # Numbered candidate regions as of this tick:
        # [{'id', 'pos'(3), 'info_gain'}]. Recomputed every tick; the id space
        # is what an incoming assignment is resolved against.
        self.regions: list = []
        # Rounds this robot has opened (leader only — a non-leader's round
        # number comes from the assignment it consumes).
        self.round_id: int = 0
        self._round_opened_s: 'float | None' = None
        self._round_reason: str = ''
        # Request awaiting publication, in the LOCAL 'map' frame. The node
        # lifts it into global ENU before it goes on the wire.
        self.pending_request: 'dict | None' = None
        # The assignment this robot is acting on:
        # {'round', 'region_id', 'pos'(3), 'ts', 'model', 'fallback'}.
        self.assignment: 'dict | None' = None
        # Last full robot->region map seen, so the leader can report each
        # robot's current_region in the next request without extra gossip.
        self.last_assignments: dict = {}
        # (round, ts) of the payload already applied — gossip redelivers the
        # same assignment every tick.
        self._applied_key: 'tuple | None' = None
        # Pursue/release transitions in the local 'map' frame, drained by the
        # node into _intent_events. Same shape VLFMBehavior uses.
        self.intent_log: list = []
        self._pursued: 'dict | None' = None
        # Human-readable last-round summary for the round_table topic.
        self.round_table_text: str = ''
        # Whether this tick's goal came from the VLM or from the fallback.
        self.used_fallback: bool = True
        self.n_rounds_assigned: int = 0
        self.n_rounds_fallback: int = 0

    def condition_check(self):
        return True

    # ── intent record ────────────────────────────────────────────────────────

    def _note_intent(self, kind, label, pos, via, why=None) -> None:
        rec = {'kind': kind, 'label': str(label or ''), 'via': via,
               'pos': [float(v) for v in np.asarray(pos, dtype=float)[:3]],
               't': self._now_s()}
        if why is not None:
            rec['why'] = why
        self.intent_log.append(rec)

    def _release(self, why: str) -> None:
        """Close the open pursuit, if any. Idempotent."""
        if self._pursued is None:
            return
        p = self._pursued
        self._note_intent('release', p['label'], p['pos'], via=p['via'],
                          why=why)
        self._pursued = None

    def _pursue(self, goal, label, via, region_id) -> None:
        """Open a pursuit unless the same one is already open."""
        g = np.asarray(goal, dtype=float)
        p = self._pursued
        if (p is not None and p['via'] == via and p['region_id'] == region_id
                and float(np.linalg.norm(g[:2] - p['pos'][:2]))
                <= self.REGION_MATCH_M):
            return
        self._release('superseded')
        self._pursued = {'pos': g.copy(), 'label': str(label or ''),
                         'via': via, 'region_id': region_id}
        self._note_intent('pursue', label, g, via=via)

    # ── assignment ───────────────────────────────────────────────────────────

    def consume_assignment(self, payload: dict, my_id: int,
                           world_to_local=None, debug_logger=None) -> bool:
        """Apply one assignment payload. True when this robot got a region.

        `world_to_local` converts a global-ENU point back into this robot's
        local frame; it is used only for the optional `regions` echo, which is
        the one part of an assignment that can carry coordinates.
        """
        if not isinstance(payload, dict):
            return False
        assigns = payload.get('assignments')
        if not isinstance(assigns, dict):
            return False
        key = (payload.get('round'), payload.get('ts'))
        if key == self._applied_key:
            return self.assignment is not None
        self._applied_key = key
        self.last_assignments = {str(k): v for k, v in assigns.items()}

        raw = assigns.get(str(int(my_id)))
        if raw is None:
            self._drop_assignment('no entry for this robot', debug_logger)
            return False
        try:
            rid = int(raw)
        except (TypeError, ValueError):
            self._drop_assignment(f'non-numeric region id {raw!r}',
                                  debug_logger)
            return False

        pos = self._region_pos(rid, payload.get('regions'), world_to_local)
        if pos is None:
            # The VLM invented a region that was never offered, or the region
            # has since been cleared. Nearest-frontier, and say so.
            self._drop_assignment(f'region {rid} not in the candidate set',
                                  debug_logger)
            return False

        # TTL is measured on OUR clock from the moment of consumption: the
        # assigner stamps wall time while raven may be on sim time, so its `ts`
        # is not comparable with ours (same trap peer_state documents).
        self.assignment = {
            'round': payload.get('round'),
            'region_id': rid,
            'pos': np.asarray(pos, dtype=float),
            'ts': self._now_s(),
            'model': str(payload.get('model', '')),
            'fallback': bool(payload.get('fallback', False)),
        }
        self.n_rounds_assigned += 1
        if debug_logger is not None:
            debug_logger.info(
                f'[conavgpt] round {payload.get("round")}: assigned region '
                f'{rid} @ ({pos[0]:.0f},{pos[1]:.0f}) '
                f'model={payload.get("model", "?")} '
                f'latency={payload.get("latency_s", -1)}')
        return True

    def _drop_assignment(self, why: str, debug_logger=None) -> None:
        self.assignment = None
        self._release(why)
        self.n_rounds_fallback += 1
        if debug_logger is not None:
            debug_logger.warn(f'[conavgpt] falling back to nearest frontier — '
                              f'{why}')

    def _region_pos(self, rid: int, echo, world_to_local):
        """Resolve a region id to a local-frame point.

        The assignment's optional `regions` echo is authoritative when present
        (it is the leader's own list, so no cross-robot numbering agreement is
        needed); otherwise the id indexes this robot's current numbering.
        """
        if isinstance(echo, list):
            for r in echo:
                try:
                    if int(r.get('id')) != rid:
                        continue
                    p = np.array([float(r['x']), float(r['y']),
                                  float(r.get('z', 0.0))])
                except (TypeError, ValueError, KeyError, AttributeError):
                    continue
                return world_to_local(p) if world_to_local is not None else p
            return None
        for r in self.regions:
            if r['id'] == rid:
                return r['pos'].copy()
        return None

    # ── round scheduling ─────────────────────────────────────────────────────

    def _round_due(self, cur_pose_np, debug_logger=None) -> 'str | None':
        """Why a new round is owed this tick, or None.

        The three assignment-driven triggers retire the assignment as they
        fire, so none of them can re-fire every tick while the answer is in
        flight — the periodic trigger alone bounds the request rate after that.
        """
        now = self._now_s()
        if self._round_opened_s is None:
            return 'first tick'
        a = self.assignment
        if a is not None:
            if now - a['ts'] >= self.assignment_ttl_s:
                self._drop_assignment(
                    f'assignment older than {self.assignment_ttl_s:.0f}s',
                    debug_logger)
                return 'assignment stale'
            if cur_pose_np is not None and float(np.linalg.norm(
                    np.asarray(cur_pose_np, float)[:2] - a['pos'][:2])) \
                    <= self.replan_on_reach_m:
                self.assignment = None
                self._release('reached')
                return 'region reached'
            if not self._region_alive(a['pos']):
                self.assignment = None
                self._release('region gone')
                return 'region gone'
        if now - self._round_opened_s >= self.round_period_s:
            return 'period elapsed'
        return None

    def _region_alive(self, pos) -> bool:
        if not self.regions:
            # No candidates at all is a starved map, not a vanished region;
            # keeping the assignment lets the drone finish flying to it.
            return True
        d = min(float(np.linalg.norm(np.asarray(pos, float)[:2] - r['pos'][:2]))
                for r in self.regions)
        return d <= self.REGION_MATCH_M

    # ── candidate regions ────────────────────────────────────────────────────

    def _candidates(self, frontiers_raw, publisher_dict, peer_state,
                    search_area_xy, completed_cells, cell_size_m):
        """FrontierBehavior's own+peer filter chain, up to the viewpoints.

        Returns ((M,3) viewpoints, (M,) info_gain). Frontiers are still
        published raw so peers keep merging them via gossip.
        """
        empty = (np.zeros((0, 3), dtype=np.float64),
                 np.zeros((0,), dtype=np.float64))
        if frontiers_raw is None or len(frontiers_raw) == 0:
            return empty
        # frontiers_raw: (N,6) RDF xyz + [empty, unobs, occ] -> FLU xyz + counts.
        has_cnts = frontiers_raw.shape[1] >= 6
        xyz_flu = np.stack([frontiers_raw[:, 2], -frontiers_raw[:, 0],
                            -frontiers_raw[:, 1]], axis=1)
        frontiers_flu = (np.concatenate([xyz_flu, frontiers_raw[:, 3:6]], axis=1)
                         if has_cnts else xyz_flu)

        raw_pub = publisher_dict.get('raw_frontiers')
        if raw_pub is not None and frontiers_flu.shape[0] > 0:
            raw_pub.publish(self._create_pointcloud2_msg(frontiers_flu[:, :3]))

        z = frontiers_flu[:, 2]
        own = frontiers_flu[(z >= self.min_altitude) & (z <= self.max_altitude)]
        if search_area_xy is not None and search_area_xy.shape[0] >= 3:
            own = own[_points_in_polygon(own[:, :2], search_area_xy)]
        if completed_cells and own.shape[0] > 0:
            own = own[~_cells_observed_mask(own[:, :2], completed_cells,
                                            cell_size_m)]
        blacklist_xy = self._blacklist_array()
        if blacklist_xy.shape[0] > 0 and own.shape[0] > 0:
            keep = _nearest_dist(own[:, :2], blacklist_xy) > self.BLACKLIST_RADIUS_M
            if keep.any():
                own = own[keep]

        kept_pub = publisher_dict.get('kept_frontiers')
        if kept_pub is not None:
            kept_pub.publish(self._create_pointcloud2_msg(own[:, :3]))

        merged = own
        n_cols = own.shape[1]
        if peer_state is not None and peer_state.peer_frontiers:
            chunks = []
            for pf in peer_state.peer_frontiers.values():
                if pf is None or pf.shape[0] == 0:
                    continue
                pz = pf[:, 2]
                f = pf[(pz >= self.min_altitude) & (pz <= self.max_altitude)]
                if search_area_xy is not None and search_area_xy.shape[0] >= 3 \
                        and f.shape[0] > 0:
                    f = f[_points_in_polygon(f[:, :2], search_area_xy)]
                if completed_cells and f.shape[0] > 0:
                    f = f[~_cells_observed_mask(f[:, :2], completed_cells,
                                                cell_size_m)]
                if f.shape[0] > 0 and blacklist_xy.shape[0] > 0:
                    keep = _nearest_dist(f[:, :2], blacklist_xy) \
                        > self.BLACKLIST_RADIUS_M
                    if keep.any():
                        f = f[keep]
                if f.shape[0] == 0:
                    continue
                if n_cols > f.shape[1]:
                    f = np.concatenate(
                        [f, np.zeros((f.shape[0], n_cols - f.shape[1]),
                                     dtype=f.dtype)], axis=1)
                chunks.append(f.astype(own.dtype, copy=False))
            if chunks:
                merged = np.vstack([own] + chunks) if own.shape[0] else \
                    np.vstack(chunks)
        if merged.shape[0] == 0:
            return empty

        viewpoints, gains = self._cluster_to_viewpoints(merged)
        if viewpoints.shape[0] == 0:
            return empty
        if search_area_xy is not None and search_area_xy.shape[0] >= 3:
            inside = _points_in_polygon(viewpoints[:, :2], search_area_xy)
            viewpoints, gains = viewpoints[inside], gains[inside]
        if viewpoints.shape[0] and blacklist_xy.shape[0] > 0:
            keep = _nearest_dist(viewpoints[:, :2], blacklist_xy) \
                > self.BLACKLIST_RADIUS_M
            if keep.any():
                viewpoints, gains = viewpoints[keep], gains[keep]
        vp_pub = publisher_dict.get('viewpoint')
        if vp_pub is not None and viewpoints.shape[0] > 0:
            vp_pub.publish(self._create_pointcloud2_msg(viewpoints[:, :3]))
        return viewpoints, gains

    def _team_centroid_xy(self, cur_pose_np, peer_state, fresh_peers):
        """Reference point for the region ordering.

        Distances are frame-invariant and the centroid is a physical point, so
        every robot that has heard the same peers derives the same ordering —
        which is what makes a bare region id mean the same thing fleet-wide.
        """
        pts = [np.asarray(cur_pose_np, dtype=float)[:2]]
        if peer_state is not None:
            for name, p in peer_state.peer_positions.items():
                if fresh_peers is None or name in fresh_peers:
                    pts.append(np.asarray(p, dtype=float)[:2])
        return np.mean(np.stack(pts), axis=0)

    def _number_regions(self, viewpoints, gains, ref_xy) -> list:
        """Rank by info_gain, then proximity to the team centroid, then bearing
        from it.

        Every term is invariant under the translation that separates two
        robots' local frames, so the ordering is too. The bearing exists only
        to make the ordering total: two regions of equal gain at equal range
        would otherwise number by whatever order they came out of DBSCAN, and
        the VLM's answer would land somewhere else on the other robot.
        """
        def _key(i):
            d = viewpoints[i][:2] - ref_xy
            return (-float(gains[i]), float(np.hypot(d[0], d[1])),
                    float(np.arctan2(d[1], d[0])))

        idx = sorted(range(viewpoints.shape[0]), key=_key)
        return [{'id': n, 'pos': viewpoints[i].astype(float),
                 'info_gain': float(gains[i])}
                for n, i in enumerate(idx[:self.max_regions])]

    # ── request ──────────────────────────────────────────────────────────────

    def _build_request(self, cur_pose_np, peer_state, my_id, fresh_peers,
                       query, search_area_xy, found_targets) -> dict:
        """The assign_request, in the LOCAL 'map' frame.

        The node rewrites every coordinate into global ENU before publishing;
        keeping the build local means this method never needs the GPS anchor.
        """
        robots = [{'id': int(my_id),
                   'x': float(cur_pose_np[0]), 'y': float(cur_pose_np[1]),
                   'z': float(cur_pose_np[2]), 'fresh': True,
                   'current_region': self._current_region_of(my_id)}]
        if peer_state is not None:
            for name, p in peer_state.peer_positions.items():
                pid = peer_state.peer_ids.get(name)
                if pid is None or int(pid) == int(my_id):
                    continue
                p = np.asarray(p, dtype=float)
                robots.append({
                    'id': int(pid), 'x': float(p[0]), 'y': float(p[1]),
                    'z': float(p[2]),
                    'fresh': bool(fresh_peers is None or name in fresh_peers),
                    'current_region': self._current_region_of(pid)})
        robots.sort(key=lambda r: r['id'])

        regions = []
        for r in self.regions:
            entry = {'id': r['id'], 'x': float(r['pos'][0]),
                     'y': float(r['pos'][1]), 'z': float(r['pos'][2]),
                     'info_gain': float(r['info_gain']),
                     'dist_by_robot': {}}
            for rb in robots:
                entry['dist_by_robot'][str(rb['id'])] = round(float(
                    np.linalg.norm(r['pos'][:2]
                                   - np.array([rb['x'], rb['y']]))), 2)
            regions.append(entry)

        found = []
        for label, center in (found_targets or []):
            c = np.asarray(center, dtype=float)
            found.append({'label': str(label), 'x': float(c[0]),
                          'y': float(c[1])})

        return {
            'round': self.round_id,
            'ts': self._now_s(),
            'leader': self.leader_id,
            'query': str(query or ''),
            # Rewritten to 'global_enu' by the node once the xy are lifted.
            'frame': 'local_map',
            'search_area': ([[float(p[0]), float(p[1])] for p in search_area_xy]
                            if search_area_xy is not None
                            and len(search_area_xy) >= 3 else []),
            'robots': robots,
            'regions': regions,
            'found': found,
        }

    def _current_region_of(self, robot_id) -> int:
        raw = self.last_assignments.get(str(int(robot_id)))
        try:
            return int(raw)
        except (TypeError, ValueError):
            return -1

    # ── execute ──────────────────────────────────────────────────────────────

    def execute(self, frontiers_raw, cur_pose_np, waypoint_locked,
                target_waypoint, target_waypoint2, publisher_dict,
                peer_state=None, my_id=0, search_area_xy=None,
                debug_logger=None, completed_zones_xy=None, cell_size_m=0.5,
                is_leader=False, query='', fresh_peers=None,
                found_targets=None, **_unused):
        if cur_pose_np is None:
            return waypoint_locked, target_waypoint, target_waypoint2

        self._update_heading(cur_pose_np)
        if self._update_stuck(cur_pose_np, target_waypoint, waypoint_locked,
                              debug_logger):
            waypoint_locked = False
            self._lock_time_s = None

        completed_cells = _cells_set_from_xys(completed_zones_xy, cell_size_m)
        viewpoints, gains = self._candidates(
            frontiers_raw, publisher_dict, peer_state, search_area_xy,
            completed_cells, cell_size_m)
        ref_xy = self._team_centroid_xy(cur_pose_np, peer_state, fresh_peers)
        self.regions = self._number_regions(viewpoints, gains, ref_xy)

        reason = self._round_due(cur_pose_np, debug_logger)
        if reason is not None:
            self._round_opened_s = self._now_s()
            self._round_reason = reason
            if is_leader:
                self.round_id += 1
                self.pending_request = self._build_request(
                    cur_pose_np, peer_state, my_id, fresh_peers, query,
                    search_area_xy, found_targets)

        goal, via, region_id = self._select_goal(cur_pose_np, viewpoints,
                                                 debug_logger)
        self.used_fallback = via == 'fallback'
        self._publish_round_table(cur_pose_np, goal, via, region_id)
        if goal is None:
            return waypoint_locked, target_waypoint, target_waypoint2
        self._pursue(goal, query, via, region_id)

        return self._steer(goal, cur_pose_np, waypoint_locked, target_waypoint,
                           target_waypoint2, publisher_dict, debug_logger, via,
                           region_id)

    def _select_goal(self, cur_pose_np, viewpoints, debug_logger=None):
        """(goal, via, region_id). Assigned region if one is live, else the
        nearest viable frontier."""
        a = self.assignment
        if a is not None:
            if self._now_s() - a['ts'] < self.assignment_ttl_s:
                # Track the region's current centroid rather than the point the
                # round froze: the cluster drifts outward as the map fills, and
                # flying to a stale centroid parks the drone behind the front.
                snapped = self._snap(a['pos'], viewpoints)
                return snapped, 'assign', a['region_id']
            self._drop_assignment('assignment expired', debug_logger)
        if viewpoints.shape[0] == 0:
            return None, 'fallback', -1
        d = np.linalg.norm(viewpoints - np.asarray(cur_pose_np, float), axis=1)
        return viewpoints[int(np.argmin(d))].astype(float), 'fallback', -1

    def _snap(self, pos, viewpoints):
        if viewpoints.shape[0] == 0:
            return np.asarray(pos, dtype=float)
        d = np.linalg.norm(viewpoints[:, :2] - np.asarray(pos, float)[:2],
                           axis=1)
        i = int(np.argmin(d))
        if float(d[i]) <= self.REGION_MATCH_M:
            return viewpoints[i].astype(float)
        return np.asarray(pos, dtype=float)

    def _steer(self, goal, cur_pose_np, waypoint_locked, target_waypoint,
               target_waypoint2, publisher_dict, debug_logger, via, region_id):
        """FrontierBehavior's lock/swap/unlock, with the VLM's choice standing
        in for the score comparison: the target only changes on a round
        boundary, so there is nothing to re-score mid-lock."""
        now_s = self._now_s()
        locked_for_s = (now_s - self._lock_time_s
                        if self._lock_time_s is not None else float('inf'))
        moved = (target_waypoint is None or float(np.linalg.norm(
            np.asarray(goal, float)[:2]
            - np.asarray(target_waypoint, float)[:2])) > self.REGION_MATCH_M)
        swap = (not waypoint_locked or target_waypoint is None
                or (moved and locked_for_s >= self.MIN_LOCK_DURATION_S))

        if swap:
            target_waypoint = np.asarray(goal, dtype=float)
            direction = target_waypoint - np.asarray(cur_pose_np, float)
            n = float(np.linalg.norm(direction))
            target_waypoint2 = (target_waypoint + 2.0 * (direction / n)
                                if n > 1e-6 else target_waypoint.copy())
            waypoint_locked = True
            self._lock_time_s = now_s

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for pt in (target_waypoint, target_waypoint2):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.position.z = float(pt[2])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        publisher_dict['path'].publish(path)

        if debug_logger is not None:
            debug_logger.info(
                f'[conavgpt] round {self.round_id} via={via} '
                f'region={region_id} of {len(self.regions)} -> '
                f'({target_waypoint[0]:.0f},{target_waypoint[1]:.0f},'
                f'{target_waypoint[2]:.0f}) t={now_s:.2f}',
                throttle_duration_sec=2.0)

        if float(np.linalg.norm(np.asarray(cur_pose_np, float)
                                - target_waypoint)) < self.UNLOCK_RADIUS_M:
            waypoint_locked = False
            self._lock_time_s = None
        return waypoint_locked, target_waypoint, target_waypoint2

    def _publish_round_table(self, cur_pose_np, goal, via, region_id) -> None:
        a = self.assignment
        lines = [
            f'round {self.round_id} ({self._round_reason or "-"})',
            f'regions: {len(self.regions)} (cap {self.max_regions})',
            (f'assigned: region {region_id} @ '
             f'({goal[0]:.0f},{goal[1]:.0f}) model={a.get("model") or "?"}'
             if via == 'assign' and goal is not None and a is not None
             else 'assigned: none — nearest-frontier fallback'),
            f'rounds: assigned={self.n_rounds_assigned} '
            f'fallback={self.n_rounds_fallback}',
        ]
        for r in self.regions:
            d = float(np.linalg.norm(
                r['pos'][:2] - np.asarray(cur_pose_np, float)[:2]))
            mark = '*' if r['id'] == region_id else ' '
            lines.append(f'  {mark} #{r["id"]}: '
                         f'({r["pos"][0]:.0f},{r["pos"][1]:.0f}) '
                         f'gain={r["info_gain"]:.0f} d={d:.0f}m')
        self.round_table_text = '\n'.join(lines)
