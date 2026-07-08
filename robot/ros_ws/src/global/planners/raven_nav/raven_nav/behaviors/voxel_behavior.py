import time
from dataclasses import dataclass

import numpy as np
import scipy.ndimage
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from raven_nav.discoveries import same_instance_xy
from raven_nav.ray_targets import ray_aabb_hits
from raven_nav.track_confirmation import TemporalConfirmer

# Relaxed fallback: cluster center is accepted if it sits within this many
# metres of the committed ray's line, AND is in front of the drone. Tolerates
# minor mis-alignment between the auctioned ray triangulation and where the
# voxel cluster actually consolidates (sensor noise, mapper smoothing).
RAY_TO_CLUSTER_RELAXED_M = 8.0

# Surface gap (m) under which two AABBs across ticks are the same cluster.
TRACK_ASSOC_MARGIN_M = 2.0

# EMA weight for the track centroid + score (size is not EMA'd, see _merge_obs).
TRACK_MERGE_ALPHA = 0.3

# Sightings at which the persistence factor of confidence saturates to 1.0.
PERSISTENCE_FULL_HITS = 10

# Instance separation: split a blob at score peaks; peaks closer than this merge.
SPLIT_PEAK_SEP_M = 5.0
SPLIT_SMOOTH_SIGMA_VOX = 1.0

# Containment suppression: drop a box this fraction (or more) of whose volume is
# inside a larger box — keeps only the bigger one of a nested pair.
CONTAIN_SUPPRESS_FRAC = 0.7

# Reachability: a waypoint within this of any voxel is blocked. Only when the
# locked approach waypoint becomes blocked (the cluster grew into it) is it
# re-stood-off in STANDOFF_STEP_M steps up to STANDOFF_MAX_M; if nothing is
# clear the object envelops the approach — it's already observed, mark visited.
WAYPOINT_CLEARANCE_M = 1.5
STANDOFF_STEP_M = 0.5
STANDOFF_MAX_M = 6.0

# Visited geometry — ONE definition, shared with the auction AABBs in
# raven_nav_node. Arrival is surface distance; same-INSTANCE is overlap-based
# (discoveries.same_instance_xy) — a gap radius cascades visits across
# adjacent buildings when boxes bloat.
VISIT_REACH_M = 3.0   # drone within this of a target AABB *surface* (0 if inside) => reached
VISIT_MATCH_M = 5.0   # point-feature (ray-lead) same-instance radius; boxes use overlap


def aabb_surface_dist(ca, sa, cb, sb) -> float:
    """Surface-to-surface gap between two axis-aligned boxes (0 if they overlap).
    Pass sa or sb as zeros to treat that argument as a point."""
    ca, sa, cb, sb = (np.asarray(v, dtype=float) for v in (ca, sa, cb, sb))
    gap = np.maximum(np.abs(ca[:3] - cb[:3]) - (sa[:3] + sb[:3]) / 2.0, 0.0)
    return float(np.linalg.norm(gap))


def waypoint_arrival_counts(cur_pose, wp2, c, s) -> bool:
    """Near-waypoint arrival only counts for the cluster the waypoint serves:
    standoffs sit within STANDOFF_MAX_M of their box, so a top-ranked cluster
    farther than that from the waypoint is a different target (re-ranked after
    a visit) and must not be marked visited from here."""
    return (wp2 is not None
            and float(np.linalg.norm(np.asarray(cur_pose, dtype=float)
                                     - np.asarray(wp2, dtype=float)))
            < VISIT_REACH_M
            and aabb_surface_dist(wp2, np.zeros(3), c, s)
            <= STANDOFF_MAX_M + 2.0)


def _contained_frac(inner_bbox, outer_bbox) -> float:
    """Fraction of inner's AABB volume that lies within outer's AABB."""
    ci, hi = np.asarray(inner_bbox[:3]), np.asarray(inner_bbox[3:6]) / 2.0
    co, ho = np.asarray(outer_bbox[:3]), np.asarray(outer_bbox[3:6]) / 2.0
    ov = np.clip(np.minimum(ci + hi, co + ho) - np.maximum(ci - hi, co - ho),
                 0.0, None)
    vol_i = float(np.prod(np.maximum(2.0 * hi, 1e-6)))
    return float(np.prod(ov)) / vol_i


@dataclass
class _VoxelObs:
    bbox: list
    label: str
    score: float = 1.0
    seen: int = 1


class VoxelBehavior:
    def __init__(self, get_clock, score_threshold=0.7, min_cluster_size=30,
                 confirm_hits=3, track_max_misses=4, proximity_engage_m=12.0,
                 min_confidence=0.0):
        self.get_clock = get_clock
        self.name = 'Voxel-based'
        self.score_threshold = score_threshold
        self.min_cluster_size = min_cluster_size
        # Drop confirmed clusters below this confidence (0 = report all).
        self.min_confidence = min_confidence
        # Engage voxel-mode on a cluster within this range even without a ray
        # commitment (fix: finish nearby targets while exploring).
        self.proximity_engage_m = proximity_engage_m
        # Hysteresis: cluster currently being approached.
        self._held_center = None
        # Remembered-box center to fly to when no live cluster (go-to-box).
        self._goto_bb_center = None
        # Deconfliction for proximity engage (set by the node each tick):
        # fresh peer XY positions (id-tagged) + points peers are committed to.
        self.fresh_peer_points = []
        self.peer_committed_points = []
        self.my_id = 0
        # {cluster_id: [cx,cy,cz,sx,sy,sz]}
        self.target_voxel_clusters = {}
        self.cluster_query_map = {}
        self.cluster_confidence = {}
        self.visited_clusters = []
        self.unvisited_clusters = []
        # Per-instance (label, center, size) visited record for STATUS only —
        # separate from visited_clusters (nav) so it never affects path choice.
        self.visited_instances = []
        # Peer-VISITED BBs [cx,cy,cz,sx,sy,sz], set each tick by the node.
        self.peer_visited_bbs = []
        self.completed_queries = set()
        self.prev_voxel_cluster_ids = 0
        self._confirmer = TemporalConfirmer(
            confirm_hits=confirm_hits, max_misses=track_max_misses)
        # Stuck-recovery: when the straight-back standoff is unreachable, rotate
        # the approach azimuth around the held cluster instead of giving up.
        # Cursor advanced by note_stuck(); 0 = head-on (unchanged behaviour).
        self._approach_azimuth_variant = 0

    def reset(self):
        self.target_voxel_clusters.clear()
        self.cluster_query_map.clear()
        self.cluster_confidence.clear()
        self.visited_clusters = []
        self.unvisited_clusters = []
        self.visited_instances = []
        self.peer_visited_bbs = []
        self.completed_queries = set()
        self.prev_voxel_cluster_ids = 0
        self._held_center = None
        self._goto_bb_center = None
        self._confirmer.reset()

    def _cluster_matches_committed_ray(self, cluster, committed_origin,
                                       committed_dir) -> bool:
        """Strict-then-relaxed: voxel cluster belongs to the drone's
        committed ray if either
          (a) the ray's slab intersects the cluster AABB, OR
          (b) the cluster center is within RAY_TO_CLUSTER_RELAXED_M of the
              ray line AND in front of the drone.
        """
        bb = np.asarray(cluster, dtype=float)
        d = np.asarray(committed_dir, dtype=float)
        d_norm = np.linalg.norm(d)
        if d_norm < 1e-6:
            return False
        d = d / d_norm
        o = np.asarray(committed_origin, dtype=float)
        # (a) strict ray-AABB
        hit, _ = ray_aabb_hits(o, d, bb)
        if hit:
            return True
        # (b) relaxed: perpendicular distance from cluster center to ray line,
        # only counting points that are in front of the drone (t > 0).
        center = bb[:3]
        v = center - o
        t = float(np.dot(v, d))
        if t <= 0.0:
            return False
        closest = o + t * d
        perp = float(np.linalg.norm(center - closest))
        return perp <= RAY_TO_CLUSTER_RELAXED_M

    def _filter_to_committed_ray(self, clusters_dict, committed_origin,
                                 committed_dir):
        """Return only clusters whose AABB matches the committed ray (strict
        or relaxed). If no commitment exists, return empty — voxel-mode must
        not activate without a specific instance to chase."""
        if committed_origin is None or committed_dir is None:
            return {}
        return {
            i: c for i, c in clusters_dict.items()
            if self._cluster_matches_committed_ray(
                c, committed_origin, committed_dir)
        }

    def _candidate_clusters(self, clusters_dict, committed_origin, committed_dir,
                            cur_pose):
        """Clusters eligible for voxel-mode: those matching the committed ray PLUS
        any within proximity_engage_m of the drone (deconflicted). The proximity
        set lets a committed drone finish a closer target instead of flying to a
        far ray/frontier one — voxel execute then approaches the nearest."""
        out = {}
        if committed_origin is None or committed_dir is None:
            # No auction commitment -> don't voxel-engage; the drone explores
            # (frontier). Prevents an unassigned drone grabbing a cluster the
            # auction gave to someone else.
            return out
        out.update(self._filter_to_committed_ray(
            clusters_dict, committed_origin, committed_dir))
        if cur_pose is not None:
            cur = np.asarray(cur_pose, dtype=float)
            for i, c in clusters_dict.items():
                if i in out:
                    continue
                center = np.asarray(c[:3], dtype=float)
                if self._cuboid_distance(cur, np.zeros(3), center,
                                         np.asarray(c[3:6], dtype=float)) \
                        > self.proximity_engage_m:
                    continue
                if self._peer_blocks_proximity(center, cur):
                    continue
                out[i] = c
        return out

    # Cluster proximity (m) under which a peer's committed point claims it.
    _PEER_CLAIM_M = 10.0

    def _peer_blocks_proximity(self, center, cur) -> bool:
        """Defer a proximity engage when a fresh peer is closer to the cluster
        (id tiebreak) or is committed to it — so two drones don't grab the same
        cluster. Self-corrects as a busy peer moves away."""
        c2 = np.asarray(center, dtype=float)[:2]
        my_d = float(np.linalg.norm(c2 - np.asarray(cur, dtype=float)[:2]))
        for pid, p in self.fresh_peer_points:
            pd = float(np.linalg.norm(c2 - np.asarray(p, dtype=float)[:2]))
            if pd < my_d or (abs(pd - my_d) < 1e-3
                             and pid is not None and pid < self.my_id):
                return True
        for cp in self.peer_committed_points:
            if float(np.linalg.norm(
                    c2 - np.asarray(cp, dtype=float)[:2])) <= self._PEER_CLAIM_M:
                return True
        return False

    def condition_check(self, vox_xyz, vox_scores, query_labels, target_objects,
                        threshold=None, committed_origin=None,
                        committed_dir=None, cur_pose=None,
                        committed_bb_center=None):
        """True when voxel-mode should run: an unvisited live cluster exists, or
        the drone is committed to a remembered (stale) box with no live voxels
        yet and should fly to it (go-to-box) until its voxels reappear."""
        self._goto_bb_center = None
        if threshold is None:
            threshold = self.score_threshold

        if vox_xyz is None or vox_scores is None or not target_objects:
            return self._engage_goto(committed_bb_center)
        if len(vox_xyz) == 0:
            return self._engage_goto(committed_bb_center)

        label_indices = [
            query_labels.index(t) for t in target_objects
            if t in query_labels
        ]
        if not label_indices:
            return self._engage_goto(committed_bb_center)

        raw_obs = self._extract_clusters(
            vox_xyz, vox_scores, label_indices, target_objects, threshold)
        confirmed = self._confirmer.update(
            raw_obs, match=self._same_cluster, merge=self._merge_obs)
        confirmed = self._suppress_contained(confirmed)

        self.target_voxel_clusters.clear()
        self.cluster_query_map.clear()
        self.cluster_confidence.clear()
        cid = 0
        for obs in confirmed:
            conf = float(obs.score) * min(1.0, obs.seen / PERSISTENCE_FULL_HITS)
            if conf < self.min_confidence:
                continue
            self.target_voxel_clusters[cid] = obs.bbox
            self.cluster_query_map[cid] = obs.label
            self.cluster_confidence[cid] = conf
            cid += 1

        # Candidates: ray-matched clusters when committed, else clusters near the
        # drone. The ray filter keeps voxel-mode from stealing House_B when the
        # drone is committed to House_A.
        ray_filtered = self._candidate_clusters(
            self.target_voxel_clusters, committed_origin, committed_dir, cur_pose)

        self.unvisited_clusters = [
            (idx, cluster) for idx, cluster in ray_filtered.items()
            if not self._is_near_visited(
                np.array(cluster[:3]), np.array(cluster[3:6]), self.visited_clusters)
            and not self._is_near_peer_visited(
                np.array(cluster[:3]), np.array(cluster[3:6]))
        ]

        return len(self.unvisited_clusters) > 0 or self._engage_goto(
            committed_bb_center)

    def _engage_goto(self, committed_bb_center) -> bool:
        """Arm go-to-box mode only when committed to a genuinely stale box: no
        live cluster near it and no peer has visited it. Live boxes fall through
        to normal voxel/ray/frontier handling."""
        if committed_bb_center is None:
            self._goto_bb_center = None
            return False
        c = np.asarray(committed_bb_center, dtype=float)[:3]
        if self._is_near_peer_visited(c, np.zeros(3)):
            self._goto_bb_center = None
            return False
        for bb in self.target_voxel_clusters.values():
            if float(np.linalg.norm(np.asarray(bb[:3], dtype=float) - c)) <= 5.0:
                self._goto_bb_center = None
                return False
        self._goto_bb_center = c
        return True

    def _suppress_contained(self, confirmed):
        """Drop a box mostly inside a larger SAME-label box (over-split fragments
        of one object); a different-label box inside another's bbox is kept."""
        order = sorted(
            confirmed,
            key=lambda o: float(np.prod(np.asarray(o.bbox[3:6]))), reverse=True)
        kept: list = []
        for o in order:
            if any(o.label == k.label
                   and _contained_frac(o.bbox, k.bbox) >= CONTAIN_SUPPRESS_FRAC
                   for k in kept):
                continue
            kept.append(o)
        return kept

    def _same_cluster(self, a: _VoxelObs, b: _VoxelObs) -> bool:
        if a.label != b.label:
            return False
        return self._cuboid_distance(
            np.array(a.bbox[:3]), np.array(a.bbox[3:6]),
            np.array(b.bbox[:3]), np.array(b.bbox[3:6])) <= TRACK_ASSOC_MARGIN_M

    def _merge_obs(self, a: _VoxelObs, b: _VoxelObs) -> _VoxelObs:
        """EMA the centroid + score; take this frame's size. Keeps the track label
        (per-label clustering + same-label matching keep it consistent)."""
        al = TRACK_MERGE_ALPHA
        a_bb = np.asarray(a.bbox, dtype=float)
        b_bb = np.asarray(b.bbox, dtype=float)
        center = (1.0 - al) * a_bb[:3] + al * b_bb[:3]
        size = b_bb[3:6]
        score = (1.0 - al) * a.score + al * b.score
        return _VoxelObs(bbox=[*center.tolist(), *size.tolist()],
                         label=a.label, score=score, seen=a.seen + 1)

    def _extract_clusters(self, vox_xyz, vox_scores, label_indices,
                          target_objects, threshold):
        """Per-label clusters: each voxel is assigned to its single highest-scoring
        target (argmax), and connected-components runs within each label — so
        adjacent objects of different classes stay separate and correctly named."""
        relevant_scores = vox_scores[:, label_indices]
        best_t = relevant_scores.argmax(axis=1)
        best_s = relevant_scores.max(axis=1)
        vox_size = 0.5
        obs = []
        for t in range(len(label_indices)):
            sel = np.where((best_t == t) & (best_s > threshold))[0]
            if len(sel) < self.min_cluster_size:
                continue
            obs.extend(self._cluster_one_label(
                vox_xyz[sel], relevant_scores[sel], t, target_objects, vox_size))
        return obs

    def _cluster_one_label(self, label_vox, label_scores, t, target_objects,
                           vox_size):
        """26-connected components + instance split for one target label's voxels."""
        out = []
        filtered_vox = np.round(label_vox, 3)
        min_coords = filtered_vox.min(axis=0)
        norm_coords = ((filtered_vox - min_coords) / vox_size).astype(int)
        max_coords = norm_coords.max(axis=0) + 1
        occupancy = np.zeros(tuple(max_coords.tolist()), dtype=np.uint8)
        occupancy[norm_coords[:, 0], norm_coords[:, 1], norm_coords[:, 2]] = 1
        labeled, num = scipy.ndimage.label(
            occupancy, structure=np.ones((3, 3, 3), dtype=np.uint8))
        comp_ids = labeled[norm_coords[:, 0], norm_coords[:, 1], norm_coords[:, 2]]
        label = target_objects[t] if t < len(target_objects) else target_objects[0]
        for cv in range(1, num + 1):
            idx = np.where(comp_ids == cv)[0]
            if len(idx) < self.min_cluster_size:
                continue
            comp_coords = norm_coords[idx]
            comp_scores = label_scores[idx]
            for g in self._split_into_instances(
                    comp_coords, comp_scores[:, t], vox_size):
                if len(g) < self.min_cluster_size:
                    continue
                coords = comp_coords[g]
                min_world = coords.min(axis=0) * vox_size + min_coords
                max_world = (coords.max(axis=0) + 1) * vox_size + min_coords
                center = (min_world + max_world) / 2
                size = max_world - min_world
                mean_scores = comp_scores[g].mean(axis=0)
                out.append(_VoxelObs(
                    bbox=[center[0], center[1], center[2],
                          size[0], size[1], size[2]],
                    label=label,
                    score=float(mean_scores[t])))
        return out

    def _split_into_instances(self, coords, win_scores, vox_size):
        """Split a connected component into separate instances at score peaks:
        find local maxima of the smoothed score field, assign each voxel to its
        nearest peak. Returns index arrays into `coords`."""
        n = len(coords)
        if n < 2 * self.min_cluster_size:
            return [np.arange(n)]
        lo = coords.min(axis=0)
        ext = coords.max(axis=0) - lo + 1
        if int(np.prod(ext)) > 2_000_000:
            return [np.arange(n)]
        cc = coords - lo
        S = np.zeros(tuple(ext.tolist()), dtype=float)
        S[cc[:, 0], cc[:, 1], cc[:, 2]] = win_scores
        occ = S > 0
        Ss = scipy.ndimage.gaussian_filter(S, SPLIT_SMOOTH_SIGMA_VOX)
        Ss[~occ] = 0.0
        win = max(1, int(round(SPLIT_PEAK_SEP_M / vox_size)))
        peaks = occ & (Ss >= scipy.ndimage.maximum_filter(Ss, size=win)) & (Ss > 0)
        plbl, npk = scipy.ndimage.label(peaks, structure=np.ones((3, 3, 3)))
        if npk <= 1:
            return [np.arange(n)]
        cents = np.array(
            scipy.ndimage.center_of_mass(peaks, plbl, range(1, npk + 1)),
            dtype=float)
        nearest = ((cc[:, None, :] - cents[None, :, :]) ** 2).sum(
            axis=2).argmin(axis=1)
        return [np.where(nearest == k)[0] for k in range(npk)]

    def execute(self, vox_xyz, vox_scores, query_labels, cur_pose_np,
                waypoint_locked, target_waypoint, target_waypoint2, publisher_dict,
                committed_origin=None, committed_dir=None,
                committed_bb_center=None):
        voxel_bbox_pub = publisher_dict.get('voxel_bbox')
        if voxel_bbox_pub:
            self._visualize_clusters(voxel_bbox_pub)

        path_pub = publisher_dict['path']

        ray_filtered = self._candidate_clusters(
            self.target_voxel_clusters, committed_origin, committed_dir, cur_pose_np)
        self.unvisited_clusters = [
            (idx, cluster) for idx, cluster in ray_filtered.items()
            if not self._is_near_visited(
                np.array(cluster[:3]), np.array(cluster[3:6]), self.visited_clusters)
            and not self._is_near_peer_visited(
                np.array(cluster[:3]), np.array(cluster[3:6]))
        ]
        if not self.unvisited_clusters:
            if self._goto_bb_center is not None and cur_pose_np is not None:
                self._publish_goto_path(self._goto_bb_center, cur_pose_np, path_pub)
            return waypoint_locked, target_waypoint, target_waypoint2

        sorted_clusters = sorted(
            self.unvisited_clusters,
            key=lambda item: np.linalg.norm(cur_pose_np - np.array(item[1][:3])))

        # Hysteresis: keep approaching the held cluster if still a candidate and
        # not much farther than the nearest, so it doesn't swap between near-equal
        # clusters.
        if self._held_center is not None and len(sorted_clusters) > 1:
            d_near = np.linalg.norm(cur_pose_np - np.array(sorted_clusters[0][1][:3]))
            for k, (idx, cl) in enumerate(sorted_clusters):
                if np.linalg.norm(np.array(cl[:3]) - self._held_center) <= 3.0:
                    if np.linalg.norm(cur_pose_np - np.array(cl[:3])) <= d_near + 5.0:
                        sorted_clusters.insert(0, sorted_clusters.pop(k))
                    break
        new_held = np.array(sorted_clusters[0][1][:3], dtype=float)
        # A different cluster is now the target -> restart approach head-on.
        if (self._held_center is None
                or float(np.linalg.norm(new_held - self._held_center)) > 3.0):
            self._approach_azimuth_variant = 0
        self._held_center = new_held

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        # Don't fire arrival detection on the tick the waypoint was first set.
        waypoint_was_locked = waypoint_locked

        for i, (idx, cluster) in enumerate(sorted_clusters):
            center = np.array(cluster[:3])
            sizes = np.array(cluster[3:])
            half_sizes = sizes / 2.0
            direction = center - cur_pose_np
            dist = np.linalg.norm(direction)
            if dist < 1e-6:
                continue
            dir_norm = direction / dist

            # Surface point facing the drone (slab-based AABB intersect).
            ray_origin_local = cur_pose_np - center
            tmin, tmax = -np.inf, np.inf
            for axis in range(3):
                if abs(dir_norm[axis]) > 1e-9:
                    t1 = (-half_sizes[axis] - ray_origin_local[axis]) / dir_norm[axis]
                    t2 = (half_sizes[axis] - ray_origin_local[axis]) / dir_norm[axis]
                    tmin = max(tmin, min(t1, t2))
                    tmax = min(tmax, max(t1, t2))
            if tmax < max(tmin, 0):
                continue
            t_hit = tmin if tmin > 0 else tmax
            surface_point = cur_pose_np + dir_norm * t_hit
            adjacent = surface_point - dir_norm * 1.0

            # Stuck recovery: after a stuck strike (note_stuck) approach the
            # cluster from a rotated azimuth instead of head-on. approach_dir
            # points drone->cluster rotated about +Z; the standoff then sits on
            # that side of the box surface (variant 0 == head-on == unchanged).
            approach_dir = self._approach_dir(center, cur_pose_np)
            if approach_dir is not None and self._approach_azimuth_variant % \
                    len(self._APPROACH_AZIMUTHS) != 0:
                half_extent = self._box_half_extent_along(half_sizes, approach_dir)
                surface_point = center - approach_dir * half_extent
                stand_dir = approach_dir
            else:
                stand_dir = dir_norm

            if i == 0:
                if not waypoint_locked:
                    target_waypoint2 = self._clear_standoff(
                        surface_point, stand_dir, vox_xyz)
                    if target_waypoint2 is None:
                        self.visited_clusters.append(cluster)
                        break
                    waypoint_locked = True
                elif self._point_near_voxels(target_waypoint2, vox_xyz):
                    new_wp = self._clear_standoff(surface_point, stand_dir, vox_xyz)
                    if new_wp is None:
                        self.visited_clusters.append(cluster)
                        waypoint_locked = False
                        target_waypoint2 = None
                        break
                    target_waypoint2 = new_wp

                alpha = 0.8
                mid = cur_pose_np * (1 - alpha) + target_waypoint2 * alpha
                target_waypoint = mid

                for wp_np in [mid, target_waypoint2]:
                    ps = PoseStamped()
                    ps.header.stamp = self.get_clock().now().to_msg()
                    ps.header.frame_id = 'map'
                    ps.pose.position.x = float(wp_np[0])
                    ps.pose.position.y = float(wp_np[1])
                    ps.pose.position.z = float(wp_np[2])
                    ps.pose.orientation.w = 1.0
                    path.poses.append(ps)

        path_pub.publish(path)

        # Completion uses the target's AABB, not voxel/standoff proximity: mark it
        # visited once the drone is within VISIT_REACH_M of the cluster AABB
        # *surface* (0 if inside it), or has reached its standoff waypoint. The
        # AABB-surface test stops the drone getting stuck circling when the exact
        # standoff is unreachable. Per-instance only (NOT completed_queries — that
        # would retire the whole class fleet-wide after one instance). visited_clusters
        # gates nav re-targeting; visited_instances is the status/auction record.
        if sorted_clusters:
            arrived_idx, arrived = sorted_clusters[0]
            c = np.array(arrived[:3], dtype=float)
            s = np.array(arrived[3:6], dtype=float)
            near_aabb = aabb_surface_dist(cur_pose_np, np.zeros(3), c, s) <= VISIT_REACH_M
            near_wp = (waypoint_was_locked and waypoint_arrival_counts(
                cur_pose_np, target_waypoint2, c, s))
            if near_aabb or near_wp:
                label = self.cluster_query_map.get(arrived_idx, '')
                if not self.is_visited(c, s, label):
                    self.visited_instances.append((label, c, s))
                if not self._is_near_visited(c, s, self.visited_clusters):
                    self.visited_clusters.append(arrived)
                waypoint_locked = False

        return waypoint_locked, target_waypoint, target_waypoint2

    def _is_near_visited(self, center, size, visited_clusters):
        return any(
            same_instance_xy(center, size, np.array(v[:3]), np.array(v[3:6]))
            for v in visited_clusters)

    def _is_near_peer_visited(self, center, size):
        """True if a peer already visited this target (same instance = overlap)."""
        return any(
            same_instance_xy(center, size, np.array(bb[:3]), np.array(bb[3:6]))
            for bb in self.peer_visited_bbs)

    def _cuboid_distance(self, ca, sa, cb, sb):
        return aabb_surface_dist(ca, sa, cb, sb)

    def mark_arrivals(self, cur_pose_np, radius_m=VISIT_REACH_M):
        """Flag detected clusters within radius_m of the drone as visited
        (per-instance, label-scoped) without driving navigation. Call after
        condition_check() has populated target_voxel_clusters."""
        if cur_pose_np is None:
            return
        cur = np.asarray(cur_pose_np, dtype=float)
        for cid, cluster in self.target_voxel_clusters.items():
            center = np.array(cluster[:3], dtype=float)
            size = np.array(cluster[3:6], dtype=float)
            label = self.cluster_query_map.get(cid, '')
            if self.is_visited(center, size, label):
                continue
            if self._cuboid_distance(cur, np.zeros(3), center, size) < radius_m:
                self.visited_instances.append((label, center, size))

    def is_visited(self, center, size, label):
        """True if (center, size, label) matches a previously marked-visited
        instance: same label and same physical instance (xy overlap)."""
        for vlabel, vcenter, vsize in self.visited_instances:
            if vlabel != label:
                continue
            if same_instance_xy(center, size, vcenter, vsize):
                return True
        return False

    def _point_near_voxels(self, point, vox_xyz, clearance=WAYPOINT_CLEARANCE_M):
        """True if the waypoint sits within clearance of any mapped voxel."""
        if vox_xyz is None or len(vox_xyz) == 0:
            return False
        d2 = ((np.asarray(vox_xyz, dtype=float)
               - np.asarray(point, dtype=float)) ** 2).sum(axis=1)
        return bool(d2.min() < clearance * clearance)

    # Approach azimuths tried on successive stuck strikes (degrees around +Z,
    # relative to head-on): head-on, then swing to either side, then behind.
    _APPROACH_AZIMUTHS = (0.0, 40.0, -40.0, 80.0, -80.0, 140.0)

    def note_stuck(self) -> None:
        """Called by the node when the pursuit stuck monitor fires in voxel
        mode: advance to the next approach azimuth for the held cluster so the
        next standoff is computed from a different side. Force a re-plan by
        clearing the hold so execute() recomputes the waypoint this tick."""
        self._approach_azimuth_variant += 1

    def _approach_dir(self, center, cur_pose_np):
        """Unit drone->cluster bearing rotated by the current stuck azimuth
        variant, so a blocked head-on corridor can be retried from the side."""
        d = np.asarray(center, dtype=float) - np.asarray(cur_pose_np, dtype=float)
        n = float(np.linalg.norm(d))
        if n < 1e-6:
            return None
        d = d / n
        idx = self._approach_azimuth_variant % len(self._APPROACH_AZIMUTHS)
        deg = self._APPROACH_AZIMUTHS[idx]
        if deg == 0.0:
            return d
        a = np.deg2rad(deg)
        c, s = float(np.cos(a)), float(np.sin(a))
        out = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])
        on = float(np.linalg.norm(out))
        return out / on if on > 1e-6 else d

    @staticmethod
    def _box_half_extent_along(half_sizes, u) -> float:
        """Distance from an AABB centre to its surface along unit direction u
        (first face hit): min_i(h_i / |u_i|) over axes with |u_i|>0."""
        h = np.asarray(half_sizes, dtype=float)
        u = np.abs(np.asarray(u, dtype=float))
        ts = [h[k] / u[k] for k in range(3) if u[k] > 1e-9]
        return float(min(ts)) if ts else float(np.max(h))

    def _clear_standoff(self, surface_point, dir_norm, vox_xyz,
                        base=1.0, step=STANDOFF_STEP_M, max_standoff=STANDOFF_MAX_M):
        """Pull the approach point back from the surface (toward the drone) until
        it clears all voxels; None if nothing within max_standoff is free."""
        s = base
        while s <= max_standoff:
            wp = surface_point - dir_norm * s
            if not self._point_near_voxels(wp, vox_xyz):
                return wp
            s += step
        return None

    def _publish_goto_path(self, target, cur_pose_np, path_pub):
        """Steer toward a remembered box center with no live voxels yet. droan
        follows /global_plan; on approach the voxels reappear and normal
        voxel-mode + mark_arrivals take over and confirm it visited."""
        target = np.asarray(target, dtype=float)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        d = target - cur_pose_np
        if float(np.linalg.norm(d)) >= 1e-6:
            for wp in (cur_pose_np + d * 0.5, target):
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x = float(wp[0])
                ps.pose.position.y = float(wp[1])
                ps.pose.position.z = float(wp[2])
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)
        path_pub.publish(path)

    def _visualize_clusters(self, pub):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i in range(self.prev_voxel_cluster_ids):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'voxel_clusters'
            m.id = i
            m.action = Marker.DELETE
            markers.markers.append(m)
        for j, (_, cluster) in enumerate(self.unvisited_clusters):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'voxel_clusters'
            m.id = j
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = cluster[0]
            m.pose.position.y = cluster[1]
            m.pose.position.z = cluster[2]
            m.scale.x = cluster[3]
            m.scale.y = cluster[4]
            m.scale.z = cluster[5]
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.2)
            m.lifetime.sec = 1
            markers.markers.append(m)
        self.prev_voxel_cluster_ids = len(self.unvisited_clusters)
        pub.publish(markers)
