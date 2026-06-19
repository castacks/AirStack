import time
from dataclasses import dataclass

import numpy as np
import scipy.ndimage
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from raven_nav.ray_targets import ray_aabb_hits
from raven_nav.track_confirmation import TemporalConfirmer

# Relaxed fallback: cluster center is accepted if it sits within this many
# metres of the committed ray's line, AND is in front of the drone. Tolerates
# minor mis-alignment between the auctioned ray triangulation and where the
# voxel cluster actually consolidates (sensor noise, mapper smoothing).
RAY_TO_CLUSTER_RELAXED_M = 8.0

# Surface gap (m) under which two AABBs across ticks are the same cluster.
TRACK_ASSOC_MARGIN_M = 2.0


@dataclass
class _VoxelObs:
    bbox: list   # [cx,cy,cz,sx,sy,sz] in metres
    label: str


class VoxelBehavior:
    def __init__(self, get_clock, score_threshold=0.7, min_cluster_size=30,
                 confirm_hits=3, track_max_misses=4, proximity_engage_m=12.0):
        self.get_clock = get_clock
        self.name = 'Voxel-based'
        self.score_threshold = score_threshold
        self.min_cluster_size = min_cluster_size
        # Engage voxel-mode on a cluster within this range even without a ray
        # commitment (fix: finish nearby targets while exploring).
        self.proximity_engage_m = proximity_engage_m
        # Hysteresis: cluster currently being approached.
        self._held_center = None
        # Deconfliction for proximity engage (set by the node each tick):
        # fresh peer XY positions (id-tagged) + points peers are committed to.
        self.fresh_peer_points = []      # [(peer_id, np.ndarray)]
        self.peer_committed_points = []  # [np.ndarray]
        self.my_id = 0
        # {cluster_id: [cx,cy,cz,sx,sy,sz]}
        self.target_voxel_clusters = {}
        # {cluster_id: query_label}
        self.cluster_query_map = {}
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

    def reset(self):
        self.target_voxel_clusters.clear()
        self.cluster_query_map.clear()
        self.visited_clusters = []
        self.unvisited_clusters = []
        self.visited_instances = []
        self.peer_visited_bbs = []
        self.completed_queries = set()
        self.prev_voxel_cluster_ids = 0
        self._held_center = None
        self._confirmer.reset()

    def _cluster_matches_committed_ray(self, cluster, committed_origin,
                                       committed_dir) -> bool:
        """Strict-then-relaxed: voxel cluster belongs to the drone's
        committed ray if either
          (a) the ray's slab intersects the cluster AABB, OR
          (b) the cluster center is within RAY_TO_CLUSTER_RELAXED_M of the
              ray line AND in front of the drone.
        """
        bb = np.asarray(cluster, dtype=float)   # [cx,cy,cz,sx,sy,sz]
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
        if committed_origin is not None and committed_dir is not None:
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
                        committed_dir=None, cur_pose=None):
        """vox_xyz: (N,3) FLU. vox_scores: (N,Q) softmax. labels parallel sim_* cols.

        Returns True iff there is at least one unvisited, temporally-confirmed
        voxel cluster for one of the target_objects. When True, voxel-mode takes
        over for fine approach + 3m arrival detection.
        """
        if threshold is None:
            threshold = self.score_threshold

        # No data this tick: leave tracks untouched (don't age on a dropped frame).
        if vox_xyz is None or vox_scores is None or not target_objects:
            return False
        if len(vox_xyz) == 0:
            return False

        label_indices = [
            query_labels.index(t) for t in target_objects
            if t in query_labels
        ]
        if not label_indices:
            return False

        raw_obs = self._extract_clusters(
            vox_xyz, vox_scores, label_indices, target_objects, threshold)
        confirmed = self._confirmer.update(raw_obs, match=self._same_cluster)

        self.target_voxel_clusters.clear()
        self.cluster_query_map.clear()
        for cid, obs in enumerate(confirmed):
            self.target_voxel_clusters[cid] = obs.bbox
            self.cluster_query_map[cid] = obs.label

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

        return len(self.unvisited_clusters) > 0

    def _same_cluster(self, a: _VoxelObs, b: _VoxelObs) -> bool:
        # Label-agnostic so a flickering best-label doesn't reset a track.
        return self._cuboid_distance(
            np.array(a.bbox[:3]), np.array(a.bbox[3:6]),
            np.array(b.bbox[:3]), np.array(b.bbox[3:6])) <= TRACK_ASSOC_MARGIN_M

    def _extract_clusters(self, vox_xyz, vox_scores, label_indices,
                          target_objects, threshold):
        """Per-frame clusters (high-conf, 26-connected, >= min_cluster_size)."""
        relevant_scores = vox_scores[:, label_indices]   # (N, len(label_indices))
        mask = (relevant_scores > threshold).any(axis=1)
        indices = np.where(mask)[0]
        if len(indices) == 0:
            return []

        # 3D connected-component labeling on high-confidence voxels.
        filtered_vox = np.round(vox_xyz[indices], 3)
        vox_size = 0.5

        min_coords = filtered_vox.min(axis=0)
        norm_coords = ((filtered_vox - min_coords) / vox_size).astype(int)
        max_coords = norm_coords.max(axis=0) + 1
        occupancy = np.zeros(tuple(max_coords.tolist()), dtype=np.uint8)
        for x, y, z in norm_coords:
            occupancy[x, y, z] = 1

        structure = np.ones((3, 3, 3), dtype=np.uint8)
        labeled, num_components = scipy.ndimage.label(occupancy, structure=structure)

        label_ids = np.array([labeled[x, y, z] for x, y, z in norm_coords])

        obs = []
        for label_val in range(1, num_components + 1):
            idx = np.where(label_ids == label_val)[0]
            if len(idx) < self.min_cluster_size:
                continue

            coords = norm_coords[idx]
            min_voxel = coords.min(axis=0)
            max_voxel = coords.max(axis=0)
            min_world = min_voxel * vox_size + min_coords
            max_world = (max_voxel + 1) * vox_size + min_coords
            center = (min_world + max_world) / 2
            size = max_world - min_world

            cluster_scores = relevant_scores[idx]
            best_local = int(cluster_scores.mean(axis=0).argmax())
            best_label = (target_objects[best_local]
                          if best_local < len(target_objects)
                          else target_objects[0])

            obs.append(_VoxelObs(
                bbox=[center[0], center[1], center[2],
                      size[0], size[1], size[2]],
                label=best_label))
        return obs

    def execute(self, vox_xyz, vox_scores, query_labels, cur_pose_np,
                waypoint_locked, target_waypoint, target_waypoint2, publisher_dict,
                committed_origin=None, committed_dir=None):
        voxel_bbox_pub = publisher_dict.get('voxel_bbox')
        if voxel_bbox_pub:
            self._visualize_clusters(voxel_bbox_pub)

        path_pub = publisher_dict['path']

        # Same candidate pool as condition_check (ray-matched, or nearby when
        # uncommitted).
        ray_filtered = self._candidate_clusters(
            self.target_voxel_clusters, committed_origin, committed_dir, cur_pose_np)
        self.unvisited_clusters = [
            (idx, cluster) for idx, cluster in ray_filtered.items()
            if not self._is_near_visited(
                np.array(cluster[:3]), np.array(cluster[3:6]), self.visited_clusters)
            and not self._is_near_peer_visited(
                np.array(cluster[:3]), np.array(cluster[3:6]))
        ]
        # If the drone's commitment moved off this cluster bucket (peer
        # outbid, target completed, peer already visited, etc.) we'd have no
        # candidates — fall back gracefully without crashing the path publish.
        if not self.unvisited_clusters:
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
        self._held_center = np.array(sorted_clusters[0][1][:3], dtype=float)

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
            adjacent = surface_point - dir_norm * 1.0  # 1m in front

            if i == 0:
                if not waypoint_locked:
                    target_waypoint2 = adjacent
                    waypoint_locked = True

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

        # Mark cluster visited only on a previously-locked waypoint within 3m.
        # NOTE: tracking is per-INSTANCE (the cluster's AABB) via visited_clusters
        # + the 10 m _is_near_visited spatial filter. We deliberately do NOT add
        # the label to completed_queries here — the label set would retire the
        # whole class fleet-wide after a single instance, so a "find all houses"
        # task would stop after one house. Per-instance dedup is enough; the
        # task-level instance_id system handles cross-drone deduplication of
        # the discovery output.
        if waypoint_was_locked and target_waypoint2 is not None and \
                np.linalg.norm(cur_pose_np - target_waypoint2) < 3.0:
            if sorted_clusters:
                arrived_idx, arrived_cluster = sorted_clusters[0]
                self.visited_clusters.append(arrived_cluster)
            waypoint_locked = False

        return waypoint_locked, target_waypoint, target_waypoint2

    def _is_near_visited(self, center, size, visited_clusters, threshold=10.0):
        return any(
            self._cuboid_distance(center, size,
                                  np.array(v[:3]), np.array(v[3:6])) < threshold
            for v in visited_clusters)

    def _is_near_peer_visited(self, center, size, threshold=10.0):
        """True if a peer already visited this target (within same-instance radius)."""
        return any(
            self._cuboid_distance(center, size,
                                  np.array(bb[:3]), np.array(bb[3:6])) < threshold
            for bb in self.peer_visited_bbs)

    def _cuboid_distance(self, ca, sa, cb, sb):
        ha, hb = sa / 2.0, sb / 2.0
        dx = max(abs(ca[0] - cb[0]) - (ha[0] + hb[0]), 0)
        dy = max(abs(ca[1] - cb[1]) - (ha[1] + hb[1]), 0)
        dz = max(abs(ca[2] - cb[2]) - (ha[2] + hb[2]), 0)
        return np.sqrt(dx**2 + dy**2 + dz**2)

    def mark_arrivals(self, cur_pose_np, radius_m=3.0):
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

    def is_visited(self, center, size, label, match_m=10.0):
        """True if (center, size, label) matches a previously marked-visited
        instance: same label and within match_m cuboid distance (the system's
        same-physical-instance proximity, matching _is_near_visited)."""
        for vlabel, vcenter, vsize in self.visited_instances:
            if vlabel != label:
                continue
            if self._cuboid_distance(center, size, vcenter, vsize) < match_m:
                return True
        return False

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
