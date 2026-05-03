import time

import numpy as np
import scipy.ndimage
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class VoxelBehavior:
    def __init__(self, get_clock):
        self.get_clock = get_clock
        self.name = 'Voxel-based'
        # {cluster_id: [cx, cy, cz, sx, sy, sz]}
        self.target_voxel_clusters = {}
        # {cluster_id: query_label} — which query each cluster belongs to
        self.cluster_query_map = {}
        self.visited_clusters = []    # list of [cx,cy,cz,sx,sy,sz]
        self.unvisited_clusters = []  # list of (cluster_id, [cx,cy,cz,sx,sy,sz])
        self.completed_queries = set()
        self.prev_voxel_cluster_ids = 0

    def reset(self):
        self.target_voxel_clusters.clear()
        self.cluster_query_map.clear()
        self.visited_clusters = []
        self.unvisited_clusters = []
        self.completed_queries = set()
        self.prev_voxel_cluster_ids = 0

    def condition_check(self, vox_xyz, vox_scores, query_labels, target_objects):
        """
        vox_xyz:      np.ndarray (N, 3) — voxel world positions
        vox_scores:   np.ndarray (N, Q) — softmax similarity per query
        query_labels: list[str]         — ordered label for each sim column
        target_objects: list[str]       — which labels we're searching for
        """
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

        threshold = 0.98
        relevant_scores = vox_scores[:, label_indices]   # (N, len(label_indices))
        mask = (relevant_scores > threshold).any(axis=1)
        indices = np.where(mask)[0]

        if len(indices) == 0:
            return False

        # Cluster the high-confidence voxels with connected-component labeling
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

        self.target_voxel_clusters.clear()
        self.cluster_query_map.clear()
        vox_cluster_count = 0

        for label_val in range(1, num_components + 1):
            idx = np.where(label_ids == label_val)[0]
            if len(idx) < 30:
                continue

            coords = norm_coords[idx]
            min_voxel = coords.min(axis=0)
            max_voxel = coords.max(axis=0)
            min_world = min_voxel * vox_size + min_coords
            max_world = (max_voxel + 1) * vox_size + min_coords
            center = (min_world + max_world) / 2
            size = max_world - min_world

            # Data is already in FLU (raven_nav converts before passing)
            cx, cy, cz = center[0], center[1], center[2]
            sx, sy, sz = size[0], size[1], size[2]

            # Tag cluster with best-scoring query label for this cluster
            cluster_scores = relevant_scores[idx]  # (cluster_size, len(label_indices))
            best_local = int(cluster_scores.mean(axis=0).argmax())
            best_label = target_objects[best_local] if best_local < len(target_objects) else target_objects[0]

            self.target_voxel_clusters[vox_cluster_count] = [cx, cy, cz, sx, sy, sz]
            self.cluster_query_map[vox_cluster_count] = best_label
            vox_cluster_count += 1

        self.unvisited_clusters = [
            (idx, cluster) for idx, cluster in self.target_voxel_clusters.items()
            if not self._is_near_visited(
                np.array(cluster[:3]), np.array(cluster[3:6]), self.visited_clusters)
        ]

        return len(self.unvisited_clusters) > 0

    def execute(self, vox_xyz, vox_scores, query_labels, cur_pose_np,
                waypoint_locked, target_waypoint, target_waypoint2, publisher_dict):
        voxel_bbox_pub = publisher_dict.get('voxel_bbox')
        if voxel_bbox_pub:
            self._visualize_clusters(voxel_bbox_pub)

        path_pub = publisher_dict['path']

        self.unvisited_clusters = [
            (idx, cluster) for idx, cluster in self.target_voxel_clusters.items()
            if not self._is_near_visited(
                np.array(cluster[:3]), np.array(cluster[3:6]), self.visited_clusters)
        ]

        sorted_clusters = sorted(
            self.unvisited_clusters,
            key=lambda item: np.linalg.norm(cur_pose_np - np.array(item[1][:3])))

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        # Track whether the waypoint was already locked before this tick.
        # Arrival detection must NOT fire in the same tick the waypoint is first set.
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

            # Find surface point facing the drone
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
            adjacent = surface_point - dir_norm * 1.0   # 1m in front of surface

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

        # Mark cluster as visited only when the drone has been navigating toward a
        # previously-locked waypoint (waypoint_was_locked) AND arrives within 3m.
        # Skipping the check on the tick the waypoint was first set prevents
        # immediate false-positive "arrived" detections.
        if waypoint_was_locked and target_waypoint2 is not None and \
                np.linalg.norm(cur_pose_np - target_waypoint2) < 3.0:
            if sorted_clusters:
                arrived_idx, arrived_cluster = sorted_clusters[0]
                self.visited_clusters.append(arrived_cluster)
                completed_label = self.cluster_query_map.get(arrived_idx)
                if completed_label:
                    self.completed_queries.add(completed_label)
            waypoint_locked = False

        return waypoint_locked, target_waypoint, target_waypoint2

    # ── helpers ──────────────────────────────────────────────────────────────

    def _is_near_visited(self, center, size, visited_clusters, threshold=10.0):
        return any(
            self._cuboid_distance(center, size,
                                  np.array(v[:3]), np.array(v[3:6])) < threshold
            for v in visited_clusters)

    def _cuboid_distance(self, ca, sa, cb, sb):
        ha, hb = sa / 2.0, sb / 2.0
        dx = max(abs(ca[0] - cb[0]) - (ha[0] + hb[0]), 0)
        dy = max(abs(ca[1] - cb[1]) - (ha[1] + hb[1]), 0)
        dz = max(abs(ca[2] - cb[2]) - (ha[2] + hb[2]), 0)
        return np.sqrt(dx**2 + dy**2 + dz**2)

    def _visualize_clusters(self, pub):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        # Delete old markers
        for i in range(self.prev_voxel_cluster_ids):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'voxel_clusters'
            m.id = i
            m.action = Marker.DELETE
            markers.markers.append(m)
        # Add new markers
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
