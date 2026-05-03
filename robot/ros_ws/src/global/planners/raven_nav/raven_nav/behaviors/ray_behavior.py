import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String


import logging
_log = logging.getLogger(__name__)


class RayBehavior:
    def __init__(self, get_clock, current_target_publisher=None, score_threshold=0.95,
                 min_altitude=1.5, max_altitude=100.0):
        self.get_clock = get_clock
        self.name = 'Ray-based'
        self.score_threshold = score_threshold
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.prev_filtered_marker_ids = 0
        self.current_target = None
        self.current_target_pub = current_target_publisher

        # Set by condition_check, consumed by execute
        self._filtered_indices = None   # indices into ray arrays that pass threshold
        self._ray_origins = None        # (N, 3) FLU — full ray set
        self._ray_dirs = None           # (N, 3) FLU
        self._per_ray_label = None      # list[str] — best label per filtered ray

        # Active target: the target the drone is committed to pursuing.
        # Set on first acquisition; stays fixed until explicitly reset.
        self._active_target = None

    def condition_check(self, ray_origins, ray_dirs, ray_scores,
                        query_labels, target_objects):
        """Returns True if any ray scores above threshold for a target object."""
        self.current_target = None
        self._filtered_indices = None

        if (ray_origins is None or ray_scores is None or
                len(ray_origins) == 0 or len(target_objects) == 0):
            return False

        # Find column indices in ray_scores for each target object
        label_indices = [query_labels.index(t) for t in target_objects
                         if t in query_labels]
        if not label_indices:
            return False

        relevant_scores = ray_scores[:, label_indices]          # (N, num_targets)

        mask = (relevant_scores > self.score_threshold).any(axis=1)
        indices = np.where(mask)[0]

        if indices.size == 0:
            return False

        # Assign each passing ray its best-scoring target label
        per_ray_label = []
        targets_found = []
        for idx in indices:
            scores_i = relevant_scores[idx]
            best_col = int(np.argmax(scores_i))
            label = target_objects[best_col]
            per_ray_label.append(label)
            targets_found.append(label)

        from collections import Counter
        self.current_target = Counter(targets_found).most_common(1)[0][0]
        if self.current_target_pub is not None:
            self.current_target_pub.publish(String(data=self.current_target))

        self._filtered_indices = indices
        self._ray_origins = ray_origins
        self._ray_dirs = ray_dirs
        self._per_ray_label = per_ray_label
        return True

    def execute(self, cur_pose_np, waypoint_locked, target_waypoint1,
                target_waypoint2, publisher_dict):
        path_publisher = publisher_dict['path']

        origins = self._ray_origins[self._filtered_indices]   # (K, 3) FLU
        dirs = self._ray_dirs[self._filtered_indices]          # (K, 3) FLU
        labels = self._per_ray_label

        # Filter rays outside altitude range (z is altitude in FLU)
        alt_mask = (origins[:, 2] >= self.min_altitude) & (origins[:, 2] <= self.max_altitude)
        origins = origins[alt_mask]
        dirs = dirs[alt_mask]
        labels = [labels[i] for i in range(len(labels)) if alt_mask[i]]

        if len(origins) == 0:
            return waypoint_locked, target_waypoint1, target_waypoint2

        xy_dirs = dirs[:, :2]
        norms = np.linalg.norm(xy_dirs, axis=1, keepdims=True)
        xy_dirs_normed = xy_dirs / (norms + 1e-6)

        # Filter rays pointing behind the robot in XY.
        # Apply valid mask to all arrays so indices stay consistent (krrish-develop).
        orig_xy = origins[:, :2]
        ray_target_xy = orig_xy + xy_dirs_normed
        to_ray_target = ray_target_xy - cur_pose_np[:2]
        dot = np.einsum('ij,ij->i', xy_dirs_normed, to_ray_target)
        valid = dot > 0

        xy_dirs_normed = xy_dirs_normed[valid]
        origins = origins[valid]
        dirs = dirs[valid]
        labels = [labels[i] for i in range(len(labels)) if valid[i]]

        if len(origins) == 0:
            return waypoint_locked, target_waypoint1, target_waypoint2

        # Greedy spatial clustering per label (45 deg XY threshold)
        angle_cos_thresh = np.cos(np.deg2rad(45))
        groups = []
        for i in range(len(origins)):
            xy_dir = xy_dirs_normed[i]
            label = labels[i]
            assigned = False
            for group in groups:
                if group['label'] != label:
                    continue
                if np.dot(xy_dir, group['centroid']) >= angle_cos_thresh:
                    group['indices'].append(i)
                    group['rays'].append(xy_dir)
                    c = np.mean(group['rays'], axis=0)
                    group['centroid'] = c / (np.linalg.norm(c) + 1e-6)
                    assigned = True
                    break
            if not assigned:
                groups.append({
                    'centroid': xy_dir.copy(),
                    'rays': [xy_dir],
                    'indices': [i],
                    'label': label,
                })

        MIN_RAYS_PER_GROUP = 1
        groups = [g for g in groups if len(g['rays']) >= MIN_RAYS_PER_GROUP]

        if not groups:
            return waypoint_locked, target_waypoint1, target_waypoint2

        # Score groups: lower is better (dist - 5 * density)
        k = 5.0
        group_avgs = []
        for group in groups:
            idxs = group['indices']
            avg_origin = origins[idxs].mean(axis=0)
            avg_dir = dirs[idxs].mean(axis=0)
            avg_dir = avg_dir / (np.linalg.norm(avg_dir) + 1e-6)
            density = len(idxs)
            score = np.linalg.norm(avg_origin - cur_pose_np) - k * density
            group_avgs.append((avg_origin, avg_dir, density, group['label'], score))

        group_avgs.sort(key=lambda g: g[4])
        best = group_avgs[0]
        best_origin, best_dir = best[0], best[1]
        direction = best_dir / (np.linalg.norm(best_dir) + 1e-6)

        target_waypoint1 = best_origin + direction * 6.0
        target_waypoint2 = best_origin + direction * 12.0

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for wp in (target_waypoint1, target_waypoint2):
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(wp[0])
            ps.pose.position.y = float(wp[1])
            ps.pose.position.z = float(wp[2])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        path_publisher.publish(path)

        published_target = best[3]
        if self._active_target is None:
            self._active_target = published_target

        self._visualize_filtered_rays(groups, origins, dirs, publisher_dict)

        return waypoint_locked, target_waypoint1, target_waypoint2

    def _visualize_filtered_rays(self, groups, origins, dirs, publisher_dict):
        pub = publisher_dict['filtered_rays']
        self._clear_filtered_rays(pub)
        colors = [
            (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
            (1.0, 1.0, 0.0), (0.0, 1.0, 1.0), (1.0, 0.0, 1.0),
            (0.5, 0.5, 0.5), (1.0, 0.5, 0.0), (0.5, 0.0, 1.0),
            (0.0, 0.5, 0.5),
        ]
        arrow_length = 2.0
        marker_array = MarkerArray()
        j = 0
        for i, group in enumerate(groups):
            rr, gg, bb = colors[i % len(colors)]
            for idx in group['indices']:
                p0 = origins[idx]
                d = dirs[idx]
                p1 = p0 + arrow_length * (d / (np.linalg.norm(d) + 1e-6))
                arrow = Marker()
                arrow.header.frame_id = 'map'
                arrow.header.stamp = self.get_clock().now().to_msg()
                arrow.ns = 'arrows'
                arrow.id = j
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.points = [
                    Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])),
                    Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])),
                ]
                arrow.scale.x = 0.6
                arrow.scale.y = 1.2
                arrow.scale.z = 0.75
                arrow.color.r = rr
                arrow.color.g = gg
                arrow.color.b = bb
                arrow.color.a = 0.5
                marker_array.markers.append(arrow)
                j += 1
        self.prev_filtered_marker_ids = j
        pub.publish(marker_array)

    def _clear_filtered_rays(self, pub):
        if self.prev_filtered_marker_ids > 0:
            clear_array = MarkerArray()
            for i in range(self.prev_filtered_marker_ids):
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'arrows'
                m.id = i
                m.action = Marker.DELETE
                clear_array.markers.append(m)
            pub.publish(clear_array)
