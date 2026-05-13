import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String


class RayBehavior:
    def __init__(self, get_clock, current_target_publisher=None,
                 score_threshold=0.68,
                 min_altitude=1.5, max_altitude=100.0):
        self.get_clock = get_clock
        self.name = 'Ray-based'
        self.score_threshold = score_threshold
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.prev_filtered_marker_ids = 0
        self.current_target = None
        self.current_target_pub = current_target_publisher

        # Set externally each tick before mode_select / execute.
        self.ray_groups = []
        self.assigned_target = None

    def condition_check(self):
        return self.assigned_target is not None and any(
            g.label == self.assigned_target for g in self.ray_groups)

    def execute(self, cur_pose_np, waypoint_locked, target_waypoint1,
                target_waypoint2, publisher_dict, assigned_target=None,
                assigned_origin=None, assigned_dir=None):
        path_publisher = publisher_dict['path']

        target = assigned_target if assigned_target is not None else self.assigned_target
        if target is None:
            return waypoint_locked, target_waypoint1, target_waypoint2

        groups = [g for g in self.ray_groups if g.label == target]
        if not groups:
            return waypoint_locked, target_waypoint1, target_waypoint2

        # Prefer forward groups when any exist — avoid backtracking toward
        # an already-passed cluster. If the assigned target's rays are all
        # behind us, fall back to all groups so the drone turns toward them.
        forward_groups = [
            g for g in groups
            if np.dot(g.avg_dir[:2],
                      g.avg_origin[:2] + g.avg_dir[:2] - cur_pose_np[:2]) > 0
        ]
        candidates = forward_groups if forward_groups else groups

        self.current_target = target
        if self.current_target_pub is not None:
            self.current_target_pub.publish(String(data=target))

        # Auction may have picked a SPECIFIC group hypothesis (per-group bids).
        # If so, lock onto the candidate whose (avg_origin, avg_dir) is
        # closest to it — keeps us pursuing the same physical target across
        # ticks rather than thrashing to whichever ray group is densest now.
        if (assigned_origin is not None and assigned_dir is not None):
            ao = np.asarray(assigned_origin, dtype=float)
            ad = np.asarray(assigned_dir, dtype=float)
            ad_n = ad / (np.linalg.norm(ad) + 1e-6)
            def _affinity(g):
                # Combine origin proximity (m) and direction alignment (deg-ish).
                origin_d = float(np.linalg.norm(g.avg_origin - ao))
                gd = np.asarray(g.avg_dir, dtype=float)
                gd_n = gd / (np.linalg.norm(gd) + 1e-6)
                dot = float(np.clip(np.dot(gd_n, ad_n), -1.0, 1.0))
                # Lower = better. 1.0 - cos goes 0..2; scale to roughly match m.
                return origin_d + 20.0 * (1.0 - dot)
            best = min(candidates, key=_affinity)
        else:
            # Fallback: pick closest dense group (old behavior).
            k = 5.0
            best = min(candidates,
                       key=lambda g: g.avg_dist_to_robot - k * g.num_rays)

        target_waypoint1 = best.avg_origin + best.avg_dir * 6.0
        target_waypoint2 = best.avg_origin + best.avg_dir * 12.0

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

        self._visualize_filtered_rays(groups, publisher_dict)
        return waypoint_locked, target_waypoint1, target_waypoint2

    def _visualize_filtered_rays(self, groups, publisher_dict):
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
        for i, g in enumerate(groups):
            rr, gg, bb = colors[i % len(colors)]
            for k in range(g.num_rays):
                p0 = g.ray_origins[k]
                d = g.ray_dirs[k]
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
