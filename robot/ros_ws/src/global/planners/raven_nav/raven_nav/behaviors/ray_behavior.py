import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

from raven_nav.behaviors.frontier_behavior import _points_in_polygon


class RayBehavior:
    # Weight on bearing continuity (same direction as / origin moving along the
    # committed bearing) when picking which ray to follow.
    DIR_WEIGHT = 20.0

    def __init__(self, get_clock, current_target_publisher=None,
                 score_threshold=0.68,
                 min_altitude=1.5, max_altitude=100.0,
                 altitude_preference_weight=0.0):
        self.get_clock = get_clock
        self.name = 'Ray-based'
        self.score_threshold = score_threshold
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.altitude_pref_weight = altitude_preference_weight
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
                assigned_origin=None, assigned_dir=None,
                search_area_xy=None):
        path_publisher = publisher_dict['path']

        target = assigned_target if assigned_target is not None else self.assigned_target
        if target is None:
            return waypoint_locked, target_waypoint1, target_waypoint2

        groups = [g for g in self.ray_groups if g.label == target]
        if not groups:
            return waypoint_locked, target_waypoint1, target_waypoint2

        # Polygon constraint: reject any ray group whose origin OR investigation
        # waypoint (origin + dir*6 m) falls outside the search area. Without
        # this, the drone will happily fly across the polygon edge to chase a
        # ray pointing outward — operator drew the polygon for a reason.
        if search_area_xy is not None and search_area_xy.shape[0] >= 3:
            pts_xy = np.array([
                [g.avg_origin[0], g.avg_origin[1]] for g in groups
            ] + [
                [g.avg_origin[0] + g.avg_dir[0] * 6.0,
                 g.avg_origin[1] + g.avg_dir[1] * 6.0] for g in groups
            ], dtype=np.float64)
            inside = _points_in_polygon(pts_xy, search_area_xy)
            n = len(groups)
            origin_in = inside[:n]
            wp_in = inside[n:]
            groups = [g for g, oi, wi in zip(groups, origin_in, wp_in)
                      if oi and wi]
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

        # Weighted pick: prefer the ray with the same direction as the committed
        # bearing AND an origin moving along it (staying on one physical target),
        # falling back to the closest when nothing matches well.
        def _alt_reward(g):
            return self.altitude_pref_weight * max(
                float(g.avg_origin[2]) - self.min_altitude, 0.0)

        if (assigned_origin is not None and assigned_dir is not None):
            ao = np.asarray(assigned_origin, dtype=float)
            ad = np.asarray(assigned_dir, dtype=float)
            ad_n = ad / (np.linalg.norm(ad) + 1e-6)
            def _cost(g):
                c = float(g.avg_dist_to_robot)
                gd = np.asarray(g.avg_dir, dtype=float)
                gd_n = gd / (np.linalg.norm(gd) + 1e-6)
                c += self.DIR_WEIGHT * (1.0 - float(np.dot(gd_n, ad_n)))
                rel = np.asarray(g.avg_origin, dtype=float) - ao
                rn = float(np.linalg.norm(rel))
                if rn > 1.0:
                    c += self.DIR_WEIGHT * (1.0 - float(np.dot(rel / rn, ad_n)))
                return c - _alt_reward(g)
            best = min(candidates, key=_cost)
        else:
            k = 5.0
            best = min(candidates,
                       key=lambda g: g.avg_dist_to_robot - k * g.num_rays
                       - _alt_reward(g))

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
