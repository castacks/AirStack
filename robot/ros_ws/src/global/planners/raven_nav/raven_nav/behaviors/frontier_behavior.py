import torch
import numpy as np
from sklearn.cluster import DBSCAN
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class FrontierBehavior:
    def __init__(self, get_clock):
        self.get_clock = get_clock
        self.name = 'Frontier-based'

    def condition_check(self):
        return True

    def execute(self, frontiers_raw, cur_pose_np, waypoint_locked, target_waypoint,
                target_waypoint2, publisher_dict):
        viewpoint_publisher = publisher_dict['viewpoint']
        path_publisher = publisher_dict['path']

        if frontiers_raw is None or len(frontiers_raw) == 0:
            return waypoint_locked, target_waypoint, target_waypoint2

        # msg_serv/frontiers are published in RDF frame (no transform applied by RayFronts)
        # Convert RDF → FLU: world_x=rdf_z, world_y=-rdf_x, world_z=-rdf_y
        frontiers = np.stack([
            frontiers_raw[:, 2],
            -frontiers_raw[:, 0],
            -frontiers_raw[:, 1],
        ], axis=1)

        frontiers = frontiers[frontiers[:, 2] > 1.5]
        if frontiers.shape[0] == 0:
            return waypoint_locked, target_waypoint, target_waypoint2

        clustering = DBSCAN(eps=2.7, min_samples=3).fit(frontiers)
        labels = clustering.labels_
        unique_labels = [l for l in set(labels) if l != -1]
        viewpoints = []
        for l in unique_labels:
            cluster_pts = frontiers[labels == l]
            centroid = cluster_pts.mean(axis=0)
            if centroid[2] > 4.0 and centroid[2] < 10.0:
                viewpoints.append(centroid)

        if len(viewpoints) == 0:
            return waypoint_locked, target_waypoint, target_waypoint2

        viewpoints = np.stack(viewpoints)
        cent_msg = self._create_pointcloud2_msg(viewpoints)
        viewpoint_publisher.publish(cent_msg)

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

        top_n = 5
        num_candidates = min(top_n, viewpoints.shape[0])
        if num_candidates == 0:
            return waypoint_locked, target_waypoint, target_waypoint2

        top_indices = np.argsort(scores)[:num_candidates]
        best_idx = top_indices[np.random.randint(0, num_candidates)]
        best_cent = viewpoints[best_idx]

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
