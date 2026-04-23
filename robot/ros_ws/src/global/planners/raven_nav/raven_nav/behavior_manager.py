from raven_nav.behaviors.frontier_behavior import FrontierBehavior
from raven_nav.behaviors.ray_behavior import RayBehavior
from raven_nav.behaviors.voxel_behavior import VoxelBehavior


class BehaviorManager:
    def __init__(self, get_clock, publisher_dict, score_threshold=0.95,
                 min_altitude=1.5, max_altitude=100.0):
        self.behavior_mode = 'Frontier-based'
        self.get_clock = get_clock
        self.frontier_behavior = FrontierBehavior(self.get_clock,
                                                   min_altitude=min_altitude,
                                                   max_altitude=max_altitude)
        current_target_pub = publisher_dict.get('current_target')
        self.ray_behavior = RayBehavior(self.get_clock, current_target_pub,
                                        score_threshold=score_threshold,
                                        min_altitude=min_altitude,
                                        max_altitude=max_altitude)
        self.voxel_behavior = VoxelBehavior(self.get_clock)
        # Priority: Voxel (navigate to confirmed detections) > Ray > Frontier
        self.behaviors = [self.voxel_behavior, self.ray_behavior, self.frontier_behavior]

    @property
    def completed_queries(self) -> set:
        return self.voxel_behavior.completed_queries

    def mode_select(self, ray_origins, ray_dirs, ray_scores,
                    query_labels, target_objects,
                    vox_xyz=None, vox_scores=None):
        for behavior in self.behaviors:
            if behavior.name == 'Voxel-based':
                # Voxel-based navigation disabled — skip
                continue
            elif behavior.name == 'Ray-based':
                if behavior.condition_check(ray_origins, ray_dirs, ray_scores,
                                            query_labels, target_objects):
                    self.behavior_mode = behavior.name
                    return
            else:
                if behavior.condition_check():
                    self.behavior_mode = behavior.name
                    return

    def behavior_execute(self, behavior_mode, frontiers, cur_pose_np,
                         waypoint_locked, target_waypoint, target_waypoint2,
                         publisher_dict,
                         vox_xyz=None, vox_scores=None, query_labels=None):
        if behavior_mode == 'Voxel-based':
            return self.voxel_behavior.execute(
                vox_xyz, vox_scores, query_labels, cur_pose_np,
                waypoint_locked, target_waypoint, target_waypoint2, publisher_dict)
        elif behavior_mode == 'Ray-based':
            return self.ray_behavior.execute(
                cur_pose_np, waypoint_locked,
                target_waypoint, target_waypoint2, publisher_dict)
        else:
            return self.frontier_behavior.execute(
                frontiers, cur_pose_np, waypoint_locked,
                target_waypoint, target_waypoint2, publisher_dict)
