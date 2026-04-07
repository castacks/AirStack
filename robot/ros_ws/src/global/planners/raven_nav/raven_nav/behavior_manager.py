from raven_nav.behaviors.frontier_behavior import FrontierBehavior
from raven_nav.behaviors.ray_behavior import RayBehavior


class BehaviorManager:
    def __init__(self, get_clock, publisher_dict, score_threshold=0.95):
        self.behavior_mode = 'Frontier-based'
        self.get_clock = get_clock
        self.frontier_behavior = FrontierBehavior(self.get_clock)
        current_target_publisher = publisher_dict.get('current_target')
        self.ray_behavior = RayBehavior(self.get_clock, current_target_publisher,
                                        score_threshold=score_threshold)
        self.behaviors = [self.ray_behavior, self.frontier_behavior]

    def mode_select(self, ray_origins, ray_dirs, ray_scores,
                    query_labels, target_objects):
        for behavior in self.behaviors:
            if behavior.name == 'Ray-based':
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
                         publisher_dict):
        if behavior_mode == 'Frontier-based':
            return self.frontier_behavior.execute(
                frontiers, cur_pose_np, waypoint_locked,
                target_waypoint, target_waypoint2, publisher_dict)
        elif behavior_mode == 'Ray-based':
            return self.ray_behavior.execute(
                cur_pose_np, waypoint_locked,
                target_waypoint, target_waypoint2, publisher_dict)
        return waypoint_locked, target_waypoint, target_waypoint2
