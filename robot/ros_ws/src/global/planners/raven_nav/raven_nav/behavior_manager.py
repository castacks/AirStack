from raven_nav.behaviors.frontier_behavior import FrontierBehavior
from raven_nav.behaviors.ray_behavior import RayBehavior
from raven_nav.behaviors.voxel_behavior import VoxelBehavior
from raven_nav.behaviors.vlfm_behavior import VLFMBehavior


class BehaviorManager:
    def __init__(self, get_clock, publisher_dict, score_threshold=0.75,
                 min_altitude=1.5, max_altitude=100.0,
                 voxel_score_threshold=0.9, voxel_min_cluster_size=30,
                 voxel_confirm_hits=3, voxel_track_max_misses=4,
                 voxel_proximity_engage_m=12.0, voxel_min_confidence=0.0):
        self.behavior_mode = 'Frontier-based'
        self.get_clock = get_clock
        self.frontier_behavior = FrontierBehavior(
            self.get_clock, min_altitude=min_altitude, max_altitude=max_altitude)
        current_target_pub = publisher_dict.get('current_target')
        self.ray_behavior = RayBehavior(self.get_clock, current_target_pub,
                                        score_threshold=score_threshold,
                                        min_altitude=min_altitude,
                                        max_altitude=max_altitude)
        self.voxel_behavior = VoxelBehavior(self.get_clock,
                                            score_threshold=voxel_score_threshold,
                                            min_cluster_size=voxel_min_cluster_size,
                                            confirm_hits=voxel_confirm_hits,
                                            track_max_misses=voxel_track_max_misses,
                                            proximity_engage_m=voxel_proximity_engage_m,
                                            min_confidence=voxel_min_confidence)
        # VLFM baseline: force-selected by the node's vlfm_baseline flag, so it is
        # NOT in the priority list (same as Frontier under frontier_only_baseline).
        self.vlfm_behavior = VLFMBehavior(
            self.get_clock, min_altitude=min_altitude, max_altitude=max_altitude,
            voxel_score_threshold=voxel_score_threshold,
            voxel_min_cluster_size=voxel_min_cluster_size)
        # Priority order: Voxel > Ray > Frontier.
        self.behaviors = [self.voxel_behavior, self.ray_behavior, self.frontier_behavior]

    @property
    def completed_queries(self) -> set:
        return self.voxel_behavior.completed_queries

    def mode_select(self, query_labels, target_objects,
                    vox_xyz=None, vox_scores=None,
                    committed_origin=None, committed_dir=None, cur_pose=None,
                    committed_bb_center=None):
        for behavior in self.behaviors:
            if behavior.name == 'Voxel-based':
                if behavior.condition_check(
                        vox_xyz, vox_scores, query_labels, target_objects,
                        committed_origin=committed_origin,
                        committed_dir=committed_dir, cur_pose=cur_pose,
                        committed_bb_center=committed_bb_center):
                    self.behavior_mode = behavior.name
                    return
            else:
                if behavior.condition_check():
                    self.behavior_mode = behavior.name
                    return

    def behavior_execute(self, behavior_mode, frontiers, cur_pose_np,
                         waypoint_locked, target_waypoint, target_waypoint2,
                         publisher_dict,
                         vox_xyz=None, vox_scores=None, query_labels=None,
                         peer_state=None, my_id=0, search_area_xy=None,
                         debug_logger=None, assigned_target=None,
                         committed_target_dir=None,
                         committed_target_origin=None,
                         completed_zones_xy=None, cell_size_m=0.5,
                         committed_bb_center=None, peer_weights=None,
                         ray_origins=None, ray_scores=None, ray_dirs=None,
                         target_objects=None):
        if behavior_mode == 'VLFM-based':
            return self.vlfm_behavior.execute(
                ray_origins, ray_scores, ray_dirs, vox_xyz, vox_scores,
                query_labels, target_objects, cur_pose_np, waypoint_locked,
                target_waypoint, target_waypoint2, publisher_dict,
                search_area_xy=search_area_xy, debug_logger=debug_logger)
        if behavior_mode == 'Voxel-based':
            return self.voxel_behavior.execute(
                vox_xyz, vox_scores, query_labels, cur_pose_np,
                waypoint_locked, target_waypoint, target_waypoint2, publisher_dict,
                committed_origin=committed_target_origin,
                committed_dir=committed_target_dir,
                committed_bb_center=committed_bb_center)
        elif behavior_mode == 'Ray-based':
            return self.ray_behavior.execute(
                cur_pose_np, waypoint_locked,
                target_waypoint, target_waypoint2, publisher_dict,
                assigned_target=assigned_target,
                assigned_origin=committed_target_origin,
                assigned_dir=committed_target_dir,
                search_area_xy=search_area_xy)
        else:
            return self.frontier_behavior.execute(
                frontiers, cur_pose_np, waypoint_locked,
                target_waypoint, target_waypoint2, publisher_dict,
                peer_state=peer_state, my_id=my_id,
                search_area_xy=search_area_xy,
                debug_logger=debug_logger,
                committed_target_dir=committed_target_dir,
                committed_target_origin=committed_target_origin,
                completed_zones_xy=completed_zones_xy,
                cell_size_m=cell_size_m,
                peer_weight=peer_weights)
