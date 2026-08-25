"""Utilities for generating 2D map
"""
import numpy as np
import open3d as o3d
import cv2
from skimage import measure
import skimage.morphology
from PIL import Image

import conavgpt2.vendor.utils.pose as pu
from conavgpt2.vendor.utils.fmm_planner import FMMPlanner


# Outdoor drone scale needs different values from upstream's indoor 4 m depth:
# a 5 cm voxel over a 60 m scene is millions of points, and a 10 cm DBSCAN eps
# discards nearly all of them as noise. The ROS wrapper overwrites these.
# Per-call frontier-detection telemetry, read by the ROS wrapper after every
# tick. `Frontier_Det` keeps at most SIX frontiers and returns no indication
# that it dropped any, so without this there is no way to see the cap binding.
#
# NOTE the selection is ASCENDING BY AREA (`reverse=False`) and breaks at the
# 6th, so what upstream keeps is the six SMALLEST regions above threshold — the
# LARGEST frontiers, the ones with the most unexplored space behind them, are
# the ones discarded. Recorded here rather than changed: it is upstream's
# method. `areas_dropped` is what says whether it matters on a given map.
LAST_DET = {}

# How many frontiers `Frontier_Det` offers. Upstream hardcodes SIX (`if i == 5:
# break`) — one BEV image per frontier in the VLM prompt, so this is a prompt-
# size bound, not a mapping one. Six is reasonable for a Habitat apartment and
# arbitrary for a 1200 m plat, where the same six cover 40,000x the area. The
# node sets it from `max_frontiers`.
MAX_FRONTIERS = 6

# Which frontiers survive the cap when there are more than MAX_FRONTIERS.
#   'smallest' upstream: ascending by area, keep the first N. Indoors a small
#              frontier is usually a DOORWAY into an unexplored room, so this is
#              a sensible prior there.
#   'largest'  descending: keep the biggest. Outdoors from the air a large
#              frontier is open ground not yet covered and a small one is a gap
#              between two buildings, so the indoor prior inverts.
FRONTIER_ORDER = 'smallest'

SCENE_VOXEL_M = 0.05
SCENE_DBSCAN_EPS_M = 0.1
SCENE_DBSCAN_MIN_POINTS = 15


def build_full_scene_pcd(depth, image, cam_K):
    height, width = depth.shape

    # cx = (width - 1.) / 2.
    # cy = (height - 1.) / 2.
    # fx = (width / 2.) / np.tan(np.deg2rad(hfov / 2.))
    # # fy = (height / 2.) / np.tan(np.deg2rad(self.args.hfov / 2.))
    
    fx = cam_K.fx,
    fy = cam_K.fy, 
    cx = cam_K.cx, 
    cy = cam_K.cy

    x = np.arange(0, width, 1.0)
    y = np.arange(0, height, 1.0)
    u, v = np.meshgrid(x, y)
    
    # Apply the mask, and unprojection is done only on the valid points
    valid_mask = depth > 0
    masked_depth = depth[valid_mask]
    u = u[valid_mask]
    v = v[valid_mask]

    # Convert to 3D coordinates
    x = (u - cx) * masked_depth / fx
    y = (v - cy) * masked_depth / fy
    z = masked_depth

    # Stack x, y, z coordinates into a 3D point cloud
    points = np.stack((x, y, z), axis=-1)
    points = points.reshape(-1, 3)
    
    # Perturb the points a bit to avoid colinearity
    points += np.random.normal(0, 4e-3, points.shape)

    color_mask = np.repeat(valid_mask[:, :, np.newaxis], 3, axis=2)
    image_flat = image[color_mask].reshape(-1, 3)  # Flatten the image array for easier indexing
    colors = image_flat / 255.0  # Normalize the colors

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    camera_object_pcd = pcd.voxel_down_sample(SCENE_VOXEL_M)

    if SCENE_DBSCAN_MIN_POINTS > 0:
        labels = np.array(
            camera_object_pcd.cluster_dbscan(eps=SCENE_DBSCAN_EPS_M,
                                             min_points=SCENE_DBSCAN_MIN_POINTS)
        )

        non_outlier_idx = np.where(labels != -1)[0]
        camera_object_pcd = camera_object_pcd.select_by_index(non_outlier_idx)
    return camera_object_pcd


def detect_frontier(explored_map, obstacle_map, current_pose, threshold_point):
    # ------------------------------------------------------------------
    ##### Get the frontier map and score
    # ------------------------------------------------------------------
    map_size = explored_map.shape[0]
    edge_map = np.zeros((map_size, map_size))
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7, 7))
    dis_obstacle_map = obstacle_map
    obstacle_map = cv2.dilate(obstacle_map, kernel)

    kernel = np.ones((5, 5), dtype=np.uint8)
    show_ex = cv2.inRange(explored_map,0.1,1)
    free_map = cv2.morphologyEx(show_ex, cv2.MORPH_CLOSE, kernel)
    contours,_=cv2.findContours(free_map, cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    if len(contours)>0:
        contour = max(contours, key = cv2.contourArea)
        cv2.drawContours(edge_map,contour,-1,1,1)

    # clear the boundary
    edge_map[0:2, 0:map_size]=0.0
    edge_map[map_size-2:map_size, 0:map_size-1]=0.0
    edge_map[0:map_size, 0:2]=0.0
    edge_map[0:map_size, map_size-2:map_size]=0.0

    target_edge = edge_map - obstacle_map

    target_edge[target_edge>0.8]=1.0
    target_edge[target_edge!=1.0]=0.0

    img_label, num = measure.label(target_edge, connectivity=2, return_num=True)#输出二值图像中所有的连通域
    props = measure.regionprops(img_label)#输出连通域的属性，包括面积等

    # selem = skimage.morphology.disk(1)
    obstacle_map[current_pose[0], current_pose[1]] = 0
    selem = skimage.morphology.disk(1)
    traversible = skimage.morphology.binary_dilation(
        dis_obstacle_map, selem) != True
    # traversible = 1 - traversible
    planner = FMMPlanner(traversible)
    goal_pose_map = np.zeros((obstacle_map.shape))
    goal_pose_map[current_pose[0], current_pose[1]] = 1
    planner.set_multi_goal(goal_pose_map)

    Goal_edge = np.zeros((img_label.shape[0], img_label.shape[1]))
    Goal_point = []
    Goal_area_list = []
    dict_cost = {}
    for i in range(0, len(props)):

        if props[i].area > threshold_point:
            # dist = planner.fmm_dist[int(props[i].centroid[0]), int(props[i].centroid[1])]
            # dict_cost[i] = props[i].area
            # print(dist)
            # print(props[i].area)
            dict_cost[i] = planner.fmm_dist[int(props[i].centroid[0]), int(props[i].centroid[1])]
            # print(dict_cost[i])

    if dict_cost:
        dict_cost = sorted(dict_cost.items(), key=lambda x: x[1], reverse=False)

        for i, (key, value) in enumerate(dict_cost):
            if value == planner.fmm_dist.max():
                continue
            Goal_edge[img_label == key + 1] = i + 1
            Goal_point.append([int(props[key].centroid[0]), int(props[key].centroid[1])])
            Goal_area_list.append(value)
            if i == 5:
                break

    return  Goal_area_list, Goal_edge, Goal_point



class Global_Map_Proc():
    
    def __init__(self, args):
        self.args = args
        map_size = self.args.map_size_cm // self.args.map_resolution
        self.explored_map = np.zeros((map_size, map_size))
        self.obstacle_map = np.zeros((map_size, map_size))
        self.top_view_map = np.zeros((map_size, map_size, 3), dtype=np.uint8)
        self.z_buffer = np.full((map_size, map_size), -np.inf)
        
    def reset(self):
        self.explored_map.fill(0)
        self.obstacle_map.fill(0)
        self.top_view_map.fill(0)
        self.z_buffer.fill(-np.inf)
        
    def Map_Extraction(self, point_sum, camera_position_z, clean_diff = True):
        map_size = self.args.map_size_cm // self.args.map_resolution
        map_real_halfsize  = self.args.map_size_cm / 100.0 / 2.0
        explored_map = np.zeros((map_size, map_size))
        obstacle_map = np.zeros((map_size, map_size))
        
        z_min = camera_position_z - self.args.map_height_cm / 100.0 /2.0
        z_max = camera_position_z + self.args.map_height_cm / 100.0 /2.0 #* 2.5
        
        points = np.asarray(point_sum.points)
        colors = np.asarray(point_sum.colors)
        
        common_mask = (
            (points[:, 0] >=  -map_real_halfsize) &
            (points[:, 0] <=   map_real_halfsize) &
            (points[:, 2] >=  -map_real_halfsize) &
            (points[:, 2] <=   map_real_halfsize)
        )

        mask_obstacle = common_mask & ((points[:, 1] >= z_min) & (points[:, 1] <= z_max))
        mask_explored = common_mask & (points[:, 1] <= z_max)
        
        points_obstacle = points[mask_obstacle]
        points_explored = points[mask_explored]
        colors_explored = colors[mask_explored]
        
        obs_i_values = np.floor((points_obstacle[:, 0])*100 / self.args.map_resolution).astype(int) + int(map_size/2)
        obs_j_values = np.floor((points_obstacle[:, 2])*100 / self.args.map_resolution).astype(int) + int(map_size/2)

        obstacle_map[obs_i_values, obs_j_values] = 1
        self.obstacle_map[obs_i_values, obs_j_values] = 1
        
        exp_i_values = np.floor((points_explored[:, 0])*100 / self.args.map_resolution).astype(int) + int(map_size/2)
        exp_j_values = np.floor((points_explored[:, 2])*100 / self.args.map_resolution).astype(int) + int(map_size/2)
        
        explored_map[exp_i_values, exp_j_values] = 1
        self.explored_map[exp_i_values, exp_j_values] = 1
        
        diff_ob_ex = explored_map - obstacle_map
        if clean_diff:
            self.obstacle_map[diff_ob_ex == 1] = 0
        
        # top_view_map = np.zeros((map_size, map_size, 3), dtype=np.uint8)
        # z_buffer = np.full((map_size, map_size), -np.inf)
        for i in range(len(points_explored)):
            if points_explored[i, 1] > self.z_buffer[exp_i_values[i], exp_j_values[i]]:
                self.z_buffer[exp_i_values[i], exp_j_values[i]] = points_explored[i, 1]
                self.top_view_map[exp_i_values[i], exp_j_values[i]] = (colors_explored[i] * 255).astype(np.uint8)
        
        return self.obstacle_map, self.explored_map, self.top_view_map
    
    
    def Frontier_Det(self, threshold_point):
        
        map_size = self.explored_map.shape[0]
        edge_map = np.zeros((map_size, map_size))
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7, 7))
        dis_obstacle_map = self.obstacle_map
        obstacle_map = cv2.dilate(self.obstacle_map, kernel)

        kernel = np.ones((5, 5), dtype=np.uint8)
        show_ex = cv2.inRange(self.explored_map,0.1,1)
        free_map = cv2.morphologyEx(show_ex, cv2.MORPH_CLOSE, kernel)
        contours,_=cv2.findContours(free_map, cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        if len(contours)>0:
            contour = max(contours, key = cv2.contourArea)
            cv2.drawContours(edge_map,contour,-1,1,1)

        # clear the boundary
        edge_map[0:2, 0:map_size]=0.0
        edge_map[map_size-2:map_size, 0:map_size-1]=0.0
        edge_map[0:map_size, 0:2]=0.0
        edge_map[0:map_size, map_size-2:map_size]=0.0

        target_edge = edge_map - obstacle_map

        target_edge[target_edge>0.8]=1.0
        target_edge[target_edge!=1.0]=0.0
        
        img_label, num = measure.label(target_edge, connectivity=2, return_num=True)#输出二值图像中所有的连通域
        props = measure.regionprops(img_label)#输出连通域的属性，包括面积等
        
        Goal_edge = np.zeros((img_label.shape[0], img_label.shape[1]))
        Goal_point = []
        Goal_area_list = []
        dict_cost = {}
        for i in range(0, len(props)):
            if props[i].area > threshold_point:
                dict_cost[i] = props[i].area

        # Telemetry only — the selection below is untouched. Same shape as
        # chat_utils.LAST_CALL: upstream discards the fact that it dropped
        # anything, so there is no way to know from the outside whether the
        # six-frontier cap is binding.
        LAST_DET.clear()
        LAST_DET.update({
            "n_regions": len(props),
            "n_above_threshold": len(dict_cost),
            "n_offered": min(len(dict_cost), MAX_FRONTIERS),
            "max_frontiers": MAX_FRONTIERS,
            "order": FRONTIER_ORDER,
            "threshold_point": threshold_point,
        })

        if dict_cost:
            dict_cost = sorted(dict_cost.items(), key=lambda x: x[1],
                               reverse=(FRONTIER_ORDER == 'largest'))
            LAST_DET["areas_all_sorted"] = [v for _, v in dict_cost]
            LAST_DET["areas_offered"] = [v for _, v in dict_cost[:MAX_FRONTIERS]]
            LAST_DET["areas_dropped"] = [v for _, v in dict_cost[MAX_FRONTIERS:]]

            for i, (key, value) in enumerate(dict_cost):
                Goal_edge[img_label == key + 1] = i + 1
                Goal_point.append([int(props[key].centroid[0]), int(props[key].centroid[1])])
                Goal_area_list.append(value)
                if i + 1 >= MAX_FRONTIERS:
                    break

        return  Goal_area_list, Goal_edge, Goal_point
