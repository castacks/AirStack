#!/usr/bin/env python3
import math
import time
import os

import torch
import open3d as o3d

from PIL import Image
import yaml
# Dead in this file: upstream's ros_multi_nav.py used these, ROS_Agent never
# does. Guarded so the package does not need tf_transformations/transforms3d.
try:
    from tf_transformations import quaternion_matrix, translation_matrix
except ImportError:  # pragma: no cover
    quaternion_matrix = translation_matrix = None

import numpy as np
import cv2
import skimage.morphology
from conavgpt2.vendor.utils.general_utils import (
    get_camera_K
)
from conavgpt2.vendor.utils.detection_segmentation import Object_Detection_and_Segmentation

from conavgpt2.vendor.constants import color_palette, category_to_id #, category_to_id_replica
from conavgpt2.vendor.utils.visualization import init_vis_image, draw_line, vis_result_fast
from conavgpt2.vendor.utils.explored_map_utils import (
    build_full_scene_pcd
)
from conavgpt2.vendor.utils.mapping import create_object_pcd, process_pcd
from conavgpt2.vendor.utils.fmm_planner import FMMPlanner
from conavgpt2.vendor.agents.vlm_multi_agents import VLM_Agent
from conavgpt2.vendor.utils.explored_map_utils import detect_frontier
  

FORWARD_KEY="w"
LEFT_KEY="a"
RIGHT_KEY="d"
UP_KEY="q"
DOWN_KEY="e"
FINISH="f"


def transform_rgb_bgr(image):
    return image[:, :, [2, 1, 0]]


def load_config(file_path):
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)
    return config


class ROS_Agent(VLM_Agent):
    def __init__(self, args, agent_id, goal_name, send_queue, receive_queue) -> None:
        
        # ------------------------------------------------------------------
        ##### Initialize basic config
        # ------------------------------------------------------------------
        self.args = args
        self.agent_id = agent_id
        self.episode_n = 0
        print("init agent " + str(agent_id) )
        np.random.seed(args.seed)
        torch.manual_seed(args.seed)
        
        
        self.send_queue = send_queue
        self.receive_queue = receive_queue

        if args.cuda:
            torch.cuda.manual_seed(args.seed)

        self.device = getattr(self.args, "device", None) \
            or "cuda:{}".format(self.args.gpu_id)
        self.dump_dir = "{}/dump/{}/".format(args.dump_location, args.nav_mode)

        if not os.path.exists(self.dump_dir):
            os.makedirs(self.dump_dir)

        # ------------------------------------------------------------------
        ##### Initialize the perception model
        # ------------------------------------------------------------------
        self.classes = ["chair", "bed", "potted plant", "toilet", "tv_screen", "couch", "person", "sink"]
        
        self.obj_det_seg = Object_Detection_and_Segmentation(self.args, self.classes, self.device)
        
        self.annotated_image = None
        self.vis_map = None
        
        self.open3d_pose = []


        # 3D mapping
        # self.camera_K = get_camera_K(
        #     self.args.frame_width, self.args.frame_height, self.args.hfov)

        self.init_map_and_pose()
        # ------------------------------------------------------------------
        
            
        # ------------------------------------------------------------------
        ##### Initialize navigation
        # ------------------------------------------------------------------
          
        self.goal_name = goal_name
        self.goal_id = self.classes.index(goal_name)
        
        self.args.turn_angle = 30
        self.init_map_and_navigation_param()        
        
        # self.init_sim_position = init_position[:3, 3]
        # self.init_sim_rotation = init_position[:3, :3]
        # ------------------------------------------------------------------

        
    def init_map_and_navigation_param(self):
        
        # 3D mapping
        self.point_sum = o3d.geometry.PointCloud()
        self.object_pcd = o3d.geometry.PointCloud()
    
        self.init_sim_position = None
        self.init_sim_rotation = None
        self.Open3D_traj = []
        self.plan_path = []
        self.nearest_point = None
        self.current_grid_pose = None
        self.camera_position = None
        
        self.relative_angle = 0
        self.eve_angle = 0

        # navigation
        self.l_step = 0
        
        self.no_frontiers_count = 0
        self.curr_frontier_count = 0
        self.greedy_stop_count = 0
        self.replan_count = 0

        self.is_running = True
        self.found_goal = False
        self.last_action = 0
        self.last_goal = None
        
        self.upstair_flag = False
        self.downstair_flag = False
        self.another_floor = False
        self.clean_diff = True

     


    def mapping(self, observations):
        time_step_info = 'Mapping time (s): \n'

        preprocess_s_time = time.time()
        
        # ------------------------------------------------------------------
        ##### At first step, get the object name and init the visualization
        # ------------------------------------------------------------------
        if self.l_step == 0:
            # self.init_sim_position = observations['pose'][:3, 3]
            # self.init_sim_rotation = observations['pose'][:3, :3]

            # self.goal_name = "chair"
            # self.goal_id = 0
            
            self.camera_K = observations['cam_K']

        # ------------------------------------------------------------------
        ##### Preprocess the observation
        # ------------------------------------------------------------------
        proc_time = time.time()
        image_rgb = observations['rgb']
        depth = observations['depth']
        image = transform_rgb_bgr(image_rgb) 
        self.annotated_image = image
        
        depth = self._preprocess_depth(depth)
        
        camera_matrix_T = self.get_transform_matrix(observations['pose'])
        self.camera_position = camera_matrix_T[:3, 3]
        self.Open3D_traj.append(camera_matrix_T)
        self.relative_angle = round(np.arctan2(camera_matrix_T[2][0], camera_matrix_T[0][0])* 57.29577951308232 + 180)
        # print("self.relative_angle: ", self.relative_angle)
        
        
        detections = self.obj_det_seg.detect(image) 
        
        n_masks = len(detections.xyxy)
        for mask_idx in range(n_masks):
            if self.goal_id == detections.class_id[mask_idx] and (detections.confidence[mask_idx] > self.args.sem_threshold or ('plant' in self.goal_name and detections.confidence[mask_idx] > 0.5)):
                mask = detections.mask[mask_idx]

                # make the pcd and color it
                camera_object_pcd = create_object_pcd(
                    depth,
                    mask,
                    self.camera_K,
                    image,
                    obj_color = None
                )
                
                if len(camera_object_pcd.points) < 10: 
                    continue
                
                camera_object_pcd.transform(camera_matrix_T)
                # camera_object_pcd = process_pcd(camera_object_pcd)
                
                self.object_pcd += camera_object_pcd

        proc_end_time = time.time()
        # print('proc_time: %.3f秒'%(proc_end_time-proc_time))
        # ------------------------------------------------------------------

        
        # ------------------------------------------------------------------
        ##### 2D Obstacle Map
        # ------------------------------------------------------------------
        map_time = time.time()
        
        local_grid_pose = [self.camera_position[0]*100/self.args.map_resolution + int(self.origins_grid[0]), 
                      self.camera_position[2]*100/self.args.map_resolution + int(self.origins_grid[1])]
        pose_x = max(1, min(int(local_grid_pose[0]), self.map_size - 1))
        pose_y = max(1, min(int(local_grid_pose[1]), self.map_size - 1))
        
        # # Adjust the centriod of the map when the robot move to the edge of the map
        # if pose_x < 100:
        #     self.move_map_and_pose(shift = 100, axis=0)
        #     pose_x += 100
        # elif pose_x > self.map_size - 100:
        #     self.move_map_and_pose(shift = -100, axis=0)
        #     pose_x -= 100
        # elif pose_y < 100:
        #     self.move_map_and_pose(shift = 100, axis=1)
        #     pose_y += 100
        # elif pose_y > self.map_size - 100:
        #     self.move_map_and_pose(shift = -100, axis=1)
        #     pose_y -= 100
        
        self.current_grid_pose = [pose_x, pose_y]
        
        # visualize trajectory
        self.visited_vis = draw_line(self.last_grid_pose, self.current_grid_pose, self.visited_vis)
        self.last_grid_pose = self.current_grid_pose
        
        full_scene_pcd = build_full_scene_pcd(depth, image_rgb, self.camera_K)
        
        # build 3D pc map
        full_scene_pcd.transform(camera_matrix_T)
        # full_scene_pcd = self.remove_full_points_cell(full_scene_pcd, self.camera_position)
        full_scene_pcd.voxel_down_sample(0.05)
        
        for pose in self.open3d_pose:
            full_scene_pcd = self.remove_robot_points_cell(full_scene_pcd, pose)
        
        self.point_sum += full_scene_pcd
      
        map_end_time = time.time()
        # print('map_time: %.3f秒'%(map_end_time-map_time))
        
           
        if self.args.visualize or self.args.print_images:
            self.annotated_image  = vis_result_fast(image, detections, self.classes)
            
            self.vis_map = self._visualize(self.obstacle_map, self.explored_map, self.goal_map, self.goal_name)
            
        if self.args.visualize:
            Open3d_goal_pose = []
            self.receive_queue.put([self.agent_id, 
                                image_rgb, 
                                depth, 
                                self.annotated_image , 
                                transform_rgb_bgr(self.vis_map),
                                np.asarray(self.point_sum.points), 
                                np.asarray(self.point_sum.colors), 
                                self.Open3D_traj,
                                self.episode_n,
                                self.plan_path,
                                Open3d_goal_pose,
                                time_step_info]
                                )    
            
        # self.l_step += 1
        # ------------------------------------------------------------------
        
    def act(self, goal_points: list, pose_all: list):
        
        # ------------------------------------------------------------------
        ##### Goal selection
        # ------------------------------------------------------------------
        act_time = time.time()
        if len(self.object_pcd.points) > 0:
            if self.found_goal == False:
                self.goal_map = np.zeros((self.local_w, self.local_h))
            goal_pcd = process_pcd(self.object_pcd)
            self.goal_map[self.object_map_building(goal_pcd)] = 1
            self.nearest_point = self.find_nearest_point_cloud(goal_pcd, self.camera_position)
                       
            self.found_goal = True
        else:
            self.found_goal = False
            
            self.goal_map = np.zeros((self.local_w, self.local_h))
            self.goal_map[goal_points[0], goal_points[1]] = 1
            
  
        # plan a path by fmm
        self.stg, self.stop, plan_path = self._get_stg(self.obstacle_map, self.current_grid_pose, pose_all, np.copy(self.goal_map))
        plan_path = np.array(plan_path) 
        # print(plan_path)
        plan_path_x = (plan_path[:, 0] - int(self.origins_grid[0])) * self.args.map_resolution / 100.0
        plan_path_y = plan_path[:, 0] * 0
        plan_path_z = (plan_path[:, 1] - int(self.origins_grid[1])) * self.args.map_resolution / 100.0

        self.plan_path = np.stack((plan_path_x, plan_path_y, plan_path_z), axis=-1)
        # print(self.plan_path)

        action = self.ffm_act()

        self.last_action = action
        
        act_end_time = time.time()
        # print('act_time: %.3f秒'%(act_end_time - act_time)) 
        return action
    
    def ffm_act(self):
        if self.is_running == False:
            return None
        
        if self.stop and self.found_goal:
            action = 0
        else:
            (stg_x, stg_y) = self.stg
            angle_st_goal = math.degrees(math.atan2(stg_x - self.current_grid_pose[0],
                                                    stg_y - self.current_grid_pose[1]))
            angle_agent = (360 - self.relative_angle) % 360.0
            if angle_agent > 180:
                angle_agent -= 360
            # angle_agent = 360 - self.relative_angle
            relative_angle = angle_agent - angle_st_goal
            if relative_angle > 180:
                relative_angle -= 360
            if relative_angle < -180:
                relative_angle += 360

            if relative_angle > self.args.turn_angle:
                action = 3  # Right
            elif relative_angle < -self.args.turn_angle:
                action = 2  # Left
            else:
                action = 1
                
        self.l_step += 1
        action_e_time = time.time()
        # print('action: %.3f秒'%(action_e_time - action_s_time)) 
        return action

    def _get_stg(self, grid, start, pose_all, goal):
        """Get short-term goal"""

        [gx1, gx2, gy1, gy2] = [0, self.local_w, 0, self.local_h] 

        x1, y1, = 0, 0
        x2, y2 = grid.shape

        # print("grid: ", grid.shape)

        def add_boundary(mat, value=1):
            h, w = mat.shape
            new_mat = np.zeros((h + 2, w + 2)) + value
            new_mat[1:h + 1, 1:w + 1] = mat
            return new_mat

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3, 3))
        erode_grid = cv2.erode(grid[x1:x2, y1:y2], kernel, iterations=1)
        
        selem = skimage.morphology.disk(10)
        traversible = skimage.morphology.binary_dilation(
            erode_grid,
            selem) != True
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3, 3))
        traversible[cv2.dilate(self.visited_vis[gx1:gx2, gy1:gy2][x1:x2, y1:y2], kernel) == 1] = 1
        traversible[cv2.dilate(self.collision_map[gx1:gx2, gy1:gy2]
                    [x1:x2, y1:y2], kernel) == 1] = 0
        traversible[int(start[0] - x1) - 1:int(start[0] - x1) + 2,
                    int(start[1] - y1) - 1:int(start[1] - y1) + 2] = 1

        # print("current_pose: ", start)
        for i in range(len(pose_all)):
            if i != self.agent_id:
                other_pose = pose_all[i]
                # print(i, other_pose)
                traversible[int(other_pose[0] - x1) - 15:int(other_pose[0] - x1) + 15,
                    int(other_pose[1] - y1) - 15:int(other_pose[1] - y1) + 15] = 0
                
        traversible = add_boundary(traversible)
        goal = add_boundary(goal, value=0)
        traversible[goal==1] = 1
        planner = FMMPlanner(traversible)
        if ("plant" in self.goal_name or "tv" in self.goal_name) and \
            np.sum(self.goal_map) > 1:
            selem = skimage.morphology.disk(15)
        else: 
            selem = skimage.morphology.disk(12)
        goal = skimage.morphology.binary_dilation(
            goal, selem) != True
        goal = 1 - goal * 1.
        planner.set_multi_goal(goal)

        path = []
        path.append([start[0], start[1]])

        state = [start[0] - x1, start[1] - y1]
        stg_x, stg_y, replan, stop_f = planner.get_short_term_goal(state)
        stg_x, stg_y = stg_x + x1 , stg_y + y1 
        for i in range(10):
            state = [stg_x - x1 , stg_y - y1 ]
            stg_x, stg_y, replan, stop = planner.get_short_term_goal(state)
            stg_x, stg_y = stg_x + x1 , stg_y + y1 
            
            path.append([stg_x, stg_y])
            if stop:
                break

        return (path[1][0], path[1][1]), stop_f, path
    


    def _preprocess_depth(self, depth, min_d=0.5, max_d=4.0):

        depth = depth / 1000.0

        # for i in range(depth.shape[1]):
        #     depth[:, i][depth[:, i] == 0.] = depth[:, i].max()

        mask = depth > max_d
        depth[mask] = 0.0

        depth[depth.shape[0]-50:, :] = 0.0
        depth[:, depth.shape[1]-50:] = 0.0
        depth[:50, :] = 0.0
        depth[:, :50] = 0.0

        return depth 


    def get_transform_matrix(self, pose_matrix):
        """
        transform the habitat-lab space to Open3D space (initial pose in habitat)
        habitat-lab space need to rotate camera from x,y,z to  x, -y, -z
        Returns Pose_diff, R_diff change of the agent relative to the initial timestep
        """
        T_world_open3d = np.eye(4)
        T_world_open3d[:3, :3] = np.array([[0, 0, 1],
                    [0, -1, 0],
                    [1, 0, 0]])
        
        T_init = np.eye(4)
        T_init[:3, :3] = self.init_sim_rotation
        T_init[:3, 3] = self.init_sim_position

        # O_camera_matrix = T_world_open3d @ pose_matrix 
        O_camera_matrix = T_world_open3d @ np.linalg.inv(T_init) @ pose_matrix 

        return O_camera_matrix
          
    def remove_robot_points_cell(self, point_sum, robot_position, radius = 0.8):
        """
        remove the influence of each other's robot pointcloud
        """
        
        points = np.asarray(point_sum.points)
        colors = np.asarray(point_sum.colors)

        mask = (((points[:, 0] >= robot_position[0] + radius) | \
                (points[:, 0] <= robot_position[0] - radius) | \
                (points[:, 2] >= robot_position[2] + radius) | \
                (points[:, 2] <= robot_position[2] - radius) )) | (points[:, 1] <= robot_position[1] - 0.44)

        points_filtered = points[mask]
        colors_filtered = colors[mask]
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_filtered)
        pcd.colors = o3d.utility.Vector3dVector(colors_filtered)

        # print("remove_robot_points_cell ", robot_position)
        return pcd
        
        
        
    def remove_full_points_cell(self, point_sum, camera_position):
        points = np.asarray(point_sum.points)
        colors = np.asarray(point_sum.colors)

        # mask = (points[:, 1] <= camera_position[1] + 0.5 )
        mask = (points[:, 1] <= camera_position[1] + 0.5 )

        points_filtered = points[mask]
        colors_filtered = colors[mask]
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_filtered)
        pcd.colors = o3d.utility.Vector3dVector(colors_filtered)

        return pcd
        
   
    def save_rgbd_image(self, rgb_image, depth):
        vis_image_rgb = np.ones((480, 1280, 3)).astype(np.uint8) * 255
        vis_image_rgb[0:480, 0:640] = rgb_image 
        # Normalize the depth values to the range 0-255
        depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # Apply a colormap (e.g., COLORMAP_JET)
        depth_color = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

        vis_image_rgb[0:480, 640:1280] = depth_color
        
        ep_dir = '{}episodes/{}/eps_rgbd_{}/'.format(
            self.dump_dir, self.args.rank, self.episode_n)
        if not os.path.exists(ep_dir):
            os.makedirs(ep_dir)
        fn = ep_dir + 'Vis-{}.png'.format(self.l_step)
        cv2.imwrite(fn, vis_image_rgb)
        
    def save_similarity_map(self, map):
        depth_normalized = cv2.normalize(map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # Apply a colormap (e.g., COLORMAP_JET)
        depth_color = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

        ep_dir = '{}episodes/{}/eps_rgbd_{}/'.format(
            self.dump_dir, self.args.rank, self.episode_n)
        if not os.path.exists(ep_dir):
            os.makedirs(ep_dir)
        fn = ep_dir + 'Vis-simi-{}.png'.format(self.l_step)
        cv2.imwrite(fn, depth_color)
    
    def _visualize(self, map_pred, exp_pred, goal_map, text_queries):

        # start_x, start_y, start_o = pose

        sem_map = np.zeros((self.local_w, self.local_h))

        # no_cat_mask = sem_map == 20
        map_mask = map_pred == 1
        exp_mask = exp_pred == 1
        vis_mask = self.visited_vis == 1
        # edge_mask = map_edge == 1

        # sem_map[no_cat_mask] = 0
        # m1 = np.logical_and(no_cat_mask, exp_mask)
        sem_map[exp_mask] = 2

        # m2 = np.logical_and(no_cat_mask, map_mask)
        sem_map[map_mask] = 1

        sem_map[vis_mask] = 3
        # sem_map[edge_mask] = 3

        selem = skimage.morphology.disk(4)
        goal_mat = 1 - skimage.morphology.binary_dilation(
            goal_map, selem) != True

        goal_mask = goal_mat == 1
        sem_map[goal_mask] = 4
        if np.sum(goal_map) == 1:
            f_pos = np.argwhere(goal_map == 1)
            # fmb = get_frontier_boundaries((f_pos[0][0], f_pos[0][1]))
            # goal_fmb = skimage.draw.circle_perimeter(int((fmb[0]+fmb[1])/2), int((fmb[2]+fmb[3])/2), 23)
            goal_fmb = skimage.draw.circle_perimeter(f_pos[0][0], f_pos[0][1], int(self.map_size/16 -1))
            goal_fmb[0][goal_fmb[0] > self.map_size-1] = self.map_size-1
            goal_fmb[1][goal_fmb[1] > self.map_size-1] = self.map_size-1
            goal_fmb[0][goal_fmb[0] < 0] = 0
            goal_fmb[1][goal_fmb[1] < 0] = 0
            # goal_fmb[goal_fmb < 0] =0
            goal_mask[goal_fmb[0], goal_fmb[1]] = 1
            sem_map[goal_mask] = 4


        color_pal = [int(x * 255.) for x in color_palette]
        sem_map_vis = Image.new("P", (sem_map.shape[1],
                                      sem_map.shape[0]))
        sem_map_vis.putpalette(color_pal)
        sem_map_vis.putdata(sem_map.flatten().astype(np.uint8))
        sem_map_vis = sem_map_vis.convert("RGB")
        sem_map_vis = np.flipud(sem_map_vis)

        sem_map_vis = sem_map_vis[:, :, [2, 1, 0]]
        vis_image = cv2.resize(sem_map_vis, (480, 480),
                                 interpolation=cv2.INTER_NEAREST)

       
        def get_contour_points(pos, origin, size=20):
            x, y, o = pos
            pt1 = (int(x) + origin[0],
                int(y) + origin[1])
            pt2 = (int(x + size / 1.5 * np.cos(o + np.pi * 4 / 3)) + origin[0],
                int(y + size / 1.5 * np.sin(o + np.pi * 4 / 3)) + origin[1])
            pt3 = (int(x + size * np.cos(o)) + origin[0],
                int(y + size * np.sin(o)) + origin[1])
            pt4 = (int(x + size / 1.5 * np.cos(o - np.pi * 4 / 3)) + origin[0],
                int(y + size / 1.5 * np.sin(o - np.pi * 4 / 3)) + origin[1])

            return np.array([pt1, pt2, pt3, pt4])

        pos = [int(self.last_grid_pose[1]*480.0/self.map_size), int((self.map_size-self.last_grid_pose[0])*480.0/self.map_size), np.deg2rad(self.relative_angle)]
        agent_arrow = get_contour_points(pos, origin=(0, 0), size=10)
        color = (int(color_palette[11] * 255),
                 int(color_palette[10] * 255),
                 int(color_palette[9] * 255))
        cv2.drawContours(vis_image, [agent_arrow], 0, color, -1)

        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        color = (20, 20, 20)  # BGR
        thickness = 2
        text = "Find {} ".format(text_queries)
        textsize = cv2.getTextSize(text, font, fontScale, thickness)[0]
        textX = (480 - textsize[0]) // 2 + 30
        textY = (50 + textsize[1]) // 2
        vis_image_show = cv2.putText(vis_image, text, (textX, textY),
                                font, fontScale, color, thickness,
                                cv2.LINE_AA)

        vis_image_rgb = init_vis_image(text_queries, self.last_action)
        vis_image_rgb[50:530, 15:655] = self.annotated_image 
        vis_image_rgb[50:530, 670:1150] = vis_image
        
        if self.args.print_images:
            ep_dir = '{}episodes/{}/eps_{}/'.format(
                self.dump_dir, self.args.rank, self.episode_n)
            if not os.path.exists(ep_dir):
                os.makedirs(ep_dir)
            fn = ep_dir + 'agent-{}-Vis-{}.png'.format(self.agent_id, self.l_step)
            cv2.imwrite(fn, vis_image_rgb)

        if self.args.visualize:
            cv2.imshow("episode_{}- agent_{}".format(self.episode_n, self.agent_id), vis_image_rgb)
            cv2.waitKey(1)

        return vis_image_show
