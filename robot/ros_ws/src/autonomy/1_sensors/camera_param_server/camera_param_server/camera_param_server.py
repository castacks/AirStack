import rclpy
from rclpy.node import Node
import tf2_ros

from sensor_msgs.msg import CameraInfo
from sensor_interfaces.srv import GetCameraParams

import yaml
import os
import numpy as np

class StereoCameraInfo:
    def __init__(self, node, camera_name, camera_info_topic, left_camera_frame_id, right_camera_frame_id):
        self.left_camera_info = CameraInfo()
        self.right_camera_info = CameraInfo()
        self.camera_type = 'stereo'

        self.tf_initialized = False
        self.left_info_initialized = False
        self.right_info_initialized = False
        self.info_initialized = False

        left_info_topic_merged = camera_name + '/left/' + camera_info_topic
        right_info_topic_merged = camera_name + '/right/' + camera_info_topic
        self.left_camera_info_sub = node.create_subscription(CameraInfo, left_info_topic_merged, self.left_camera_info_callback, 10)
        self.right_camera_info_sub = node.create_subscription(CameraInfo, right_info_topic_merged, self.right_camera_info_callback, 10)
        
        self.camera_name = camera_name
        self.left_camera_frame_id = left_camera_frame_id
        self.right_camera_frame_id = right_camera_frame_id
        self.base_link_frame_id = node.base_link_frame_id
        self.left_camera_transform_to_baselink = None
        self.right_camera_transform_to_baselink = None
        self.baseline = None

    def left_camera_info_callback(self, msg):
        self.left_camera_info = msg
        self.left_info_initialized = True
        if self.left_info_initialized and self.right_info_initialized:
            self.info_initialized = True

    def right_camera_info_callback(self, msg):
        self.right_camera_info = msg
        self.right_info_initialized = True
        if self.left_info_initialized and self.right_info_initialized:
            self.info_initialized = True

class MonoCameraInfo:
    def __init__(self, node, camera_name, camera_info_topic, camera_frame_id):
        self.camera_info = CameraInfo()
        self.camera_type = 'mono'

        self.info_initialized = False

        info_topic_merged = camera_name + '/' + camera_info_topic
        self.camera_info_sub = node.create_subscription(CameraInfo, info_topic_merged, self.camera_info_callback, 10)
        
        self.camera_name = camera_name
        self.camera_frame_id = camera_frame_id
        self.base_link_frame_id = node.base_link_frame_id
        self.camera_transform_to_baselink = None

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.info_initialized = True
    
class CameraParamServer(Node):
    def __init__(self):
        super().__init__('cam_param_server')

        self.camera_dict = {}
        self.base_link_frame_id = None

        self.tfs_initialized = False
        self.info_initialized = False
        self.server_initialized = False

        self.declare_parameter('camera_config', rclpy.parameter.Parameter.Type.STRING)
        camera_config_file = self.get_parameter('camera_config').value
        if not os.path.exists(camera_config_file):
            self.get_logger().error('Camera configuration file not found: %s' % camera_config_file)
            return
        
        self.parse_camera_config(camera_config_file)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_checking_timer = self.create_timer(0.1, self.check_tf)

    def parse_camera_config(self, config_file_path):
        try:
            config_file = self.load_config(config_file_path)
            camera_list = config_file.get('camera_list')
            self.base_link_frame_id = config_file.get('base_link_frame_id')
        except Exception as e:
            self.get_logger().error(f"Error parsing camera config file: {e}")

        for camera in camera_list:
            camera_name = camera.get('camera_name')
            if camera.get('camera_type') == 'stereo':
                self.camera_dict[camera_name] = StereoCameraInfo(self, camera_name, 
                                                                       camera.get('camera_info_sub_topic'),
                                                                       camera.get('left_camera_frame_id'), 
                                                                       camera.get('right_camera_frame_id'))
            elif camera.get('camera_type') == 'mono':
                self.camera_dict[camera_name] = MonoCameraInfo(self, camera_name, camera.get('camera_info_sub_topic'), camera.get('camera_frame_id'))
            else:
                self.get_logger().error('Invalid camera type: %s' % camera.get('type'))
        
    def load_config(self, file):
        try:
            with open(file, 'r') as file:
                config = yaml.safe_load(file)
            return config
        except FileNotFoundError:
            self.get_logger().error(f"Error: The file '{file}' does not exist.")
            raise
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file '{file}': {e}")
            raise

    def check_tf(self):
        if not self.tfs_initialized:
            for camera in self.camera_dict.values():
                if isinstance(camera, StereoCameraInfo):
                    try:
                        camera.left_camera_transform_to_baselink = self.tf_buffer.lookup_transform(camera.left_camera_frame_id, camera.base_link_frame_id, rclpy.time.Time())
                        camera.right_camera_transform_to_baselink = self.tf_buffer.lookup_transform(camera.right_camera_frame_id, camera.base_link_frame_id, rclpy.time.Time())
                        left_cam_location = np.array([camera.left_camera_transform_to_baselink.transform.translation.x, camera.left_camera_transform_to_baselink.transform.translation.y, camera.left_camera_transform_to_baselink.transform.translation.z])
                        right_cam_location = np.array([camera.right_camera_transform_to_baselink.transform.translation.x, camera.right_camera_transform_to_baselink.transform.translation.y, camera.right_camera_transform_to_baselink.transform.translation.z])
                        camera.baseline = np.linalg.norm(left_cam_location - right_cam_location)
                        camera.tf_initialized = True
                    except Exception as e:
                        self.get_logger().error(f"Error looking up transform: {e}")
                elif isinstance(camera, MonoCameraInfo):
                    try:
                        camera.camera_transform_to_baselink = self.tf_buffer.lookup_transform(camera.camera_frame_id, camera.base_link_frame_id, rclpy.time.Time())
                    except Exception as e:
                        self.get_logger().error(f"Error looking up transform: {e}")
            
            if all(camera.tf_initialized for camera in self.camera_dict.values()):
                self.tfs_initialized = True
                self.get_logger().info('Camera transforms initialized')
        
        if not self.info_initialized:
            if all(camera.info_initialized for camera in self.camera_dict.values()):
                self.info_initialized = True
                self.get_logger().info('Camera parameters initialized')
        
        if self.tfs_initialized and self.info_initialized and not self.server_initialized:
            self.camera_params_srv = self.create_service(GetCameraParams, 'get_camera_params', self.get_camera_params)
            self.server_initialized = True
            self.get_logger().info('Camera parameter server initialized')

    
    def get_camera_params(self, request, response):
        # Check if camera parameters are initialized
        if self.tfs_initialized and self.info_initialized:
            camera_name_list = request.camera_names
            camera_type_list = request.camera_types
            if len(camera_name_list) != len(camera_type_list):
                self.get_logger().error('Camera name and type list length mismatch')
                response.success = False
                return response
            for i, camera_name in enumerate(camera_name_list):
                camera_type = camera_type_list[i]
                response = self.get_camera_params_single(response, camera_name, camera_type)
            return response
        else:
            self.get_logger().error('Camera parameters not initialized')
            response.success = False
            return response

    def get_camera_params_single(self, response, incoming_camera_name, incoming_camera_type):
        if incoming_camera_name in self.camera_dict:
            camera = self.camera_dict[incoming_camera_name]
            if camera.camera_type == incoming_camera_type:
                if isinstance(camera, StereoCameraInfo):
                    response.camera_frame_ids.append(camera.left_camera_frame_id)
                    response.camera_infos.append(camera.left_camera_info)
                    response.camera_frame_ids.append(camera.right_camera_frame_id)
                    response.camera_infos.append(camera.right_camera_info)
                    response.baselines.append(camera.baseline)
                    response.success = True
                elif isinstance(camera, MonoCameraInfo):
                    response.camera_info = camera.camera_info
                    response.camera_transform_to_baselink = camera.camera_transform_to_baselink
                    response.success = True
            else:
                self.get_logger().error('Camera type mismatch: %s' % incoming_camera_name)
                response.success = False
            return response
        else:
            self.get_logger().error('Camera not found: %s' % incoming_camera_name)
            return response
    


def main(args=None):
    rclpy.init(args=args)

    camera_param_server = CameraParamServer()

    rclpy.spin(camera_param_server)

    camera_param_server.destroy_node()
    rclpy.shutdown()

