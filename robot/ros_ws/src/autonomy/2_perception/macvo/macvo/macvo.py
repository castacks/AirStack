import rclpy
import torch
import numpy as np
import cv2
from cv_bridge import CvBridge

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber

from pathlib import Path
from typing import TYPE_CHECKING
from torchvision.transforms.functional import center_crop, resize
import os, sys
import argparse

from .MessageFactory import to_stamped_pose, from_image, to_pointcloud, to_image
from sensor_interfaces.srv import GetCameraParams

# Add the src directory to the Python path
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'src'))
sys.path.insert(0, src_path)
if TYPE_CHECKING:
    # To make static type checker happy : )
    from src.Odometry.MACVO import MACVO
    from src.DataLoader import SourceDataFrame, MetaInfo
    from src.Utility.Config import load_config
else:
    from Odometry.MACVO import MACVO                
    from DataLoader import SourceDataFrame, MetaInfo
    from Utility.Config import load_config

class MACVONode(Node):
    def __init__(self):
        super().__init__("macvo_node")

        self.bridge = None
        self.time = None
        self.prev_time = None
        self.frame = None
        self.camera_info = None
        self.baseline = None
        self.prev_frame = None
        self.odometry = None

        # Declare subscriptions and publishers ----------------
        self.declare_parameter("imageL_sub_topic", rclpy.Parameter.Type.STRING)
        self.declare_parameter("imageR_sub_topic", rclpy.Parameter.Type.STRING)
        imageL_topic = self.get_parameter("imageL_sub_topic").get_parameter_value().string_value
        imageR_topic = self.get_parameter("imageR_sub_topic").get_parameter_value().string_value
        self.imageL_sub = Subscriber(self, Image, imageL_topic, qos_profile=1)
        self.imageR_sub = Subscriber(self, Image, imageR_topic, qos_profile=1)

        self.sync_stereo = ApproximateTimeSynchronizer(
            [self.imageL_sub, self.imageR_sub], queue_size=2, slop=0.1
        )
        self.sync_stereo.registerCallback(self.receive_frame)
        
        self.declare_parameter("pose_pub_topic", rclpy.Parameter.Type.STRING)
        pose_topic = self.get_parameter("pose_pub_topic").get_parameter_value().string_value
        self.pose_pipe  = self.create_publisher(PoseStamped, pose_topic, qos_profile=1)
        
        self.declare_parameter("point_pub_topic", rclpy.Parameter.Type.STRING)
        point_topic = self.get_parameter("point_pub_topic").get_parameter_value().string_value
        if point_topic is not None:
            self.point_pipe = self.create_publisher(PointCloud, point_topic, qos_profile=1)
        else:
            self.point_pipe = None
        
        self.declare_parameter("img_pub_topic", rclpy.Parameter.Type.STRING)
        img_stream = self.get_parameter("img_pub_topic").get_parameter_value().string_value
        if img_stream is not None:
            self.img_pipes = self.create_publisher(Image, img_stream, qos_profile=1)
        else:
            self.img_pipes = None

        self.frame = "map"
        
        # Load the MACVO model ------------------------------------
        self.declare_parameter("camera_config", rclpy.Parameter.Type.STRING)
        camera_config = self.get_parameter("camera_config").get_parameter_value().string_value
        self.get_logger().info(f"Loading macvo model from {camera_config}, this might take a while...")
        cfg, _ = load_config(Path(camera_config))
        self.frame_idx  = 0
        self.odometry   = MACVO.from_config(cfg)
        self.declare_parameter("camera_name", rclpy.Parameter.Type.STRING)
        self.camera_name = self.get_parameter("camera_name").get_parameter_value().string_value

        self.odometry.register_on_optimize_finish(self.publish_latest_pose)
        self.odometry.register_on_optimize_finish(self.publish_latest_points)
        # self.odometry.register_on_optimize_finish(self.publish_latest_stereo)
        self.odometry.register_on_optimize_finish(self.publish_latest_matches)
        self.get_logger().info(f"MACVO Model loaded successfully! Initializing MACVO node ...")

        self.declare_parameter("inference_dim_u", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("inference_dim_v", rclpy.Parameter.Type.INTEGER)
        u_dim = self.get_parameter("inference_dim_u").get_parameter_value().integer_value
        v_dim = self.get_parameter("inference_dim_v").get_parameter_value().integer_value
        
        self.time  , self.prev_time  = None, None
        self.meta = None

        # get camera params from the camera param server ----------------------------------------------------------
        self.declare_parameter("camera_param_server_client_topic", rclpy.Parameter.Type.STRING)
        camera_param_server_topic = self.get_parameter("camera_param_server_client_topic").get_parameter_value().string_value
        self.camera_param_client = self.create_client(GetCameraParams, camera_param_server_topic)
        self.get_camera_params(camera_param_server_topic)
        
        self.bridge = CvBridge()
        self.scale_u = float(self.camera_info.width / u_dim)
        self.scale_v = float(self.camera_info.height / v_dim)

        self.rot_correction_matrix = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.rot_correction_matrix = np.eye(3)

        # self.get_logger().info(f"scale u: {self.scale_u}, scale v: {self.scale_v}, u_dim: {u_dim}, v_dim: {v_dim}, width: {self.camera_info.width}, height: {self.camera_info.height}")

        self.get_logger().info(f"MACVO Node initialized with camera config: {camera_config}")

    def get_camera_params(self, camera_param_server_topic, client_time_out: int = 1):
        while True:
            while not self.camera_param_client.wait_for_service(timeout_sec=client_time_out):
                self.get_logger().error(f"Service {camera_param_server_topic} not available, waiting again...")
            req = GetCameraParams.Request()
            req.camera_names.append(self.camera_name)
            req.camera_types.append("stereo")
            future = self.camera_param_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.camera_info = future.result().camera_infos[0]
                self.baseline = future.result().baselines[0]
                self.get_logger().info(f"Received camera config from server!")
                return
            else:
                self.get_logger().error(f"Failed to get camera params from server")
        
    def publish_latest_pose(self, system: MACVO):
        pose = system.gmap.frames.pose[-1]
        frame = self.frame
        time  = self.time if self.prev_time is None else self.prev_time
        assert frame is not None and time is not None
        
        out_msg = to_stamped_pose(pose, frame, time)

        # Correction for the camera coordinate frame
        out_msg.pose.position.x, out_msg.pose.position.y, out_msg.pose.position.z = np.dot(self.rot_correction_matrix, np.array([out_msg.pose.position.x, out_msg.pose.position.y, out_msg.pose.position.z]))
        
        self.pose_pipe.publish(out_msg)
   
    def publish_latest_points(self, system: MACVO):
        if self.point_pipe is None: return
        
        latest_frame  = system.gmap.frames[-1]
        latest_points = system.gmap.get_frame_points(latest_frame)
        latest_obs    = system.gmap.get_frame_observes(latest_frame)
        
        frame = self.frame
        time  = self.time if self.prev_time is None else self.prev_time
        assert frame is not None and time is not None
        
        out_msg = to_pointcloud(
            position  = latest_points.position,
            keypoints = latest_obs.pixel_uv,
            frame_id  = frame,
            colors    = latest_points.color,
            time      = time
        )

        # Correction for the camera coordinate frame
        for pt in out_msg.points:
            pt.x, pt.y, pt.z = np.dot(self.rot_correction_matrix, np.array([pt.x, pt.y, pt.z]))

        self.point_pipe.publish(out_msg)
  
    def publish_latest_stereo(self, system: MACVO):
        if self.img_pipes is None: return
        
        source = system.prev_frame
        if source is None: return
        frame = self.frame
        time  = self.time if self.prev_time is None else self.prev_time
        assert frame is not None and time is not None
        
        img = (source.imageL[0].permute(1, 2, 0).numpy() * 255).copy().astype(np.uint8)
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.frame_id = frame
        msg.header.stamp = time

        self.img_pipes.publish(msg)
    
    def publish_latest_matches(self, system: MACVO):
        if self.img_pipes is None: return

        source = system.prev_frame
        if source is None: return
        frame = self.frame
        time  = self.time if self.prev_time is None else self.prev_time
        assert frame is not None and time is not None
        latest_frame  = system.gmap.frames[-1]

        # pixels are given from the reduced image size, need to be scaled back to original size
        pixels_uv    = system.gmap.get_frame_observes(latest_frame).pixel_uv.int().numpy()
        img = (source.imageL[0].permute(1, 2, 0).numpy() * 255).copy().astype(np.uint8)
        if pixels_uv.size > 0:
            for i in range(pixels_uv.shape[0]):
                x, y = pixels_uv[i]
                img = cv2.circle(img, (x, y), 2, (0, 255, 0), -1)
            msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            msg.header.frame_id = frame
            msg.header.stamp = time
            self.img_pipes.publish(msg)

    def receive_frame(self, msg_L: Image, msg_R: Image) -> None:
        if self.frame is None or self.bridge is None:
            self.get_logger().error("MACVO Node not initialized yet, skipping frame")
            return
        
        self.prev_frame, self.prev_time = self.frame, self.time
        
        self.frame        = msg_L.header.frame_id

        self.time = msg_L.header.stamp
        imageL = self.bridge.imgmsg_to_cv2(msg_L, desired_encoding="passthrough")
        imageR = self.bridge.imgmsg_to_cv2(msg_R, desired_encoding="passthrough")

        camera_fx, camera_fy = self.camera_info.k[0], self.camera_info.k[4]
        camera_cx, camera_cy = self.camera_info.k[2], self.camera_info.k[5]
        meta = MetaInfo(
            idx=self.frame_idx,
            baseline=self.baseline,
            width=self.camera_info.width,
            height=self.camera_info.height,
            K=torch.tensor([[camera_fx, 0., camera_cx],
                            [0., camera_fy, camera_cy],
                            [0., 0., 1.]]).float())
        
        frame = SourceDataFrame(
                meta=meta,
                imageL=torch.tensor(imageL)[..., :3].float().permute(2, 0, 1).unsqueeze(0) / 255.,
                imageR=torch.tensor(imageR)[..., :3].float().permute(2, 0, 1).unsqueeze(0) / 255.,
                imu=None,
                gtFlow=None, gtDepth=None, gtPose=None, flowMask=None
            ).resize_image(scale_u=self.scale_u, scale_v=self.scale_v)
        
        start_time = self.get_clock().now()
        self.odometry.run(frame)
        end_time = self.get_clock().now()   
        # self.get_logger().info(f"Frame {self.frame_idx} processed in {end_time - start_time}")
        self.frame_idx += 1

def main():
    rclpy.init()
    node = MACVONode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
