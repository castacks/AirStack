import rclpy
import torch
import pypose as pp

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time

from pathlib import Path
from typing import TYPE_CHECKING
import os, sys
import logging

from .DispartyPublisher import DisparityPublisher
from .MessageFactory import to_stamped_pose, from_image, to_pointcloud, to_image, to_nav_msgs_odmetry
from sensor_interfaces.srv import GetCameraParams

# Add the src directory to the Python path
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "src"))
sys.path.insert(0, src_path)
if TYPE_CHECKING:
    # To make static type checker happy : )
    from src.Odometry.MACVO import MACVO
    from src.DataLoader import StereoFrameData, StereoData, SmartResizeFrame
    from src.Utility.Config import load_config
else:
    import DataLoader
    from Odometry.MACVO import MACVO
    from DataLoader import StereoFrameData, StereoData, SmartResizeFrame
    from Utility.Config import load_config


PACKAGE_NAME = "macvo2"


class MACVO2Node(Node):
    def __init__(self) -> None:
        super().__init__("macvo2_node")
        self.coord_frame = "map_ned"  # FIXME: this is probably wrong? Should be the camera optical center frame
        self.frame_id = 0      # Frame ID
        self.init_time = None  # ROS2 time stamp
        self.get_logger().set_level(logging.INFO)
        self.get_logger().info(f"{os.getcwd()}")
        self.declared_parameters = set()

        self.coord_frame = self.get_string_param("coordinate_frame")  # FIXME: this is probably wrong? Should be the camera optical center frame
        self.frame_id = 0  # Frame ID

        # Load the Camera model ------------------------------------
        self.get_camera_params()
        # End

        # Declare subscriptions and publishers ----------------
        # Subscriptions
        self.imageL_sub = Subscriber(
            self, Image, self.get_string_param("imageL_sub_topic"), qos_profile=1
        )
        self.imageR_sub = Subscriber(
            self, Image, self.get_string_param("imageR_sub_topic"), qos_profile=1
        )
        self.sync_stereo = ApproximateTimeSynchronizer(
            [self.imageL_sub, self.imageR_sub], queue_size=2, slop=0.1
        )
        self.sync_stereo.registerCallback(self.receive_stereo)

        # Publishers
        # self.pose_send = self.create_publisher(
        #     PoseStamped, self.get_string_param("pose_pub_topic"), qos_profile=1
        # )
        self.odom_send = self.create_publisher(
            Odometry, self.get_string_param("odom_pub_topic"), qos_profile=1
        )

        self.map_send = self.create_publisher(
            PointCloud, self.get_string_param("point_pub_topic"), qos_profile=1
        )
        self.img_send = self.create_publisher(
            Image, self.get_string_param("img_pub_topic"), qos_profile=1
        )
        # End

        # Load the MACVO model ------------------------------------
        macvo_config_path = self.get_string_param("camera_config")
        self.get_logger().info(
            f"Loading macvo model from {macvo_config_path}, this might take a while..."
        )
        cfg, _ = load_config(Path(macvo_config_path))

        original_cwd = os.getcwd()
        try:
            os.chdir(get_package_share_directory(PACKAGE_NAME))
            self.get_logger().info(get_package_share_directory(PACKAGE_NAME))
            self.odometry = MACVO[StereoFrameData].from_config(cfg)
            self.odometry.register_on_optimize_finish(self.publish_data)
        finally:
            os.chdir(original_cwd)

        # Publish disparity if needed.
        self.disparity_publisher = DisparityPublisher(
            self,
            self.odometry.Frontend,
            publish_topic=self.get_string_param("disp_pub_topic"),
            frame_id=self.coord_frame,
        )
        self.odometry.Frontend = self.disparity_publisher
        # End

        # Load the Camera model ------------------------------------
        self.camera_info = None
        self.get_camera_params()
        # End

        self.time, self.prev_time = None, None

    def get_integer_param(self, parameter_name: str) -> int:
        if parameter_name not in self.declared_parameters:
            self.declare_parameter(parameter_name, rclpy.Parameter.Type.INTEGER)
            self.declared_parameters.add(parameter_name)
        return self.get_parameter(parameter_name).get_parameter_value().integer_value

    def get_string_param(self, parameter_name: str) -> str:
        if parameter_name not in self.declared_parameters:
            self.declare_parameter(parameter_name, rclpy.Parameter.Type.STRING)
            self.declared_parameters.add(parameter_name)
        return self.get_parameter(parameter_name).get_parameter_value().string_value

    def get_camera_params(self, client_time_out: int = 1):
        camera_param_server_topic = self.get_string_param(
            "camera_param_server_client_topic"
        )
        camera_name = self.get_string_param("camera_name")
        camera_param_client = self.create_client(
            GetCameraParams, camera_param_server_topic
        )

        while True:
            while not camera_param_client.wait_for_service(timeout_sec=client_time_out):
                self.get_logger().error(
                    f"Service {camera_param_server_topic} not available, waiting again..."
                )
            req = GetCameraParams.Request()
            req.camera_names.append(camera_name)
            req.camera_types.append("stereo")
            future = camera_param_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.camera_info = future.result().camera_infos[0]
                self.baseline = future.result().baselines[0]
                self.get_logger().info(f"Received camera config from server!")
                return
            else:
                self.get_logger().error(f"Failed to get camera params from server")
                raise Exception("Failed to get camera params from server")

    def publish_data(self, system: MACVO):
        # Latest pose
        pose = pp.SE3(system.graph.frames.data["pose"][-1])
        self.get_logger().info(f"Publish {pose}")
        time_ns = int(system.graph.frames.data["time_ns"][-1].item())

        time = Time()
        time.sec = (time_ns // 1_000_000_000) + self.init_time.sec
        time.nanosec = (time_ns % 1_000_000_000) + self.init_time.nanosec

        # pose_msg = to_stamped_pose(pose, self.coord_frame, time)
        odom_msg = to_nav_msgs_odmetry(pose, self.coord_frame, time)

        # Latest map
        if system.mapping:
            points = system.graph.get_frame2map(system.graph.frames[-2:-1])
        else:
            points = system.graph.get_match2point(
                system.graph.get_frame2match(system.graph.frames[-1:])
            )

        map_pc_msg = to_pointcloud(
            position=points.data["pos_Tw"],
            keypoints=None,
            frame_id=self.coord_frame,
            colors=points.data["color"],
            time=time,
        )

        # self.pose_send.publish(pose_msg)
        self.odom_send.publish(odom_msg)
        self.map_send.publish(map_pc_msg)

    @staticmethod
    def time_to_ns(time: Time) -> int:
        return int(time.sec * 1e9) + time.nanosec

    def receive_stereo(self, msg_imageL: Image, msg_imageR: Image) -> None:
        if self.camera_info is None:
            self.get_logger().error("Skipped a frame since camera info is not received yet")
            return
        self.get_logger().info(f"Frame {self.frame_id}")
        imageL, timestamp = from_image(msg_imageL), msg_imageL.header.stamp
        imageR = from_image(msg_imageR)
        if self.init_time is None:
            self.init_time = timestamp
        elapsed = int(self.time_to_ns(timestamp) - self.time_to_ns(self.init_time))
        self.disparity_publisher.curr_timestamp = timestamp
        
        # Instantiate a frame and scale to the desired height & width
        stereo_frame = SmartResizeFrame(
            {
                "height": self.get_integer_param("inference_dim_u"),
                "width": self.get_integer_param("inference_dim_v"),
                "interp": "bilinear",
            }
        )(
            StereoFrameData(
                idx=torch.tensor([self.frame_id], dtype=torch.long),
                time_ns=[elapsed],
                stereo=StereoData(
                    T_BS=pp.identity_SE3(1, dtype=torch.float64),
                    K=torch.tensor(
                        [
                            [
                                [self.camera_info.k[0], 0.0, self.camera_info.k[2]],
                                [0.0, self.camera_info.k[4], self.camera_info.k[5]],
                                [0.0, 0.0, 1.0],
                            ]
                        ],
                        dtype=torch.float,
                    ),
                    baseline=torch.tensor([self.baseline], dtype=torch.float),
                    time_ns=[elapsed],
                    height=imageL.shape[0],
                    width=imageL.shape[1],
                    imageL=torch.tensor(imageL)[..., :3]
                    .float()
                    .permute(2, 0, 1)
                    .unsqueeze(0)
                    / 255.0,
                    imageR=torch.tensor(imageR)[..., :3]
                    .float()
                    .permute(2, 0, 1)
                    .unsqueeze(0)
                    / 255.0,
                ),
            )
        )
        self.odometry.run(stereo_frame)

        # Pose-processing
        self.frame_id += 1

    def destroy_node(self):
        self.odometry.terminate()


def main():
    rclpy.init()
    node = MACVO2Node()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
