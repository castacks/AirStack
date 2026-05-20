# Standard library imports
import os
import math
import unittest
from copy import deepcopy

# Third-party imports
import pytest
import yaml

# ROS2 imports
import rclpy
import rclpy.time
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

# Airstack-specific imports
from airstack_msgs.msg import TrajectoryXYZVYaw
from airstack_msgs.msg import Odometry as TrackingPoint
from airstack_msgs.srv import TakeoffLandingCommand

# Launch-related imports
import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


@pytest.mark.launch_test
def generate_test_description():
    # pkg_name = "takeoff_landing_planner"
    pkg_path = get_package_share_directory("takeoff_landing_planner")
    param_file = os.path.join(
        pkg_path, "test", "config", "test_takeoff_landing_planner.yaml"
    )

    # Launch the TakeoffLandingPlanner node
    takeoff_landing_planner = Node(
        package="takeoff_landing_planner",
        executable="takeoff_landing_planner",
        name="takeoff_landing_planner",
        parameters=[
            ParameterFile(
                param_file=param_file,
                allow_substs=True,
            )
        ],
        arguments=["--ros-args", "--log-level", "fatal"],
    )

    return (
        launch.LaunchDescription(
            [
                takeoff_landing_planner,
                launch.actions.TimerAction(
                    period=1.0, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ]
        ),
        {"param_file": param_file},
    )


class TestTakeoff(unittest.TestCase):
    @classmethod
    def setUpClass(cls, param_file):
        # Initialize ROS2
        rclpy.init()

        # Create a ROS2 node for testing
        cls.node = rclpy.create_node("test_takeoff_node")

        # Load the YAML file
        with open(param_file, "r") as file:
            param_file = yaml.safe_load(file)

        # Parameter dictionary
        cls.params = param_file["/**"]["ros__parameters"]

        # Different commands
        cls.CMD_TAKEOFF = TakeoffLandingCommand.Request.TAKEOFF
        cls.CMD_LAND = TakeoffLandingCommand.Request.LAND
        cls.CMD_RESET = TakeoffLandingCommand.Request.NONE

        # Different fake odoms for testing different scenarios
        cls.TYPE_ODOM_GROUNDED = 0
        cls.TYPE_ODOM_TAKEOFF_PENDING = 1
        cls.TYPE_ODOM_TAKEOFF_COMPLETE = 2
        cls.TYPE_ODOM_LAND_PENDING = 3
        cls.TYPE_ODOM_LAND_COMPLETE = 4

        # Service Client
        cls.client = cls.node.create_client(
            TakeoffLandingCommand, "set_takeoff_landing_command"
        )

        # Publishers
        cls.odom_pub = cls.node.create_publisher(Odometry, "odometry", 1)
        cls.ref_point_pub = cls.node.create_publisher(
            TrackingPoint, "tracking_point", 1
        )
        cls.progress_pub = cls.node.create_publisher(
            Float32, "trajectory_completion_percentage", 1
        )

        # Other vars
        cls.flag_recvd_traj = False
        cls.latest_traj = None
        cls.takeoff_state = String(data="NONE")
        cls.landing_state = String(data="NONE")

    @classmethod
    def tearDownClass(cls):
        # Clean up the node
        cls.node.destroy_node()
        # Shutdown ROS2
        rclpy.shutdown()

    def setUp(cls):
        pass

    def tearDown(cls):
        pass

    def test_takeoff_parameters(cls):
        # Test that the parameters are loaded correctly
        cls.assertIn("takeoff_height", cls.params)
        cls.assertIn("high_takeoff_height", cls.params)
        cls.assertIn("takeoff_landing_velocity", cls.params)
        cls.assertIn("takeoff_acceptance_distance", cls.params)
        cls.assertIn("takeoff_acceptance_time", cls.params)
        cls.assertIn("landing_stationary_distance", cls.params)
        cls.assertIn("landing_acceptance_time", cls.params)
        cls.assertIn("landing_tracking_point_ahead_time", cls.params)
        cls.assertIn("takeoff_path_roll", cls.params)
        cls.assertIn("takeoff_path_pitch", cls.params)
        cls.assertIn("takeoff_path_relative_to_orientation", cls.params)

    def test_node_active(cls):
        # Test that the takeoff service is available
        cls.assertTrue(cls.client.wait_for_service(timeout_sec=5.0))

    # @classmethod
    def call_service(cls, command):
        request = TakeoffLandingCommand.Request()
        request.command = command
        future = cls.client.call_async(request)

        rclpy.spin_until_future_complete(cls.node, future, timeout_sec=5.0)

        cls.assertTrue(future.done(), "Service call timed out")
        resp = future.result()
        cls.assertIsNotNone(resp)
        return resp

    # @classmethod
    def reset_states(cls):
        cls.call_service(cls.CMD_RESET)
        cls.flag_recvd_traj = False
        cls.latest_traj = None
        cls.progress_pub.publish(Float32(data=0.0))

    def test_service_response(cls):
        # Takeoff Service Response Check
        resp = cls.call_service(cls.CMD_TAKEOFF)
        cls.assertIsNotNone(resp)
        cls.assertTrue(resp.accepted)

        # # Reset internal states
        cls.reset_states()

        # Landing Service Response Check
        resp = cls.call_service(cls.CMD_LAND)
        cls.assertIsNotNone(resp)
        cls.assertTrue(resp.accepted)

        # Reset internal states
        cls.reset_states()

    def yaw_to_quat(cls, yaw):
        q = Quaternion()
        q.x = q.y = 0.0
        q.z = math.sin(yaw / 2)
        q.w = math.cos(yaw / 2)

        return q

    def get_fake_odom(cls, kind, yaw=0, curr_odom=Odometry()):
        fake_odom = Odometry()
        fake_odom.header.stamp = cls.node.get_clock().now().to_msg()
        fake_odom.header.frame_id = "map"
        fake_odom.child_frame_id = "robot"
        if kind == cls.TYPE_ODOM_GROUNDED:
            fake_odom.pose.pose.position.x = 0.0
            fake_odom.pose.pose.position.y = 0.0
            fake_odom.pose.pose.position.z = 0.0
            fake_odom.pose.pose.orientation = cls.yaw_to_quat(yaw)
        elif kind == cls.TYPE_ODOM_TAKEOFF_PENDING:
            if cls.flag_recvd_traj:
                fake_odom.pose.pose.position = deepcopy(
                    cls.latest_traj.waypoints[-1].position
                )
                fake_odom.pose.pose.position.z -= (
                    1.5 * cls.params["takeoff_acceptance_distance"]
                )
                fake_odom.pose.pose.orientation.w = 1.0
        elif kind == cls.TYPE_ODOM_TAKEOFF_COMPLETE:
            if cls.flag_recvd_traj:
                fake_odom.pose.pose.position = cls.latest_traj.waypoints[-1].position
                fake_odom.pose.pose.orientation.w = 1.0
        elif kind == cls.TYPE_ODOM_LAND_PENDING:
            if cls.flag_recvd_traj and curr_odom is not None:
                fake_odom.pose.pose.position = deepcopy(curr_odom.pose.pose.position)
                fake_odom.pose.pose.position.z -= 0.5 * cls.params["takeoff_height"]
        # This one is for tracking point
        elif kind == cls.TYPE_ODOM_LAND_COMPLETE:
            if cls.flag_recvd_traj:
                fake_odom.pose.pose.position = deepcopy(curr_odom.pose.pose.position)
                fake_odom.pose.pose.position.z = -1e5

        return fake_odom

    # Convert Odometry to TrackingPoint
    def get_fake_ref_point(cls, kind):
        odom = cls.get_fake_odom(kind)
        ref_point = TrackingPoint()
        ref_point.header = odom.header
        ref_point.child_frame_id = odom.child_frame_id
        ref_point.pose = odom.pose.pose

        return ref_point

    # TODO (@kavin-cmu): Split this into smaller unit tests
    def check_takeoff_trajectory(cls):
        # Check if we have a trajectory
        cls.assertIsNotNone(cls.latest_traj, "Dont have a valid trajectory!")

        # Get first and last waypoints
        beg_wp = cls.latest_traj.waypoints[0]
        end_wp = cls.latest_traj.waypoints[-1]

        # Check frame
        cls.assertEqual(cls.latest_traj.header.frame_id, "map")

        # Check if first waypoint is same as the odom that we sent
        cls.assertEqual(
            beg_wp.position,
            cls.get_fake_odom(cls.TYPE_ODOM_GROUNDED).pose.pose.position,
        )

        # Check if the velocity at first waypoint is as expected
        cls.assertAlmostEqual(
            beg_wp.velocity, cls.params["takeoff_landing_velocity"], delta=1e-7
        )

        dx = end_wp.position.x - beg_wp.position.x
        dy = end_wp.position.y - beg_wp.position.y
        dz = end_wp.position.z - beg_wp.position.z

        # Check if final waypoint is at the correct height
        cls.assertEqual(dz, cls.params["takeoff_height"])

        # Check path pitch angle
        cls.assertAlmostEqual(
            math.degrees(math.asin(dx / dz)),
            cls.params["takeoff_path_pitch"],
            delta=1e-5,
        )

        # Check path roll angle
        cls.assertAlmostEqual(
            math.degrees(math.asin(dy / dz)),
            cls.params["takeoff_path_roll"],
            delta=1e-5,
        )

    def wait_for(cls, secs):
        end_time = cls.node.get_clock().now() + rclpy.duration.Duration(seconds=secs)
        while cls.node.get_clock().now() < end_time:
            rclpy.spin_once(cls.node, timeout_sec=0.01)

    def test_takeoff_and_landing(cls):
        # Test that a trajectory override message is published after takeoff
        cls.reset_states()

        cls.odom_pub.publish(cls.get_fake_odom(cls.TYPE_ODOM_GROUNDED))
        cls.ref_point_pub.publish(cls.get_fake_ref_point(cls.TYPE_ODOM_GROUNDED))

        def trajectory_callback(msg):
            cls.latest_traj = msg
            cls.flag_recvd_traj = True

        def takeoff_state_callback(msg):
            cls.takeoff_state = msg
            # print(msg.data)

        def landing_state_callback(msg):
            cls.landing_state = msg

        takeoff_state_sub = cls.node.create_subscription(
            String,
            "takeoff_state",
            takeoff_state_callback,
            1,
        )

        landing_state_sub = cls.node.create_subscription(
            String,
            "landing_state",
            landing_state_callback,
            1,
        )

        traj_sub = cls.node.create_subscription(
            TrajectoryXYZVYaw,
            "trajectory_override",
            trajectory_callback,
            1,
        )

        # Send takeoff request
        cls.call_service(cls.CMD_TAKEOFF)

        # Wait for trajectory override message
        cls.wait_for(1.0)

        cls.assertTrue(
            cls.flag_recvd_traj, "Did not receive generated trajectory in time!"
        )

        cls.check_takeoff_trajectory()

        # Emulate traj controller setpoint already at end
        cls.ref_point_pub.publish(
            cls.get_fake_ref_point(cls.TYPE_ODOM_TAKEOFF_COMPLETE)
        )

        # Emulate robot Odometry in progress
        cls.odom_pub.publish(cls.get_fake_odom(cls.TYPE_ODOM_TAKEOFF_PENDING))
        cls.progress_pub.publish(Float32(data=75.0))
        cls.wait_for(1.0)
        cls.assertEqual(cls.takeoff_state.data, "TAKING_OFF")

        # Emulate robot Odometry complete but time check will fail
        odom_end = cls.get_fake_odom(cls.TYPE_ODOM_TAKEOFF_COMPLETE)
        cls.odom_pub.publish(odom_end)
        cls.progress_pub.publish(Float32(data=100.0))
        cls.wait_for(1.0)
        cls.assertEqual(cls.takeoff_state.data, "TAKING_OFF")

        # Send same odometry but with passing timestamp
        odom_end.header.stamp.sec += int(cls.params["takeoff_acceptance_time"])
        cls.odom_pub.publish(odom_end)
        cls.wait_for(1.0)
        cls.assertEqual(cls.takeoff_state.data, "COMPLETE")

        # Landing Checks
        cls.reset_states()

        # Send landing request
        cls.call_service(cls.CMD_LAND)
        cls.wait_for(1.0)
        cls.assertTrue(
            cls.flag_recvd_traj, "Did not receive generated trajectory in time!"
        )

        # Emulate pending landing
        cls.odom_pub.publish(cls.get_fake_odom(cls.TYPE_ODOM_LAND_PENDING))
        cls.ref_point_pub.publish(cls.get_fake_ref_point(cls.TYPE_ODOM_LAND_PENDING))
        cls.wait_for(1.0)
        cls.assertEqual(cls.landing_state.data, "LANDING")

        # Emulate completed landing
        gnd_odom = cls.get_fake_odom(cls.TYPE_ODOM_GROUNDED)
        gnd_odom.header.stamp.sec += int(cls.params["landing_acceptance_time"])

        # need to publish multiple messages to pass the odom_list count check
        for i in range(10):
            cls.odom_pub.publish(gnd_odom)

        cls.ref_point_pub.publish(cls.get_fake_ref_point(cls.TYPE_ODOM_LAND_COMPLETE))
        cls.wait_for(1.0)
        cls.assertEqual(cls.landing_state.data, "COMPLETE")

        traj_sub.destroy()
        takeoff_state_sub.destroy()
        landing_state_sub.destroy()
