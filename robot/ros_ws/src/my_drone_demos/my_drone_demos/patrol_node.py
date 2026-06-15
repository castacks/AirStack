import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from enum import Enum
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped, Vector3
from nav_msgs.msg import Odometry
from airstack_msgs.srv import RobotCommand

import numpy as np

WAYPOINTS = np.array([
    [ 1.5,  1.5, 1.5],
    [-1.5,  1.5, 1.5],
    [-1.5, -1.5, 1.5],
    [ 1.5, -1.5, 1.5],
])

class State(Enum):
    IDLE = 0
    ARM = 1
    TAKEOFF = 2
    PATROL = 3
    LAND = 4

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.state = State.IDLE
        
        # Declare parameters
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('cruise_altitude', 1.5)
        self.declare_parameter('patrol_speed', 0.5)
        self.declare_parameter('arrival_threshold', 0.3)
        self.declare_parameter('land_altitude', 0.15)
        
        # Read back parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.cruise_altitude = self.get_parameter('cruise_altitude').value
        self.patrol_speed = self.get_parameter('patrol_speed').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value
        self.land_alt = self.get_parameter('land_altitude').value
        
        self.get_logger().info(f'robot_name parameter set to: {self.robot_name}')
        self.get_logger().info(f'cruise_altitude parameter set to: {self.cruise_altitude}')
        self.get_logger().info(f'patrol_speed parameter set to: {self.patrol_speed}')
        self.get_logger().info(f'arrival_threshold parameter set to: {self.arrival_threshold}')
        self.get_logger().info(f'land_alt parameter set to: {self.land_alt}')
        
        self.vel_cmd_publisher = self.create_publisher(TwistStamped, f'/{self.robot_name}/interface/velocity_command', 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odometry_subscriber = self.create_subscription(Odometry,f'/{self.robot_name}/odometry_conversion/odometry', self.odometry_callback, qos)
        self.odometry_subscriber
        
        # Robot command clients
        self.robot_command_client = self.create_client(RobotCommand, f'/{self.robot_name}/interface/robot_command')
    
        # Start and landing services
        self.start_srv = self.create_service(Trigger, '~/start', self.start_callback)
        self.land_srv = self.create_service(Trigger, '~/land', self.land_callback)
        
        # Loop timer at 20 Hz
        self.loop_timer = self.create_timer(
            0.05,
            self.loop
        )
        
        self.arm_time = 0 # For arm timing
        self.arm_steps_done = set()
        
        self.position = None
        self.waypoint_idx = 0
        
    def start_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.state = State.ARM
        
        response.success = True
        response.message = "Drone armed succesfully"
        
        self.get_logger().info('Drone armed succesfully!')
        
        return response
        
    
    def land_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.state = State.LAND
        
        response.success = True
        response.message = "Initiating drone landing"
        
        self.get_logger().info('Initiating drone landing!')
        
        return response
    
    def odometry_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self.position = np.array([position.x, position.y, position.z])
        
    def robot_command_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Success: {response.success}")
        except Exception as e:
            self.get_logger().error(f"Robot Command service called failed: {e}")
    
    def publish_velocity(self, x=0.0, y=0.0, z=0.0):
        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = self.get_clock().now().to_msg()
        vel_cmd.header.frame_id = 'map'
        vel_cmd.twist.linear = Vector3(x=x, y=y, z=z)
        self.vel_cmd_publisher.publish(vel_cmd)

    def p_controller(self, goal_position):
        Kp = 1.0 * (goal_position - self.position)
        Kp = Kp.clip(-5, 5)
        return Kp
    
    def loop(self):
        if self.position is None:
            return
        if self.state == State.IDLE:
            pass
        if self.state == State.ARM:
            if self.arm_time == 0:
                self.arm_time = self.get_clock().now().nanoseconds * 1e-9
                
            self.publish_velocity()
            
            current_seconds = self.get_clock().now().nanoseconds * 1e-9
            elapsed = current_seconds - self.arm_time
            if elapsed >= 1 and 'request_control' not in self.arm_steps_done:
                if not self.robot_command_client.service_is_ready():
                    self.get_logger().warn('robot_command not ready yet, retrying')
                else:
                    # send request command
                    req = RobotCommand.Request()
                    req.command = RobotCommand.Request.REQUEST_CONTROL
                    self.future = self.robot_command_client.call_async(req)
                    self.future.add_done_callback(self.robot_command_response_callback)
                    
                    self.get_logger().info('Request Command service request sent')
                    
                    self.arm_steps_done.add('request_control')
                
            if elapsed >= 1.5 and 'send_arm' not in self.arm_steps_done:
                # send request command
                if not self.robot_command_client.service_is_ready():
                    self.get_logger().warn('robot_command not ready yet, retrying')
                else:
                    req = RobotCommand.Request()
                    req.command = RobotCommand.Request.ARM
                    self.future = self.robot_command_client.call_async(req)
                    self.future.add_done_callback(self.robot_command_response_callback)
                    
                    self.get_logger().info('Request Command service request sent')
                    
                    self.arm_steps_done.add('send_arm')
                
            if elapsed >= 2.5 and 'transition_to_takeoff' not in self.arm_steps_done:
                self.state = State.TAKEOFF
                self.arm_steps_done.add('transition_to_takeoff')
                
        elif self.state == State.TAKEOFF:
            # Reset the state vars for arm
            if self.arm_time != 0:
                self.arm_time = 0
            
            if len(self.arm_steps_done) != 0:
                self.arm_steps_done.clear()
            
            goal_position = np.array([0, 0, self.cruise_altitude])
            velocity = self.p_controller(goal_position)
            self.publish_velocity(x=float(velocity[0]), y=float(velocity[1]), z=float(velocity[2]))

            if np.linalg.norm(goal_position - self.position) < self.arrival_threshold:
                self.state = State.PATROL

        elif self.state == State.PATROL:
            goal_position = WAYPOINTS[self.waypoint_idx % 4]
            velocity = self.p_controller(goal_position)
            self.publish_velocity(x=float(velocity[0]), y=float(velocity[1]), z=float(velocity[2]))
            
            if np.linalg.norm(goal_position - self.position) < self.arrival_threshold:
                self.waypoint_idx += 1
            
            
        elif self.state == State.LAND:
            self.publish_velocity(z=-0.3)
            if self.position[2] <= self.land_alt:
                req = RobotCommand.Request()
                req.command = RobotCommand.Request.DISARM
                self.future = self.robot_command_client.call_async(req)
                self.future.add_done_callback(self.robot_command_response_callback)
                self.state = State.IDLE
                self.get_logger().info('Landed')
            
            
            
                
def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
            
            
                