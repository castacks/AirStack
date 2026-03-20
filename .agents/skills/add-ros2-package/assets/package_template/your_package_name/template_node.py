#!/usr/bin/env python3
"""
Template ROS 2 Node in Python

This template demonstrates:
- Subscribing to topics with callbacks
- Publishing to topics
- Using parameters
- Using timers for periodic processing
- Error handling and logging

TODO: Update this docstring with your module description
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class TemplateNode(Node):
    """
    Template ROS 2 node showing common patterns
    
    Attributes:
        update_rate (float): Processing frequency in Hz
        threshold (float): Detection/processing threshold
        enable_debug (bool): Enable debug outputs
        input_frame (str): Reference frame for inputs
        output_frame (str): Reference frame for outputs
    """
    
    def __init__(self):
        """Initialize the template node"""
        super().__init__('template_node')
        
        self.get_logger().info('Initializing TemplateNode...')
        
        # State variables
        self.current_odom = None
        self.initialized = False
        
        # Setup
        self.setup_parameters()
        self.setup_pubsub()
        self.setup_timer()
        
        self.initialized = True
        self.get_logger().info('TemplateNode initialized successfully')
    
    def setup_parameters(self):
        """Declare and get ROS 2 parameters"""
        
        # Declare parameters with defaults
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('threshold', 0.5)
        self.declare_parameter('enable_debug', False)
        self.declare_parameter('input_frame', 'base_link')
        self.declare_parameter('output_frame', 'map')
        
        # TODO: Declare your module-specific parameters
        
        # Get parameter values
        self.update_rate = self.get_parameter('update_rate').value
        self.threshold = self.get_parameter('threshold').value
        self.enable_debug = self.get_parameter('enable_debug').value
        self.input_frame = self.get_parameter('input_frame').value
        self.output_frame = self.get_parameter('output_frame').value
        
        # Log parameters
        self.get_logger().info('Parameters:')
        self.get_logger().info(f'  update_rate: {self.update_rate:.2f} Hz')
        self.get_logger().info(f'  threshold: {self.threshold:.2f}')
        self.get_logger().info(f'  enable_debug: {self.enable_debug}')
        self.get_logger().info(f'  input_frame: {self.input_frame}')
        self.get_logger().info(f'  output_frame: {self.output_frame}')
    
    def setup_pubsub(self):
        """Create publishers and subscribers"""
        
        # Create subscribers
        # Note: Topic names are generic and will be remapped in launch file
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry',
            self.odom_callback,
            10)
        
        # TODO: Add more subscribers as needed
        
        # Create publishers
        self.cmd_pub = self.create_publisher(Twist, 'output', 10)
        
        if self.enable_debug:
            self.debug_pub = self.create_publisher(Float64, 'debug/value', 10)
        
        # TODO: Add more publishers as needed
        
        self.get_logger().info('Publishers and subscribers initialized')
    
    def setup_timer(self):
        """Create timer for periodic processing"""
        timer_period = 1.0 / self.update_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def odom_callback(self, msg):
        """
        Callback for odometry messages
        
        Args:
            msg (Odometry): Incoming odometry message
        """
        # Store latest odometry
        self.current_odom = msg
        
        if self.enable_debug:
            pos = msg.pose.pose.position
            self.get_logger().debug(
                f'Received odometry: pos=[{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]')
        
        # TODO: Add your callback logic
    
    def timer_callback(self):
        """Timer callback for periodic processing"""
        if not self.initialized or self.current_odom is None:
            self.get_logger().warn(
                'Waiting for odometry data...',
                throttle_duration_sec=1.0)
            return
        
        # Process and publish
        self.process()
    
    def process(self):
        """
        Process data and generate output
        
        TODO: Implement your algorithm here
        """
        # Example: Create output message
        output_msg = Twist()
        
        # Example: Simple processing
        # In a real implementation, replace with your algorithm
        output_msg.linear.x = 0.5  # Example value
        output_msg.linear.y = 0.0
        output_msg.linear.z = 0.0
        output_msg.angular.x = 0.0
        output_msg.angular.y = 0.0
        output_msg.angular.z = 0.1  # Example value
        
        # Publish output
        self.cmd_pub.publish(output_msg)
        
        # Publish debug information if enabled
        if self.enable_debug and hasattr(self, 'debug_pub'):
            debug_msg = Float64()
            debug_msg.data = 123.45  # Example debug value
            self.debug_pub.publish(debug_msg)
        
        self.get_logger().debug('Processing complete, output published')


def main(args=None):
    """Main function to run the node"""
    rclpy.init(args=args)
    
    try:
        node = TemplateNode()
        node.get_logger().info('Node running...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        # Cleanup
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
