import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import random
import math

X_MIN, X_MAX = -3.0, 0.0
Y_MIN, Y_MAX = -2.0, 2.0
Z_MIN, Z_MAX = -1.5, -0.5

def yaw_to_quaternion(yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def sample_waypoints(N=10):
    points = []
    
    print(f"sampling {N} goal point...")
    
    x = random.uniform(X_MIN, X_MAX)
    y = random.uniform(Y_MIN, Y_MAX)
    z = random.uniform(Z_MIN, Z_MAX)
    yaw = 0
    points.append((x, y, z, yaw))
    
    attempts = 0
    while len(points) < N:
        attempts += 1
        
        # 1. Sample a new point
        nx = random.uniform(X_MIN, X_MAX)
        ny = random.uniform(Y_MIN, Y_MAX)
        nz = random.uniform(Z_MIN, Z_MAX)
        nyaw = random.uniform(-math.pi, math.pi)
        
        # previous point
        px, py, pz, pyaw = points[-1]
        
        # 2. Check the diff to see if satisfied
        dist = math.sqrt((nx - px)**2 + (ny - py)**2 + (nz - pz)**2)
        dyaw_rad = abs(normalize_angle(nyaw - pyaw))
        dyaw_deg = math.degrees(dyaw_rad)
        
        # 3. Check validity
        # condition A: 0.5m
        if dist <= 0.5:
            continue
            
        # condition B: cannot turn too fast
        max_allowed_dyaw = dist * 45.0 
        if dyaw_deg > max_allowed_dyaw:
            continue
            
        points.append((nx, ny, nz, nyaw))
        
    print(f"Total attempts: {attempts}.\n")
    return points

class WaypointPublisher(Node):
    def __init__(self, waypoints):
        super().__init__('waypoint_sampler_node')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.publisher_ = self.create_publisher(PoseArray, '/goal_point_list', qos_profile)
        self.waypoints = waypoints
        
        self.timer = self.create_timer(1.0, self.publish_and_exit)

    def publish_and_exit(self):
        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        print("Generated goal point list: (X, Y, Z, Yaw_deg):")
        for i, (x, y, z, yaw) in enumerate(self.waypoints):
            print(f" P{i+1}: ({x:.2f}, {y:.2f}, {z:.2f})  Yaw: {math.degrees(yaw):.1f}°")
            
            pose = Pose()
            pose.position = Point(x=x, y=y, z=z)
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            msg.poses.append(pose)
            
        self.publisher_.publish(msg)
        self.get_logger().info('✅ PoseArray 已经成功发布到 /goal_point_list')
        
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    
    valid_points = sample_waypoints(N=10)
    
    # 2. 启动节点并发布
    node = WaypointPublisher(valid_points)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()