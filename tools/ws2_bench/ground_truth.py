"""Receive simulation truth for evidence/controlled test-flight feedback."""
import json,socket
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

rclpy.init();node=rclpy.create_node("ws2_ground_truth_receiver")
pub=node.create_publisher(PoseStamped,"/ws2/ground_truth/pose",qos_profile_sensor_data)
oracle_pub=node.create_publisher(String,"/ws2/ground_truth/oracle",10)
last_oracle=-1.
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(("0.0.0.0",9877));sock.settimeout(.1)
while rclpy.ok():
    try:data=json.loads(sock.recv(65536))
    except socket.timeout:
        rclpy.spin_once(node,timeout_sec=0.);continue
    msg=PoseStamped();msg.header.frame_id="ws2_world"
    ns=round(data["sim_time"]*1e9);msg.header.stamp.sec=ns//10**9;msg.header.stamp.nanosec=ns%10**9
    msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=data["position"]
    msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=data["attitude_xyzw"]
    pub.publish(msg)
    if data['sim_time']-last_oracle>=.1 and 'oracle' in data:
        oracle_pub.publish(String(data=json.dumps(data['oracle'])))
        last_oracle=data['sim_time']
    rclpy.spin_once(node,timeout_sec=0.)
