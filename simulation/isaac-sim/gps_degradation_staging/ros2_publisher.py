"""
Publishes GPS degradation state to a ROS 2 topic so the autonomy stack
(MACVIO, mission planner) can observe GPS quality in real time.

Topic:   /robot_{vehicle_id}/gps/degradation_state
Type:    std_msgs/Float32MultiArray  (avoids a custom message dependency)

Field layout (0-based index):
  [0]  state             (int cast to float: 0=OPEN_SKY … 4=RECOVERY)
  [1]  n_los_sats
  [2]  hdop
  [3]  vdop
  [4]  eph_m
  [5]  epv_m
  [6]  delta_lat_deg
  [7]  delta_lon_deg
  [8]  delta_alt_m
"""

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .state_machine import DegradationOutput


class GpsDegradationPublisher:

    def __init__(self, vehicle_id: int, node: Node):
        topic = f"/robot_{vehicle_id}/gps/degradation_state"
        self._pub = node.create_publisher(Float32MultiArray, topic, 10)

    def publish(self, output: DegradationOutput, n_los: int) -> None:
        msg = Float32MultiArray()
        msg.data = [
            float(output.state),
            float(n_los),
            float(output.hdop),
            float(output.vdop),
            float(output.eph_m),
            float(output.epv_m),
            float(output.delta_lat_deg),
            float(output.delta_lon_deg),
            float(output.delta_alt_m),
        ]
        self._pub.publish(msg)
