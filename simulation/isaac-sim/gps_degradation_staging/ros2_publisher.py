"""ROS 2 publisher for GPS degradation state (Float32MultiArray, 9 fields)."""
from __future__ import annotations


class Ros2StatePublisher:
    # Field order matches what subscribers expect
    _FIELDS = ["state", "n_los", "hdop", "vdop", "eph_m", "epv_m",
               "delta_lat", "delta_lon", "delta_alt"]

    def __init__(self, ros2_node, robot_id: int = 1):
        self._pub = None
        if ros2_node is not None:
            try:
                from std_msgs.msg import Float32MultiArray
                topic = f"/robot_{robot_id}/gps/degradation_state"
                self._pub = ros2_node.create_publisher(Float32MultiArray, topic, 10)
            except Exception as exc:
                print(f"[Ros2StatePublisher] publisher setup failed: {exc}")

    def publish(self, output) -> None:
        if self._pub is None:
            return
        try:
            from std_msgs.msg import Float32MultiArray
            msg = Float32MultiArray()
            msg.data = [
                float(output.state),
                float(output.n_los),
                float(output.hdop),
                float(output.vdop),
                float(output.eph_m),
                float(output.epv_m),
                float(output.delta_lat),
                float(output.delta_lon),
                float(output.delta_alt),
            ]
            self._pub.publish(msg)
        except Exception as exc:
            print(f"[Ros2StatePublisher] publish error: {exc}")
