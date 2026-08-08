#!/usr/bin/env python3
"""
ZED ObjectsStamped → vision_msgs/Detection2DArray converter
============================================================
Subscribes to:
    /robot_1/sensors/front_stereo/obj_det/objects  (zed_msgs/msg/ObjectsStamped)

Publishes:
    /robot_1/gimbal/boundingbox                    (vision_msgs/msg/Detection2DArray)

The bounding box is derived from the 2D corner points in the ZED detection.
Corner layout (BoundingBox2Di):
      0 ------- 1
      |         |
      3 ------- 2
Each corner is Keypoint2Di.kp = [x, y] (uint32).

Only objects whose tracking_state != 0 (OFF/invalid) are forwarded.
Confidence is remapped from ZED range [1, 99] → score [0.0, 1.0].
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from zed_msgs.msg import ObjectsStamped
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose


class ZedToDetection2DNode(Node):

    def __init__(self):
        super().__init__("zed_to_detection2d")

        self.declare_parameter("input_topic",  "/robot_1/sensors/front_stereo/obj_det/objects")
        self.declare_parameter("output_topic", "/robot_1/gimbal/boundingbox")
        self.declare_parameter("min_confidence", 30.0)   # ZED scale: 1-99

        input_topic  = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._min_conf = self.get_parameter("min_confidence").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._sub = self.create_subscription(
            ObjectsStamped, input_topic, self._cb, qos)
        self._pub = self.create_publisher(
            Detection2DArray, output_topic, qos)

        self.get_logger().info(
            f"Forwarding {input_topic} → {output_topic} "
            f"(min_confidence={self._min_conf})")

    # ------------------------------------------------------------------ #

    def _cb(self, msg: ObjectsStamped):
        out = Detection2DArray()
        out.header = msg.header

        for obj in msg.objects:
            # tracking_state 0 = OFF (not valid)
            if obj.tracking_state == 0:
                continue
            if obj.confidence < self._min_conf:
                continue

            det = Detection2D()
            det.header = msg.header
            det.id = str(obj.label_id)

            # ---- bounding box from 2D corners ----
            # corners[0]=top-left, [1]=top-right, [2]=bottom-right, [3]=bottom-left
            corners = obj.bounding_box_2d.corners
            xs = [float(c.kp[0]) for c in corners]
            ys = [float(c.kp[1]) for c in corners]

            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)

            bbox = BoundingBox2D()
            bbox.center.position.x = (x_min + x_max) / 2.0
            bbox.center.position.y = (y_min + y_max) / 2.0
            bbox.center.theta = 0.0
            bbox.size_x = x_max - x_min
            bbox.size_y = y_max - y_min
            det.bbox = bbox

            # ---- hypothesis (class label + score) ----
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = obj.label     # e.g. "Person", "Vehicle"
            hyp.hypothesis.score    = float(obj.confidence) / 99.0
            det.results.append(hyp)

            out.detections.append(det)

        self._pub.publish(out)
        self.get_logger().debug(
            f"Published {len(out.detections)} detection(s)")


def main(args=None):
    rclpy.init(args=args)
    node = ZedToDetection2DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
