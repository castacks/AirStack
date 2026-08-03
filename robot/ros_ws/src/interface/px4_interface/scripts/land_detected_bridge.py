#!/usr/bin/env python3

"""
Land Detected Bridge

Translates PX4's landing detector into the message the autonomy stack expects.

    PX4 ──VehicleLandDetected──▶ [this node] ──ExtendedState──▶ takeoff_landing_planner
          land_detected                        extended_state
          (/{robot}/fmu/out/vehicle_land_detected)

takeoff_landing_planner's LandTask only reports success once it observes
mavros_msgs/ExtendedState with landed_state == ON_GROUND, and it has no altitude
fallback. On the MAVROS path that message comes from MAVROS itself; over
uXRCE-DDS nothing publishes it, so a land command would run until timeout even
with the drone sitting on the ground. This node closes that gap without changing
the autonomy stack, keeping the simulation and hardware code paths identical
above the interface layer.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from mavros_msgs.msg import ExtendedState
from px4_msgs.msg import VehicleLandDetected


class LandDetectedBridgeNode(Node):
    """Republishes px4_msgs/VehicleLandDetected as mavros_msgs/ExtendedState."""

    def __init__(self):
        super().__init__('land_detected_bridge')

        self.declare_parameter('log_transitions', True)
        self.log_transitions = bool(self.get_parameter('log_transitions').value)

        # PX4 /fmu/out/* topics are published BEST_EFFORT + VOLATILE.
        px4_qos = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST)

        self.land_sub = self.create_subscription(
            VehicleLandDetected, 'land_detected', self._on_land_detected, px4_qos)
        self.state_pub = self.create_publisher(ExtendedState, 'extended_state', 1)

        self._last_state = None
        self.get_logger().info('Land detected bridge started')

    def _on_land_detected(self, msg: VehicleLandDetected):
        state = ExtendedState()
        state.landed_state = (ExtendedState.LANDED_STATE_ON_GROUND if msg.landed
                              else ExtendedState.LANDED_STATE_IN_AIR)
        self.state_pub.publish(state)

        if self.log_transitions and state.landed_state != self._last_state:
            self._last_state = state.landed_state
            self.get_logger().info(
                'PX4 land detector: '
                + ('ON_GROUND' if msg.landed else 'IN_AIR'))


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    try:
        node = LandDetectedBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
