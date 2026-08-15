"""Host-side orchestration of the AirStack Docker stack for S.A.F.E. evaluation.

Everything here talks to the stack the way AirStack itself does: `airstack up`
for bring-up, `docker exec` + the stock ros2 CLI for actions, and one
long-lived in-container rclpy bridge (container_bridge.py) for the topics
that need real publish rates. No custom planner protocols.
"""
