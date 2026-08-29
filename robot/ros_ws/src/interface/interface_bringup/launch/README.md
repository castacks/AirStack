# interface_bringup launch files

- `interface.launch.py` — the canonical interface bringup, included by the
  stack entry files under `stacks/*/launch/`. Launches MAVROS (via
  `mavros_px4.launch.xml`, skipped when `SIM_TYPE=simple`), the
  `robot_interface_node`, the position setpoint publisher, and the odometry
  conversion node.
- `mavros_px4.launch.xml` — wraps the upstream `mavros` `node.launch` with the
  AirStack MAVROS config (`../config/px4_config.yaml`) and PX4 plugin list.

There is no `sim` launch argument. The MAVLink connection is computed from
environment variables (see `interface.launch.py`):

```text
OFFBOARD_PORT = OFFBOARD_BASE_PORT (default 14540) + ROS_DOMAIN_ID
ONBOARD_PORT  = ONBOARD_BASE_PORT  (default 14580) + ROS_DOMAIN_ID
FCU_URL       = udp://:<OFFBOARD_PORT>@<SIM_IP>:<ONBOARD_PORT>
                (unless FCU_URL is set; SIM_IP default 172.31.0.200)
TGT_SYSTEM    = 1 + ROS_DOMAIN_ID (unless TGT_SYSTEM is set)
```

One launch argument: `interface_odometry_in_topic` — the odometry topic
remapped into `odometry_conversion` (default
`/$ROBOT_NAME/interface/mavros/local_position/odom`).

The drone safety monitor is NOT launched here — the stack entry files launch it
(`drone_safety_monitor.launch.xml`).
