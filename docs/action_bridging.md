# ROS 2 action bridging across DDS domains

*Summary for AirStack team discussion.*

## Problem

Each robot runs on its own `ROS_DOMAIN_ID` (robot_1 → 1, robot_2 → 2, …); the GCS runs on domain 0. We bridge with eProsima **`ddsrouter`**. Topics work. **Actions don't.**

From the GCS container:
- `ros2 service list --include-hidden-services -t` shows `/robot_1/tasks/takeoff/_action/send_goal` — name + type are bridged.
- `ros2 action info /robot_1/tasks/takeoff` → `Action servers: 0`.
- `ros2 action send_goal` hangs; Foxglove reports "Service … is not available".

### Root cause

`ddsrouter` bridges the DDS `rq/…Request` / `rr/…Reply` topics underneath a ROS 2 service, but every ROS 2 action client calls `rcl_service_server_is_available()` first. That walks the RMW endpoint graph for a **matched DDS server signature**; ddsrouter's proxy endpoints don't match, so the call aborts. It's an RMW endpoint-matching problem, not a naming/visibility problem.

Same symptom reported in the wild: [RTI community — "ROS2 Router: Service Call Request made, but no Response"](https://community.rti.com/forum-topic/ros2-router-service-call-request-made-no-response).

### "Are actions failing because they're hidden? If we unhide them, will it work?"

No.
- `_action/*` naming is a **display-only filter** in ROS CLI tools; DDS doesn't care ([ROS 2 Actions Design](https://design.ros2.org/articles/actions.html)).
- Foxglove already runs with `include_hidden: true` — the services do appear, the liveliness check still fails.
- The `_action/*` prefix is hardcoded in `rclcpp_action::create_server()`; unhiding would require forking rclcpp and wouldn't change DDS behavior.

---

## Options

### A — Swap ddsrouter for `zenoh_bridge_ros2dds`
Replace the bridging layer with Eclipse Zenoh's ROS 2 plugin. Zenoh represents services/actions as **Queryables** and side-steps the DDS liveliness-matching problem.
- **Pros:** native topic + service + action bridging; actively developed; OSRF's current recommendation for distributed ROS 2; same config works for same-host Docker and remote robots.
- **Cons:** rip-and-replace of the whole bridging layer (compose + launch); new dep (`ros-jazzy-zenoh-bridge-ros2dds`); team has to learn it; cross-machine deploys need explicit `connect:` peer config.
- **Refs:** [zenoh-plugin-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) · [Jazzy docs](https://docs.ros.org/en/jazzy/p/zenoh_bridge_dds/) · [OSRF discourse](https://discourse.openrobotics.org/t/new-zenoh-bridge-for-ros-2/34163) · [Autoware writeup](https://autoware.org/driving-autoware-with-zenoh/) · [Humble tutorial](https://medium.com/@piliwilliam0306/use-zenoh-bridge-ros2dds-with-ros2-humble-459ab70ce9c7)

### B — Add `ros2/domain_bridge` alongside ddsrouter
Keep ddsrouter for topics; run a per-robot `domain_bridge` with an `actions:` block. Supposed to create a real `rclcpp_action::Server` on domain 0 that forwards to the robot.
- **We tried this in-session.** Bridge ran, config interpolated, but `ros2 action info` still showed `Action servers: 0`. Reverted.
- **Pros:** smallest config change, no new protocol.
- **Cons:** action support is a [4-year-old open issue (#11)](https://github.com/ros2/domain_bridge/issues/11); services support similarly thin ([#10](https://github.com/ros2/domain_bridge/issues/10)); [design doc](https://github.com/ros2/domain_bridge/blob/main/doc/design.md) admits QoS/liveliness limits; silent failure mode makes it hard to debug.
- **Refs:** [ros2/domain_bridge](https://github.com/ros2/domain_bridge) · [#11](https://github.com/ros2/domain_bridge/issues/11) · [#10](https://github.com/ros2/domain_bridge/issues/10) · [design doc](https://github.com/ros2/domain_bridge/blob/main/doc/design.md)

### C — Custom per-robot action relay node
One new ROS 2 package (`action_relay`). Per robot, one binary with two `rclcpp::Context` objects: a real `rclcpp_action::Server` on domain 0 and a real `rclcpp_action::Client` on domain N. Server forwards goals to client; feedback/result/cancel propagate through. ~150 lines of C++, templated over the 5 action types.
- **Pros:** bulletproof — it *is* a real server on domain 0, so the liveliness check trivially passes; no new deps; additive (ddsrouter still handles topics/images); scales per-robot.
- **Cons:** we maintain the code; new task executor → one line in a registry; careful two-executor / two-context threading; no widely-used package to vendor.
- **Refs:** [ROS 2 Actions Design](https://design.ros2.org/articles/actions.html) · [rclcpp_action API](https://docs.ros.org/en/jazzy/p/rclcpp_action/) · [multi_robot_tf_relay (related pattern)](https://github.com/Gabryss/multi_robot_tf_relay) · [Jazzy action server tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)

---

## Related reading

- [eProsima DDS Router docs](https://eprosima-dds-router.readthedocs.io/) — what we use today
- [ROS on DDS (design)](https://design.ros2.org/articles/ros_on_dds.html) — why services/actions use request/reply DDS topics
- [ROS 2 QoS — Deadline, Liveliness, Lifespan](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html) — explains `rcl_service_server_is_available` semantics
- [Programming Multiple Robots with ROS 2 (OSRF)](https://osrf.github.io/ros2multirobotbook/) — currently recommends Zenoh for cross-domain
- [Husarnet — Bridge Remote DDS Networks](https://husarnet.com/blog/ros2-dds-router)
