# (a) Judge-CLI cross-distro interop — 2026-09-11 04:55–05:00 EDT (host, docker only)

Publisher: `aerostack2/nightly-humble:82d85abc…` (ROS 2 Humble, FastDDS),
`ros2 topic pub -r 10 /drone0/odom nav_msgs/msg/Odometry`, `--network host --ipc host`, `ROS_DOMAIN_ID=1`.

| Observer | topic list -t | echo --once --csv | hz | node list |
|---|---|---|---|---|
| `ros:jazzy-ros-base` (daemon), stale host daemon present | ✗ (only rosout/parameter_events) | ✗ (type unknown) | ✓ 9.996 Hz | ✗ |
| `ros:humble-ros-base` (daemon), stale host daemon present | ✗ | ✗ | – | ✗ |
| `ros:jazzy-ros-base` `--no-daemon --spin-time 5` | ✓ `/drone0/odom [nav_msgs/msg/Odometry]` | – | – | (hidden node, expected empty) |
| `ros:humble-ros-base` (daemon) after `ros2 daemon stop` on host domain 1 | ✓ | ✓ `0,0,odom,,1.0,2.0,3.0,…` | – | (hidden node) |
| `ros:jazzy-ros-base` (daemon) after host daemon stop | ✓ | ✓ | – | (hidden node) |

Root cause of the ✗ rows: with host networking every process on the host
shares the ros2 daemon at `localhost:11511+domain`; a stale daemon from
another user (`/opt/ros/humble` CLI run earlier at domain 1 on this box,
plus a `fieldai` cyclonedds daemon at domain 0) answered the queries and
knew nothing of the FastDDS publisher. Humble↔Jazzy CLI interop itself
works. Decision: the pod's `ros2` is a wrapper into a persistent Humble
container (`study-judge-cli`) so the shared daemon is the platforms'
distro whichever side spawns it (`agent_study/osmo/pod/ros2-wrapper.sh`).
