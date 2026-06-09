# mocap_planner

A simple local planner that uses mocap-provided odometry for indoor GPS-free flight. Designed as a sanity-check baseline with a pluggable policy interface for dropping in a real planner later.

## Overview

OptiTrack → NatNet → `natnet_ros2` → MAVROS → PX4 EKF → `/odometry`

This node subscribes to the clean odometry that comes out of that pipeline and publishes trajectories to the trajectory controller. It doesn't interact with mocap directly.

## Modes

| Mode | Behavior |
|------|----------|
| `hold` | Latches to the first received pose and holds there indefinitely |
| `waypoints` | Visits a list of (x, y, z, yaw) waypoints in order, holds at the last one |

Set `mode` in [config/mocap_planner.yaml](config/mocap_planner.yaml).

## Configuration

```yaml
mode: "hold"              # 'hold' or 'waypoints'
publish_rate: 10.0        # Hz
velocity: 0.5             # m/s (waypoints mode only)
acceptance_radius: 0.15   # m — how close counts as "reached"

# Flat list: x y z yaw  x y z yaw  ...  (yaw in radians)
waypoints: [0.0, 0.0, 1.0, 0.0,
            1.0, 0.0, 1.0, 0.0]
```

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/{robot_name}/odometry` | `nav_msgs/Odometry` | in |
| `/{robot_name}/trajectory_controller/trajectory_override` | `airstack_msgs/TrajectoryXYZVYaw` | out |

## Build & Run

### 1. Start the containers

In two separate terminals:

```bash
# Terminal 1 — robot stack
AUTOLAUNCH=false airstack up robot-desktop

# Terminal 2 — Isaac Sim
AUTOLAUNCH=false airstack up isaac-sim
```

Then attach to the Isaac Sim tmux session to launch it manually:

```bash
docker exec -it isaac-sim tmux attach -t isaac
```

Wait for Isaac Sim to fully load, then spawn a drone using the Pegasus extension UI.

### 2. Build the package

```bash
docker exec airstack-robot-desktop-1 bash -c "source /opt/ros/jazzy/setup.bash && cd /root/AirStack/robot/ros_ws && colcon build --packages-select mocap_planner"
```

### 3. Launch the planner

```bash
docker exec airstack-robot-desktop-1 bash -c "source /opt/ros/jazzy/setup.bash && source /root/AirStack/robot/ros_ws/install/setup.bash && ros2 launch mocap_planner mocap_planner.launch.xml"
```

You should see:
```
[mocap_planner_node-1] [INFO] [...] [mocap_planner]: mocap_planner started in 'hold' mode
```

## Verifying It Works

Work through these checks in order. Each one confirms a layer of the pipeline before moving to the next. All `docker exec` commands assume the robot container is running and Isaac Sim has a drone spawned.

### 1. Odometry is flowing

The planner won't publish anything until it receives odometry — confirm it's coming in:

```bash
docker exec airstack-robot-desktop-1 bash -c "source /opt/ros/jazzy/setup.bash && source /root/AirStack/robot/ros_ws/install/setup.bash && ros2 topic hz /robot_1/odometry"
```

Expected: ~50–100 Hz. If 0 Hz, the autonomy stack isn't running or PX4 hasn't initialized. Check that Isaac Sim has a drone spawned and PX4 SITL is connected.

### 2. Mocap data is arriving (real hardware only)

Skip this step when testing in simulation — Isaac Sim provides odometry directly. On real hardware with OptiTrack:

```bash
docker exec airstack-robot-desktop-1 bash -c "source /opt/ros/jazzy/setup.bash && source /root/AirStack/robot/ros_ws/install/setup.bash && ros2 topic hz /robot_1/mavros/vision_pose/pose"
```

Expected: ~120 Hz (OptiTrack default streaming rate).

- **No output / 0 Hz:** `natnet_ros2` is not connecting to Motive. Check `server_ip` in `natnet_config.yaml` matches the PC running Motive, and that the drone and Motive machine are on the same subnet.
- **Low Hz (< 50):** Network packet loss. Check WiFi signal, switch to a dedicated 5GHz AP if on a home router.

Then move the drone by hand and confirm the position in odometry tracks it:

```bash
docker exec airstack-robot-desktop-1 bash -c "source /opt/ros/jazzy/setup.bash && source /root/AirStack/robot/ros_ws/install/setup.bash && ros2 topic echo /robot_1/odometry --once"
```

- **Position is zeros or NaN:** EKF2 is not accepting the vision pose. In QGroundControl, check that `EKF2_EV_CTRL` has the position bit enabled and `EKF2_HGT_REF` is set to vision.
- **Position is wildly wrong:** Frame mismatch between OptiTrack world frame and PX4's NED frame. Check `frame_id` in `vision_pose_converter.yaml`.
- **Position jumps around:** Quaternion sign flips — make sure `canonical_quaternion: true` in `vision_pose_converter.yaml`.

### 3. The planner is publishing

```bash
docker exec airstack-robot-desktop-1 bash -c "source /opt/ros/jazzy/setup.bash && source /root/AirStack/robot/ros_ws/install/setup.bash && ros2 topic hz /robot_1/trajectory_controller/trajectory_override"
```

Expected: ~10 Hz (your `publish_rate`). If 0 Hz, odometry hasn't arrived yet (check step 1) or the topic remap is wrong.

Inspect what's being sent:

```bash
docker exec airstack-robot-desktop-1 bash -c "source /opt/ros/jazzy/setup.bash && source /root/AirStack/robot/ros_ws/install/setup.bash && ros2 topic echo /robot_1/trajectory_controller/trajectory_override"
```

In `hold` mode you should see a single waypoint whose position matches where the drone was when the node first received odometry.

### 4. The drone actually holds / moves

**In simulation:** Watch the drone in the Isaac Sim viewport. Arm it via QGroundControl and take off, then observe behavior.

**On real hardware:** Arm the drone, take off manually to ~1m, then hand control to the planner.

**Hold mode:** The drone should stay at the position it was at when the planner started. If it:
- **Drifts slowly** — mocap→EKF fusion is slightly off, check covariances in `natnet_config.yaml`
- **Oscillates** — trajectory controller gains need tuning, not a mocap issue
- **Flies away** — frame mismatch, stop immediately and recheck step 2

**Waypoints mode:** The drone should fly smoothly between the positions defined in `waypoints`. If the drone skips waypoints, increase `acceptance_radius`. If it never advances, decrease it.

If the drone holds position tightly in a stable hover, the mocap state estimation pipeline is healthy and ready to be trusted as feedback for the CBF planner.

## Plugging in a Real Planner

The policy interface is in [mocap_planner/policy/base.py](mocap_planner/policy/base.py):

```python
class PlannerPolicy(ABC):
    @abstractmethod
    def compute(self, odom: Odometry) -> TrajectoryXYZVYaw:
        ...
```

1. Write a class that inherits `PlannerPolicy` and implements `compute`
2. Instantiate it in `_build_policy` in [mocap_planner/node.py](mocap_planner/node.py)
