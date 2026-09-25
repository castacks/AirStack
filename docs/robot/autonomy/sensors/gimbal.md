
# **Gimbal**

Two gimbals are available in Isaac Sim: the native, scripted mount of the MTL search scene
(below, recommended for new work), and the older UI **Gimbal Extension**.

## **Native MTL gimbal (`search_mission_scene.py`)**

`simulation/isaac-sim/launch_scripts/search_mission_scene.py` adds a scripted gimbal to every
drone it spawns. The [MTL search stack](../../../../stacks/mtl_search/README.md) uses it, and
it needs no UI extension. The stock Iris and its optional rigid ZED mount stay untouched; the
gimbal is authored alongside them:

```
/World/drone{i}/base_link/camera_gimbal          Xform: pose written every frame
/World/drone{i}/base_link/camera_gimbal/camera   UsdGeom.Camera, horizontal FOV = mission sensor.fov_deg
/World/MTL/Gimbal_robot_{i}                       OmniGraph with its own ROS2Context(domain_id = i)
```

| Topic | Type | Direction |
|---|---|---|
| `/robot_{i}/gimbal/cmd_pitch_yaw` | `geometry_msgs/Vector3` | in: `x = roll`, `y = pitch`, `z = yaw` [rad] |
| `/robot_{i}/gimbal/state` | `geometry_msgs/Vector3` | out: the angles the camera actually has (after limits and slew) |
| `/robot_{i}/gimbal/rgb` | `sensor_msgs/Image` | out, `frame_id: camera_optical_frame` |
| `/robot_{i}/gimbal/camera_info` | `sensor_msgs/CameraInfo` | out |

**Convention.** The angles are the Z-Y-X Euler angles `R = Rz(yaw)·Ry(pitch)·Rx(roll)` of the
camera frame in the **earth (ENU) frame**. The camera frame has `x` = boresight, `y` = image
left and `z` = image up. `pitch > 0` looks down, so nadir is `pitch = +π/2`. `yaw` is the
compass direction of the boresight, with `0` pointing East and `π/2` pointing North. The
earth frame is used because the mount is **earth-stabilised**, like a real 3-axis gimbal:
airframe attitude does not move the image. The camera position rides the airframe
(`p = p_body + R_body·mount_offset`).

**Actuator model.** Commands are clamped to the travel limits (roll ±80°, pitch −20°…110°).
Each axis then slews toward its command at a bounded rate (120°/s by default); yaw takes the
short way round. Until the first command arrives, the camera parks at
`initial_pitch_deg` (60°) along the airframe heading.

**Configuration.** All of these live in `airstack.sim_gimbal` of the scenario, from
`stacks/mtl_search/config/mission.yaml`:

- image size and publish rate;
- mount offset;
- slew rate;
- travel limits;
- initial pitch.

The FOV comes from `sensor.fov_deg`, so the rendered image and the detection model always
agree.

**How it moves.** An app-update callback reads the subscriber's outputs and steps the
actuator on sim time. It takes the airframe pose from the Pegasus vehicle state, because the
rigid body's pose lives in fabric, not USD. It then writes the gimbal Xform's local pose
relative to the (static) `base_link` spawn Xform, and publishes the resulting angles. On the
robot, `mtl_trajectory_follower` commands the gimbal and publishes the TF chain
`base_link → camera_gimbal_link → camera_optical_frame` from `gimbal/state`.

The Isaac-free parts are unit-tested in `simulation/isaac-sim/utils/mtl_scene/test`: the
kinematics, the USD camera axis fix-up, and the actuator. That includes a run of the scene
script against real `pxr` with the Kit, Pegasus and OmniGraph modules stubbed.

## **Legacy: Gimbal Extension (UI)**

### **Overview**  
The **Gimbal Extension** provides an easy way to integrate a controllable gimbal into an existing drone model within the scene. This extension is designed to facilitate the attachment and operation of a camera-equipped gimbal, allowing for real-time adjustments to pitch and yaw angles via ROS 2 messages.


### **Installation and Activation**  
To enable the **Gimbal Extension**, follow these steps:

1. Open the **Extensions** window by navigating to:  
   **Window** → **Extensions**

2. Under the **THIRD PARTIES** section, go to the **User** tab.
3. Locate the **Gimbal Extension** and turn it on.
4. Once enabled, a new **Gimbal Extension** window should appear.

### **Adding a Gimbal to a Drone**  
To attach a gimbal to an existing UAV model:

1. Copy the **prim path** of the UAV to which you want to add the gimbal.
2. In the **Gimbal Extension** window, paste the copied path into the **Robot Prim Path** text box.
3. Set the **Robot Index** based on the `DOMAIN_ID` of the drone.  
   - The `DOMAIN_ID` should match the identifier used for the robot to ensure proper communication.

For a step-by-step demonstration, refer to the video tutorial below:

<iframe src="https://drive.google.com/file/d/1pN0Pxe4nYQL1qs40oZTDqsOXsMrApLPM/preview" width="840" height="480" allow="autoplay" allowfullscreen="allowfullscreen"></iframe>

### **Gimbal Camera Image Topic**
Once the gimbal is successfully added, the camera image feed from the gimbal will be published on the following ROS 2 topic: `/robot_<ID>/gimbal/rgb`.

### **Controlling the Gimbal**
The gimbal pitch and yaw angles can be controled by the ros2 messages `/robot_<ID>/gimbal/desired_gimbal_pitch` and `/robot_<ID>/gimbal/desired_gimbal_yaw` of type `std_msgs/msg/Float64`, respectively.