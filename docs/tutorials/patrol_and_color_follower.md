# Patrol & Color Follower — AirStack + ROS 2 Hands-On Tutorial

You will build two ROS 2 nodes from scratch inside AirStack:

1. **Patrol node** — arms the drone, flies a square waypoint loop, and records everything (camera + odometry + commands) to a rosbag the whole time.
2. **Color follower node** — arms the drone, takes off, then uses the camera to find a colored blob and yaws / climbs to keep it centered.

Both nodes plug into AirStack the same way the SVG swarm commander does: they publish velocity commands and call the `robot_command` service. No changes to the autonomy stack needed.

**What you'll practice:**

- Creating a Python ROS 2 package inside AirStack's Docker workflow
- Publishers, subscribers, timer callbacks, and async service clients
- Reading `sensor_msgs/Image` with `cv_bridge`
- Recording, inspecting, and replaying rosbags
- A simple P/PD controller for flight

**Prerequisites:** You have completed the getting started tutorial (containers up, drone takes off in Foxglove).

---

## How to use this tutorial

Each section has a **"What to do"** block telling you what to implement, then a **"Solution"** you can expand to check your work, followed by a **Checkpoint** to verify things are working before moving on.

Try to write the code yourself first. The checkpoint tells you exactly what to run to confirm it works — don't skip them.

---

## 0. Background: how AirStack's robot interface works

Before writing any code, spend 5 minutes understanding what you'll be talking to.

Every drone in AirStack exposes three things under its namespace (`/robot_1/` in sim):

| What | ROS name | Type | Direction |
|---|---|---|---|
| Current position & velocity | `/robot_1/odometry_conversion/odometry` | `nav_msgs/Odometry` | **subscribe** (read) |
| Velocity command | `/robot_1/interface/velocity_command` | `geometry_msgs/TwistStamped` | **publish** (write) |
| Arm / offboard / disarm | `/robot_1/interface/robot_command` | `airstack_msgs/srv/RobotCommand` | **service client** |
| Left camera image | `/robot_1/sensors/front_stereo/left/image_rect` | `sensor_msgs/Image` | **subscribe** (read) |

The `robot_command` service accepts three commands (integer constants on `RobotCommand.Request`):

- `REQUEST_CONTROL` — switch PX4 to offboard mode so your node can fly it
- `ARM` — spin up motors
- `DISARM` — cut motors

**Before takeoff you must send them in order:** stream zero-velocity commands for ~1 s so PX4 accepts offboard, then `REQUEST_CONTROL`, then `ARM`. This is the same sequence the SVG commander uses.

### Verify the topics exist

Start the sim (if not already running) and confirm the topics are there:

```bash
# Host
airstack connect robot --command=bash

# Inside container
source ~/AirStack/robot/ros_ws/install/setup.bash
ros2 topic list | grep robot_1
```

You should see `/robot_1/odometry_conversion/odometry` and `/robot_1/interface/velocity_command` in the list. If they're missing, the autonomy stack hasn't launched yet — check `AUTOLAUNCH` in `.env`.

---

## 1. Create the package

All development happens inside the robot container. Connect and navigate to the workspace:

```bash
airstack connect robot --command=bash
cd ~/AirStack/robot/ros_ws
```

Create a new Python ROS 2 package. A ROS 2 package is the unit of code organisation — it has a `package.xml` declaring its name and dependencies, a `setup.py` (for Python packages) listing its executables, and a Python module directory for your source files.

```bash
ros2 pkg create --build-type ament_python my_drone_demos \
  --dependencies rclpy geometry_msgs nav_msgs sensor_msgs std_srvs airstack_msgs cv_bridge
```

This creates `src/my_drone_demos/` with the standard scaffolding. Your node source files will go in `src/my_drone_demos/my_drone_demos/`.

> **Why `--dependencies`?** These get written into `package.xml` so `colcon build` knows what to compile first. If you forget a dependency, your node will fail to import at runtime with a cryptic `ModuleNotFoundError`.

Build it now to confirm the scaffolding is valid:

```bash
bws --packages-select my_drone_demos && sws
```

### ✅ Checkpoint 1

```bash
ros2 pkg list | grep my_drone_demos
```

You should see `my_drone_demos`. If you don't, the build failed — check the colcon output above for errors.

---

## 2. Part 1 — Waypoint Patrol

### 2a. Plan the state machine

The drone can't just receive a "go to X" command — PX4 in offboard mode requires a **continuous stream of commands** (at least 2 Hz, ideally 20 Hz) or it will fall back to hold mode. That means your control loop runs on a timer, and you need to track what phase of the flight you're in.

Design a simple state machine with these states:

```
IDLE → ARM → TAKEOFF → PATROL → LAND → IDLE
```

- **IDLE**: doing nothing, streaming nothing
- **ARM**: streaming zero-velocity, waiting 1 s, then sending `REQUEST_CONTROL` + `ARM`
- **TAKEOFF**: P-controller climbing to cruise altitude, transition to PATROL on arrival
- **PATROL**: P-controller seeking waypoints in a loop, advancing index on arrival
- **LAND**: descending at fixed speed, disarming when low enough

An operator service (`~/start`) kicks things off from IDLE → ARM, and `~/land` jumps to LAND from anywhere.

### 2b. Understand the P-controller

A **proportional controller** computes a velocity proportional to the error (distance to goal):

```
velocity = Kp × (goal_position − current_position)
```

With `Kp = 1.0` and a 2 m error the output is 2 m/s. You should clamp this to a maximum speed so the drone doesn't sprint and overshoot. A drone "arrives" at a waypoint when the error is below an **arrival threshold** (e.g. 0.3 m).

### 2c. Write the node

Create `src/my_drone_demos/my_drone_demos/patrol_node.py`.

**What to implement, in order:**

1. A `PatrolNode` class that inherits from `rclpy.node.Node`.
2. In `__init__`: declare parameters for `robot_name`, `cruise_altitude`, `patrol_speed`, `arrival_threshold`. Use `self.declare_parameter(name, default)` and `self.get_parameter(name).value` to read them back.
3. A publisher for `TwistStamped` on `/{robot_name}/interface/velocity_command`.
4. A subscriber for `Odometry` on `/{robot_name}/odometry_conversion/odometry`. Store the position as a `numpy` array.
5. A service client for `RobotCommand` on `/{robot_name}/interface/robot_command`. Make it async with `call_async`.
6. A 20 Hz timer that calls your control loop.
7. Two services (`std_srvs/Trigger`): `~/start` and `~/land`.
8. The state machine logic inside the timer callback (ARM timing, TAKEOFF P-controller, PATROL waypoint cycling, LAND descent).
9. A `main()` function that calls `rclpy.init`, constructs the node, `rclpy.spin`s it, and shuts down cleanly.

**Waypoints to use** (a 3 m square at cruise altitude):

```python
import numpy as np
WAYPOINTS = np.array([
    [ 1.5,  1.5, 1.5],
    [-1.5,  1.5, 1.5],
    [-1.5, -1.5, 1.5],
    [ 1.5, -1.5, 1.5],
])
```

**ARM timing sequence** (copy this pattern exactly — PX4 is fussy):

```
t=0:   start streaming zero velocity
t=1.0: send REQUEST_CONTROL
t=1.5: send ARM
t=2.5: transition to TAKEOFF
```

Use `self.get_clock().now()` and `.nanoseconds * 1e-9` to get elapsed seconds.

<details>
<summary>Solution: patrol_node.py</summary>

```python
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from airstack_msgs.srv import RobotCommand

WAYPOINTS = np.array([
    [ 1.5,  1.5, 1.5],
    [-1.5,  1.5, 1.5],
    [-1.5, -1.5, 1.5],
    [ 1.5, -1.5, 1.5],
])


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('cruise_altitude', 1.5)
        self.declare_parameter('patrol_speed', 0.5)
        self.declare_parameter('arrival_threshold', 0.3)
        self.declare_parameter('land_altitude', 0.15)

        name  = self.get_parameter('robot_name').value
        self._cruise_alt   = self.get_parameter('cruise_altitude').value
        self._speed        = self.get_parameter('patrol_speed').value
        self._arrival      = self.get_parameter('arrival_threshold').value
        self._land_alt     = self.get_parameter('land_altitude').value

        self._position = None
        self._state    = 'IDLE'
        self._wp_idx   = 0
        self._arm_start = None

        self._cmd_pub = self.create_publisher(
            TwistStamped, f'/{name}/interface/velocity_command', 10)
        self.create_subscription(
            Odometry, f'/{name}/odometry_conversion/odometry',
            self._odom_cb, 10)
        self._rc_client = self.create_client(
            RobotCommand, f'/{name}/interface/robot_command')

        self.create_service(Trigger, '~/start', self._handle_start)
        self.create_service(Trigger, '~/land',  self._handle_land)
        self.create_timer(0.05, self._loop)

        self.get_logger().info('PatrolNode ready — call ~/start to fly')

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._position = np.array([p.x, p.y, p.z])

    def _handle_start(self, _, resp):
        if self._position is None:
            resp.success = False
            resp.message = 'No odometry yet'
            return resp
        self._state = 'ARM'
        self._arm_start = self.get_clock().now()
        resp.success = True
        resp.message = 'Arming...'
        return resp

    def _handle_land(self, _, resp):
        self._state = 'LAND'
        resp.success = True
        resp.message = 'Landing'
        return resp

    def _send_robot_cmd(self, command: int, label: str):
        if not self._rc_client.service_is_ready():
            self.get_logger().warn(f'{label}: service not ready')
            return
        req = RobotCommand.Request()
        req.command = command
        future = self._rc_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'{label} → success={f.result().success}'))

    def _publish_vel(self, v: np.ndarray):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x = float(v[0])
        msg.twist.linear.y = float(v[1])
        msg.twist.linear.z = float(v[2])
        self._cmd_pub.publish(msg)

    def _p_vel(self, goal: np.ndarray) -> np.ndarray:
        error = goal - self._position
        dist  = np.linalg.norm(error)
        speed = min(self._speed, 1.0 * dist)
        return error / max(dist, 1e-6) * speed

    def _loop(self):
        if self._position is None:
            return
        now = self.get_clock().now()

        if self._state == 'IDLE':
            return

        elif self._state == 'ARM':
            self._publish_vel(np.zeros(3))
            elapsed = (now - self._arm_start).nanoseconds * 1e-9
            if elapsed > 1.0:
                self._send_robot_cmd(
                    RobotCommand.Request.REQUEST_CONTROL, 'offboard')
            if elapsed > 1.5:
                self._send_robot_cmd(RobotCommand.Request.ARM, 'arm')
            if elapsed > 2.5:
                self._state = 'TAKEOFF'
                self.get_logger().info('Ascending...')

        elif self._state == 'TAKEOFF':
            target = np.array([
                self._position[0], self._position[1], self._cruise_alt])
            vel = self._p_vel(target)
            self._publish_vel(vel)
            if np.linalg.norm(target - self._position) < self._arrival:
                self._state = 'PATROL'
                self.get_logger().info('Starting patrol')

        elif self._state == 'PATROL':
            goal = WAYPOINTS[self._wp_idx]
            vel  = self._p_vel(goal)
            self._publish_vel(vel)
            if np.linalg.norm(goal - self._position) < self._arrival:
                self._wp_idx = (self._wp_idx + 1) % len(WAYPOINTS)
                self.get_logger().info(
                    f'Waypoint reached → next: {self._wp_idx}')

        elif self._state == 'LAND':
            self._publish_vel(np.array([0.0, 0.0, -0.3]))
            if self._position[2] <= self._land_alt:
                self._send_robot_cmd(RobotCommand.Request.DISARM, 'disarm')
                self._state = 'IDLE'
                self.get_logger().info('Landed')


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

</details>

### 2d. Register the node executable

Open `src/my_drone_demos/setup.py`. Find the `entry_points` dict and add your node:

```python
entry_points={
    'console_scripts': [
        'patrol_node = my_drone_demos.patrol_node:main',
    ],
},
```

This is what lets `ros2 run my_drone_demos patrol_node` find your `main()`.

### 2e. Build and run

```bash
cd ~/AirStack/robot/ros_ws
bws --packages-select my_drone_demos && sws

# Terminal A — run the node
ros2 run my_drone_demos patrol_node

# Terminal B — trigger takeoff
ros2 service call /patrol_node/start std_srvs/srv/Trigger

# Terminal B — land early if needed
ros2 service call /patrol_node/land std_srvs/srv/Trigger
```

### ✅ Checkpoint 2

Watch the node's log output in Terminal A. You should see, in order:

```
[patrol_node]: PatrolNode ready — call ~/start to fly
[patrol_node]: Arming...          ← after calling /start
[patrol_node]: offboard → success=True
[patrol_node]: arm → success=True
[patrol_node]: Ascending...
[patrol_node]: Starting patrol
[patrol_node]: Waypoint reached → next: 1
[patrol_node]: Waypoint reached → next: 2
...
```

In Foxglove, open a **3D** panel with the `map` fixed frame and add an **Odometry** display for `/robot_1/odometry_conversion/odometry` — you should see the drone tracing the square.

If `arm → success=False`, the EKF hasn't converged yet (odometry not stable). Wait 30 seconds after the sim starts playing and try again.

---

## 3. Rosbags

A **rosbag** is a recording of ROS 2 topics serialised to disk. You can play it back later as if the system were running live — great for debugging, post-processing, or showing someone a flight without re-running the sim.

### 3a. What rosbags are made of

A bag directory contains:
- One or more `.mcap` files (the serialised message data)
- A `metadata.yaml` describing the topics, types, message counts, and time range

`Ctrl-C` on `ros2 bag record` writes the `metadata.yaml`. If you kill it with `kill -9`, the bag is unreadable — always use `Ctrl-C`.

### 3b. Important: where to save the bag

Inside the container, only paths under `~/AirStack/robot/ros_ws/` are bind-mounted to the host. Bags saved anywhere else (e.g. `~/bags/`) disappear when the container is removed.

```bash
mkdir -p ~/AirStack/robot/ros_ws/bags
```

### 3c. Record manually during a flight

Open a third terminal **before calling `/start`**:

```bash
airstack connect robot --command=bash
cd ~/AirStack/robot/ros_ws && sws

ros2 bag record -o bags/patrol_$(date +%H%M%S) \
  /robot_1/sensors/front_stereo/left/image_rect \
  /robot_1/odometry_conversion/odometry \
  /robot_1/interface/velocity_command
```

Let the drone fly one full loop, then `Ctrl-C` the recording.

### 3d. Inspect and replay

```bash
# How long, how many messages, what types?
ros2 bag info bags/patrol_HHMMSS

# Play back at real time (topics re-published as if live)
ros2 bag play bags/patrol_HHMMSS

# Play at half speed
ros2 bag play bags/patrol_HHMMSS --rate 0.5

# Play, pause at first message
ros2 bag play bags/patrol_HHMMSS --start-paused
# press Space to play/pause, Ctrl-C to stop
```

While a bag is playing, you can subscribe to the topics just like a live system:

```bash
# In another terminal
ros2 topic echo /robot_1/odometry_conversion/odometry --once
```

### 3e. Automate recording inside the node

**What to do:** Add rosbag recording to `PatrolNode` so it starts automatically when `~/start` is called and stops when the drone lands. Use `subprocess.Popen` to launch `ros2 bag record` as a child process, and `proc.terminate()` + `proc.wait()` to stop it cleanly.

Hints:
- `import subprocess, datetime`
- Build the bag path with a timestamp: `bags/patrol_` + `datetime.datetime.now().strftime('%H%M%S')`
- `subprocess.Popen(['ros2', 'bag', 'record', '-o', path, topic1, topic2, ...])`
- Call terminate in `_handle_land` and also in `_loop` when transitioning to IDLE after disarm

<details>
<summary>Solution: add recording to PatrolNode</summary>

Add to `__init__`:

```python
import subprocess, datetime

self._bag_proc = None
self._bag_topics = [
    f'/{name}/sensors/front_stereo/left/image_rect',
    f'/{name}/odometry_conversion/odometry',
    f'/{name}/interface/velocity_command',
]
```

Add these two methods:

```python
def _start_bag(self, name):
    stamp = datetime.datetime.now().strftime('%H%M%S')
    path  = f'/root/AirStack/robot/ros_ws/bags/patrol_{stamp}'
    cmd   = ['ros2', 'bag', 'record', '-o', path] + self._bag_topics
    self._bag_proc = subprocess.Popen(cmd)
    self.get_logger().info(f'Recording → {path}')

def _stop_bag(self):
    if self._bag_proc and self._bag_proc.poll() is None:
        self._bag_proc.terminate()
        self._bag_proc.wait()
        self.get_logger().info('Bag saved')
```

In `_handle_start`, after setting `self._state = 'ARM'`:

```python
robot_name = self.get_parameter('robot_name').value
self._start_bag(robot_name)
```

In the LAND branch, after `self._state = 'IDLE'`:

```python
self._stop_bag()
```

Also call `self._stop_bag()` in `finally` inside `main()` for clean Ctrl-C shutdown.

</details>

### ✅ Checkpoint 3

After a flight with automated recording:

```bash
ls ~/AirStack/robot/ros_ws/bags/
ros2 bag info ~/AirStack/robot/ros_ws/bags/patrol_HHMMSS
```

You should see three topics, a non-zero duration, and message counts for all three. The `image_rect` count will be the largest (camera runs at ~30 Hz vs 20 Hz odometry).

---

## 4. Part 2 — Color Follower

Now you'll use the camera. This node takes off and then tracks a colored object by yawing left/right and adjusting altitude.

### 4a. cv_bridge: from ROS to OpenCV

`sensor_msgs/Image` messages carry raw pixel data in a ROS-specific encoding. To work with them in Python you need `cv_bridge` to convert them to a numpy array (what OpenCV uses):

```python
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def image_callback(msg):
    # Convert ROS Image → OpenCV BGR numpy array
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # frame is now shape (H, W, 3), dtype uint8
```

To go the other way (publish a debug image):

```python
debug_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
pub.publish(debug_msg)
```

### 4b. HSV color detection

OpenCV's `inRange` in HSV (Hue-Saturation-Value) space is much more robust to lighting changes than RGB thresholding:

```python
hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, low_bound, high_bound)
# mask is uint8: 255 where the color matches, 0 elsewhere
```

HSV ranges in OpenCV: H ∈ [0,179], S ∈ [0,255], V ∈ [0,255].

Common starting ranges:

| Color | Low | High | Note |
|---|---|---|---|
| Red | `[0, 150, 100]` | `[10, 255, 255]` | Also check `[170,150,100]`→`[180,255,255]` (red wraps) |
| Green | `[40, 100, 100]` | `[80, 255, 255]` | |
| Blue | `[100, 150, 100]` | `[130, 255, 255]` | |
| Yellow | `[20, 150, 100]` | `[35, 255, 255]` | |

### 4c. Finding the blob centroid

Once you have the mask, find the largest connected blob and compute its centroid:

```python
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if contours:
    largest = max(contours, key=cv2.contourArea)
    M  = cv2.moments(largest)
    cx = M['m10'] / M['m00']   # x centroid in pixels
    cy = M['m01'] / M['m00']   # y centroid in pixels
```

### 4d. PD control for yaw and altitude

You want the blob to be at the center of the image: `(w/2, h/2)`. The **normalized error** maps the pixel offset to `[-1, +1]`:

```
yaw error = (cx - w/2) / (w/2)    # +1 means blob is at the right edge
alt error = (h/2 - cy) / (h/2)    # +1 means blob is at the top edge
```

A **PD controller** adds a derivative term (rate of change of error) to dampen oscillation:

```
output = Kp × error + Kd × (error − prev_error) / dt
```

For yaw: a positive yaw error (blob to the right) should produce a **negative** yaw rate (turn right in ENU — angular.z convention: positive = counter-clockwise). So:

```
yaw_rate = −(Kp_yaw × yaw_err + Kd_yaw × d_yaw_err/dt)
```

For altitude: a positive alt error (blob above center) should produce a **positive** vz (climb):

```
vz = Kp_alt × alt_err + Kd_alt × d_alt_err/dt
```

Always clamp the outputs:

```python
yaw_rate = np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE)
vz       = np.clip(vz,       -MAX_VZ,        MAX_VZ)
```

To publish yaw rate, use `msg.twist.angular.z`.

### 4e. Write the node

Create `src/my_drone_demos/my_drone_demos/color_follower_node.py`.

**What to implement:**

1. Same ARM → TAKEOFF state machine as the patrol node (you can copy that part).
2. A separate image callback that only runs control when state is `'TRACKING'`. TAKEOFF transitions to TRACKING when altitude is reached.
3. In the image callback:
   - Convert with `cv_bridge`
   - Build HSV mask
   - Find the largest contour; if none, publish zero velocity and return
   - Compute centroid and normalized errors
   - PD control → yaw_rate and vz
   - Publish velocity with a small constant `vx` (forward creep, e.g. 0.2 m/s)
4. A debug image publisher on `/color_follower/debug_image` showing the camera frame with a bounding box and the binary mask side by side. This is what you'll watch in Foxglove to tune the color.
5. The arm/takeoff timer still needs to run at 20 Hz to stream commands — the image callback fires at camera rate (~30 Hz) and only controls when tracking.

**Good starting gains:** `Kp_yaw=0.8`, `Kd_yaw=0.1`, `Kp_alt=0.5`, `Kd_alt=0.05`, `MAX_YAW_RATE=0.6`, `MAX_VZ=0.3`

<details>
<summary>Solution: color_follower_node.py</summary>

```python
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from airstack_msgs.srv import RobotCommand

# Tune these to your target object
HSV_LOW  = np.array([  0, 150, 100])   # red low
HSV_HIGH = np.array([ 10, 255, 255])   # red high
HSV_LOW2  = np.array([170, 150, 100])  # red wraps in HSV
HSV_HIGH2 = np.array([180, 255, 255])


class ColorFollowerNode(Node):

    def __init__(self):
        super().__init__('color_follower')
        self._bridge = CvBridge()

        self.declare_parameter('robot_name',    'robot_1')
        self.declare_parameter('cruise_alt',     1.5)
        self.declare_parameter('forward_speed',  0.2)
        self.declare_parameter('kp_yaw',  0.8)
        self.declare_parameter('kd_yaw',  0.1)
        self.declare_parameter('kp_alt',  0.5)
        self.declare_parameter('kd_alt',  0.05)
        self.declare_parameter('max_yaw_rate', 0.6)
        self.declare_parameter('max_vz',       0.3)
        self.declare_parameter('land_alt',     0.15)

        name = self.get_parameter('robot_name').value
        self._cruise_alt    = self.get_parameter('cruise_alt').value
        self._fwd           = self.get_parameter('forward_speed').value
        self._kp_yaw        = self.get_parameter('kp_yaw').value
        self._kd_yaw        = self.get_parameter('kd_yaw').value
        self._kp_alt        = self.get_parameter('kp_alt').value
        self._kd_alt        = self.get_parameter('kd_alt').value
        self._max_yaw       = self.get_parameter('max_yaw_rate').value
        self._max_vz        = self.get_parameter('max_vz').value
        self._land_alt      = self.get_parameter('land_alt').value

        self._position  = None
        self._state     = 'IDLE'
        self._arm_start = None

        self._prev_yaw_err = 0.0
        self._prev_alt_err = 0.0
        self._prev_t       = None

        self._cmd_pub = self.create_publisher(
            TwistStamped, f'/{name}/interface/velocity_command', 10)
        self._debug_pub = self.create_publisher(
            Image, '/color_follower/debug_image', 10)

        self.create_subscription(
            Odometry, f'/{name}/odometry_conversion/odometry',
            self._odom_cb, 10)
        self.create_subscription(
            Image, f'/{name}/sensors/front_stereo/left/image_rect',
            self._image_cb, 10)

        self._rc = self.create_client(
            RobotCommand, f'/{name}/interface/robot_command')

        self.create_service(Trigger, '~/start', self._handle_start)
        self.create_service(Trigger, '~/land',  self._handle_land)
        self.create_timer(0.05, self._arm_loop)

        self.get_logger().info('ColorFollower ready — call ~/start')

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self._position = np.array([p.x, p.y, p.z])

    def _handle_start(self, _, resp):
        if self._position is None:
            resp.success, resp.message = False, 'No odometry'
            return resp
        self._state     = 'ARM'
        self._arm_start = self.get_clock().now()
        resp.success, resp.message = True, 'Arming...'
        return resp

    def _handle_land(self, _, resp):
        self._state = 'LAND'
        resp.success, resp.message = True, 'Landing'
        return resp

    def _send_cmd(self, command, label):
        if not self._rc.service_is_ready():
            return
        req = RobotCommand.Request()
        req.command = command
        self._rc.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(
                f'{label} → {f.result().success}'))

    def _pub_vel(self, vx=0., vy=0., vz=0., yaw=0.):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x  = float(vx)
        msg.twist.linear.y  = float(vy)
        msg.twist.linear.z  = float(vz)
        msg.twist.angular.z = float(yaw)
        self._cmd_pub.publish(msg)

    def _arm_loop(self):
        if self._position is None or self._state == 'IDLE':
            return
        now = self.get_clock().now()

        if self._state == 'ARM':
            self._pub_vel()
            e = (now - self._arm_start).nanoseconds * 1e-9
            if e > 1.0: self._send_cmd(RobotCommand.Request.REQUEST_CONTROL, 'offboard')
            if e > 1.5: self._send_cmd(RobotCommand.Request.ARM, 'arm')
            if e > 2.5:
                self._state = 'TAKEOFF'

        elif self._state == 'TAKEOFF':
            err = np.array([0., 0., self._cruise_alt]) \
                - np.array([0., 0., self._position[2]])
            vz = float(np.clip(1.0 * err[2], -0.5, 0.5))
            self._pub_vel(vz=vz)
            if abs(err[2]) < 0.25:
                self._state = 'TRACKING'
                self.get_logger().info('Tracking — show me something colored!')

        elif self._state == 'LAND':
            self._pub_vel(vz=-0.3)
            if self._position[2] <= self._land_alt:
                self._send_cmd(RobotCommand.Request.DISARM, 'disarm')
                self._state = 'IDLE'

    def _image_cb(self, msg: Image):
        if self._state != 'TRACKING':
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w  = frame.shape[:2]
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask  = cv2.bitwise_or(
            cv2.inRange(hsv, HSV_LOW, HSV_HIGH),
            cv2.inRange(hsv, HSV_LOW2, HSV_HIGH2))

        now = self.get_clock().now()
        dt  = ((now - self._prev_t).nanoseconds * 1e-9
               if self._prev_t else 0.05)
        self._prev_t = now

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours or cv2.contourArea(max(contours, key=cv2.contourArea)) < 200:
            self._pub_vel()
            self._pub_debug(frame, mask)
            return

        largest = max(contours, key=cv2.contourArea)
        M  = cv2.moments(largest)
        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']

        yaw_err = (cx - w / 2) / (w / 2)
        alt_err = (h / 2 - cy) / (h / 2)

        yaw_rate = -(self._kp_yaw * yaw_err
                     + self._kd_yaw * (yaw_err - self._prev_yaw_err) / dt)
        vz       =   self._kp_alt * alt_err \
                     + self._kd_alt * (alt_err - self._prev_alt_err) / dt

        self._prev_yaw_err = yaw_err
        self._prev_alt_err = alt_err

        yaw_rate = float(np.clip(yaw_rate, -self._max_yaw, self._max_yaw))
        vz       = float(np.clip(vz,       -self._max_vz,  self._max_vz))

        self._pub_vel(vx=self._fwd, vz=vz, yaw=yaw_rate)

        x, y, bw, bh = cv2.boundingRect(largest)
        cv2.rectangle(frame, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
        cv2.circle(frame, (int(cx), int(cy)), 6, (0, 0, 255), -1)
        self._pub_debug(frame, mask)

    def _pub_debug(self, frame, mask):
        combined = np.hstack([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        self._debug_pub.publish(
            self._bridge.cv2_to_imgmsg(combined, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = ColorFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

</details>

### 4f. Register the entry point

In `setup.py`:

```python
entry_points={
    'console_scripts': [
        'patrol_node    = my_drone_demos.patrol_node:main',
        'color_follower = my_drone_demos.color_follower_node:main',
    ],
},
```

### 4g. Add a test object in Isaac Sim

You need something for the drone to see. In Isaac Sim:

1. **Create → Shapes → Sphere**
2. In the **Property** panel, create a new Material with a solid bright-red color (R=1, G=0, B=0)
3. Place the sphere about 3 m in front of the drone's spawn point at `z=1.5`

### 4h. Build and run

```bash
bws --packages-select my_drone_demos && sws

# Terminal A
ros2 run my_drone_demos color_follower

# Terminal B
ros2 service call /color_follower/start std_srvs/srv/Trigger
```

### 4i. Tune the color with the debug image

In Foxglove, add a **Raw Image** panel and set its topic to `/color_follower/debug_image`. You'll see:
- **Left half**: the raw camera feed with a green bounding box around what the node found
- **Right half**: the binary mask — white = detected, black = not detected

Adjust `HSV_LOW` / `HSV_HIGH` at the top of the file until **only your sphere** is white in the mask. Rebuild after each change (`bws --packages-select my_drone_demos && sws`).

> **Tip:** if the mask looks mostly empty, your color range is too tight. Start wider (S > 50, V > 50) and narrow in.

### ✅ Checkpoint 4

After calling `~/start` and waiting for takeoff:

1. The node log says `Tracking — show me something colored!`
2. The Foxglove debug image shows the sphere highlighted with a green box
3. The drone slowly rotates to keep the sphere near the center of frame
4. Moving the sphere left/right in Isaac Sim causes the drone to yaw to follow it

If the drone oscillates wildly, reduce `kp_yaw`. If it's sluggish, increase it.

---

## 5. Putting it all together

You now have two working nodes. Some things to try:

**Combine them:** Add image recording to the color follower using the same `subprocess.Popen` pattern from Part 1. Play the bag back after the flight and watch the drone's perspective.

**Override parameters at launch** (no rebuild needed):

```bash
ros2 run my_drone_demos patrol_node \
  --ros-args -p patrol_speed:=0.8 -p cruise_altitude:=2.0
```

**Echo the velocity commands** while flying to see what the controller is outputting:

```bash
ros2 topic echo /robot_1/interface/velocity_command
```

**Plot odometry from a bag** to see the patrol path:

```bash
ros2 bag play bags/patrol_HHMMSS
# In Foxglove: add a Plot panel, field /robot_1/odometry_conversion/odometry.pose.pose.position.x
```

---

## Reference: useful ROS 2 commands

```bash
# What topics are publishing right now?
ros2 topic list

# Is my topic actually sending data? (shows Hz)
ros2 topic hz /robot_1/odometry_conversion/odometry

# Print one message
ros2 topic echo /robot_1/interface/velocity_command --once

# What nodes are running?
ros2 node list

# What topics does my node publish/subscribe to?
ros2 node info /patrol_node

# Call a service manually
ros2 service call /patrol_node/start std_srvs/srv/Trigger

# Bag operations
ros2 bag record -o my_bag topic1 topic2 ...
ros2 bag info   my_bag/
ros2 bag play   my_bag/ --rate 0.5
```
