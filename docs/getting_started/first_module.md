# Write Your First Module

A ~45 minute lesson. By the end you will have scaffolded an in-tree module,
implemented a small ROS 2 node inside it, built it in the robot container,
wired it into a stack of your own, and watched it appear in the stack's
observed wiring — the full life of an AirStack capability, in miniature.

**Prerequisite:** you finished the
[Modular AirStack Walkthrough](modular_airstack.md), so
`airstack up --stack full_default --sim isaac` works on your machine. Our
module, `odom_echo`, logs the arrival rate of the robot's canonical odometry
topic — trivial on purpose, so the *boundary motions* stay in focus.

## 1. Scaffold the module

```bash
airstack module create --in-tree odom_echo
```

This scaffolds a complete module boundary at
`robot/ros_ws/src/modules/odom_echo/`: a stub `module.yaml` manifest
([field reference](../../common/module_schema/README.md)) and an
`ament_python` package skeleton whose launch file declares its topic endpoint
as an **arg defaulting to the canonical name** (never a `<remap>` — wiring
belongs to whoever includes you). Trunk gitignores this path, so in a fork
commit with `git add -f`.

**Check:** `find robot/ros_ws/src/modules/odom_echo -type f | sort` prints
(paths relative to the module):

```text
module.yaml                          README.md
odom_echo/package.xml                odom_echo/setup.py
odom_echo/setup.cfg                  odom_echo/resource/odom_echo
odom_echo/odom_echo/__init__.py      odom_echo/odom_echo/odom_echo_node.py
odom_echo/launch/odom_echo.launch.xml
odom_echo/test/test_import.py
```

## 2. Implement the node

Open `robot/ros_ws/src/modules/odom_echo/odom_echo/odom_echo/odom_echo_node.py`
and replace the stub class with a rate logger (keep the generated `main()`):

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomEchoNode(Node):
    def __init__(self):
        super().__init__("odom_echo")
        self.declare_parameter("odometry_topic", "odometry")
        topic = self.get_parameter("odometry_topic").value
        self.count = 0
        self.create_subscription(Odometry, topic, self.on_odometry, 10)
        self.create_timer(2.0, self.report)

    def on_odometry(self, msg):
        self.count += 1

    def report(self):
        self.get_logger().info(f"odometry rate: {self.count / 2.0:.1f} Hz")
        self.count = 0
```

Two small edits alongside it: in `package.xml`, add
`<exec_depend>nav_msgs</exec_depend>` next to `rclpy` (every dependency gets
declared). In `launch/odom_echo.launch.xml`, delete the `robot_name` arg and
the node's `namespace="$(var robot_name)"` attribute — the stack dispatcher
already pushes `/$ROBOT_NAME` onto everything it includes, so keeping the
stub's namespace would nest your node at `/robot_1/robot_1/odom_echo`.

**Check:** `python3 -m py_compile <path to odom_echo_node.py>` exits silently
(no syntax errors).

## 3. Build it in the robot container

All builds happen inside Docker — start the robot container without
autolaunch, then hop in with `airstack connect` (it attaches to the
container's tmux session; open a fresh shell window with ++ctrl+b++ then
++c++) and build just your package with the `bws` alias:

```bash
airstack up robot-desktop --no-autolaunch
airstack connect robot-desktop
```

Then, inside the container:

```bash
bws --packages-select odom_echo
sws && ros2 pkg list | grep odom_echo
```

**Check:** the last command prints `odom_echo` — colcon found the package
through the `ros_ws` bind mount, and the build persists across restarts.
Detach from the container with ++ctrl+b++ then ++d++ (or `exit`).

## 4. Wire it into a stack of your own

Never edit a reference stack — copy one, then add your include. Run
`airstack stack new full_default my_stack`, open
`stacks/my_stack/launch/stack.launch.xml`, and append one line before
`</launch>` (a bare include means "wired canonically"):

```xml
<include file="$(find-pkg-share odom_echo)/launch/odom_echo.launch.xml" />
```

Relaunch on your stack (stack launch files are bind-mounted — no rebuild):

```bash
airstack down
airstack up --stack my_stack --sim isaac
airstack ready
```

**Check:** the node is alive under the robot namespace, reporting a live rate.
Connect to the container (`airstack connect robot-desktop`, new window with
++ctrl+b++ ++c++) and run:

```bash
ros2 node list | grep odom_echo
# /robot_1/odom_echo
```

Your node's log lines (`odometry rate ...`) are also visible from the host via
`docker logs airstack-robot-desktop-1` — tmux output is mirrored there.

## 5. See it in the wiring

Your stack has no `wiring.md` yet (`stack new` deliberately doesn't copy it —
that file is *observed*, never inherited). With the stack still running,
snapshot the live graph, then confirm there's no drift:

```bash
airstack doctor --snapshot --stack my_stack
airstack doctor --live --stack my_stack
```

**Check:** `grep odom_echo stacks/my_stack/wiring.md` shows your node and its
odometry edge, and `--live` reports `graph matches wiring.md`. (The CI-grade
route that stamps verified provenance:
`airstack test -m wiring --stack my_stack --sim isaacsim --num-robots 1`.)

## Congratulations

You scaffolded a module boundary, built a node inside it, composed it into
your own stack, and captured the result as observed wiring — the loop every
AirStack capability grows through. Next, one line each:

- Track extraction debt as your module grows: `airstack module doctor --drift`
  ([AirStack Modules](../development/modules.md))
- Graduate it to a standalone repo: the
  [extract-module skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/extract-module/SKILL.md)
- Give it CI and register it: [Module CI](../development/module_ci.md) and the
  [Module & Stack Catalog](../modules/index.md)
- Deeper stack authoring (splits, lints, `stack diff`):
  [Creating a Custom Stack Topology](../development/creating_a_stack.md)
