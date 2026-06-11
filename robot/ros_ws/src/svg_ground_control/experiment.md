# SVG Ground Control — Command Reference

> **Maintained file.** This is the canonical, copy-pasteable command
> reference for the SVG multi-drone CBF experiments. It is updated whenever
> the package or workflow changes. Every command block assumes a **fresh
> terminal**.

**Domain convention: `ROS_DOMAIN_ID=1` everywhere.** The robot container
auto-derives 1 from its name (`airstack-robot-desktop-1`); manually-started
containers and the mocap PC need an explicit `export ROS_DOMAIN_ID=1`.
Sanity-check `echo $ROS_DOMAIN_ID` in every shell before the first `ros2`
command — a mismatch shows up as "service unavailable" / missing topics.

## What runs where

| Node | Purpose |
|---|---|
| `swarm_commander` | central controller: scenario nominal → **CBF filter** → per-drone velocity commands; takeoff/start/hold/land services |
| `mocap_bridge` | hardware only: `/{name}/pose` → px4_interface `visual_odometry_in` |
| `natnet_ros2_node` | hardware only: OptiTrack Motive → per-body PoseStamped |
| `keyboard_teleop` | drives one teleop drone (`-p drone:=drone_3`) |
| per-drone interface stacks | MAVROS (sim) or px4_interface/uXRCE-DDS (real) |

**Scenarios** (`scenario:=` launch arg or config): `hover`, `random_walk`,
`random_goals`, `head_on`, `antipodal`, `squeeze`.
`teleop_drones` (comma-separated string in config) lists operator-driven,
CBF-exempt drones; empty string = fully autonomous swarm.
`squeeze` needs `drone_names` ordered `[holder, holder, intruder]`.

## Single-terminal workflow (tmux)

Instead of one host terminal per step, attach once (`./airstack.sh connect
robot`, no `--command=bash`) and use tmux windows — prefix is `Ctrl-b`
(press Ctrl+b, release, then the key):

- `Ctrl-b c` new window (tab) · `Ctrl-b n`/`p` or `Ctrl-b 0..9` switch
- `Ctrl-b ,` rename window · `Ctrl-b %`/`"` split panes · `Ctrl-b arrows` move · `Ctrl-b x` close focused pane
- `Ctrl-b [` scroll logs (`q` exits) · `Ctrl-b d` detach (everything keeps running)

One window each for A3 (interfaces), A4 (commander), A5 (teleop), A6
(service calls). Every new window is a fresh shell: repeat
`cd ~/AirStack/robot/ros_ws && sws`. Detach with `Ctrl-b d`, never `exit`,
to keep the session alive.

---

# Part A — Simulation

## A1. Start containers (host)

```bash
cd ~/AirStack
git checkout yikuan/SVG_ground_control

# .env requirements:
#   COMPOSE_PROFILES="desktop,isaac-sim"
#   AUTOLAUNCH="false"      <- the full autonomy stack would fight the commander
#   NUM_ROBOTS="1"          <- ONE robot container (sim drone count passed inline)
grep -E '^(COMPOSE_PROFILES|AUTOLAUNCH|NUM_ROBOTS|PLAY_SIM_ON_START)' .env

./airstack.sh up
./airstack.sh status        # robot-desktop-1 and isaac-sim must be Up
```

## A2. Isaac Sim — spawn drones (fresh terminal)

```bash
cd ~/AirStack
./airstack.sh connect isaac-sim --command=bash
```

Inside the container (`PLAY_SIM_ON_START=true` is REQUIRED — PX4 SITL only
launches once the timeline plays):

```bash
NUM_ROBOTS=3 SVG_DOMAIN_ID=1 PLAY_SIM_ON_START=true \
PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
/isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py \
  --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts
```

This terminal is then owned by the sim (logs, no prompt — normal). Look for
`Spawning 3 drone(s) on ROS domain 1`, then `PX4 Autolaunch: True` per drone.
If those `PX4 Autolaunch` lines never appear, the timeline isn't playing —
press Play in the Isaac window.

## A3. Build + MAVROS interfaces (fresh terminal)

```bash
cd ~/AirStack
./airstack.sh connect robot --command=bash
```

Inside:

```bash
echo $ROS_DOMAIN_ID                 # must print 1
cd ~/AirStack/robot/ros_ws
bws && sws                          # full build first time; later just sws

./src/svg_ground_control/scripts/launch_sim_interfaces.sh 3
```

Wait for each MAVROS to connect (benign warnings: `AUTOPILOT_VERSION ...
switched to default capabilities`). Verify from any other shell:

```bash
ros2 topic echo /drone_1/interface/mavros/state --once   # connected: true
ros2 topic hz /drone_1/odometry_conversion/odometry      # streams after EKF converges (~30 s)
```

## A4. Ground controller (fresh terminal)

```bash
cd ~/AirStack
./airstack.sh connect robot --command=bash
```

Inside (`sws` first in every shell):

```bash
cd ~/AirStack/robot/ros_ws && sws

# Default config (hover + teleop drone_3):
ros2 launch svg_ground_control ground_control.launch.py

# Scenario override on the default config:
ros2 launch svg_ground_control ground_control.launch.py scenario:=head_on
ros2 launch svg_ground_control ground_control.launch.py scenario:=random_goals
ros2 launch svg_ground_control ground_control.launch.py scenario:=random_walk

# Squeeze profile (3 drones: 2 steady holders + intruder through the gap):
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/squeeze_3drone.yaml
```

Squeeze geometry is fully explicit in `squeeze_3drone.yaml`:
`squeeze_holder_positions` (flat `[x1,y1,z1, x2,y2,z2]`) sets the two steady
drones' posts; `squeeze_intruder_waypoints` (flat `[ax,ay,az, bx,by,bz]`)
sets the two endpoints the moving drone shuttles between (takes off at A,
flies A→B→A→…). Rules: posts must be > 2r apart (the commander refuses
otherwise), and the A→B line should pass within 2r of a post or nothing
interesting happens. NOTE: edits to config YAMLs need a rebuild (`bws`) to
reach the installed copy — or pass `config:=` pointing at the source file
under `src/svg_ground_control/config/`.

To hand-fly the squeeze intruder instead, edit `teleop_drones: "drone_3"` in
`squeeze_3drone.yaml` (or your own copy) and drive it per A5.

## A5. Teleop (fresh terminal — needs its own TTY)

```bash
cd ~/AirStack
./airstack.sh connect robot --command=bash
```

Inside:

```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 run svg_ground_control keyboard_teleop --ros-args -p drone:=drone_3
# w/s = ±x, a/d = ±y, r/f = up/down, space = stop, +/- = speed step, q = quit
```

## A6. Fly (fresh terminal)

```bash
cd ~/AirStack
./airstack.sh connect robot --command=bash
```

Inside:

```bash
cd ~/AirStack/robot/ros_ws && sws

ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
# all drones arm, ascend to the scenario's initial layout, and HOLD there

ros2 service call /swarm_commander/start std_srvs/srv/Trigger
# scenario goes live (retry if it answers "not all drones holding yet")

ros2 service call /swarm_commander/hold std_srvs/srv/Trigger
# PANIC BUTTON: pause scenario, every drone freezes at its current position

ros2 service call /swarm_commander/land std_srvs/srv/Trigger
# descend + disarm
```

Watch the commander terminal: `CBF active on: drone_1, drone_2 (...)` lines
show the filter intervening; `CBF emergency push-apart engaged` means the QP
went infeasible (drones too close — investigate before continuing).

---

# Part B — Real Starlings + OptiTrack mocap

## B0. One-time setup

Per drone (QGC / VOXL shell):

- `uxrce_dds_client start -n drone_1` (each its own name) pointing at the
  ground PC IP, port 8888, and PX4 param **`UXRCE_DDS_DOM_ID = 1`**
  (defaults to 0 — a silent mismatch otherwise).
- `EKF2_EV_CTRL` = fuse external vision; GPS fusion off indoors.
- RC kill switch + offboard-loss failsafe configured.

Motive (OptiTrack): create one rigid body per drone, named exactly
`drone_1`, `drone_2`, ... (the natnet node publishes per-body topics by this
name). Enable NatNet streaming (unicast to the ground PC, or multicast).

NatNet SDK (ground PC, once — proprietary, downloaded not committed):

```bash
cd ~/AirStack
./robot/ros_ws/src/perception/natnet_ros2/scripts/download-natnet-sdk.sh
```

Edit `robot/ros_ws/src/perception/natnet_ros2/config/natnet_config.yaml`:
`server_ip:` = Motive PC IP, `body_id: -1` (track ALL bodies),
`publish_to_mavros: false`.

## B1. Host-network container + uXRCE agent (fresh terminal)

The stock container's bridge network can't hear the drones — use host
networking (image tag: `grep VERSION ~/AirStack/.env`):

```bash
cd ~/AirStack
docker run --rm -it --network host --name svg_ground \
  -v ~/AirStack:/home/robot/AirStack \
  airlab-docker.andrew.cmu.edu/airstack/airstack:v0.18.0_robot-x86-64_dev bash
```

Inside (manually-run containers do NOT auto-derive the domain):

```bash
export ROS_DOMAIN_ID=1
MicroXRCEAgent udp4 -p 8888
# each powered drone logs a session; /drone_i/fmu/out/* topics appear
```

## B2. ALL per-drone interfaces — one command (fresh terminal)

```bash
docker exec -it svg_ground bash
export ROS_DOMAIN_ID=1
cd ~/AirStack/robot/ros_ws && bws && sws    # bws once per code change

ros2 launch svg_ground_control real_interfaces.launch.py \
  drones:=drone_1,drone_2,drone_3
```

## B3. NatNet bridge (fresh terminal)

```bash
docker exec -it svg_ground bash
export ROS_DOMAIN_ID=1
cd ~/AirStack/robot/ros_ws && sws

ros2 launch natnet_ros2 natnet_ros2.launch.py
ros2 topic list | grep -i optitrack    # note the per-body pose topic paths
```

Set `mocap_topic_template` in `swarm_real.yaml` to match that path with
`{name}` in place of the body name (e.g.
`"/robot/perception/optitrack/{name}"`).

## B4. Commander + mocap bridge (fresh terminal)

```bash
docker exec -it svg_ground bash
export ROS_DOMAIN_ID=1
cd ~/AirStack/robot/ros_ws && sws

ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(pwd)/src/svg_ground_control/config/swarm_real.yaml \
  use_mocap:=true
# add scenario:=squeeze etc. to override the config's scenario
```

## B5. Preflight, then fly (fresh terminal)

```bash
docker exec -it svg_ground bash
export ROS_DOMAIN_ID=1
cd ~/AirStack/robot/ros_ws && sws

# per drone, props off / drone in hand:
ros2 topic hz   /drone_1/pose                                # mocap arriving (rename per your template)
ros2 topic echo /drone_1/odometry_conversion/odometry --once # tracks hand movement, meters, right signs

# teleop in its own docker-exec terminal (B-A5), then:
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger
ros2 service call /swarm_commander/hold    std_srvs/srv/Trigger   # panic
ros2 service call /swarm_commander/land    std_srvs/srv/Trigger
```

First hardware flight: ONE drone (`drone_names: ["drone_1"]`,
`teleop_drones: ""`, scenario `hover`), thumb on the RC kill switch. Then
two, then the full demo.

---

# Tests

```bash
# CBF + scenario unit tests (pure numpy, run anywhere):
cd ~/AirStack/robot/ros_ws/src/svg_ground_control
python3 -m pytest test/test_cbf.py test/test_scenarios.py -q

# Closed-loop functional test (fake drones, no sim needed) — terminal A:
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/squeeze_3drone.yaml
# terminal B:
python3 test/functional_squeeze_test.py
```

---

# Troubleshooting (issues actually hit)

| Symptom | Cause / fix |
|---|---|
| `[ERROR] Docker daemon is not running` but it is | user not in `docker` group: `sudo usermod -aG docker $USER`, re-login |
| `airstack connect` shows no prompt | it attaches to the container tmux; `Ctrl-b c` new window, `Ctrl-b d` detach — or use `--command=bash` |
| 3 robot containers appear | `.env NUM_ROBOTS` also scales container replicas; keep it `"1"`, pass drone count inline to the sim script |
| service `waiting for service to become available...` forever | `ROS_DOMAIN_ID` mismatch between shells; also try `ros2 daemon stop` |
| MAVROS `connected: false`, no odometry topics | PX4 SITL not launched: Isaac timeline not playing (`PLAY_SIM_ON_START=true`, or press Play) |
| `The parameter 'X' is not initialized` | empty YAML list can't override a typed param — `teleop_drones`/`external_drones` are comma-separated STRINGS |
| `start` answers "not all drones holding yet" | drones still converging to takeoff targets; retry after a few seconds |
