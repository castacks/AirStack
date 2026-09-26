# SVG Ground Control — Guide & Command Reference

> **Maintained file.** Canonical, copy-pasteable reference for the SVG
> multi-drone CBF experiments, updated whenever the package or workflow
> changes. Every command block assumes a **fresh terminal**.

## Contents
1. [How AirStack is structured](#1-how-airstack-is-structured)
2. [How SVG ground control is structured](#2-how-svg-ground-control-is-structured)
3. [Topic & service wiring](#3-topic--service-wiring)
4. [Conventions (domain, tmux, rebuilds)](#4-conventions)
5. [Part A — Simulation](#part-a--simulation)
6. [Part B — Bring in a real drone (connect + verify)](#part-b--bring-in-a-real-drone-connect--verify)
7. [Part C — Tasks: any drone in any mode](#part-c--tasks-any-drone-in-any-mode)
8. [Part D — Real hardware: first flight & reference](#part-d--real-hardware-first-flight--reference)
9. [RViz visualization](#rviz-visualization)
10. [Foxglove visualization (SVG Basestation panel)](#foxglove-visualization)
11. [Geofence](#geofence)
12. [Recording rosbags / monitoring](#recording-rosbags)
13. [Automated tests](#automated-tests)
14. [Troubleshooting](#troubleshooting)

---

## 1. How AirStack is structured

AirStack is a layered ROS 2 (Jazzy) autonomy stack that runs in Docker
containers. The pieces relevant to us:

```
┌─────────────────────────────────────────────────────────────────────┐
│ Docker containers (started by ./airstack.sh up, one bridge network)   │
│                                                                       │
│  isaac-sim ───── PX4 SITL (one per drone) ── MAVLink UDP / uXRCE-DDS  │
│   (Isaac Sim + Pegasus; physics, sensors, flight dynamics)            │
│                                                                       │
│  robot-desktop-1 ── the ROS 2 workspace (robot/ros_ws), where our     │
│                     nodes run. /AirStack is bind-mounted from host.    │
└─────────────────────────────────────────────────────────────────────┘
```

**The autonomy workspace** (`robot/ros_ws/src`) is organized in layers; the
ones we touch:

- `interface/` — talks to the flight controller. Two interchangeable plugins
  behind the same ROS API (`robot_interface_node`):
  - `mavros_interface` → PX4 over **MAVROS/MAVLink** (used for SIM/SITL).
  - `px4_interface` → PX4 over **uXRCE-DDS** (`/fmu/*` topics, used for
    HARDWARE). It converts ENU↔NED, runs the 10 Hz offboard heartbeat, and
    accepts `velocity_command` / `pose_command` / `robot_command`.
  - `odometry_conversion` — both plugins feed this; it republishes the
    canonical `…/odometry_conversion/odometry` (ENU `nav_msgs/Odometry`) and
    the `map→base_link` TF that everything downstream consumes.
- `perception/natnet_ros2` — OptiTrack Motive → ROS bridge (hardware mocap).
- `svg_ground_control/` — **our package** (this one). It sits *on top of*
  the interface layer: it reads each drone's odometry and writes each drone's
  velocity command, and otherwise ignores the stock AirStack planners.

Normally AirStack auto-launches the full autonomy stack; we run with
`AUTOLAUNCH=false` and launch only our nodes, so nothing fights us for
control.

---

## 2. How SVG ground control is structured

Five executables (`robot/ros_ws/src/svg_ground_control/svg_ground_control/`):

| Script | Node | What it does |
|---|---|---|
| `swarm_commander.py` | `swarm_commander` | **The brain.** 20 Hz loop: build each drone's *nominal* velocity (from the scenario or teleop) → run the **CBF safety filter** → publish a per-drone command (real: reference position + velocity + acceleration; sim: velocity). Owns each drone's reference point, takeoff/start/hold/land/reset_fence services, the geofence, and the RViz markers. |
| `scenarios.py` | (library) | Nominal-velocity policies: `hover`, `goal`, `random_walk`, `random_goals`, `head_on`, `antipodal`, `squeeze`. Pure NumPy, ported from `~/drone_soccer`. |
| `trajectory.py` | (library) | The go-to-goal law: PX4-style braking law + acceleration-limited reference profile (`goal_accel_mps2`, `goal_settle_s`). See [C1](#c1-single-drone-goal-goal_singleyaml). |
| `cbf_filter.py` | (library) | The velocity-CBF collision filter (`filter_velocities`), a verbatim port of `drone_soccer/cbf.py`. |
| `mocap_bridge.py` | `mocap_bridge` | Hardware only: `/{name}/pose` (mocap) → `/{name}/fmu/visual_odometry_in` for the PX4 EKF. |
| `safe_teleop/` | `safe_teleop` | Gamepad teleop for one `teleop_drones` drone: `/joy` → altitude-held ENU velocity on the teleop topic. Device = `teleop_controller` (`xbox_usb`; registry `safe_teleop/controllers.py`). See [teleop.md](teleop.md). |

**Data flow inside `swarm_commander` each tick:**

```
 per-drone odometry  ──► (add drone_position_offsets → shared world frame)
        │
        ▼
   reference point per drone (where it was told to be; PX4 holds it)
        │
        ▼
   scenario.nominal_velocity()   ── OR ──  teleop / goal-command input
        │  (per-drone desired velocity + acceleration feedforward, ENU;
        │   accel-limited profile evaluated at the reference, trajectory.py)
        ▼
   cbf_filter.filter_velocities()   ◄── sees ALL drones' world positions
        │  (collision-safe velocities; cbf_exempt rows restored after)
        ▼
   geofence check (hold_all: latch + freeze all if any drone outside the box;
                   keep_in: clip each command at the walls instead)
        │
        ▼
   publish  real: /{name}/fmu/trajectory_command (reference + velocity + accel)
            sim:  /{name}/interface/velocity_command
         +  /svg/viz/markers (RViz)
```

**Three independent per-drone axes** — set any combination in *any* task
config (see [Part C](#part-c--tasks-any-drone-in-any-mode)):

- **Mode** (`drone_modes`): `sim` (commands via MAVROS `/interface/…`) or
  `real` (commands via px4_interface `/fmu/…`). A `real` drone also shows up in
  the Isaac viewport at its live pose via an avatar (Part A `DRONE_MODES`).
  Mixed per run → hybrid.
- **Role** (`teleop_drones`, `external_drones`): `auto` (scenario-driven),
  `teleop` (operator-driven via a teleop topic), `external` (tracked for the
  CBF but never commanded — e.g. RC-flown). Unlisted = `auto`.
  **Convention: the standard experiments never use teleop** — a drone is
  `sim`, `real`, or `external`; teleop remains a debugging utility only (A5).
- **CBF-exempt** (`cbf_exempt_drones`): the filter still *sees* these drones
  (so everyone else avoids them) but leaves their *own* command uncorrected —
  they play the moving obstacle. Independent of role: a policy-driven (`auto`)
  drone or a `teleop` drone can be exempt. Teleop is **not** auto-exempt; list
  it here if you want its manual commands left unfiltered. (The `squeeze`
  scenario additionally self-designates its intruder via
  `squeeze_intruder_cbf_exempt`; the two union.)

**Lifecycle services** (`std_srvs/Trigger`):
`~/takeoff` (arm+offboard+ascend to the scenario's initial layout, then hold)
→ `~/start` (scenario goes live) → `~/hold` (panic freeze) →
`~/land` (descend+disarm). Plus `~/reset_fence` (clear a geofence latch).

---

## 3. Topic & service wiring

For each drone `{name}` (e.g. `drone_1`):

| Topic / service | Dir | Type | Who |
|---|---|---|---|
| `/{name}/odometry_conversion/odometry` | in | `nav_msgs/Odometry` | from interface layer → commander & RViz |
| `/{name}/interface/velocity_command` (sim) | out | `geometry_msgs/TwistStamped` | commander → MAVROS interface |
| `/{name}/fmu/velocity_command` (real) | out | `geometry_msgs/TwistStamped` | commander → px4_interface |
| `/{name}/interface/robot_command` or `/{name}/fmu/robot_command` | call | `airstack_msgs/srv/RobotCommand` | commander → arm/offboard/disarm |
| `/svg/{name}/teleop_command` | in | `geometry_msgs/TwistStamped` | safe_teleop → commander (teleop drones) |
| `/svg/{name}/goal_command` | in | `geometry_msgs/PoseStamped` | you → commander (`goal` scenario) |
| `/svg/{name}/speed_command` | in | `std_msgs/Float32` | you → commander (`goal` scenario) |
| `/{name}/pose` | in | `geometry_msgs/PoseStamped` | mocap → mocap_bridge (hardware) |
| `/{name}/fmu/visual_odometry_in` | out | `nav_msgs/Odometry` | mocap_bridge → px4_interface (hardware) |
| `/svg/viz/markers` | out | `visualization_msgs/MarkerArray` | commander → RViz (all drones, world frame) |
| `/swarm_commander/{takeoff,start,hold,land,reset_fence}` | call | `std_srvs/Trigger` | you → commander |

The state topic is the same for sim and real; only the command topic/service
namespace changes (`/interface/` vs `/fmu/`), which is exactly what
`drone_modes` selects per drone.

---

## 4. Conventions

**ROS domain = 1 everywhere.** The robot container's
[`.bashrc`](../../../docker/.bashrc) **hard-pins `ROS_DOMAIN_ID=1`** (overriding the
robot-name mapping), so every shell you open in it is already on domain 1 — this is
an image-level change, so rebuild the image after pulling (see [A1](#a1-containers-host)).
Manually-started containers and the mocap PC still need `export ROS_DOMAIN_ID=1`.
Check `echo $ROS_DOMAIN_ID` in every shell — a mismatch shows up as "service
unavailable" / missing topics.

**tmux** (when you `./airstack.sh connect robot` without `--command=bash`):
the `bringup` window opens as a 5-over-3 grid — top-left pane runs the autonomy
launch, the other 7 are shells that wait for that pane's `bws` to finish and
then `sws` automatically, so they are ready to use once the build is done
(layout and auto-source set by the `after-new-session` hook in
[`common/.tmux.conf`](../../../../common/.tmux.conf), helper `sws_after_build`
in [`robot/docker/.bashrc`](../../../docker/.bashrc)). `Ctrl-b` + arrow or
`Ctrl-b q [n]` jumps between panes, `Ctrl-b z` zooms one pane full-screen.
`Ctrl-b c` new window · `Ctrl-b n/p` or `Ctrl-b 0..9` switch · `Ctrl-b ,`
rename · `Ctrl-b %`/`"` split · `Ctrl-b x` close pane · `Ctrl-b [` scroll
(`q` exits) · `Ctrl-b d` detach (keeps running). Every new window is a fresh
shell: re-run `cd ~/AirStack/robot/ros_ws && sws`.

**Rebuild after edits.** `ros2 launch` reads the *installed* copy. After
editing any `.py`/`.yaml`/`.rviz` in the package, run `bws` (or pass
`config:=` pointing straight at the source file under `src/.../config/`).

**Drone BS** drone_1, using DDS Port 8888

---

# Part A — Simulation

The standard demo: 3 SITL drones, scenario from the config.

### A1. Containers (host)

```bash
cd ~/AirStack
git checkout yikuan/SVG_ground_control
./airstack.sh setup                         # FIRST TIME on a machine only — see note
./airstack.sh image-build robot-desktop     # REQUIRED after pulling this branch — see note
# .env: COMPOSE_PROFILES="desktop,isaac-sim", AUTOLAUNCH="false", NUM_ROBOTS="1"
grep -E '^(COMPOSE_PROFILES|AUTOLAUNCH|NUM_ROBOTS)' .env
./airstack.sh up
./airstack.sh status        # robot-desktop-1 and isaac-sim Up
```

> **First time on a machine: `./airstack.sh setup`.** It adds the `airstack`
> command to your shell profile (open a new terminal afterwards) and runs
> `config`, which creates two git-ignored files the Isaac Sim compose mounts:
> `simulation/isaac-sim/docker/omni_pass.env` and `user.config.json`. Without
> them `./airstack.sh up` fails with `env file ... omni_pass.env not found`.
> Press Enter at the Nucleus API-token prompt to keep the `guest` defaults, or
> copy the two `*_TEMPLATE*` files yourself (`omni_pass_TEMPLATE.env →
> omni_pass.env`, `user_TEMPLATE.config.json → user.config.json`). Also make
> sure your user is in the `docker` group (`sudo usermod -aG docker $USER`,
> then log out/in) — otherwise every command reports
> `Docker daemon is not running` even though it is.
>
> **⚠️ Always rebuild the robot image after pulling this branch.** This branch
> changes the robot **Docker image** itself (not just the bind-mounted workspace) —
> e.g. `MicroXRCEAgent` is now baked into the image
> ([`Dockerfile.robot`](../../../docker/Dockerfile.robot)), and the container
> [`.bashrc`](../../../docker/.bashrc) hard-pins `ROS_DOMAIN_ID=1`. Image contents
> only update on a rebuild, so a stale image will be missing the agent and may sit on
> the wrong domain. Rebuild with `./airstack.sh image-build robot-desktop` (or
> `./airstack.sh up --build`); add `--no-cache` if a layer looks stale. A plain
> `git pull` + `./airstack.sh up` is **not** enough. (Editing `.py`/`.yaml` inside the
> workspace still only needs `bws` — that's the bind mount, §4 — but anything that
> touches the Dockerfile or `.bashrc` needs an image rebuild.)

### A2. Isaac Sim — spawn drones (fresh terminal)

```bash
cd ~/AirStack && ./airstack.sh connect isaac-sim --command=bash
```
Inside (`PLAY_SIM_ON_START=true` is REQUIRED — PX4 SITL only launches when the
timeline plays; `ISAAC_SIM_HEADLESS=true` is REQUIRED unless you specifically
need the Isaac window — see note below):
```bash
NUM_ROBOTS=3 SVG_DOMAIN_ID=1 PLAY_SIM_ON_START=true ISAAC_SIM_HEADLESS=true \
PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
/isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py \
  --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts
```
Expect `Spawning 3 drone(s) on ROS domain 1` then `PX4 Autolaunch: True` per
drone. Drones spawn at x = −2, 0, +2 (this is why the sim configs set
`drone_position_offsets: [-2,0,0, 0,0,0, 2,0,0]`).

> **Real-drone avatars (`DRONE_MODES`).** For a hybrid run, tell the sim which
> drones are real so it spawns a SITL body only for the sim ones and a
> visual-only **avatar** for each real one — the avatar is teleported every
> step to that drone's `…/odometry_conversion/odometry`, so a real (mocap)
> drone appears in the Isaac viewport at its live pose. Add
> `DRONE_MODES="real,real,sim"` (length `NUM_ROBOTS`, matching the commander's
> `drone_modes`) to the launch, and run with a **GUI** viewport
> (`ISAAC_SIM_HEADLESS=false`) so you can see it. Used by the hybrid squeeze in
> [Part C](#part-c--tasks-any-drone-in-any-mode). The avatar's rclpy node joins
> the drones' domain automatically (it sets `ROS_DOMAIN_ID=SVG_DOMAIN_ID`); for
> true hardware the Isaac container must also be able to reach the real drones'
> DDS traffic (host networking / discovery server) — see Troubleshooting.

> **Run headless.** For SVG ground control you never need the Isaac viewport —
> physics, PX4 SITL, and the ROS topics all run headless, and you watch the
> drones in RViz (`/svg/viz/markers`) instead. The launcher defaults to GUI
> mode (`ISAAC_SIM_HEADLESS` unset → `false`), which opens a viewport window;
> running headless avoids the viewport entirely and is the right default. Pass
> `ISAAC_SIM_HEADLESS=true`. (When launched via `./airstack.sh up` with
> `AUTOLAUNCH=true`, set `ISAAC_SIM_HEADLESS=true` in `.env` instead.)
>
> ⚠️ **Headless does NOT fix an RTX renderer segfault.** If Isaac crashes with a
> `Segmentation fault` whose backtrace is in `librtx.scenedb.plugin.so` /
> `libcarb.scenerenderer-rtx.plugin.so` at `carbOnPluginStartup` — and it still
> crashes headless, and even a bare empty `SimulationApp({"headless":True})`
> crashes the same way — that is a **GPU driver ↔ Isaac Sim version
> incompatibility**, not an AirStack bug. Seen on RTX 5080 / Blackwell with
> NVIDIA driver 595.x and Isaac Sim 5.1.0: the app boots to `app ready`, then
> the RTX renderer faults on the first frame. Clearing the shader cache does
> not help. Fix = run a driver Isaac Sim 5.1 supports (Linux **580.65.06**, or
> **591.74** which a Blackwell user confirmed works — driver **595.x crashes**),
> or move to a newer Isaac Sim release. See Troubleshooting below.

### A3. Build + per-drone MAVROS interfaces (fresh terminal)

```bash
cd ~/AirStack && ./airstack.sh connect robot --command=bash
echo $ROS_DOMAIN_ID                       # 1
cd ~/AirStack/robot/ros_ws && bws && sws  # bws first time / after edits
./src/svg_ground_control/scripts/launch_sim_interfaces.sh 3
```
Verify (any other shell): `ros2 topic echo /drone_1/interface/mavros/state
--once` → `connected: true`, then `ros2 topic hz
/drone_1/odometry_conversion/odometry` (~30 Hz after EKF converges, ~30 s).

### A4. Ground controller (fresh terminal)

> **Prerequisite — the per-drone interfaces must already be running.** The
> commander reads each drone's state from `/{name}/odometry_conversion/odometry`,
> produced by the interface layer — start it **before** this step:
> * **sim** → `./src/svg_ground_control/scripts/launch_sim_interfaces.sh N` (A3, MAVROS)
> * **real** → `ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1,...`
>   (px4_interface, uXRCE-DDS — see [C0](#c0-start-the-per-drone-interfaces-required-before-any-task))
>
> Without it the commander logs `no drone eligible for takeoff (missing odometry)`.

```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 launch svg_ground_control ground_control.launch.py            # default (hover, all-auto)
# or pick a scenario:
ros2 launch svg_ground_control ground_control.launch.py scenario:=head_on
# or the squeeze profile:
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/squeeze_3drone.yaml
```

### A5. Gamepad teleop (optional — NOT used by any standard experiment)

> In the standard experiments a drone is **sim**, **real**, or **external**
> (RC-flown, tracked-only); the one Part C task that uses teleop is
> [C5](#c5-squeeze-with-a-hand-flown-intruder-squeeze_rc_intruderyaml), the
> hardware squeeze with a gamepad-flown intruder. Hand-flying otherwise has
> its own configs (`teleop_single.yaml` sim, `teleop_real.yaml` one real
> drone via `./svg_teleop.sh real`) — see [teleop.md](teleop.md). The
> keyboard teleop has been removed.

Any config takes a hand-flown drone by listing it in `teleop_drones` (or
`teleop_drones:=` on the commander launch). Teleop has its own launch: start
it FIRST, check the pad line it prints once a second, then start the
commander in a second terminal. Which device is the **`teleop_controller`**
parameter (config `safe_teleop` block or `teleop_controller:=`), an entry of
`svg_ground_control/safe_teleop/controllers.py`: `dragonrise_usb` (the generic
SHANWAN/DragonRise "Android gamepad" on the bench — the configs' default) or
`xbox_usb` (a real Xbox 360 pad); new devices are added there. The two differ
in axis numbers, not in how they fly. Identify an unknown pad with
`ros2 run svg_ground_control joy_map` before flying it — see
[teleop.md](teleop.md).

The pad must be visible **inside the container**: `docker exec
airstack-robot-desktop-1 ls /dev/input/js0`. If it is missing, recreate the
container (`AUTOLAUNCH=false airstack up robot-desktop`); `/dev` is populated
once at container start, so a pad plugged in later needs the `/dev/input`
bind mount that `robot-base-docker-compose.yaml` now sets.

```bash
# terminal 1: the pad (prints "pad: fwd .. left .. climb .. yaw .. | cmd vx .." — move the sticks)
cd ~/AirStack/robot/ros_ws && sws
ros2 launch svg_ground_control teleop.launch.py drone:=drone_3
# terminal 2: the commander, same config
ros2 launch svg_ground_control ground_control.launch.py scenario:=squeeze teleop_drones:=drone_3
# right stick = move, left stick = up/down + yaw, LB = lock. Position mode: release
# the sticks and the commander holds the drone where it is (teleop_kp / teleop_lead_m).
```

### A6. Fly (fresh terminal)

```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger   # arm+ascend+hold
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger   # scenario live
ros2 service call /swarm_commander/hold    std_srvs/srv/Trigger   # PANIC freeze
ros2 service call /swarm_commander/land    std_srvs/srv/Trigger   # descend+disarm
```

---

# Part B — Bring in a real drone (connect + verify)

Get one real drone talking to the stack and confirm it is tracked — the
hardware analogue of Part A's sim bring-up, with **no flight**. Once this
passes, a real drone is just `drone_modes: "...real..."` in any
[Part C](#part-c--tasks-any-drone-in-any-mode) task.

Use the **same `airstack`-managed robot container as Part A** — connect with
`./airstack.sh connect robot --command=bash`. Its [`robot/docker/.bashrc`](../../../docker/.bashrc)
already exports `ROS_DOMAIN_ID` (resolved per container, =1 here) and sources
the workspace at shell startup, and `bws`/`sws` are available — so the commands
below need **no `export ROS_DOMAIN_ID` and no manual `source`**. The state topic
is identical to sim (`…/odometry_conversion/odometry`); only the source changes
(mocap + px4_interface instead of SITL + MAVROS).

> **Prerequisite — robot container on host networking.** Hardware mocap (NatNet
> from Motive) and the drone's uXRCE-DDS link are on your LAN, which the default
> Docker bridge can't reach. Put `robot-desktop` on the host's network stack in
> [`robot/docker/docker-compose.yaml`](../../../docker/docker-compose.yaml): comment
> out its `networks:`/`ports:` and set `network_mode: host`, then `./airstack.sh
> up`. (Host mode ⇒ `NUM_ROBOTS=1`, since replicas would clash on ports — which
> is what this workflow uses anyway.) Verify with `./airstack.sh status`.

### B0. Get the drone onto your LAN (Wi-Fi / DHCP)

The uXRCE-DDS link (B2/B3) needs the drone reachable on the same subnet as this
PC. ADB into the VOXL and check the Wi-Fi interface:
```bash
adb shell
ip addr show wlan0            # is there an inet, and is it on the router's subnet?
```
**If `wlan0` has a stale static IP** (e.g. a hard-coded `192.168.30.20` from a
previous network) or no lease, flush it and request DHCP from the router:
```bash
ip addr flush dev wlan0       # drop the old/static address
ip link set wlan0 up
udhcpc -i wlan0               # busybox DHCP client (common on VOXL/embedded)
# or, if udhcpc isn't present:
dhclient -v wlan0            # ISC client
ip addr show wlan0           # should now show a router-assigned address
```
If `udhcpc` gets a lease, the router/DHCP path is fine. To make it **persist
across reboots** when `systemd-networkd` manages the interface, find the pinning
file and switch it to DHCP:
```bash
ls /etc/systemd/network/      # look for a *wlan0*.network with a static Address=
networkctl status wlan0       # shows who manages it + current address
```
Edit (or add) that `.network` file so it reads:
```ini
[Match]
Name=wlan0

[Network]
DHCP=yes
```
then `systemctl restart systemd-networkd`.

Notes:
- If DHCP keeps failing, `wlan0` probably isn't associated — confirm with
  `voxl-wifi status` / `iw wlan0 link` before chasing DHCP.
- Don't run a manual `udhcpc` **and** `voxl-wifi`'s managed client at once — they
  fight over the interface. For a drone you'll fly, prefer `voxl-wifi station` so
  it reconnects after every reboot.
- Want a fixed address per drone? Use a **DHCP reservation on the router**, not a
  static IP on the VOXL — avoids pool collisions and survives re-imaging.

### B1. Per-drone one-time setup

**(a) VOXL2 comms — one-shot script.** [`scripts/voxl_setup_real_drone.sh`](scripts/voxl_setup_real_drone.sh)
does the entire comms bring-up *on the VOXL*: points PX4's client at the ground
PC, namespaces topics to `/{name}/fmu/...`, pins the DDS domain, disables the
onboard `voxl-microdds-agent`, restarts `voxl-px4`, and verifies the session.
Idempotent — safe to re-run or re-point to a new IP/name.

*Getting it onto the drone:*
```bash
# from the ground PC (repo root):
adb push robot/ros_ws/src/svg_ground_control/scripts/voxl_setup_real_drone.sh /usr/bin/
# in the VOXL adb shell (root):
chmod +x /usr/bin/voxl_setup_real_drone.sh
voxl_setup_real_drone.sh <robot_name> <ground_pc_ip> [domain_id=1] [port=8888]
#   e.g.  voxl_setup_real_drone.sh drone_1 192.168.123.134 1 8888
# confirm the edit landed + the client connected:
grep -n 'microdds_client start' /usr/bin/voxl-px4-start   # -h <ip> -p 8888 -n <name>
px4-microdds_client status                                # "connected", Agent IP=<ground_pc_ip>
# If not connected:
px4-microdds_client start -t udp -h 192.168.50.2 -p <port> -n <name>
```

*Required on the ground side* — the agent must run where the topics are created
(B2), then verify on the **ground PC** (robot container, `ROS_DOMAIN_ID=1`):
```bash
cd ~/AirStack && ./airstack.sh connect robot --command=bash
MicroXRCEAgent udp4 -p 8888 -v4            # leave running; logs "session established <VOXL_IP>"
# another shell on the ground PC:
ros2 topic list | grep drone_1/fmu         # /drone_1/fmu/out/vehicle_status, .../vehicle_odometry, ...
ros2 topic echo /drone_1/fmu/out/vehicle_status --qos-reliability best_effort --once
```
`/fmu/*` topics live on the **ground PC** (the agent host), **never** on the VOXL
— that is the XRCE-DDS design (the VOXL runs only the thin client).

> **Why the script exists (facts learned the hard way).** The agent host IP lives
> only in the `-h` flag of `microdds_client start` in `/usr/bin/voxl-px4-start` on
> the VOXL (no PX4 param stores it on this SDK); a passive reboot is unreliable —
> `systemctl restart voxl-px4` is what re-reads the edit. The DDS domain is the
> **client** param (`UXRCE_DDS_DOM_ID`, or older `XRCE_DDS_DOM_ID` on ModalAI
> builds), **not** the agent's `ROS_DOMAIN_ID` — it must match your ground
> consumers' `ROS_DOMAIN_ID` (=1). VOXL2 runs ROS 2 **Foxy**; keep its native
> topics off the Jazzy ground domain (the XRCE bridge re-emits `/fmu` as
> Jazzy-native, which is safe — see [B6](#b6-voxl2-diagnostics-cheat-sheet)). On
> the VOXL you verify the bridge **only** with `px4-microdds_client status`.

**(b) Flight params (separate — NOT done by the script).** For an actual flight
you still need, per drone (QGC or `px4-param`): `EKF2_EV_CTRL` to fuse external
vision (GPS off indoors), an RC kill switch, and an offboard-loss failsafe — see
[Part D](#part-d--real-hardware-first-flight--reference). The script wires up
*comms only*.

> **`MAV_SYS_ID` (per drone).** Give each drone a distinct id (drone_N → N) so
> QGC can show all of them. PX4's commander **drops any VehicleCommand whose
> `target_system` ≠ its own `MAV_SYS_ID`** — this filter applies to uXRCE-DDS
> commands too, the DDS domain id has nothing to do with it. `px4_interface`
> therefore has a `target_system` parameter; `real_interfaces.launch.py` sets it
> from the trailing number of each name by default (`drone_2` → 2), or pass
> `target_systems:=1,2,3` explicitly. Symptom of a mismatch: "Arm command sent"
> / `arm -> success=True` in the logs, the drone never arms, and **no**
> `fmu/out/vehicle_command_ack` ever appears.

**(c) Motive / NatNet (mocap) — one-time.** Name one rigid body per drone
`drone_1`, `drone_2`, … in Motive, set the OptiTrack streaming **Up Axis = Z**,
enable Broadcast Frame, and pick the right Local Interface IP. The vendored
[`natnet_ros2`](../../perception/natnet_ros2) package (L2S-lab) **auto-downloads
the NatNet SDK** into `deps/NatNetSDK` on its **first build** (needs internet) and
**must be built with `--symlink-install`** — which `bws` already passes:
```bash
bws --packages-select natnet_ros2 && sws
```
The OptiTrack server/client IPs are launch args (`serverIP`/`clientIP`), already
defaulted to this rig in
[`natnet_ros2.launch.py`](../../perception/natnet_ros2/launch/natnet_ros2.launch.py)
together with `pub_rigid_body:=true` (so per-body `/…/pose` topics are published,
not just TF). Override per run if needed:
`ros2 launch natnet_ros2 natnet_ros2.launch.py serverIP:=… clientIP:=…`.

> **Topic naming.** Each Motive rigid body is published on its own
> `geometry_msgs/PoseStamped` topic **`/<body-name>/pose`** (e.g. `/drone_1/pose`)
> and broadcast as a TF frame — when `pub_rigid_body:=true` (now the default; with
> it `false` you get **only** `/tf` and no `/…/pose`). Name your Motive bodies
> `drone_1`/`drone_2`/… and discover them with `ros2 topic list | grep pose`.
> Unlabeled markers are configured in `config/initiate.yaml`.

**(d) Onboard LED strip — one-time per drone.** Each drone's NeoPixel strip
(11 RGBW pixels on the ESC LED output) is driven by a small **no-ROS** daemon on
the VOXL, [`scripts/svg_led_daemon.py`](scripts/svg_led_daemon.py), which
writes the ESC LED packet into voxl-px4's `/run/mpa/modal_io_bridge` pipe (port
of the ModalAI `modal_io.c` reference in `led_ws/`) and takes color commands
over **UDP** from the ground node `led_controller` (started by
`ground_control.launch.py`, `use_led:=true` by default). No ROS on the VOXL on
purpose — its Foxy DDS must stay off the Jazzy ground domain ([B6](#b6-voxl2-diagnostics-cheat-sheet)).
The daemon shows **green at boot** (before any ground link), sends a 1 Hz
heartbeat to the ground PC (IP read from the `-h` flag in `voxl-px4-start`, so
run (a) first), and falls back to green 10 s after the ground goes silent.
```bash
# over Wi-Fi, from the ground PC (package dir) — scp + ssh + installer in one go
# (VOXL root password: oelinux123; `ssh-copy-id root@<ip>` once to stop the prompts):
scripts/voxl_push_led.sh drone_1 <drone_ip>            # [ground_pc_ip] [num_leds=11]
#   = scp scripts/svg_led_daemon.py scripts/voxl_setup_led.sh root@<drone_ip>:/usr/bin/
#     ssh root@<drone_ip> 'chmod +x /usr/bin/voxl_setup_led.sh && voxl_setup_led.sh drone_1'
#   RGB (not RGBW) strip / other brightness:  LED_EXTRA_ARGS="--rgb --brightness 60" scripts/voxl_push_led.sh drone_1 <ip>
# over USB instead:
adb push scripts/svg_led_daemon.py scripts/voxl_setup_led.sh /usr/bin/
adb shell 'chmod +x /usr/bin/voxl_setup_led.sh && voxl_setup_led.sh drone_1'
# on the VOXL, to check:
systemctl status svg-led ; journalctl -u svg-led -n 20   # "opened MAVLink tunnel sink", "PX4 ESC LED bits muted"
```
*Ground side, once:* the heartbeats arrive on **UDP 47901** — this host runs
`ufw`, so `sudo ufw allow 47901/udp`. The robot container is `network_mode: host`,
so nothing else to map. Verify in the commander terminal:
`[led_controller]: drone_1: LED daemon online at <ip>:47900`. Recolor live:
```bash
ros2 topic pub --once /svg/led_command std_msgs/msg/String "{data: 'drone_1 blue'}"        # name …
ros2 topic pub --once /svg/led_command std_msgs/msg/String "{data: 'drone_1 255,60,0'}"    # … or r,g,b[,w]
ros2 topic pub --once /svg/led_command std_msgs/msg/String "{data: 'all green blink'}"     # all drones, blink
# same thing as a service:
ros2 service call /svg/drone_1/set_led_color airstack_msgs/srv/SetLedColor "{color: blue}"
```
Works whenever `led_controller` is running (it is part of `ground_control.launch.py`;
standalone: `ros2 run svg_ground_control led_controller --ros-args --params-file <config>.yaml`),
regardless of arming or flight state.
The daemon also mutes PX4's ESC status-LED bits (`px4-qshell voxl_esc -l 0 led`),
otherwise the driver repaints the strip with its own red/green/blue arm state and
it flickers orange — see the troubleshooting row. Colors: off/red/green/blue/white/yellow/cyan/magenta/orange/purple, scaled by
`led_controller.brightness` (80/255 default — bright white washes out the
OptiTrack IR view). A manual color is the drone's *base* color; the CBF red
(below) overrides it while active, then returns to it.

### B2. uXRCE-DDS agent (ground PC)

The agent bridges the drone's PX4 client to ROS `/fmu/*` topics and **creates
them on the ground PC** (not the VOXL). The robot image ships `MicroXRCEAgent`,
so run it directly in the robot container (host network, `ROS_DOMAIN_ID=1`):
```bash
MicroXRCEAgent udp4 -p 8888 -v4
```
`-v4` logs each session/datawriter, so you can watch the drone attach
(`create_client … session established … <VOXL_IP>`). The `/fmu/*` topics land on
the domain the **PX4 client** requested (the `domain_id` the B1 script set, =1),
so make sure your ground consumers (`px4_interface`, the commander, your `ros2`
shells) are on `ROS_DOMAIN_ID=1` too — the agent's own env domain is not the
lever.

> **Where `MicroXRCEAgent` comes from — two interchangeable installs:**
> 1. **Baked into the robot image** (the default):
>    [`Dockerfile.robot`](../../../docker/Dockerfile.robot) builds eProsima
>    Micro-XRCE-DDS-Agent v2.4.3 into `/opt/uxrce` (builder stage ~L198) and
>    copies it + adds it to `PATH` in the runtime stage (~L364). Ships with the
>    image on every machine; needs `./airstack.sh image-build robot-desktop`
>    after pulling.
> 2. **Built in the ROS workspace** (per-machine, no image rebuild — the repo
>    is colcon-buildable):
>    ```bash
>    cd ~/AirStack/robot/ros_ws/src
>    git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
>    cd ~/AirStack/robot/ros_ws && bws --packages-select microxrcedds_agent && sws
>    ```
>    Lives on the host bind mount (survives container restarts), but is lost on
>    `cws`/fresh checkout and must be rebuilt per machine. First build needs
>    internet (the superbuild fetches Fast-DDS).
>
> Last-resort fallback if neither is available:
> `docker run --rm -it --network host -e ROS_DOMAIN_ID=1 microros/micro-ros-agent:jazzy udp4 --port 8888`.

### B3. Per-drone px4_interface (fresh terminal)

```bash
cd ~/AirStack && ./airstack.sh connect robot --command=bash
ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1   # add ,drone_2,...
```

### B4. NatNet mocap — launch + UNIT-TEST (fresh terminal)

Build natnet first (it needs `--symlink-install`, which `bws` passes; the first
build also downloads the NatNet SDK — needs internet):
```bash
cd ~/AirStack && ./airstack.sh connect robot --command=bash
bws --packages-select natnet_ros2 && sws
ros2 launch natnet_ros2 natnet_ros2.launch.py   # serverIP/clientIP default to this rig
ros2 topic list | grep pose                      # each Motive body -> /<body-name>/pose
```
Then verify mocap is actually streaming (this is the unit-test — do NOT skip):
```bash
ros2 topic hz   /drone_1/pose                 # ~180 Hz (or your Motive rate)
ros2 topic echo /drone_1/pose --once          # sane x,y,z = where the drone sits
# move the drone by hand: position must change smoothly, no NaNs / jumps
```
- No topic / 0 Hz → Motive not streaming, wrong `serverIP`, or the rigid body
  isn't named `drone_1`. **Only `/tf` and no `/…/pose`** → `pub_rigid_body` is
  `false` (the vendored launch defaults it `true`).
- `mocap_bridge` consumes `/<name>/pose` (`mocap_topic_template: "/{name}/pose"`
  in `swarm_real.yaml`), forwarding mocap → PX4 visual odometry.

### B4b. External vision → EKF2 (the arm blocker)

Indoors with no GPS/VIO, PX4 EKF2 has **no position source** unless mocap is
fed in as external vision. Until it fuses one it produces no estimate, emits
no `/fmu/out/vehicle_odometry`, and **refuses to arm ("fuse failure")**. The
feed path (with `px4_vio_mode: direct`, the default):

```
/{name}/pose ─ mocap_bridge ─► /{name}/fmu/in/vehicle_visual_odometry
            (px4_msgs/VehicleOdometry: timestamp 0, quality 100, pose-only)
                                   │
                                   ▼  EKF2 (needs EKF2_EV_CTRL set)
                            /{name}/fmu/out/vehicle_odometry
```

**1. PX4 params for pure-mocap flight (per drone, once — QGC or `px4-param`;
the comms script does NOT set these).** Without them PX4 ignores
`vehicle_visual_odometry` entirely, or keeps fusing mag/GPS against it:
```
# --- external vision in ---
EKF2_EV_CTRL = 11      # bitmask: horiz pos(1) + vert pos(2) + yaw(8) = 11
                       #   yaw bit 8 REQUIRED for mocap yaw; without it EKF
                       #   takes heading from the mag -> "yaw estimate error"
EKF2_HGT_REF = 3       # height reference = Vision
EKF2_EV_DELAY ≈ 50     # ms; mocap-over-WiFi latency (tune)

# --- competing sources OFF (indoors, mocap-only) ---
EKF2_GPS_CTRL = 0      # no GPS indoors
EKF2_MAG_TYPE = 5      # magnetometer = None (EKF side)
SYS_HAS_MAG   = 0      # SYSTEM side — easy to miss; without it the EKF still
                       #   waits on / fuses the mag (cs_mag_hdg stays true)
# EKF2_BARO_CTRL = 0   # optional: also drop baro height (EV height only)
```
Then **save and reboot** — both are required:
```bash
px4-param save                    # unsaved params DIE on power loss
systemctl restart voxl-px4        # mag/EV fusion is configured at EKF INIT;
                                  # a live param change does not take effect
```
> Older ModalAI builds have `EKF2_AID_MASK` instead of `EKF2_EV_CTRL`
> (vision-position + vision-yaw bits) — check with `px4-param show EKF2_EV_CTRL`.

**1b. Disable onboard VIO (pure mocap only).** A stock ModalAI drone runs its
own VIO (`voxl-qvio-server` → `voxl-vision-hub`) which injects a SECOND,
competing pose into the same EV input — EKF sees two disagreeing sources and
degrades/rejects. On the VOXL:
```bash
sed -i 's/"en_vio":.*true/"en_vio": false/' /etc/modalai/voxl-vision-hub.conf   # or edit by hand
systemctl restart voxl-vision-hub          # keep it RUNNING (other services need it)
systemctl disable --now voxl-qvio-server   # stop the VIO estimator itself
# do NOT touch voxl-mavlink-server — that is the QGC link
```

**1c. Verify fusion state (VOXL — note it's `px4-listener`, not `listener`):**
```bash
px4-listener vehicle_visual_odometry     # mocap EV arriving? timestamp a few ms old, steadily
px4-listener estimator_status_flags      # WANT: cs_ev_pos/cs_ev_hgt/cs_ev_yaw = True,
                                         #       cs_mag_hdg = False  (mag truly off)
px4-listener vehicle_local_position      # xy_valid / z_valid = True once converged (~20-30 s still)
```
`cs_mag_hdg: True` after all of the above → params didn't take (not saved /
no reboot / wrong drone — they are PER DRONE).

**2. Verify the feed reaches PX4 — mind the QoS.** PX4 `/fmu/*` topics are
**best_effort**; a plain `ros2 topic echo` (reliable) shows **nothing** and
looks broken when it isn't. Always:
```bash
ros2 topic hz   /drone_1/fmu/in/vehicle_visual_odometry            # ~mocap rate (mocap_bridge alive)
ros2 topic echo /drone_1/fmu/out/vehicle_odometry --once \
  --qos-reliability best_effort --qos-durability volatile          # EKF IS fusing -> position appears
```
If `in/…` streams but `out/…` stays silent, EKF2 isn't accepting it → re-check
the params above, or the timestamp/frame below.

**3. Frame hand-check (do before every first flight).** Carry the drone a
metre toward PX4 **North** (the agreed forward), watch `out/vehicle_odometry`:
`position[0]` (N) must **increase**; carrying East increases `position[1]`;
lifting it increases nothing in z down (`position[2]` decreases). If axes are
swapped/mirrored, your mocap isn't ROS-ENU — flip `px4_vio_frame:
"modalai_flip"` in `swarm_real.yaml` (the reference transform) and re-check.
This `direct`/`modalai_flip` path reproduces the proven `model_ai_tfpub.cpp`.

### B5. See the drone in RViz (no flight)

Bring up the commander **without taking off** + the mocap bridge, then watch
the drone's marker track as you carry it. Confirms mocap → odometry → world
before anything arms.

```bash
cd ~/AirStack && ./airstack.sh connect robot --command=bash
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/swarm_real.yaml \
  use_mocap:=true
# in another shell — does odometry track your hand?
ros2 topic echo /drone_1/odometry_conversion/odometry --once
```
Open RViz (see [RViz visualization](#rviz-visualization)) and move the drone by
hand: its **red** sphere should follow on `/svg/viz/markers`. (To also see it in
the Isaac 3D viewport, launch Isaac with `DRONE_MODES` set — see Part A and the
flagship in C4.) Do **not** call `takeoff` here — this is preflight only.

### B6. VOXL2 diagnostics cheat sheet

Everything we reach for during a real-drone bring-up. **VOXL** lines run in the
`adb shell`; **ground PC** lines run in the robot container (`ROS_DOMAIN_ID=1`).

*Services (VOXL):*
```bash
voxl-inspect-services                       # which voxl-* services are enabled/running
systemctl status voxl-px4                   # PX4 flight stack
systemctl is-enabled voxl-microdds-agent    # should be 'disabled' (we use the remote agent)
```

*Wi-Fi / network (VOXL):*
```bash
ip addr show wlan0                          # current IP — on the router's subnet?
voxl-wifi status     ;  iw wlan0 link       # is it associated to the AP?
networkctl status wlan0                     # who manages wlan0 + the DHCP lease
ping -c2 <ground_pc_ip>                      # reachability to the agent host
```

*PX4 ↔ XRCE bridge (VOXL — the ONLY VOXL-side bridge checks):*
```bash
px4-microdds_client status                  # connected? Agent IP? Payload tx/rx nonzero?
px4-param show -a | grep -i -E 'dom|xrce|dds'   # discover the DDS-domain param name
px4-param show UXRCE_DDS_DOM_ID              # or XRCE_DDS_DOM_ID — the DDS domain
```

*Did a file change correctly? (VOXL):*
```bash
grep -n 'microdds_client start' /usr/bin/voxl-px4-start   # -h/-p/-n actually applied?
ls -l  /usr/bin/voxl-px4-start*                            # timestamped .bak.* the script made
diff   /usr/bin/voxl-px4-start.bak.* /usr/bin/voxl-px4-start   # exactly what changed
awk --version 2>/dev/null || awk -W version               # which awk (mawk 1.3.3 lacks [[:space:]])
```

*Topics & data (ground PC — where `/fmu/*` actually lives):*
```bash
ros2 topic list | grep <name>/fmu
ros2 topic hz   /<name>/fmu/out/vehicle_odometry --qos-reliability best_effort
ros2 topic echo /<name>/fmu/out/vehicle_status   --qos-reliability best_effort --once
```

> **Cross-distro hazard.** VOXL2 is ROS 2 **Foxy**, the ground stack is **Jazzy**.
> If the VOXL's *native* Foxy DDS traffic shares a `ROS_DOMAIN_ID` with the Jazzy
> ground stack, `ros2 topic list` can crash with `deserialize_change` /
> `std::bad_alloc` (incompatible RTPS wire formats). Keep them apart: the XRCE
> bridge is safe (the agent re-emits `/fmu` as Jazzy-native on the ground PC), and
> the on-VOXL `voxl-microdds-agent` stays **disabled** (the B1 script does this).

---

# Part C — Tasks: any drone in any mode

One framework, not "sim tests vs hardware tests". Every task is a config; each
config exposes the **three per-drone axes** (see §2) and you pick them freely:

```yaml
drone_modes:        "sim,sim,sim"   # per drone: sim -> SITL/MAVROS, real -> hardware/fmu
teleop_drones:      ""              # operator-driven (else scenario-driven)
external_drones:    ""              # tracked by CBF, never commanded
cbf_exempt_drones:  ""              # CBF won't correct these (still obstacles)
```

"Pure sim", "all real", and "hybrid" are just different `drone_modes` vectors on
the **same** task. To make a drone real: set its slot to `real` (commands route
to `/fmu/…`; it must be connected per [Part B](#part-b--bring-in-a-real-drone-connect--verify),
and Isaac shows it as an avatar). Nothing else in the task changes.

All-sim tasks assume Part A (A1–A3) is up; any `real` drone assumes Part B.

### C0. Start the per-drone interfaces (required before any task)

The commander reads every drone's state from `/{name}/odometry_conversion/odometry`
— which is **always** produced by an interface node, in *both* modes. `drone_modes`
only switches the *command* routing; the *state* side still needs the matching
interface running, or the commander reports `no drone eligible for takeoff (missing
odometry)`:

```bash
# SIM drones — MAVROS/SITL interfaces (same as A3); arg = number of sim drones:
./src/svg_ground_control/scripts/launch_sim_interfaces.sh 1

# REAL drones — px4_interface stack (uXRCE-DDS, no MAVROS); comma-separate names:
ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1   # ,drone_2,...
# target_system (= the drone's MAV_SYS_ID) defaults to the name's number (drone_2 -> 2);
# override with target_systems:=1,2,3 if your ids differ. Check the startup line:
#   [drone_2.fmu.px4_interface]: PX4Interface initialized (uXRCE-DDS), target_system=2
```

For a **real** drone this is the analogue of A3 — it brings up `px4_interface`
(converts `/{name}/fmu/out/vehicle_odometry` → `…/odometry_conversion/odometry`) and
`odometry_conversion`. Confirm it before launching the commander:
```bash
ros2 node list | grep -E 'px4_interface|odometry_conversion'   # both present, per drone
ros2 topic echo /drone_1/odometry_conversion/odometry --once   # a pose appears (tracks reality)
```
If `odometry_conversion/odometry` is empty even though `/{name}/fmu/out/vehicle_odometry`
streams, check the interface stack is actually up (a dead `microdds_client` or a
crashed `px4_interface` is the usual cause).

### C1. Single-drone goal (`goal_single.yaml`)

One drone flies to a goal you set, at a speed you set. **One config does both
sim and real — `drone_modes` is the only switch** (`"sim"` → SITL/MAVROS,
`"real"` → hardware/`/fmu/`). Launch with **`use_mocap:=true` always**: on a
`real` drone the mocap bridge feeds PX4 EKF2 the external vision it needs to arm
(mocap → `/drone_1/fmu/in/vehicle_visual_odometry`, the **only** way EKF2 fuses a
position indoors — see [B4b](#b4b-external-vision--ekf2-the-arm-blocker)); in
`sim` it's a harmless no-op (no `/drone_1/pose`, SITL self-estimates). For a
1-drone sim, spawn with `NUM_ROBOTS=1` in A2 and `./launch_sim_interfaces.sh 1`
in A3; for `real`, connect the drone per [Part B](#part-b--bring-in-a-real-drone-connect--verify)
and set the EKF2 params (B4b) first.

Before flying:
```bash
# REAL drones — px4_interface stack (uXRCE-DDS, no MAVROS); comma-separate names.
# target_system = MAV_SYS_ID is taken from the name (drone_2 -> 2), see C0:
ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1   # ,drone_2,...
```
If flying Drone 1:
```bash
MicroXRCEAgent udp4 -p 8888 -v4
```
If flying Drone 2:
```bash
MicroXRCEAgent udp4 -p 8889 -v4
```
If flying Drone 3:
```bash
MicroXRCEAgent udp4 -p 8892 -v4
```
```bash
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/goal_single.yaml \
  use_mocap:=true
```
**How it flies (since 2026-09-20).** Real drones no longer get a bare
velocity setpoint. The commander keeps a *reference point* per drone (where
it was told to be, integrated from the published velocity) and flies an
acceleration-limited profile toward the goal from it, using PX4's own
braking law `v = -aL + sqrt((aL)² + 2ad)`; the reference, the velocity and
the acceleration go to PX4 in one `trajectory_command`, so PX4 closes the
position loop onboard with feedforward — the same structure as its Position
mode. Measured on drone_2 (px4_logs/, bag `drone_2_auto_goal_0920_192853`):
the old `1.5 × distance` P-law overshot a 7 m / 5 m/s leg by **1.0 m** and
needed 4.7 s to settle; PX4 Position mode stops from 5.8 m/s in 4.2 m with
0.35 m overshoot; the new law stops on the goal within ~0.05 m in the same
plant model (`test/test_trajectory.py`). Knobs, all live with
`ros2 param set /swarm_commander …`:

| param | default | meaning |
|---|---|---|
| `scenario_speed_mps` / `speed_command` | config | cruise cap; reached only if the goal is farther than `v²/(2a) + v·settle` (the commander logs this distance for every speed it receives) |
| `goal_accel_mps2` | 3.0 | acceleration and braking of the profile (drone_2 managed 5.5 in the logs; PX4 auto uses 3) |
| `goal_settle_s` | 0.3 | exponential tail into the goal; larger = softer stop, slower arrival |
| `goal_lead_m` | 2.0 | leash: how far the reference may get ahead of the drone (tracking lag at speed, a wall, a gust) before it is pulled back and the profile restarts from the drone's speed; the CBF does not need it (the reference only moves by the *published* velocity). PX4's `MPC_XY_ERR_MAX` |
| `real_command_mode` | `trajectory` | `velocity` sends the old TwistStamped instead (the px4_interface must be rebuilt for `trajectory`: `bws --packages-select px4_interface`) |
| `takeoff_speed_mps` | 0.5 | climb speed of the takeoff profile (braking law into the takeoff target, no P-law step) |
| `hold_lead_m` | 0.2 | reference leash while taking off / landing / holding outside a mission — keep small, PX4's altitude loop is stiff |

**Top speed the room allows.** A leg of length S needs `v²/a` to accelerate
and brake plus `v·settle` to ease in: `v² / goal_accel + v·goal_settle = S`.
Fence box 10 × 9.7 m, goals 0.5 m inside the walls: along y (8 m) 5.9 m/s at
6 m/s², 6.9 at 8; on the diagonal (−4,−4.7) → (5,4) = 12.5 m: **7.8 m/s at
6 m/s², 8.9 at 8**. 10 m/s needs ~15.5 m even at 8 m/s² — not in this room.
Keep `fence_brake_accel_mps2 >= goal_accel_mps2`, or the wall envelope caps
the cruise first (it did in run_035852: 4.7 m/s with brake 4).

Braking distance is `v²/(2·goal_accel_mps2) + v·goal_settle_s`: to brake later,
raise `goal_accel_mps2` (the airframe did 5.5 m/s² in the logs; PX4's own
manual braking asks up to 8) and/or lower `goal_settle_s` (0.2 is still a
clean stop in the plant model; below that the tail gets sharp). At 5 m/s:
3.0/0.3 → 5.7 m, 4.0/0.2 → 4.1 m, 5.5/0.2 → 3.3 m.

```bash
# control terminal:
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger
# goal with heading: [x, y, z, theta]; theta in DEGREES, 0 = +X, CLOCKWISE
ros2 topic pub --once /svg/drone_1/goal_xyzt std_msgs/msg/Float64MultiArray "{data: [1.0, 0.5, 1.4, 90.0]}"
# (position-only form still works; its heading is +X = 0 deg)
ros2 topic pub --once /svg/drone_1/goal_command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5, z: 1.4}}}"
ros2 topic pub --once /svg/drone_1/speed_command std_msgs/msg/Float32 "{data: 0.8}"
ros2 service call /swarm_commander/land std_srvs/srv/Trigger
```
**Safety with the CBF.** The CBF still filters velocities, and that is
still everything that moves a drone: the reference point (PX4's position
setpoint) is integrated from the *published*, filtered velocity, so it stops
when the CBF says stop; the acceleration feedforward is dropped whenever the
CBF or the fence alters a velocity; and while they do, the reference is held
on the short `hold_lead_m` leash so PX4's own, unfiltered pull toward the
reference stays below ~0.2 m/s.

**Heading.** Real drones on the trajectory output get an absolute yaw with
every setpoint: the goal's `theta` in the goal scenario, and **nose on +X
(0°) in every other scenario** (hover, squeeze, random goals, …) and while
taking off / holding. `theta` is degrees, 0 = +X of the mocap frame,
clockwise positive seen from above (90 = nose on −Y). A `goal_command`
PoseStamped may carry the heading as its quaternion (ENU yaw, the usual ROS
sense); an all-zero quaternion means 0°. Teleop drones yaw with the stick:
while it is deflected the setpoint carries the yaw *rate* and an all-zero
rotation (x, y, z **and** w — a `Quaternion` message defaults to w = 1,
which px4_interface read as "hold ENU yaw 0" and dropped the rate, so the
yaw stick did nothing in bag `run_053740`); the moment it is centred the
measured heading is adopted and held as an absolute yaw, as PX4's own
Position mode does. RViz shows the commanded heading as a white arrow. Sim /
velocity-only drones are not heading-controlled.

**Stick acceleration.** `teleop_accel_mps2` (live) ramps the stick velocity
at that rate and feeds the ramp's acceleration forward with the setpoint —
PX4's own Position mode (`MPC_ACC_HOR_MAX`, default 5). Without it a stick
step is followed at only ~4 m/s² by the velocity loop, which is why drone_2
peaked at 3.9 m/s with an 8 m/s stick in the 7.7 m teleop box (bag
`run_053740`): it never reached the wall's cap. Plant model, that box, brake
4: step 4.3 m/s, ramp 5 → 5.2 m/s, ramp 8 → 5.4 m/s (~40° bank); in the
9.7 m geofence span 5.0 / 5.8 / 6.1 m/s. Note that
8 m/s is not reachable there with a stop at the wall — from wall to wall the
drone accelerates for half the span and brakes for the other half, 6.2 m/s
at 5 m/s² with no lag at all — so a faster run needs a bigger box. Keep the
wall's `fence_brake_accel_mps2` at 4 with the ramp: 5-6 buy only
+0.2-0.4 m/s of peak in the model, and a slow-responding vehicle (0.2 s
attitude lag) then overshoots 0.2-0.7 m where 4 stays within 0.05 m.
**LEDs:** the strip is **green** throughout (daemon default + `led_controller`
block in `goal_single.yaml`; a single drone is never CBF-corrected). Recolor at
**any** time — armed or not, before takeoff, on the bench — the same way you
retarget a formation:
```bash
ros2 topic pub --once /svg/led_command std_msgs/msg/String "{data: 'drone_1 blue'}"        # name …
ros2 topic pub --once /svg/led_command std_msgs/msg/String "{data: 'drone_1 255,60,0'}"    # … or r,g,b[,w]
ros2 topic pub --once /svg/led_command std_msgs/msg/String "{data: 'drone_1 red blink'}"   # blink
ros2 topic pub --once /svg/led_command std_msgs/msg/String "{data: 'all green'}"           # every drone
```
Or as a **service call** (`airstack_msgs/srv/SetLedColor`: `color` = a name or
`"r,g,b[,w]"`, `mode` 0 = solid / 1 = blink; the reply says whether the strip
took it):
```bash
ros2 service call /svg/drone_1/set_led_color airstack_msgs/srv/SetLedColor "{color: blue}"
ros2 service call /svg/drone_1/set_led_color airstack_msgs/srv/SetLedColor "{color: '255,60,0', mode: 0}"
ros2 service call /svg/drone_1/set_led_color airstack_msgs/srv/SetLedColor "{color: red, mode: 1}"   # blink
ros2 service call /svg/set_led_color         airstack_msgs/srv/SetLedColor "{color: green}"          # every drone
```
(`ros2 service list | grep led` shows one `/svg/<drone>/set_led_color` per
drone in the `led_controller` block; quote the `r,g,b` form so YAML does not
read it as a list.)
Colors: off red green blue white yellow cyan magenta orange purple. Setup per
drone: [B1(d)](#b1-per-drone-one-time-setup); disable with `use_led:=false`.

### C2. Multi-drone goal (`goal_tracking.yaml`)

Assign different goals/speeds to different drones while flying; the CBF keeps
them apart when paths cross. Same dual-mode design as C1: **one config,
`drone_modes` is the only switch, per drone** (`"real,real"` = both hardware;
mix like `"real,sim"` for hybrid). Launch with **`use_mocap:=true` always** —
the config's `mocap_bridge` block feeds every real drone's EKF2 (sim drones
are a no-op, same as C1).

**Real-drone prerequisites, PER DRONE — for EVERY drone in the config's
`drone_names`** (the multi-drone part people miss; each drone needs its own
full chain in THIS session):
* each drone connected per [Part B](#part-b--bring-in-a-real-drone-connect--verify)
  on its **own agent port** (drone_1→8888, drone_2→8889, drone_3→8892) → run
  **one `MicroXRCEAgent udp4 -p <port> -v4` per drone** on the ground PC —
  including any agent you had running for a previous single-drone test (a
  closed C1 terminal ≠ a running agent);
* EKF2/mag params set on **each** drone (B4b.1 — they are per-drone, saved,
  rebooted);
* one `natnet_ros2` instance serves all bodies (`/drone_1/pose`, …);
* interfaces up **for all listed drones**:
  `ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1,drone_2,drone_3`
  ([C0](#c0-start-the-per-drone-interfaces-required-before-any-task)) —
  a name missing here is exactly "drone_X: no odometry / odometry stale,
  refuses takeoff" while the others fly. Per-drone triage:
  `ros2 node list | grep drone_X` → `ros2 topic hz /drone_X/pose` →
  best_effort echo `/drone_X/fmu/out/vehicle_odometry` →
  `ros2 topic hz /drone_X/odometry_conversion/odometry`;
* `drone_position_offsets` is **per drone**: a `real` slot is `0,0,0` (mocap is
  absolute — all real drones share the mocap origin), a `sim` slot is that
  drone's Isaac spawn (`x = 2*(i-1)-(N-1)`), e.g. hybrid `"real,sim"` →
  `[0,0,0, 1,0,0]`. All-real (current config) = all zeros; the commander's
  "offsets are all zero" startup warning is expected/benign in that case.
Before flying:
```bash
# REAL drones — px4_interface stack (uXRCE-DDS, no MAVROS); comma-separate names:
ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1   # ,drone_2,...
```
If flying Drone 1:
```bash
MicroXRCEAgent udp4 -p 8888 -v4
```
If flying Drone 2:
```bash
MicroXRCEAgent udp4 -p 8889 -v4
```
If flying Drone 3:
```bash
MicroXRCEAgent udp4 -p 8892 -v4
```
```bash
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/goal_tracking.yaml \
  use_mocap:=true
# takeoff BOTH drones together + start, then retarget any drone any time:
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger
ros2 topic pub --once /svg/drone_1/goal_command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0, z: 1.2}}}"
ros2 topic pub --once /svg/drone_2/goal_command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: -1.5, y: 0.0, z: 1.2}}}"
ros2 topic pub --once /svg/drone_1/speed_command std_msgs/msg/Float32 "{data: 0.6}"
ros2 topic pub --once /svg/drone_2/speed_command std_msgs/msg/Float32 "{data: 1.0}"
```
**LEDs:** all real drones **green**; a drone turns **red for as long as the CBF
is altering its command** (paths crossing — the same moments the commander logs
`CBF active on: …`), then back to green. Recolor any time (works disarmed too), formation-style:
`ros2 topic pub --once /svg/led_command std_msgs/msg/String "{data: 'drone_2 blue'}"`,
`"{data: 'all white'}"`, `"{data: 'drone_1 red blink'}"` (per-drone topic
`/svg/<name>/led_command` takes just `"<color> [blink]"`; services
`/svg/<name>/set_led_color` still exist). Only drones listed in the config's
`led_controller.drone_names` (the real ones) are driven — setup per drone in
[B1(d)](#b1-per-drone-one-time-setup).

**Formation profiles — retarget the whole swarm with ONE command.** The config
defines named profiles (`formation_profiles` + one `formation_<name>` array
each, one x,y,z row per drone in `drone_names` order — edit/add/remove freely
in `goal_tracking.yaml`; keep pairs > 2*`cbf_safety_radius_m` apart and inside
the fence). Publishing a profile name sends every scenario-driven drone to its
slot simultaneously (any time after `start`; the CBF deconflicts the crossing;
external drones are skipped). Shipped examples: `home` (= the takeoff layout),
`line`, `triangle`, `diagonal`:

```bash
ros2 topic pub --once /svg/formation_command std_msgs/msg/String "{data: triangle}"
ros2 topic pub --once /svg/formation_command std_msgs/msg/String "{data: home}"   # back to start
ros2 topic pub --once /svg/formation_command std_msgs/msg/String "{data: next}"   # roll to the next profile
```

`next` cycles through the profiles in their `formation_profiles` order,
wrapping around — repeat the same command to step through the whole set. An
explicit profile name re-anchors the cycle there (`next` continues from it).
An unknown name is ignored with a warning listing the available profiles (check
the commander log). Per-drone `goal_command` / `speed_command` still work and
can fine-tune individual drones after a formation switch.

### C3. Squeeze — all-sim rehearsal (`squeeze_3drone.yaml`)

Holders (drone_1,2) hold their posts; the intruder (drone_3) shuttles through
the gap. drone_3 is **CBF-exempt** (`cbf_exempt_drones: "drone_3"`) so it
presses through and the holders alone yield.

**Why no `use_mocap` here (unlike C1/C2):** this config is the deliberately
**all-sim rehearsal** — `drone_modes: "sim,sim,sim"`, no real drone anywhere,
so no PX4 EKF needs external vision and `mocap_bridge` would have no
`/{name}/pose` inputs to forward. `use_mocap` only matters when at least one
drone is `real`. The two **hardware** squeeze variants are the next sections:
* real holders + **sim** intruder → [C4](#c4-flagship--hybrid-squeeze-real-holders--sim-intruder-hybrid_squeezeyaml)
* real holders + **hand-flown (gamepad) teleop** intruder → [C5](#c5-squeeze-with-a-hand-flown-intruder-squeeze_rc_intruderyaml)

```bash
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/squeeze_3drone.yaml
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger
```

Run this rehearsal before either hardware variant — same scenario geometry,
zero risk.

### C4. Flagship — hybrid squeeze: real holders + sim intruder (`hybrid_squeeze.yaml`)

Your experiment plan #1: **drone_1,2 real holders** (mocap hardware — the
config's `mocap_bridge` block forwards `/drone_1/pose` and `/drone_2/pose`
into their PX4 EKFs), **drone_3 sim** (Isaac SITL), **CBF-exempt +
policy-controlled**. All three appear in the Isaac viewport — real holders as
live avatars, the intruder as its SITL body — and the real holders react (via
the CBF) to the virtual intruder squeezing through. Config already set:
`drone_modes: "real,real,sim"`, `cbf_exempt_drones: "drone_3"`, and
`drone_position_offsets: [0,0,0, 0,0,0, 2,0,0]` (holders mocap-anchored →
zero; sim intruder → its Isaac spawn x=+2).

```bash
# 0. real holders connected + verified — Part B (px4_interface + NatNet + mocap)
#    for drone_1,drone_2.

# 1. Isaac (GUI): SITL for the sim intruder + avatars for the real holders.
#    DRONE_MODES matches the commander's drone_modes.  [isaac-sim container]
NUM_ROBOTS=3 DRONE_MODES="real,real,sim" SVG_DOMAIN_ID=1 \
PLAY_SIM_ON_START=true ISAAC_SIM_HEADLESS=false \
PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
/isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py \
  --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts

# 2. MAVROS interface for the sim intruder (drone_3) only.  [robot container]
ROBOT_NAME=drone_3 FCU_URL='udp://:14543@<sim_ip>:14583' TGT_SYSTEM=4 \
  ros2 launch svg_ground_control sim_drone_interface.launch.xml drone_name:=drone_3

# 3. ONE commander for all three (use_mocap feeds the real holders' EKFs).
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/hybrid_squeeze.yaml \
  use_mocap:=true
```
One CBF sees all three (the state topic is identical for real and sim), so the
real holders dodge the simulated intruder. In RViz the holders are **red**, the
intruder **cyan**, all in one `map` frame.

> **Dry-run the routing first (no hardware).** `test/functional_hybrid_test.py`
> fakes the real+sim drones on their respective topics and asserts each drone's
> commands land on the correct namespace and the squeeze still works — run it
> before trusting a real flight (see [Automated tests](#automated-tests)).

### C5. Squeeze with a HAND-FLOWN intruder (`squeeze_rc_intruder.yaml`)

Your experiment plan #2: **drone_1,2 real holders** (commander-flown, hold the
posts and yield via the CBF) + **drone_3 real, flown by a human pilot on the
gamepad** as the intruder. drone_3 is a **`teleop_drones`** entry: the
commander arms it, lifts it to intruder waypoint A at `takeoff`, hands it to
the sticks at `start` (position mode — released sticks hold), and lands it
with the holders. It is listed in **`cbf_exempt_drones`**, so its stick goes
out uncorrected and the holders alone yield; the CBF sees its *commanded*
(ramped, capped) velocity as a fixed row, so the holders start moving before
it arrives. A **teleop fence** (amber box inside the geofence) is the soft
wall the pilot meets; the keep_in geofence bounds all three.

(The earlier setup — drone_3 on its own RC link, `external_drones`, merely
tracked with its *measured* velocity in the filter — is a two-line switch
listed in the config's header. An external drone may NOT also be in
`cbf_exempt_drones`; the commander rejects that config.)

**Mocap goes to ALL THREE drones** (the config's `mocap_bridge` lists
drone_1,2,3): every drone flies offboard on the commander's setpoints and
needs external vision to arm — set the B4b.1 EKF2/mag params on all three.

Prerequisites (all three drones per [Part B](#part-b--bring-in-a-real-drone-connect--verify)):
* own agent port per drone (8888/8889/8892) → **three** `MicroXRCEAgent`s;
* B4b.1 params set + saved + rebooted on **each** drone;
* Motive bodies `drone_1..3` streaming; interfaces for **all three**:
  `ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1,drone_2,drone_3`
* the gamepad, checked before anything is armed ([teleop.md](teleop.md)).

```bash
# terminal 1 — the pad (prints what it reads; move the sticks, nothing flies yet):
ros2 launch svg_ground_control teleop.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/squeeze_rc_intruder.yaml \
  drone:=drone_3

# terminal 2 — commander (mocap always on):
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/squeeze_rc_intruder.yaml \
  use_mocap:=true

# fly (terminal 3):
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger   # arms/lifts ALL THREE:
                                                                  # holders to their posts, drone_3 to waypoint A
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger   # holders on posts; drone_3 on the sticks
# fly drone_3 through the gap — watch the yellow (teleop) marker + red holders yield in RViz.
ros2 service call /swarm_commander/land    std_srvs/srv/Trigger   # lands ALL THREE
```

> ⚠️ **Safety — the fences are velocity clips, not a motor cutoff.** In
> `keep_in` (this config) drone_3 is braked at the teleop fence and the
> holders at the geofence; in `hold_all` a breach by anyone freezes all three.
> The RC kill switch remains the true cutoff for every drone. If drone_3's
> odometry goes stale (mocap dropout) the commander sends it zero velocity
> and the scenario pauses; if the pad goes stale (`teleop_timeout_s`) the
> stick reads zero and position hold keeps drone_3 where it is.

**LEDs (`led_controller` block in `squeeze_rc_intruder.yaml`, all three drones
set up per [B1(d)](#b1-per-drone-one-time-setup)):** everyone is **green**. A
**holder turns red while the CBF is pushing it out of the intruder's way** (the
commander publishes the corrected names on `/svg/cbf_active` every tick; the LED
node holds red ≥ 0.5 s so short corrections are visible) and returns to green
when its command is no longer altered. drone_3 is CBF-exempt — its stick is
never "corrected" — and stays green; give the pilot's drone its own color if
useful: `ros2 topic pub --once /svg/led_command std_msgs/msg/String "{data: 'drone_3 blue'}"`.
A CBF **emergency push-apart** turns every holder red. Watch the signal itself
with `ros2 topic echo /svg/cbf_active`.

---

# Part D — Real hardware: first flight & reference

Once [Part B](#part-b--bring-in-a-real-drone-connect--verify) confirms tracking,
fly. The flight services are identical to sim ([A6](#a6-fly-fresh-terminal)) —
only the config (real modes) and the safety discipline differ.

### D1. Preflight + fly (fresh terminal)

```bash
cd ~/AirStack && ./airstack.sh connect robot --command=bash
ros2 topic hz   /drone_1/pose                                  # mocap arriving?
ros2 topic echo /drone_1/odometry_conversion/odometry --once   # tracks reality?
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger
ros2 service call /swarm_commander/land    std_srvs/srv/Trigger
```

### D2. First-flight safety

- **One drone first.** `drone_names: ["drone_1"]`, `drone_modes: "real"`,
  scenario `hover`, thumb on the **RC kill switch**. Then two. Then the demo.
- The geofence is a freeze-in-place, **not** a motor cutoff — the RC kill
  switch is the true cutoff ([Geofence](#geofence)).
- Fit `arena_*` and `fence_*` to your capture volume before arming.
- Keep `cbf_max_speed_mps` conservative on hardware (`swarm_real.yaml` uses
  1.0).

---

## RViz visualization

The commander publishes all drones' **world** positions (offset-corrected, so
real + simulated share one frame) as a `MarkerArray` on `/svg/viz/markers`:
an Iris body mesh per drone coloured by planner status — green = planner not
launched, blue = running, red = stopped after running, dim gray = landing; with
overrides orange = frozen-on-breach, yellow = teleop, gray = external —
translucent safety sphere (2r), name/mode/role label, goal points, and the
geofence box. The mesh is
`package://robot_descriptions/iris/meshes/base_link_body_body.stl`, so
`robot_descriptions` must be built in this workspace (`bws` does it).

```bash
# from a robot-container shell (./airstack.sh connect robot --command=bash):
# ROS_DOMAIN_ID + workspace are already set by .bashrc. Needs an X display.
rviz2 -d $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/svg_drones.rviz
```
The config sets fixed frame `map` and adds the MarkerArray display. If you
open a bare `rviz2`: set Fixed Frame = `map`, Add → By topic →
`/svg/viz/markers`. This is the unified "see all drones" view for hybrid runs.

For the operator view with the safety stop and telemetry, see
[Foxglove visualization](#foxglove-visualization) — same markers, plus the
SVG Basestation panel.

**Hand-carry / preflight (no flight needed).** The markers come from
`swarm_commander`, not the drones directly, so the chain is: interface layer
→ `/{name}/odometry_conversion/odometry` → commander → `/svg/viz/markers` →
RViz. To watch drones move by hand with nothing armed:
1. bring up the per-drone interfaces (Part A A3 for sim, or Part B for
   hardware: px4_interface + NatNet + mocap bridge) so odometry flows;
2. launch `ground_control.launch.py` but **do NOT call takeoff** — the
   commander idles in IDLE, publishes zero commands, and still publishes
   markers every tick;
3. launch RViz.
Now move each drone by hand and its sphere tracks live — the ideal hardware
preflight to confirm mocap→odometry matches reality before arming. If RViz is
empty: `ros2 topic hz /svg/viz/markers` (should be ~20 Hz; if silent the
commander isn't running) and `ros2 topic echo
/{name}/odometry_conversion/odometry --once` (a drone with no odometry is
skipped in the markers).

---

## Foxglove visualization

Foxglove is the operator-facing alternative to RViz: the same `/svg/viz/markers`
3D view (Iris body mesh per drone, coloured by planner status — see the RViz
section for the legend) plus the **SVG Basestation** panel (agent wiring,
two-click land-all safety stop, Hold All, link safety, battery / RTB, formation
dropdown). The panel, a ready-made layout and its installer live in this
package's [`foxglove/`](foxglove/) directory — see the panel's
[README](foxglove/svg-basestation/README.md) for what every column means. (The
general AirStack panels — Robot Tasks, Waypoint / Polygon editors — stay in
`gcs/foxglove_extensions/`.)

**Everything runs from the robot container — nothing to start by hand.**

* [`ground_control.launch.py`](launch/ground_control.launch.py) starts
  `foxglove_bridge` next to the commander (`use_foxglove_bridge:=false` to opt
  out, `foxglove_port:=` to move it off 8765). Expect
  `[foxglove_bridge]: Server listening on 0.0.0.0:8765` in the A4 terminal.
* The robot container runs `svg_ground_control/foxglove/install.py` (SVG
  Basestation, from the mounted `ros_ws`) and `gcs/foxglove_extensions/install.py`
  (Robot Tasks, Waypoint / Polygon editors) at start-up, so all four panels are
  installed in the container's own Foxglove Studio. Studio's config/layouts persist in `robot/docker/Foxglove/`
  (git-ignored, mounted at `/root/.config/Foxglove`).
* `robot-desktop` is on `network_mode: host` and pins `ROS_DOMAIN_ID=1`, so a
  Studio on the **host** reaches the bridge at `ws://localhost:8765` too. The
  `gcs` container is deliberately not used: it sits on the Docker bridge network
  at domain 0 and never sees the drone topics.

> **After pulling this change** (once): recreate the robot container so the new
> mounts appear — `./airstack.sh up` recreates on compose changes, which kills
> anything running inside — then rebuild the packages it touches:
> `bws --packages-select robot_descriptions interface_bringup svg_ground_control`
> (drone mesh, per-drone TF frames, launch file). If you were running a
> hand-started `foxglove_bridge`, stop it first: two bridges on one port is a
> bind error and a respawn loop.

### F1. Studio on the host (default)

```bash
cd ~/AirStack
python3 robot/ros_ws/src/svg_ground_control/foxglove/install.py   # once per pull: installs the
                                                                  # panel into ~/.foxglove-studio/extensions
foxglove-studio                                                   # (re)start Studio AFTER installing
```
Then **Open connection** → Foxglove WebSocket → `ws://localhost:8765`, and
**Layouts → Import from file…** →
`~/AirStack/robot/ros_ws/src/svg_ground_control/foxglove/svg_basestation.json`. Pick the imported
layout from the layout dropdown (top-right).

### F2. Studio inside the container (alternative)

```bash
# with the rest of ground control (pre-connected to the bridge):
ros2 launch svg_ground_control ground_control.launch.py use_foxglove_studio:=true
# or on its own, from any robot-container shell:
foxglove-studio --no-sandbox
```
Import the layout once from
`/root/AirStack/robot/ros_ws/src/svg_ground_control/foxglove/svg_basestation.json`; it is kept in the
mounted config dir, so it is still there after the container is recreated.
`install.py` prints one `Installed Foxglove extension: airlab-cmu.<name>-<ver>`
line per panel in `docker logs airstack-robot-desktop-1`; **Extensions** in
Studio's left sidebar lists what is loaded.

The layout is preset for `drone_1,drone_2,drone_3` with **Modes** blank, so each
agent is detected from the wire (`/{name}/interface/…` ⇒ sim,
`/{name}/fmu/…` ⇒ real) — the panel's **Wiring** card says which it decided.
For a different drone list or explicit modes, edit the panel settings (gear icon)
and mirror the config's `drone_names` / `drone_modes` / `drone_position_offsets`.
The 3D panel's display frame is `map`: the markers are published in bare `map`
while each drone's TF is namespaced (`drone_N/map → drone_N/base_link`, see
[`sim_drone_interface.launch.xml`](launch/sim_drone_interface.launch.xml)), so
three drones no longer fight over one `map → base_link` transform.

Each drone carries a name label (`drone_1` …; mode and role are in the panel,
not the label). Foxglove always draws a text marker on a contrasting box —
black behind light text, white behind dark — with the box as opaque as the
text, and uses its own sans-serif font; neither can be turned off from the
marker. The label is therefore the panel's dark slate on a white chip
(`LABEL_COLOR` in `swarm_commander.py`), slightly translucent.

### F3. What you should see

| Stage | Panel |
| --- | --- |
| Only the bridge up | Everything rendered but reading `--` (no topic list yet) |
| A3 interfaces up | Agents listed, Battery & Power live from `/{name}/interface/mavros/battery`, Link Safety Rate/Drop from odometry |
| A4 commander up | 3D view shows the drone meshes (green until `start`) + geofence; Tasks chip names the scenario topics it found |
| Config with `formation_profiles` (e.g. `cbf_sim.yaml scenario:=goal`) | Formation dropdown lists the profiles; Tasks chip shows `formation` |
| Real drone (Part B) | Mocap age, EKF, Ping (uXRCE-DDS `timesync_status`) columns appear for that agent |

Sections hide themselves when nothing publishes what they need (**Sections:
Auto**); set **Show all** in the settings to force every card on.

**Safety controls.** The red **LAND ALL** bar is two-click (arm → fire within
4 s) and calls `/swarm_commander/land`; **Hold All** calls `/swarm_commander/hold`.
Takeoff / Start / Reset Fence are in the command strip below. These are the same
services as A6, so the CLI and the panel can be mixed freely.

**If the panel is empty:** in a robot shell `ros2 topic list | grep drone_1`
must show the interface topics and `ros2 topic hz /svg/viz/markers` must tick
(~20 Hz). If the topics exist but Foxglove sees none, the bridge is on the wrong
domain — `echo $ROS_DOMAIN_ID` in the A4 shell must print `1`. If the panel's
services all fail, check the A4 terminal: a dead `swarm_commander` leaves stale
service names in `ros2 service list`. If the **SVG Basestation** panel type is
missing from *Add panel*, `install.py` ran for a different user / `HOME` than the
one running `foxglove-studio`. If the drones render as nothing / a warning about
`package://robot_descriptions/...`, `robot_descriptions` is not built in this
workspace (`bws`).

---

## Geofence

The box `[fence_min, fence_max]` (world ENU) in `swarm_commander`, watched
for every role — commanded drones once they are ACTIVE (climb-out and landing
pass through the floor on purpose), external RC-flown drones whenever they
are airborne (above `land_complete_altitude_m`, fresh odometry). What a
breach does is `fence_behavior`:

- **`hold_all`** (default, every autonomous config): a safety latch. Any
  watched drone outside the box freezes *every* drone at its current
  position, stops the scenario, and refuses `start` until `~/reset_fence`.
  An external drone trips it too — the holders stop; the RC pilot must bring
  their own drone back.
- **`keep_in`** (the teleop configs): nobody stops. Each commanded drone's
  velocity is clipped per axis so it cannot cross a wall, and a drone found
  outside is pushed back in. The wall is a **braking envelope**
  (`fence.wall_speed`, PX4's own braking law): the outward speed may not
  exceed the speed from which a stop *at* the wall is still reachable
  decelerating at `fence_brake_accel_mps2` — `sqrt(2·a·d)` far out, so the
  cruise speed is kept until the true braking distance
  `v²/(2a) + v/gain` and the brake is then firm — and, in the last stretch,
  `fence_keep_in_gain` × the distance left (the tail into the wall and the
  push-back rate from outside). `1/gain` is the response lag the envelope
  allows for. On the trajectory output the envelope's deceleration goes to
  PX4 as the acceleration feedforward on the limited axes
  (`fence.keep_in_acceleration`), so the vehicle brakes with the command
  instead of a velocity-loop lag later. A hand-flown drone's position target
  is clamped into the box as well. External drones cannot be steered; keep_in
  only logs them. `fence_margin_m` shrinks the box so the wall is met that
  early. All three dynamics are live (`ros2 param set`).

  *Why the envelope:* bag `run_045417` (drone_2, 8 m/s stick, keep_in, gain
  1, brake 0) went **0.35-0.68 m past the wall on every one of eleven
  approaches at ~6 m/s**. The old `gain × distance` cap starts falling 6 m
  out and is zero *at* the wall, but PX4 follows a bare velocity setpoint
  with ~0.1 s of delay and a ~0.55 s velocity-loop time constant (no
  feedforward was sent while the fence was active), so the drone was still
  doing 1.5 m/s when it crossed. Raising that gain makes it worse — the
  command drops faster than the vehicle can follow. In the plant model of
  `test_trajectory.py` (which overshoots 0.4 m at 6 m/s and 2.3 m at 8 m/s
  with the old cap), brake 4 m/s² + gain 2 + feedforward stops within
  0.02 m from 0.5-8 m/s and is at rest on the wall in 2.4-4.3 s, sooner
  than the old cap needs to overshoot and come back
  (`test_fence_and_position_hold.py`). The gain is also the stiffness of
  the last stretch: 3 rings at the wall through the 0.3 s loop delay, so
  use 2 on the trajectory output. Sim (velocity-only) drones get no
  feedforward and lag ~0.7 s: gain 0.7 (a 1.4 s margin) and brake 2.

- **Teleop fence** (`teleop_fence_enabled`, `teleop_fence_min` /
  `teleop_fence_max`): a second, smaller box for **hand-flown drones only**,
  which must lie inside the geofence (the commander refuses to start
  otherwise). The sticks meet it as a keep_in wall — same envelope, gain and
  margin as above — *whatever* `fence_behavior` is, so a pilot never reaches
  the geofence and the geofence stays the outer safety net (a `hold_all`
  geofence still latches if something else goes wrong). Scenario-driven and
  external drones ignore it. Drawn amber in the 3D view; the status snapshot
  carries both boxes (`fence`, `teleop_fence`). A floor above the ground is
  fine: only an ACTIVE drone is held inside a keep_in box — velocity clip
  *and* reference clamp — so take-off and landing pass through it. (The
  reference used to be clamped in every state, and a landing drone with
  `teleop_fence_min` z = 0.3 hovered at 0.3 m: PX4's stiff altitude loop
  held the clamped position setpoint against the descent command.)

Config (per profile):
```yaml
fence_enabled: true
fence_behavior: "hold_all"      # or "keep_in"
fence_brake_accel_mps2: 4.0     # keep_in: braking deceleration of the envelope (m/s2); 0 = plain gain*d
fence_keep_in_gain: 2.0         # keep_in: near-wall speed <= gain*distance (1/s); 1/gain = lag margin (sim: 0.7, brake 2)
fence_margin_m: 0.0             # keep_in: m
fence_min: [-2.5, -2.5, 0.3]    # x,y,z lower limits (world ENU, m)
fence_max: [ 2.5,  2.5, 2.5]    # x,y,z upper limits
teleop_fence_enabled: true      # hand-flown drones: a smaller keep_in box inside the geofence
teleop_fence_min: [-1.5, -1.5, 0.5]
teleop_fence_max: [ 1.5,  1.5, 2.0]
```
Recover — **no relaunch needed**:
```bash
ros2 service call /swarm_commander/reset_fence std_srvs/srv/Trigger
# or the "Reset Fence" button in the SVG Basestation panel
```
`reset_fence` only clears the `hold_all` latch — it does not move anything. If
a drone is **still hovering outside** the box, clearing is not enough: the
check runs every control tick and re-latches immediately (the reply warns
`still outside: drone_1`). The recovery is:

1. `land` — descent is fence-exempt; the drone touches down where it is and
   disarms.
2. `takeoff` — the climb-out is fence-exempt too and flies to the drone's
   takeoff target (`hover_positions` / the initial goal), which is inside the
   box, so it arrives holding inside and `start` is accepted again.

(`land` / `takeoff` act on every commanded drone, so the others cycle with it.)
A drone that **landed** outside needs only step 2. Also fix what sent it out —
a `goal_command` or formation profile beyond the wall will do it again on the
next `start`; under `hold_all` the commander does not clamp goals to the fence.

`hold_all` is a freeze-in-place, not a motor cutoff, and `keep_in` is a
velocity clip, not a wall — the RC kill switch remains the true cutoff. The
fence box is drawn in RViz / Foxglove (green normally, red when latched),
together with a **ground grid on the fence floor** clipped to the fence
footprint: lines on world multiples of `fence_grid_cell_m` (default 0.5 m; `0`
disables), whole metres brighter, so x=0 / y=0 are on the grid and a drone's
position reads straight off it. It follows whatever fence the loaded config
has — the 3D panel's own grid is a fixed 8 m square on the origin and is
turned off in `svg_basestation.json` for that reason.

---

## Recording rosbags

Record INTO the mounted workspace (`~/AirStack/robot/ros_ws/...`); paths
outside the bind mounts stay trapped in the container. Ctrl-C stops AND writes
`metadata.yaml`.

```bash
ros2 bag record -o ~/AirStack/robot/ros_ws/bags/run_$(date +%H%M%S) \
  /drone_1/odometry_conversion/odometry /drone_2/odometry_conversion/odometry \
  /drone_3/odometry_conversion/odometry \
  /drone_1/interface/velocity_command /drone_2/interface/velocity_command \
  /drone_3/interface/velocity_command
ros2 bag info <bag_dir>          # sanity
```
Live position of one drone: `ros2 topic echo
/drone_1/odometry_conversion/odometry --field pose.pose.position`. To rescue a
bag from a non-mounted path: `docker cp
airstack-robot-desktop-1:/root/AirStack/<path> ~/AirStack/...`.

---

## Automated tests

```bash
cd ~/AirStack/robot/ros_ws/src/svg_ground_control
# unit (pure numpy, no ROS):
python3 -m pytest test/test_cbf.py test/test_scenarios.py -q
# node-level (needs rclpy; constructs the commander, no launch/interfaces):
python3 -m pytest test/test_exempt.py -q          # cbf_exempt_drones / teleop decoupling

# closed-loop functional (fake drones; launch the matching commander first):
#   ground_control.launch.py config:=<share>/config/<cfg>.yaml   then:
python3 test/functional_single_goal_test.py     # cfg: goal_single.yaml
python3 test/functional_multi_goal_test.py      # cfg: goal_tracking.yaml
python3 test/functional_hybrid_test.py          # cfg: hybrid_squeeze.yaml
python3 test/functional_squeeze_test.py         # cfg: squeeze_3drone.yaml
python3 test/functional_squeeze_lag_test.py     # cfg: squeeze_3drone.yaml (PX4-lag model)
python3 test/functional_fence_test.py           # cfg: goal_single.yaml
```
The fake-drone tests integrate the commander's velocity commands and publish
odometry in per-drone local frames, so they also exercise the offset
correction and (for hybrid) the command routing. Give the commander ~5 s to
come up before starting a test.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `[ERROR] Docker daemon is not running` but it is | user not in `docker` group: `sudo usermod -aG docker $USER`, then a real logout/login (lock screen doesn't count), or `newgrp docker` per shell |
| LED strip **dark** on a drone | On the VOXL: `systemctl status svg-led`, `journalctl -u svg-led -n 30`. "cannot open /run/mpa/modal_io_bridge" = this voxl-px4 build has no FIFO (Starling 2 Max SDK) — the daemon then sends the same packet as a **MAVLink TUNNEL** through `voxl-mavlink-server` (`/run/mpa/mavlink_onboard/control`; needs that service active; journal: "opened MAVLink tunnel sink"). The tunnel payload type is a ModalAI enum; `voxl_setup_led.sh` reads it from the mavlink headers (211 on SDK 1.8) — wrong value = dark strip. Still dark: strip is RGB not RGBW (`LED_EXTRA_ARGS="--rgb"`), `--brightness 0`, or the strip is not on ESC **id 0** (fw ≥ 39 only lets id 0 drive it). Hardware check with PX4 stopped: `voxl-esc` wrapper procedure + `python3 voxl-esc-neopixel-test.py --mode all -n 11 --brightness 40 --id 0` in `/usr/share/modalai/voxl-esc-tools` |
| LED strip **flickers orange** (green shows for an instant, then orange blink; steady only while voxl-px4 is stopped) | PX4's `voxl_esc` driver stamps its **status-LED bits** into every motor command (disarmed: red; armed: blue / position: green / **offboard: red**) and ESC fw 39.21 mirrors them onto an active NeoPixel strip, alternating with our frames at the 20 Hz passthrough rate. Fix = freeze the driver's LED bits: `px4-qshell voxl_esc -l 0 led` (**options before the verb**, `voxl_esc led -l 0` is silently ignored). `svg_led_daemon.py` does this itself at start and after every voxl-px4 restart (journal: "PX4 ESC LED bits muted"); `--no-px4-led-mute` disables it. Side effect: the ESCs' own tiny status LEDs no longer show the arm state. Verify: `systemctl restart voxl-px4` → strip flickers for ~10 s, then back to steady green |
| `led_controller`: **"no LED heartbeat yet"** for a drone | Daemon not running (`systemctl status svg-led` on the VOXL), its ground IP is wrong (`grep -- --ground-ip /etc/systemd/system/svg-led.service` — re-run `voxl_setup_led.sh <name> <ground_ip>` after re-pointing the drone), UDP 47901 blocked on the ground PC (`sudo ufw allow 47901/udp`), or the name is missing from the config's `led_controller.drone_names`. `ros2 topic list` does not show it — it is plain UDP; use `sudo tcpdump -ni any udp port 47901` |
| drone stays **red** after the CBF episode / colors do not change | red is only held while `/svg/cbf_active` is fresh (< `cbf_stale_s`, 1 s) — a dead commander clears it by itself; a color that never changes = the daemon is not receiving (see the row above) or is replaying its 10 s fallback (ground silent → green). `ros2 topic echo /svg/cbf_active` shows the raw per-tick signal |
| `airstack connect` shows no prompt | it attaches to the container tmux; `Ctrl-b c` new window, `Ctrl-b d` detach — or use `--command=bash` |
| 3 robot containers appear | `.env NUM_ROBOTS` also scales container replicas; keep it `"1"`, pass drone count inline to the sim script |
| service `waiting for service to become available…` forever | `ROS_DOMAIN_ID` mismatch between shells; also `ros2 daemon stop` |
| `package 'svg_ground_control' not found` / `bws`/`sws` not found / `topic list` shows only `/parameter_events`,`/rosout` | you're not in a robot-container shell. Use `./airstack.sh connect robot --command=bash` — its `.bashrc` sets `ROS_DOMAIN_ID` and sources the workspace, and `bws`/`sws` exist. A raw `docker exec … sh` skips all that |
| hardware: can't reach the drone / Motive (no `/fmu/*`, no `/<body>/pose` topics) | the robot container is on the Docker **bridge** net, not your LAN. Set `robot-desktop` to `network_mode: host` (comment out `networks:`/`ports:`) and `./airstack.sh up` — Part B prerequisite. Host mode ⇒ `NUM_ROBOTS=1` |
| `MicroXRCEAgent: command not found` | your robot image predates the bake-in — rebuild it (`./airstack.sh image-build robot-desktop`; [`Dockerfile.robot`](../../../docker/Dockerfile.robot) ~L198/L364 installs v2.4.3 to `/opt/uxrce`). No-rebuild alternative: build it in the workspace (clone eProsima Micro-XRCE-DDS-Agent into `ros_ws/src`, `bws --packages-select microxrcedds_agent && sws` — see B2). Last resort: the `microros/micro-ros-agent:jazzy` host container. Keep `ROS_DOMAIN_ID=1` on the agent |
| `px4-microdds_client` keeps stopping (must ssh in and `start` it by hand) | Two layers of auto-restart, both installed by re-running [`voxl_setup_real_drone.sh`](scripts/voxl_setup_real_drone.sh): a boot-time retry loop in `voxl-px4-start` (survives slow Wi-Fi at boot) and the **`svg-microdds-watchdog` systemd service** (checks every 1 s, restarts a stopped client mid-session). Verify: `systemctl status svg-microdds-watchdog`; find why it died: `journalctl -u voxl-px4 -b \| grep -i microdds`. "Running, disconnected" is NOT dead — the client reconnects itself once the ground agent is back |
| Drone not reachable / wrong or stale IP (e.g. old static `192.168.30.x`) | ADB in and reset `wlan0` to DHCP: `ip addr flush dev wlan0 && ip link set wlan0 up && udhcpc -i wlan0` (or `dhclient -v wlan0`), then `ip addr show wlan0`. Make it persistent via the `systemd-networkd` `*wlan0*.network` (`DHCP=yes`) or a router-side DHCP reservation — see [B0](#b0-get-the-drone-onto-your-lan-wi-fi--dhcp) |
| "Arm command sent" / `arm -> success=True` but the drone **never arms**, and `ros2 topic echo /<name>/fmu/out/vehicle_command_ack --qos-reliability best_effort` prints **nothing** | Those "success" values only mean the command was *published*. No ack at all = PX4's commander dropped it: `target_system` ≠ the drone's `MAV_SYS_ID` (`px4-param show MAV_SYS_ID` on the VOXL). `real_interfaces.launch.py` derives it from the name (drone_2 → 2) or takes `target_systems:=…`; confirm in the `PX4Interface initialized … target_system=N` line. An ack that says `DENIED`/`TEMPORARILY_REJECTED` (now logged as WARN by px4_interface) means PX4 *did* hear you and refused — run `px4-commander check` on the VOXL for the reason (usually the B4b EKF2 params, or an RC/kill-switch requirement) |
| real drone **won't arm** ("fuse failure" / "no position"), no `/fmu/out/vehicle_odometry` | EKF2 has no position source. Set `EKF2_EV_CTRL`/`EKF2_HGT_REF=Vision`/`EKF2_GPS_CTRL=0` (B4b), and verify `/{name}/fmu/in/vehicle_visual_odometry` is streaming. The SVG real path feeds it via `mocap_bridge` (`px4_vio_mode: direct`) — **not** MAVROS |
| QGC: **"yaw estimate error"**, won't get ready | EKF's yaw is contested or has no source. `px4-listener estimator_status_flags`: `cs_mag_hdg: True` → mag still fused — set `EKF2_MAG_TYPE=5` **and** `SYS_HAS_MAG=0`, `px4-param save`, restart voxl-px4 (B4b.1); `cs_ev_yaw: False` → yaw bit missing — `EKF2_EV_CTRL=11`. If both look right: mocap yaw itself is bad — quaternion flipping while still (`ros2 topic echo /<name>/pose --field pose.orientation`; symmetric Motive markers → re-create body), wrong frame (`px4_vio_frame`), or lossy Wi-Fi EV stream |
| QGC: **"no local position estimate"** (after mag/GPS were disabled) | Disabling mag+GPS removed the old sources but EV isn't fusing in their place: `EKF2_EV_CTRL` must include position bits (11, not yaw-only 8), `EKF2_HGT_REF=3`; params saved + PX4 restarted (unsaved params die on power loss; fusion config applies at EKF init). Params are **per drone** — re-check after re-provisioning/renaming. Verify with `px4-listener estimator_status_flags` (`cs_ev_pos/cs_ev_hgt`) and `px4-listener vehicle_local_position` (`xy_valid`) |
| `ros2 topic echo /…/fmu/out/…` shows nothing (but the topic exists) | PX4 `/fmu/*` are **best_effort**; add `--qos-reliability best_effort --qos-durability volatile` to echo. Not a real outage |
| **one drone behaves differently** from the others (e.g. drone_1 **balloons 15-20 cm at every stop** in Position mode, then sinks back over ~2 s; drone_2 does 4-8 cm for the same 30-37° bank reversals) | First diff PX4 params between a good and a bad drone's ulog: `python3 scripts/ulog_param_diff.py bad.ulg good.ulg MPC_ EKF2_` (needs `pip install pyulog`). drone_1 vs drone_2: **identical** params, thrust curve, motor/ESC response, attitude tracking, mocap latency (~49 ms, `EKF2_EV_DELAY`=50 is right), no lever arm, no saturation. What differed: drone_1's EKF **vertical velocity lags truth by ~0.2 m/s for ~0.5 s after braking with a negative roll** (only that direction), so the height controller keeps pushing after the reversal; drone_2's EKF tracks within 0.07 m/s. Cause consistent with a small IMU-to-airframe misalignment on drone_1 (EKF-vs-mocap roll offset −0.5°, pitch disagreement ±1.5° with roll; vertical-accel error antisymmetric in roll). Fixes, in order: (1) tighten mocap fusion so the EKF trusts EV height over the IMU transient — `EKF2_EVP_NOISE 0.03` (or `EKF2_EV_NOISE_MD 1`, which uses the bridge's 1 cm variance, as drone_3 already does) on **all** drones; (2) redo accel + level-horizon calibration on drone_1 (`CAL_ACC0_*SCALE`=1.0 on every drone = never fully calibrated); (3) recreate drone_1's Motive rigid body with the airframe squared up (EV yaw is fused; its yaw is 1.5° off the IMU). Does not affect offboard swarm flights (≤1.2 m/s, no 30° banks). Do NOT "fix" by copying drone_3's `MPC_VEL_MANUAL`=2 — that is drone_3's intentional speed limit, not the cause |
| drone **yaws slowly by itself** in Position/Altitude mode (steady ~10-20°/s, sticks released) while EKF yaw, mocap yaw and gyro all agree in the ulog | Not the estimator, not the controller: PX4 is being *commanded* a yaw rate by the RC yaw channel. In the ulog, `manual_control_setpoint.yaw` sits at a non-zero value (e.g. +0.21) with the stick released, because `RC<n>_TRIM` (n = `RC_MAP_YAW`) no longer matches the transmitter's centre (seen: TRIM=1427 vs centre 1540 µs; ch1-3 fine). PX4 applies `MPC_HOLD_DZ`=0.1 then `MPC_YAW_EXPO` and `MPC_MAN_Y_MAX` — (0.21−0.1)/0.9 → expo → ×150°/s ≈ 12.7°/s, exactly the observed drift. Fix: recalibrate the radio in QGC (or `px4-param set RC4_TRIM <centre>` + `px4-param save`) and check the transmitter's yaw trim/subtrim. Offboard flights ignore sticks, but a Position-mode takeover will spin until this is fixed. Verify at rest: `px4-listener manual_control_setpoint` → `yaw` ≈ 0 |
| EV accepted but drone drifts / flies the wrong way / position mirrored | mocap frame ≠ ROS-ENU. Do the B4b hand-check; set `px4_vio_frame: "modalai_flip"` (the reference transform) in `swarm_real.yaml` |
| Isaac Sim segfaults at startup, backtrace in `librtx.scenedb.plugin.so` / `libcarb.scenerenderer-rtx.plugin.so` at `carbOnPluginStartup` — **also crashes headless**, and a bare empty `SimulationApp({"headless":True})` crashes identically | GPU driver ↔ Isaac Sim RTX incompatibility, NOT an AirStack bug. App boots to `app ready` then the RTX renderer faults on the first frame. Confirmed on RTX 5080 / Blackwell + NVIDIA driver **595.x** + Isaac Sim 5.1.0. Headless and clearing the shader cache do **not** help (the renderer plugin loads at app init regardless; there is no renderer-less path through Kit). **Fix:** install a driver Isaac Sim 5.1 supports — Linux **580.65.06**, or **591.74** (a Blackwell user's confirmed-good version) — using the *open* kernel module variant required for RTX 50-series; or upgrade to a newer Isaac Sim release. ([NVIDIA forum report](https://forums.developer.nvidia.com/t/isaac-sim-5-1-gui-crash-access-violation-on-rtx-5070-ti-blackwell-fixed-by-driver-downgrade-to-591-74/365335)) |
| MAVROS `connected: false`, no odometry | PX4 SITL not launched: Isaac timeline not playing (`PLAY_SIM_ON_START=true`, or press Play) |
| takeoff returns success=false right after launch | commander hasn't received odometry yet — wait a few seconds and retry |
| one drone "odometry stale"/won't arm in a MULTI-drone task but flies fine in a single-drone task | that drone's chain isn't up in THIS session — infra is per drone: its `MicroXRCEAgent` (own port) running? its name included in `real_interfaces.launch.py drones:=…`? its `/pose` streaming? Triage in order: `ros2 node list \| grep drone_X` → `hz /drone_X/pose` → best_effort echo `/drone_X/fmu/out/vehicle_odometry` → `hz /drone_X/odometry_conversion/odometry`. If ALL stream but staleness is intermittent with 3 drones on Wi-Fi: link congestion — check `hz` max interval vs `state_timeout_s` (0.5 s), improve AP or raise the timeout slightly |
| `The parameter 'X' is not initialized` | empty YAML list can't override a typed param — `teleop_drones`/`external_drones`/`drone_modes`/`cbf_exempt_drones` are comma-separated STRINGS (`""` = none) |
| teleop drone gets shoved around / won't act as the obstacle | teleop is **no longer auto-CBF-exempt** — add it to `cbf_exempt_drones` to leave its commands uncorrected (and let others dodge it). Conversely, drop it from the list to have the filter protect your manual commands |
| commander rejects config: `"X" is in both external_drones and cbf_exempt_drones` | external drones are never commanded, so they can't be "exempt" — remove the name from one of the two lists |
| Isaac avatar (real drone) doesn't appear / doesn't move | (a) launch Isaac with a **GUI** viewport (`ISAAC_SIM_HEADLESS=false`) and `DRONE_MODES` listing that drone as `real`; (b) the avatar tracks `…/odometry_conversion/odometry` — confirm it's flowing (`ros2 topic hz`); (c) domain: the script sets `ROS_DOMAIN_ID=SVG_DOMAIN_ID`, so the drone must publish on that domain; (d) **true hardware**: the real drone's odometry is on a host-network container while Isaac is on the bridge network — DDS must cross them (run Isaac with host networking or a discovery server). Pure-sim dry-run: publish a synthetic odom or run that drone as a throwaway SITL |
| `start` says "not all drones holding yet" | drones still converging to takeoff targets; retry after a few seconds |
| takeoff & land work but the drone **won't move to a goal** | goal-seeking needs a SUCCESSFUL `/start` (takeoff/land don't). Re-run `/start` and read the response: "not all drones holding yet" → loosen `arrival_threshold_m` or wait; "geofence breached" → `~/reset_fence` + widen fence. Then confirm motion with `ros2 topic echo /<name>/fmu/velocity_command` and that the goal arrived (`ros2 topic echo /svg/<name>/goal_command --once`). The velocity path itself is fine — that's what takeoff used |
| `start` says "geofence breached" | a drone left the box; `ros2 service call /swarm_commander/reset_fence std_srvs/srv/Trigger` after recovering |
| drones fly right *shapes* in wrong *places*; intruder misses the gap | per-drone PX4 local origins: `drone_position_offsets` must equal the sim spawn positions (`x = 2*(i-1) - (N-1)` → `[-2,0,0, 0,0,0, 2,0,0]` for 3). Zeros only for mocap-anchored hardware |
| hybrid: a "real" drone never moves | nothing is consuming `/{name}/fmu/velocity_command` — real-mode drones need px4_interface up (Part B); validate the routing first with `functional_hybrid_test.py` |
| teleop: `pad: NO /joy` although the pad **is** plugged in | the container cannot see the device. `ls /dev/input/js0` on the host, then `docker exec airstack-robot-desktop-1 ls /dev/input/js0` — `privileged` populates `/dev` only at container start, so a pad plugged in afterwards is invisible. `robot-base-docker-compose.yaml` bind-mounts `/dev/input`; a container created before that needs **recreating** (`AUTOLAUNCH=false airstack up robot-desktop`), not restarting. Confirm with `ros2 run joy joy_enumerate_devices` (SDL's `Failed loading udev_device_get_action` line is harmless). |
| teleop: `REFUSING TO COMMAND: forward (axis 4) rests at +1.00` | wrong `teleop_controller` for this pad: that axis is an analog trigger, which rests at full scale and would command full speed untouched. The bench pad is `dragonrise_usb` (right stick on axes 2/3), an Xbox pad is `xbox_usb` (3/4). Check with `ros2 run svg_ground_control joy_map`. |
| teleop: sticks move the wrong drone axis (but nothing is refused) | a rearranged stick layout is not detectable automatically — only a resting trigger is. Verify each direction on the ground against the printed `cmd vx/vy/vz`, then fix the profile's axis numbers in `safe_teleop/controllers.py` (or override `forward_axis` etc. in the config's `safe_teleop` block). |
| **speed does not change** (`scenario_speed_mps`, `speed_command`, `ros2 param set`) | Three separate things. (1) `ros2 param set /swarm_commander scenario_speed_mps X` used to answer *successful* and do nothing (read once at startup) — it is now applied live, and other params answer with a reason. (2) The speed is a cruise cap: the drone brakes at `goal_accel_mps2` and eases in over `goal_settle_s`, so it reaches the setting only if the goal is farther than `v²/(2a) + v·settle` (0.6 m at 1.2 m/s, 5.7 m at 5 m/s with the defaults) — the commander prints that distance with every speed it receives. (3) `cbf_max_speed_mps` caps everything (1.2 in the goal configs). The bags show the drone tracks the commanded speed within 0.05 m/s, so if the log says 1.0 the drone flies 1.0. |
| **teleop speed does not change** (raised `max_speed_mps`) | Three caps, lowest wins: `safe_teleop.max_speed_mps` (stick scaling, read at launch — relaunch `teleop.launch.py`), the commander's `teleop_max_speed_mps` (default **1.2**, live via `ros2 param set`; the commander now warns `stick velocity … capped`), and `cbf_max_speed_mps` unless the drone is `cbf_exempt`. |
| **`~/hold` makes a fast drone fly back ("bounce")** | Fixed: hold now brakes on the goal law to a predicted stop point `v²/(2·goal_accel_mps2) + v/hover_kp` ahead (clamped into the fence) and holds there, instead of chasing the position at the instant of the call (bag run_041842: 1.5 m bounce from 6 m/s). The service reply says `braking, stops X m ahead`. |
| **drone oscillates / "outside the fence, pushing it back" at a goal** | The goal is on or beyond a fence wall (e.g. y = 4.5 with `fence_max` y = 4.5): the keep_in envelope goes to zero speed at the wall, so the drone can never quite arrive and the profile keeps pushing. Put goals ≥ 0.5 m inside the box. |
| **every commander log line appears twice, jerky flight** | Two `swarm_commander` instances are running (a second `ground_control.launch.py` in another terminal), both publishing setpoints to the same drone. `ros2 node list` must show one `/swarm_commander`; kill the extra. |
| **takeoff shoots up past the hover height, then drops** | The reference point ran ahead of a drone that could not follow yet (PX4's takeoff thrust ramp, ~1.5 s of no lift) and PX4's stiff altitude loop (`MPC_Z_P` 5) then chased it: bag `C1_0920_203148` reached 1.94 m for a 1 m target. Fixed by `hold_lead_m` (0.2 m leash while ascending/landing/holding — do not raise it) and the `takeoff_speed_mps` climb profile; check the commander is the rebuilt one (`bws`). |
| **overshoots the goal / stops sluggishly / oscillates around it** | Make sure the drone is on the trajectory output: the startup log says `real output: trajectory` and `ros2 topic hz /drone_N/fmu/trajectory_command` shows 20 Hz (it needs the rebuilt px4_interface; with `velocity` output PX4 has no position loop and no feedforward, and the ground loop cannot beat its ~0.7 s velocity lag). Then `goal_accel_mps2` too high for the airframe (drop to 2), or `goal_settle_s` too small (raise to 0.5). A sim/MAVROS drone always flies the softer velocity-only law (`goal_velocity_only_settle_s`). |
| RViz empty | Fixed Frame must be `map`; check `ros2 topic hz /svg/viz/markers`; needs an X display (`echo $DISPLAY`) |
