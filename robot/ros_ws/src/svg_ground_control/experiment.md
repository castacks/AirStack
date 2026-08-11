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
10. [Geofence](#geofence)
11. [Recording rosbags / monitoring](#recording-rosbags)
12. [Automated tests](#automated-tests)
13. [Troubleshooting](#troubleshooting)

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
| `swarm_commander.py` | `swarm_commander` | **The brain.** 20 Hz loop: build each drone's *nominal* velocity (from the scenario or teleop) → run the **CBF safety filter** → publish a per-drone velocity command. Owns takeoff/start/hold/land/reset_fence services, the geofence, and the RViz markers. |
| `scenarios.py` | (library) | Nominal-velocity policies: `hover`, `goal`, `random_walk`, `random_goals`, `head_on`, `antipodal`, `squeeze`. Pure NumPy, ported from `~/drone_soccer`. |
| `cbf_filter.py` | (library) | The velocity-CBF collision filter (`filter_velocities`), a verbatim port of `drone_soccer/cbf.py`. |
| `mocap_bridge.py` | `mocap_bridge` | Hardware only: `/{name}/pose` (mocap) → `/{name}/fmu/visual_odometry_in` for the PX4 EKF. |
| `keyboard_teleop.py` | `keyboard_teleop` | Drives one teleop drone (`-p drone:=drone_3`) with the keyboard. |

**Data flow inside `swarm_commander` each tick:**

```
 per-drone odometry  ──► (add drone_position_offsets → shared world frame)
        │
        ▼
   scenario.nominal_velocity()   ── OR ──  teleop / goal-command input
        │  (per-drone desired velocity, ENU)
        ▼
   cbf_filter.filter_velocities()   ◄── sees ALL drones' world positions
        │  (collision-safe velocities; cbf_exempt rows restored after)
        ▼
   geofence check (latch + freeze all if any drone outside the box)
        │
        ▼
   publish /{name}/<iface>/velocity_command   +   /svg/viz/markers (RViz)
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
| `/svg/{name}/teleop_command` | in | `geometry_msgs/TwistStamped` | keyboard_teleop → commander (teleop drones) |
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
./airstack.sh image-build robot-desktop     # REQUIRED after pulling this branch — see note
# .env: COMPOSE_PROFILES="desktop,isaac-sim", AUTOLAUNCH="false", NUM_ROBOTS="1"
grep -E '^(COMPOSE_PROFILES|AUTOLAUNCH|NUM_ROBOTS)' .env
./airstack.sh up
./airstack.sh status        # robot-desktop-1 and isaac-sim Up
```

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

### A5. Keyboard teleop (optional utility — NOT used by any standard experiment)

> In the standard experiments a drone is **sim**, **real**, or **external**
> (RC-flown, tracked-only) — none of the Part C tasks use teleop. This tool
> remains available for ad-hoc debugging only (requires putting a drone in
> `teleop_drones`, which no shipped config does).

```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 run svg_ground_control keyboard_teleop --ros-args -p drone:=drone_3
# w/s=±x  a/d=±y  r/f=up/down  space=stop  +/-=speed  q=quit
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

```bash
cd ~/AirStack/robot/ros_ws && sws
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/goal_single.yaml \
  use_mocap:=true
# control terminal:
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger
ros2 topic pub --once /svg/drone_1/goal_command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5, z: 1.4}}}"
ros2 topic pub --once /svg/drone_1/speed_command std_msgs/msg/Float32 "{data: 0.8}"
ros2 service call /swarm_commander/land std_srvs/srv/Trigger
```

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
  on its **own agent port** (drone_1→8888, drone_2→8889, drone_3→8890) → run
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
* real holders + **RC-flown external** intruder → [C5](#c5-squeeze-with-an-rc-flown-intruder-squeeze_rc_intruderyaml)

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

### C5. Squeeze with an RC-FLOWN intruder (`squeeze_rc_intruder.yaml`)

Your experiment plan #2: **drone_1,2 real holders** (commander-flown, hold the
posts and yield via the CBF) + **drone_3 real, RC-flown by a human pilot** as
the intruder. drone_3 is **`external_drones`**: the commander never arms or
commands it, but it IS tracked — its mocap-fed odometry enters the CBF, so the
holders dodge the pilot's drone. (An external drone may NOT also be in
`cbf_exempt_drones`; there is no command to exempt — the commander rejects
that config.)

**Mocap goes to ALL THREE drones** (the config's `mocap_bridge` lists
drone_1,2,3): the holders need external vision to arm and fly offboard, and
**drone_3's PX4 needs it too** so its onboard controller can fuse a position
for indoor RC **Position mode** — set the B4b.1 EKF2/mag params on all three
drones, not just the holders.

Prerequisites (all three drones per [Part B](#part-b--bring-in-a-real-drone-connect--verify)):
* own agent port per drone (e.g. 8888/8889/8890) → **three** `MicroXRCEAgent`s;
* B4b.1 params set + saved + rebooted on **each** drone;
* Motive bodies `drone_1..3` streaming; interfaces for **all three** (drone_3's
  px4_interface is state-only, but without it the commander can't see drone_3
  and the scenario pauses):
  `ros2 launch svg_ground_control real_interfaces.launch.py drones:=drone_1,drone_2,drone_3`

```bash
# commander (mocap always on):
ros2 launch svg_ground_control ground_control.launch.py \
  config:=$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/squeeze_rc_intruder.yaml \
  use_mocap:=true

# fly:
ros2 service call /swarm_commander/takeoff std_srvs/srv/Trigger   # arms/lifts ONLY the holders
ros2 service call /swarm_commander/start   std_srvs/srv/Trigger   # holders on posts (requires only
                                                                  # commanded drones holding)
# now the RC pilot takes off drone_3 (Position mode) and flies it through the
# gap — watch the gray (external) marker + red holders yield in RViz.
ros2 service call /swarm_commander/land    std_srvs/srv/Trigger   # lands ONLY the holders;
                                                                  # the RC pilot lands drone_3
```

> ⚠️ **Safety — the geofence does NOT police drone_3.** The fence only watches
> commander-flown (ACTIVE) drones; an external drone never trips it. The RC
> pilot (and their kill switch) is drone_3's only safety layer. A breach by a
> *holder* still freezes the holders as usual. Also: if drone_3's odometry goes
> stale (mocap dropout), the commander pauses the scenario and the holders fall
> back to holding position — by design.

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
solid sphere per drone (red=real, cyan=sim, yellow=teleop, gray=external,
orange=frozen-on-breach), translucent safety sphere (2r), name/mode/role
label, goal points, and the geofence box.

```bash
# from a robot-container shell (./airstack.sh connect robot --command=bash):
# ROS_DOMAIN_ID + workspace are already set by .bashrc. Needs an X display.
rviz2 -d $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/svg_drones.rviz
```
The config sets fixed frame `map` and adds the MarkerArray display. If you
open a bare `rviz2`: set Fixed Frame = `map`, Add → By topic →
`/svg/viz/markers`. This is the unified "see all drones" view for hybrid runs.

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

## Geofence

A safety latch in `swarm_commander`. If any **airborne, holding/active** drone
leaves the box `[fence_min, fence_max]` (world ENU), the commander latches a
breach: every drone freezes at its current position, the scenario stops, and
`start` is refused until you call `~/reset_fence`. Climb-out and landing pass
through the floor on purpose and are exempt from detection.

Config (per profile):
```yaml
fence_enabled: true
fence_min: [-2.5, -2.5, 0.3]    # x,y,z lower limits (world ENU, m)
fence_max: [ 2.5,  2.5, 2.5]    # x,y,z upper limits
```
Recover:
```bash
ros2 service call /swarm_commander/reset_fence std_srvs/srv/Trigger
```
This is a freeze-in-place, not a motor cutoff — the RC kill switch remains the
true cutoff. The fence box is drawn in RViz (green normally, red when latched).

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
| `airstack connect` shows no prompt | it attaches to the container tmux; `Ctrl-b c` new window, `Ctrl-b d` detach — or use `--command=bash` |
| 3 robot containers appear | `.env NUM_ROBOTS` also scales container replicas; keep it `"1"`, pass drone count inline to the sim script |
| service `waiting for service to become available…` forever | `ROS_DOMAIN_ID` mismatch between shells; also `ros2 daemon stop` |
| `package 'svg_ground_control' not found` / `bws`/`sws` not found / `topic list` shows only `/parameter_events`,`/rosout` | you're not in a robot-container shell. Use `./airstack.sh connect robot --command=bash` — its `.bashrc` sets `ROS_DOMAIN_ID` and sources the workspace, and `bws`/`sws` exist. A raw `docker exec … sh` skips all that |
| hardware: can't reach the drone / Motive (no `/fmu/*`, no `/<body>/pose` topics) | the robot container is on the Docker **bridge** net, not your LAN. Set `robot-desktop` to `network_mode: host` (comment out `networks:`/`ports:`) and `./airstack.sh up` — Part B prerequisite. Host mode ⇒ `NUM_ROBOTS=1` |
| `MicroXRCEAgent: command not found` | your robot image predates the bake-in — rebuild it (`./airstack.sh image-build robot-desktop`; [`Dockerfile.robot`](../../../docker/Dockerfile.robot) ~L198/L364 installs v2.4.3 to `/opt/uxrce`). No-rebuild alternative: build it in the workspace (clone eProsima Micro-XRCE-DDS-Agent into `ros_ws/src`, `bws --packages-select microxrcedds_agent && sws` — see B2). Last resort: the `microros/micro-ros-agent:jazzy` host container. Keep `ROS_DOMAIN_ID=1` on the agent |
| `px4-microdds_client` keeps stopping (must ssh in and `start` it by hand) | Two layers of auto-restart, both installed by re-running [`voxl_setup_real_drone.sh`](scripts/voxl_setup_real_drone.sh): a boot-time retry loop in `voxl-px4-start` (survives slow Wi-Fi at boot) and the **`svg-microdds-watchdog` systemd service** (checks every 1 s, restarts a stopped client mid-session). Verify: `systemctl status svg-microdds-watchdog`; find why it died: `journalctl -u voxl-px4 -b \| grep -i microdds`. "Running, disconnected" is NOT dead — the client reconnects itself once the ground agent is back |
| Drone not reachable / wrong or stale IP (e.g. old static `192.168.30.x`) | ADB in and reset `wlan0` to DHCP: `ip addr flush dev wlan0 && ip link set wlan0 up && udhcpc -i wlan0` (or `dhclient -v wlan0`), then `ip addr show wlan0`. Make it persistent via the `systemd-networkd` `*wlan0*.network` (`DHCP=yes`) or a router-side DHCP reservation — see [B0](#b0-get-the-drone-onto-your-lan-wi-fi--dhcp) |
| real drone **won't arm** ("fuse failure" / "no position"), no `/fmu/out/vehicle_odometry` | EKF2 has no position source. Set `EKF2_EV_CTRL`/`EKF2_HGT_REF=Vision`/`EKF2_GPS_CTRL=0` (B4b), and verify `/{name}/fmu/in/vehicle_visual_odometry` is streaming. The SVG real path feeds it via `mocap_bridge` (`px4_vio_mode: direct`) — **not** MAVROS |
| QGC: **"yaw estimate error"**, won't get ready | EKF's yaw is contested or has no source. `px4-listener estimator_status_flags`: `cs_mag_hdg: True` → mag still fused — set `EKF2_MAG_TYPE=5` **and** `SYS_HAS_MAG=0`, `px4-param save`, restart voxl-px4 (B4b.1); `cs_ev_yaw: False` → yaw bit missing — `EKF2_EV_CTRL=11`. If both look right: mocap yaw itself is bad — quaternion flipping while still (`ros2 topic echo /<name>/pose --field pose.orientation`; symmetric Motive markers → re-create body), wrong frame (`px4_vio_frame`), or lossy Wi-Fi EV stream |
| QGC: **"no local position estimate"** (after mag/GPS were disabled) | Disabling mag+GPS removed the old sources but EV isn't fusing in their place: `EKF2_EV_CTRL` must include position bits (11, not yaw-only 8), `EKF2_HGT_REF=3`; params saved + PX4 restarted (unsaved params die on power loss; fusion config applies at EKF init). Params are **per drone** — re-check after re-provisioning/renaming. Verify with `px4-listener estimator_status_flags` (`cs_ev_pos/cs_ev_hgt`) and `px4-listener vehicle_local_position` (`xy_valid`) |
| `ros2 topic echo /…/fmu/out/…` shows nothing (but the topic exists) | PX4 `/fmu/*` are **best_effort**; add `--qos-reliability best_effort --qos-durability volatile` to echo. Not a real outage |
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
| RViz empty | Fixed Frame must be `map`; check `ros2 topic hz /svg/viz/markers`; needs an X display (`echo $DISPLAY`) |
