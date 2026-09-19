#!/usr/bin/env bash
# Bring up a whole teleop experiment from one command, on the host.
#
#   ./svg_teleop.sh solo             one drone, nothing to avoid
#   ./svg_teleop.sh squeeze          you fly the intruder, the holders yield
#   ./svg_teleop.sh hover            three drones hovering, the CBF corrects you
#   ./svg_teleop.sh squeeze --headless   same, no Isaac viewport
#
#   ./svg_teleop.sh real             ONE REAL DRONE (mocap + uXRCE-DDS), no sim.
#                                    Read teleop.md "Real drone" first.
#
#   ./svg_teleop.sh takeoff      arm and ascend
#   ./svg_teleop.sh start        hand control to the sticks
#   ./svg_teleop.sh hold         stop where you are
#   ./svg_teleop.sh land
#   ./svg_teleop.sh reset-fence  clear a geofence breach
#   ./svg_teleop.sh monitor      live sticks + commanded velocity (this terminal)
#   ./svg_teleop.sh status       what is running, odometry, roles
#   ./svg_teleop.sh logs <name>  isaac | agent | natnet | iface | commander | teleop | joy
#   ./svg_teleop.sh stop         kill everything, leave containers up
#
# Everything runs in tmux inside the containers, so nothing needs a terminal
# of its own. Attach to any of it with:
#   docker exec -it isaac-sim tmux attach -t isaac
#   docker exec -it airstack-robot-desktop-1 tmux attach -t <iface|commander|teleop|joy|rviz>

set -uo pipefail

ROBOT=airstack-robot-desktop-1
ISAAC=isaac-sim
WS=/root/AirStack/robot/ros_ws
SHARE="\$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control"
SIM_SCRIPT=/isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py

red()  { printf '\033[0;31m%s\033[0m\n' "$*"; }
grn()  { printf '\033[0;32m%s\033[0m\n' "$*"; }
ylw()  { printf '\033[0;33m%s\033[0m\n' "$*"; }
step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }

# The containers' .bashrc greets every login shell, so strip that chatter or
# it lands in the middle of captured values.
denoise() { grep -vE '^(Sourcing /|Usage: .*resolve_robot_name\.py)' || true; }
rexec()  { docker exec "$ROBOT" bash -lc "$1" 2>/dev/null | denoise; }
iexec()  { docker exec "$ISAAC" bash -lc "$1" 2>/dev/null | denoise; }
# A tmux SERVER keeps the environment of whatever shell first started it, and
# every later session inherits that, not the caller's. A server started before
# the container resolved ROS_DOMAIN_ID leaves each session on domain 0 while
# the rest of the stack is on domain 1 — nodes run, MAVROS still reaches PX4
# over UDP, and yet no ROS topic is visible to anything. Teardown kills the
# server, and every session exports the domain explicitly as a backstop.
ROS_DOMAIN=""
detect_domain() {
    ROS_DOMAIN=$(docker exec "$ROBOT" bash -lc 'echo "DOMAIN:$ROS_DOMAIN_ID"' 2>/dev/null \
                 | sed -n 's/^DOMAIN:\([0-9][0-9]*\)$/\1/p' | head -1)
    ROS_DOMAIN=${ROS_DOMAIN:-0}
}
rtmux()  { docker exec "$ROBOT" bash -lc "tmux kill-session -t $1 2>/dev/null; tmux new -d -s $1 \"export ROS_DOMAIN_ID=$ROS_DOMAIN; $2\"" 2>/dev/null; }

need_containers() {   # need_containers <container...>  (default: robot only)
    local list=("$@"); [ "${#list[@]}" -eq 0 ] && list=("$ROBOT")
    for c in "${list[@]}"; do
        docker ps --format '{{.Names}}' | grep -qx "$c" || {
            red "Container '$c' is not running."
            echo "  cd ~/AirStack && AUTOLAUNCH=false ./airstack.sh up robot-desktop"
            [ "$c" = "$ISAAC" ] && echo "  cd ~/AirStack && AUTOLAUNCH=false ./airstack.sh up isaac-sim"
            exit 1
        }
    done
}

# Everything that has ever leaked between runs. A stale odom_modifier is the
# nastiest: it double-publishes one drone's odometry and the commander then
# chases two conflicting estimates, which looks like a drone that will not
# settle rather than like an error.
teardown() {
    iexec "pkill -f svg_multi_drone_single_domain.py; pkill -f 'bin/px4'" || true
    rexec "pkill -f 'ros2 launch'; pkill -f launch_sim_interfaces; \
           pkill -9 -f swarm_commander; pkill -9 -f safe_teleop; pkill -9 -f joy_node; \
           pkill -9 -f rviz2; pkill -9 -f odom_modifier; pkill -9 -f odometry_conversion; \
           pkill -9 -f robot_interface; pkill -9 -f position_setpoint; pkill -9 -f mavros; \
           pkill -9 -f mocap_bridge; pkill -9 -f natnet; pkill -f MicroXRCEAgent" || true
    # Kill the whole server, not just our sessions: a server left over from an
    # earlier shell carries that shell's environment into everything it spawns.
    rexec "tmux kill-server 2>/dev/null" || true
    sleep 4
}

wait_for() {   # wait_for <desc> <timeout_s> <shell test>
    local desc="$1" timeout="$2" test_cmd="$3" waited=0
    printf '    %s ' "$desc"
    while ! eval "$test_cmd" >/dev/null 2>&1; do
        sleep 5; waited=$((waited + 5))
        printf '.'
        if [ "$waited" -ge "$timeout" ]; then
            echo; red "    timed out after ${timeout}s"
            return 1
        fi
    done
    echo " ok (${waited}s)"
    return 0
}

bringup() {
    local opt="$1" headless="$2"
    local n config overrides drone

    case "$opt" in
        solo)    n=1; config=teleop_single.yaml; overrides=""; drone=drone_1
                 desc="one drone, nothing to avoid" ;;
        squeeze) n=3; config=swarm_sim.yaml; overrides="scenario:=squeeze teleop_drones:=drone_3"; drone=drone_3
                 desc="you fly the intruder, the holders yield" ;;
        hover)   n=3; config=swarm_sim.yaml; overrides="teleop_drones:=drone_3"; drone=drone_3
                 desc="three drones hovering — the CBF corrects you" ;;
        *) red "Unknown experiment '$opt' (want solo, squeeze or hover)"; exit 1 ;;
    esac

    need_containers "$ROBOT" "$ISAAC"
    detect_domain
    grn "$opt: $desc"
    echo "  $n drone(s), $config ${overrides:-(no overrides)}, you fly $drone"

    step "Clearing anything left from a previous run"
    teardown
    grn "    clean"

    step "Isaac Sim ($n drone(s), $([ "$headless" = true ] && echo headless || echo GUI), camera)"
    iexec "tmux kill-session -t isaac 2>/dev/null; tmux new -d -s isaac \
        \"NUM_ROBOTS=$n SVG_DOMAIN_ID=1 PLAY_SIM_ON_START=true ISAAC_SIM_HEADLESS=$headless ENABLE_CAMERA=true CAMERA_DRONES=$drone \
          PYTHONPATH=\\\$ISAAC_SIM_PYTHONPATH /isaac-sim/python.sh $SIM_SCRIPT \
          --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts 2>&1 | tee /tmp/isaac.log\""
    # 'Ready for takeoff!' per drone is the real readiness signal. The log is
    # mostly deprecation warnings until then, so it looks stalled when it is not.
    wait_for "waiting for $n PX4(s) to boot (minutes with the GUI)" 900 \
        "[ \"\$(docker exec $ISAAC bash -lc 'grep -c \"Ready for takeoff\" /tmp/isaac.log 2>/dev/null' 2>/dev/null)\" -ge $n ]" || {
        red "Isaac never came up. ./svg_teleop.sh logs isaac"; exit 1; }

    step "Drone interfaces"
    rtmux iface "cd $WS && source install/setup.bash && ./src/svg_ground_control/scripts/launch_sim_interfaces.sh $n 2>&1 | tee /tmp/iface.log"
    for d in $(seq 1 "$n"); do
        wait_for "drone_$d odometry" 180 \
            "docker exec $ROBOT bash -lc 'source $WS/install/setup.bash >/dev/null 2>&1; timeout 6 ros2 topic hz /drone_$d/odometry_conversion/odometry 2>&1 | grep -q average'" || {
            red "drone_$d never produced odometry. ./svg_teleop.sh logs iface"; exit 1; }
    done

    # One publisher per drone. Two means a stale interface survived and the
    # drone will wander instead of holding station.
    step "Checking for duplicate publishers"
    local bad=0
    for d in $(seq 1 "$n"); do
        local c
        c=$(rexec "source $WS/install/setup.bash >/dev/null 2>&1; timeout 12 ros2 topic info /drone_$d/odometry_conversion/odometry 2>/dev/null | grep -m1 'Publisher count' | awk '{print \$3}'")
        c=${c:-0}
        if [ "$c" = "1" ]; then
            echo "    drone_$d: 1 publisher"
        else
            red "    drone_$d: $c publishers — a stale interface is still running"
            bad=1
        fi
    done
    [ "$bad" = 1 ] && { red "Run ./svg_teleop.sh stop and try again."; exit 1; }

    step "Ground controller"
    rtmux commander "cd $WS && source install/setup.bash && ros2 launch svg_ground_control ground_control.launch.py config:=$SHARE/config/$config $overrides 2>&1 | tee /tmp/commander.log"
    wait_for "commander up" 60 \
        "docker exec $ROBOT bash -lc 'grep -q \"SwarmCommander up\" /tmp/commander.log 2>/dev/null'" || {
        red "Commander never started. ./svg_teleop.sh logs commander"; exit 1; }
    sleep 3
    # The banner prints before the first publish, so it is not proof of life.
    if rexec "grep -q 'process has died' /tmp/commander.log"; then
        red "    commander started then DIED — see ./svg_teleop.sh logs commander"
        rexec "grep -A2 'symbol lookup error' /tmp/commander.log | head -3" && \
            ylw "    stale message package; see teleop.md troubleshooting"
        exit 1
    fi
    rexec "grep -m1 'SwarmCommander up' /tmp/commander.log | sed 's/.*SwarmCommander/    SwarmCommander/'"

    step "joy_node (your gamepad)"
    rtmux joy "cd $WS && source install/setup.bash && ros2 run joy joy_node 2>&1 | tee /tmp/joy.log"
    sleep 4
    if rexec "grep -q 'Opened joystick' /tmp/joy.log"; then
        rexec "grep -m1 'Opened joystick' /tmp/joy.log | sed 's/.*\\[joy_node\\]:/   /'"
    else
        ylw "    no joystick opened — is the pad plugged in? (ls /dev/input/js*)"
    fi

    step "safe_teleop on $drone"
    # max_speed_mps 2.0: full stick = 2 m/s (the node's own default is a
    # conservative 1.0). The config's teleop/cbf caps are 2.0 to match.
    rtmux teleop "cd $WS && source install/setup.bash && ros2 run svg_ground_control safe_teleop --ros-args -p drone:=$drone -p max_speed_mps:=2.0 2>&1 | tee /tmp/teleop.log"
    sleep 3
    grn "    driving $drone"

    step "RViz"
    # Drop camera panels for drones this run never spawned, so nothing shows
    # a permanently blank feed.
    # Only the teleop drone carries a camera, so it is the only one with
    # panels worth keeping.
    local drones=" $drone"
    rtmux rviz "cd $WS && source install/setup.bash && \
        python3 src/svg_ground_control/scripts/rviz_for_run.py \
            \$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/svg_drones.rviz \
            /tmp/svg_drones_run.rviz$drones && \
        rviz2 -d /tmp/svg_drones_run.rviz 2>&1 | tee /tmp/rviz.log"
    sleep 3
    grn "    yellow = the drone you fly, cyan = scenario-driven, orange = geofence breached"

    echo
    grn "Ready. 1) Let the drone takeoff. 2) Start to teleop drone. 3) Land when finished."
    echo "  ./svg_teleop.sh takeoff"
    echo "  ./svg_teleop.sh start"
    echo "  ./svg_teleop.sh land"
    echo
    echo "  ./svg_teleop.sh monitor"
    echo "  ./svg_teleop.sh status"
    echo "  ./svg_teleop.sh stop"
}

# One real drone over mocap + uXRCE-DDS. Same commander, same teleop node,
# same takeoff/start/land — only the transport differs (px4_interface instead
# of MAVROS, mocap instead of Isaac). No Isaac container needed.
#
# NOT covered here (do them first — experiment.md Part B): VOXL provisioning
# (voxl_setup_real_drone.sh), EKF2 external-vision params + the frame
# hand-check (B4b), and the RViz hand-carry preflight (B5).
bringup_real() {
    local drone=drone_1 config=teleop_real.yaml n=1

    need_containers "$ROBOT"
    detect_domain
    grn "real: one REAL drone ($drone), gamepad teleop, mocap + uXRCE-DDS"
    red "  This flies hardware. PX4 failsafes and the RC kill switch are the"
    red "  safety net — keep a thumb on the kill switch. Yaw sign is UNVERIFIED"
    red "  by sim (real path negates it, sim does not): test yaw slowly, low."

    step "Clearing anything left from a previous run"
    teardown
    grn "    clean"

    step "uXRCE-DDS agent (port 8888)"
    rtmux agent "MicroXRCEAgent udp4 -p 8888 -v4 2>&1 | tee /tmp/agent.log"
    wait_for "agent process" 30 \
        "docker exec $ROBOT bash -lc 'pgrep -f MicroXRCEAgent >/dev/null'" || {
        red "MicroXRCEAgent did not start (not in the image? see experiment.md B2)"; exit 1; }
    # The client on the VOXL retries on its own; give it a moment but do not
    # block forever — a missing session shows up as no odometry below anyway.
    wait_for "drone session (is the drone powered + on wifi?)" 60 \
        "docker exec $ROBOT bash -lc 'grep -q \"session established\" /tmp/agent.log'" || \
        ylw "    no session yet — continuing, but expect the odometry gate to fail"

    step "Real drone interfaces (px4_interface)"
    rtmux iface "cd $WS && source install/setup.bash && ros2 launch svg_ground_control real_interfaces.launch.py drones:=$drone 2>&1 | tee /tmp/iface.log"

    step "NatNet mocap"
    if ! rexec "source $WS/install/setup.bash >/dev/null 2>&1; ros2 pkg prefix natnet_ros2 >/dev/null 2>&1; echo BUILT:\$?" | grep -q 'BUILT:0'; then
        red "natnet_ros2 is not built. In the robot container:"
        echo "  bws --packages-select natnet_ros2 && sws     (first build needs internet)"
        exit 1
    fi
    rtmux natnet "cd $WS && source install/setup.bash && ros2 launch natnet_ros2 natnet_ros2.launch.py 2>&1 | tee /tmp/natnet.log"
    wait_for "$drone mocap pose (is Motive streaming?)" 120 \
        "docker exec $ROBOT bash -lc 'source $WS/install/setup.bash >/dev/null 2>&1; timeout 6 ros2 topic hz /$drone/pose 2>&1 | grep -q average'" || {
        red "$drone never appeared on /$drone/pose. Motive not streaming, wrong"
        red "serverIP, or the rigid body is not named $drone. ./svg_teleop.sh logs natnet"; exit 1; }

    step "Ground controller + mocap bridge"
    rtmux commander "cd $WS && source install/setup.bash && ros2 launch svg_ground_control ground_control.launch.py config:=$SHARE/config/$config use_mocap:=true 2>&1 | tee /tmp/commander.log"
    wait_for "commander up" 60 \
        "docker exec $ROBOT bash -lc 'grep -q \"SwarmCommander up\" /tmp/commander.log 2>/dev/null'" || {
        red "Commander never started. ./svg_teleop.sh logs commander"; exit 1; }
    sleep 3
    if rexec "grep -q 'process has died' /tmp/commander.log"; then
        red "    commander started then DIED — see ./svg_teleop.sh logs commander"
        exit 1
    fi
    rexec "grep -m1 'SwarmCommander up' /tmp/commander.log | sed 's/.*SwarmCommander/    SwarmCommander/'"

    # Mocap -> EKF -> odometry is the whole real state chain; no odometry here
    # means the EKF is not fusing external vision (experiment.md B4b).
    wait_for "$drone odometry (mocap -> EKF -> odometry_conversion)" 120 \
        "docker exec $ROBOT bash -lc 'source $WS/install/setup.bash >/dev/null 2>&1; timeout 6 ros2 topic hz /$drone/odometry_conversion/odometry 2>&1 | grep -q average'" || {
        red "$drone never produced odometry. Check the EKF2 external-vision"
        red "params + QoS checks in experiment.md B4b. ./svg_teleop.sh logs iface"; exit 1; }

    step "Checking for duplicate publishers"
    local c
    c=$(rexec "source $WS/install/setup.bash >/dev/null 2>&1; timeout 12 ros2 topic info /$drone/odometry_conversion/odometry 2>/dev/null | grep -m1 'Publisher count' | awk '{print \$3}'")
    c=${c:-0}
    if [ "$c" = "1" ]; then
        echo "    $drone: 1 publisher"
    else
        red "    $drone: $c publishers — a stale interface is still running"
        red "Run ./svg_teleop.sh stop and try again."; exit 1
    fi

    step "joy_node (your gamepad)"
    rtmux joy "cd $WS && source install/setup.bash && ros2 run joy joy_node 2>&1 | tee /tmp/joy.log"
    sleep 4
    if rexec "grep -q 'Opened joystick' /tmp/joy.log"; then
        rexec "grep -m1 'Opened joystick' /tmp/joy.log | sed 's/.*\\[joy_node\\]:/   /'"
    else
        ylw "    no joystick opened — is the pad plugged in? (ls /dev/input/js*)"
    fi

    step "safe_teleop on $drone"
    rtmux teleop "cd $WS && source install/setup.bash && ros2 run svg_ground_control safe_teleop --ros-args -p drone:=$drone 2>&1 | tee /tmp/teleop.log"
    sleep 3
    grn "    driving $drone"

    step "RViz"
    rtmux rviz "cd $WS && source install/setup.bash && \
        python3 src/svg_ground_control/scripts/rviz_for_run.py \
            \$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/svg_drones.rviz \
            /tmp/svg_drones_run.rviz $drone && \
        rviz2 -d /tmp/svg_drones_run.rviz 2>&1 | tee /tmp/rviz.log"
    sleep 3

    echo
    grn "Stack is up. GROUND CHECK BEFORE FLYING (teleop.md \"Real drone\"):"
    echo "  ./svg_teleop.sh monitor          # wiggle sticks, check directions"
    echo "  # and/or, in the container:"
    echo "  ros2 topic echo /svg/$drone/teleop_command"
    echo "  # carry the drone by hand: its RViz marker must track (B5 preflight)"
    echo
    red "Yaw cannot be ground-checked. First flight: yaw slowly, low altitude."
    echo
    grn "Then, thumb on the RC kill switch:"
    echo "  ./svg_teleop.sh takeoff"
    echo "  ./svg_teleop.sh start"
    echo "  ./svg_teleop.sh land"
}

call_srv() {
    local srv="$1"
    local out
    out=$(rexec "source $WS/install/setup.bash >/dev/null 2>&1; timeout 25 ros2 service call /swarm_commander/$srv std_srvs/srv/Trigger 2>&1 | grep Trigger_Response")
    if [ -z "$out" ]; then
        red "No response from /swarm_commander/$srv — is the commander running? (./svg_teleop.sh status)"
        return 1
    fi
    if echo "$out" | grep -q "success=True"; then
        grn "$(echo "$out" | sed 's/.*message=//; s/)$//')"
    else
        ylw "$(echo "$out" | sed 's/.*message=//; s/)$//')"
        case "$out" in
            *"not all drones holding"*)
                echo "  A drone has not reached its post yet. Wait and retry;"
                echo "  if it never settles, watch it with ./svg_teleop.sh status" ;;
            *"missing odometry"*)
                echo "  Interfaces are not up. ./svg_teleop.sh logs iface" ;;
        esac
        return 1
    fi
}

monitor() {
    need_containers
    detect_domain
    # Which drone is being flown is whatever safe_teleop was started on.
    local drone
    drone=$(rexec "grep -m1 -o 'driving drone_[0-9]*' /tmp/teleop.log 2>/dev/null | grep -o 'drone_[0-9]*'")
    drone=${drone:-drone_1}
    if ! rexec "tmux has-session -t teleop 2>/dev/null"; then
        ylw "safe_teleop is not running — bring an experiment up first."
    fi
    echo "Monitoring $drone. Ctrl-C to quit."
    # -it so curses gets a real terminal; this one deliberately runs attached.
    docker exec -it "$ROBOT" bash -lc \
        "export ROS_DOMAIN_ID=$ROS_DOMAIN; cd $WS && source install/setup.bash && \
         ros2 run svg_ground_control teleop_monitor --ros-args -p drone:=$drone"
}

# The monitor is a curses view, so it needs its own terminal. There is no host
# tmux and no way to split the VSCode terminal from a script, so the best
# available is a separate window; if that is not possible, say what to run.
open_monitor() {
    local here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    if [ -n "${DISPLAY:-}" ] && command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="svg teleop monitor" \
            -- bash -lc "cd '$here' && ./svg_teleop.sh monitor" >/dev/null 2>&1 &
        grn "  monitor opened in a new window"
    else
        echo "  watch the sticks with:  ./svg_teleop.sh monitor"
    fi
}

status() {
    need_containers
    step "Processes"
    for s in iface commander teleop joy rviz; do
        if rexec "tmux has-session -t $s 2>/dev/null"; then grn "    $s: running"; else red "    $s: not running"; fi
    done
    # Real-mode-only sessions: absent on a sim run, so only report when up.
    for s in agent natnet; do
        rexec "tmux has-session -t $s 2>/dev/null" && grn "    $s: running (real mode)"
    done
    if iexec "pgrep -f svg_multi_drone_single_domain.py >/dev/null"; then
        grn "    isaac: running ($(iexec "pgrep -cf 'bin/px4'" | tr -d '\n') PX4 instance(s))"
    else
        red "    isaac: not running (expected on a real-drone run)"
    fi

    step "Drones"
    local roles
    roles=$(rexec "grep -m1 'SwarmCommander up' /tmp/commander.log 2>/dev/null | sed 's/.*scenario=/scenario=/'")
    [ -n "$roles" ] && echo "    $roles"
    for d in 1 2 3; do
        local hz pos
        hz=$(rexec "source $WS/install/setup.bash >/dev/null 2>&1; timeout 5 ros2 topic hz /drone_$d/odometry_conversion/odometry 2>&1 | grep -m1 average | awk '{printf \"%.1f\", \$3}'")
        [ -z "$hz" ] && continue
        pos=$(rexec "source $WS/install/setup.bash >/dev/null 2>&1; timeout 5 ros2 topic echo --once /drone_$d/odometry_conversion/odometry 2>/dev/null | grep -A3 'position:' | sed -n '2,4p' | awk '{printf \"%7.2f\", \$2}'")
        echo "    drone_$d  ${hz} Hz  raw pos[$pos ]"
    done

    step "Recent commander output"
    rexec "grep -vE 'CBF active' /tmp/commander.log 2>/dev/null | tail -5 | sed 's/^/    /'"
}

logs() {
    case "${1:-}" in
        isaac)     iexec "tail -40 /tmp/isaac.log" ;;
        agent)     rexec "tail -40 /tmp/agent.log" ;;
        natnet)    rexec "tail -40 /tmp/natnet.log" ;;
        iface)     rexec "tail -40 /tmp/iface.log" ;;
        commander) rexec "grep -vE 'CBF active' /tmp/commander.log | tail -40" ;;
        teleop)    rexec "tail -30 /tmp/teleop.log" ;;
        joy)       rexec "tail -20 /tmp/joy.log" ;;
        rviz)      rexec "tail -20 /tmp/rviz.log" ;;
        *) red "logs <isaac|agent|natnet|iface|commander|teleop|joy|rviz>"; exit 1 ;;
    esac
}

usage() { sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'; }

cmd="${1:-}"; shift || true
headless=false
for a in "$@"; do [ "$a" = "--headless" ] && headless=true; done

case "$cmd" in
    solo|squeeze|hover) bringup "$cmd" "$headless" ;;
    real)         bringup_real ;;
    takeoff)      call_srv takeoff ;;
    start)        call_srv start && open_monitor ;;
    hold)         call_srv hold ;;
    land)         call_srv land ;;
    reset-fence)  call_srv reset_fence ;;
    monitor)      monitor ;;
    status)       status ;;
    logs)         logs "${1:-}" ;;
    stop)         need_containers; detect_domain; teardown; grn "Stopped. Containers left running." ;;
    ""|-h|--help) usage ;;
    *)            red "Unknown command '$cmd'"; echo; usage; exit 1 ;;
esac
