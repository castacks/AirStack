# Hardware vs Isaac velocity-command replay (ZOH)

Primary comparison for `drone1_20260812_030151`. Hardware flew offboard
**velocity** commands at 20 Hz. This path replays that `TwistStamped` stream
into PX4 SITL. It does **not** use Takeoff / Navigate / Land (`replay_in_sim.py`).

## Pieces

- Plant: `simulation/isaac-sim/assets/robots/starling2max.usd`
- Scene: `simulation/isaac-sim/launch_scripts/starling2max_velocity_replay.py`
- Node: `robot/ros_ws/src/interface/mavros_interface/scripts/replay_velocity_zoh.py`
- Host runner: `tests/hw_sim_compare/run_velocity_replay.py`
- Overlay: `tests/hw_sim_compare/plot_velocity_overlay.py`

## Bring-up

```bash
VERSION=0.19.0-alpha.5 PLAY_SIM_ON_START=true \
  ISAAC_SIM_SCRIPT_NAME=starling2max_velocity_replay.py \
  airstack up isaac-sim

VERSION=0.19.0-alpha.5 AUTOLAUNCH=false airstack up robot-desktop
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select mavros_interface"
docker exec airstack-robot-desktop-1 bash -c "sws && ros2 launch desktop_bringup robot.launch.xml role:=full"
```

Then:

```bash
python3 tests/hw_sim_compare/run_velocity_replay.py --mode step
python3 tests/hw_sim_compare/run_velocity_replay.py --mode full
python3 tests/hw_sim_compare/plot_velocity_overlay.py \
  --sim tests/hw_sim_compare/results/velocity_zoh_result.npz \
  --out tests/hw_sim_compare/results/overlay.png
```
