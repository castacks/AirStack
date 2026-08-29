# Change a Parameter and See the Effect

A ~10 minute lesson in the core AirStack developer loop: **edit a config file
on the host → relaunch → watch the behavior change in sim → revert**. By the
end you will have made the drone take off at a different climb speed, and
you'll know exactly when an edit needs a rebuild and when it doesn't.

**Prerequisite:** you finished [Getting Started](index.md) and the stack is
running in Isaac Sim, with Foxglove open. Our vehicle for the lesson is
`takeoff_velocity` in
[takeoff_landing_planner](../../robot/ros_ws/src/local/planners/takeoff_landing_planner/README.md)
— the ascent speed used whenever a Takeoff goal's `velocity_m_s` is `0`.

!!! note "Why a YAML edit needs no rebuild"

    Host `robot/ros_ws/` is **bind-mounted** as
    `/root/AirStack/robot/ros_ws/` in the robot container
    (`robot/docker/robot-base-docker-compose.yaml`), and the `bws` build alias
    runs `colcon build --symlink-install` (`robot/docker/.bashrc`), so the
    *installed* config YAML is a symlink back to the source file you edit on
    the host. But ROS 2 nodes read parameters **once at startup** — so a YAML
    edit needs no rebuild, only a relaunch.

## 1. Fly the baseline

In Foxglove's Robot Tasks panel, open the **Takeoff** tab and set
`velocity_m_s` to `0` — zero means "use the config default", currently
1.0 m/s. Keep `target_altitude_m` at 10.0, pick your robot, **Send**.
**Check:** the drone reaches 10 m in about 10 seconds. **Land** it.

## 2. Edit the parameter on the host

In `robot/ros_ws/src/local/planners/takeoff_landing_planner/config/takeoff_landing_planner.yaml`, change:

```yaml
takeoff_velocity: 1.0   # -> 0.2
```

**Check:** the *installed* copy inside the container already shows your edit —
bind mount plus symlink-install, no `bws` needed. Hop into the container with
`airstack connect robot-desktop` (opens the container's tmux session; new
shell window with ++ctrl+b++ ++c++) and run:

```bash
grep takeoff_velocity ~/AirStack/robot/ros_ws/install/takeoff_landing_planner/share/takeoff_landing_planner/config/takeoff_landing_planner.yaml
```

## 3. Relaunch the robot stack

The node read its parameters at startup, so restart just the robot container
(the sim keeps running):

```bash
airstack down robot-desktop
airstack up robot-desktop
airstack ready
```

**Check:** the running node now holds your value. Connect again
(`airstack connect robot-desktop`, ++ctrl+b++ ++c++ for a shell window) and
run — it prints `Double value is: 0.2`:

```bash
sws && ros2 param get /robot_1/takeoff_landing_planner/takeoff_landing_task takeoff_velocity
```

## 4. See the effect

Send the same Takeoff as step 1 (`velocity_m_s` = `0`, altitude 10 m).

**Check:** the drone now crawls upward — roughly 50 seconds to 10 m instead
of 10. That's your edit, flying.

## 5. Revert

Change `takeoff_velocity` back to `1.0`, then repeat step 3.
**Check:** `ros2 param get` reports `1.0` again.

Congratulations! You just ran the loop every AirStack change goes through:
edit → relaunch → observe → revert.

!!! tip "When you *do* need to rebuild"

    Editing C++/Python source, message definitions, or launch/CMake files
    means running `bws` (optionally `bws --packages-select <package>`) inside
    the container before relaunching; `sws` sources the built workspace.

**Next stops:** the [robot configuration reference](../robot/configuration/index.md)
for the rest of the knobs, `robot/docker/.bashrc` for the `bws` / `sws` /
`autolaunch` helpers in full, and the
[debug-module skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/debug-module/SKILL.md)
when a change doesn't behave.
