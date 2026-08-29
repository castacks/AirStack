# Adding a State Estimator

A state estimator in AirStack is any node that produces the robot's primary state estimate — the odometry surface every downstream consumer (safety monitor, PID controller, DROAN, random_walk, trajectory controller, task servers) subscribes to. The contract is [Interface Conventions §2](../interface_conventions.md#2-odometry-primary-state-estimate): `nav_msgs/Odometry` on `odometry_conversion/odometry` (RELIABLE QoS), `pose` in the `map` frame (ENU, meters), `twist` in the body frame (`child_frame_id`), plus the `map → base_link` TF. This guide assumes you have run the stack before and know the [layered architecture](../index.md); it swaps the estimator, not the consumers.

Today's default estimator path is PX4's EKF: MAVROS publishes `/{robot_name}/interface/mavros/local_position/odom`, and the `odometry_conversion` node (from the `robot_interface` package, launched inside `robot/ros_ws/src/interface/interface_bringup/launch/interface.launch.py`) normalizes it onto the canonical surface — it restamps `frame_id`/`child_frame_id` to `map`/`base_link`, republishes on `odometry_conversion/odometry`, and broadcasts the `map → base_link` TF (`convert_odometry_to_transform: true`). Your estimator replaces the *input* to that node, not the node itself.

## Package or module?

Decide early where the estimator lives:

- **In-tree package** — fastest for trunk work: a normal ROS 2 package under `robot/ros_ws/src/perception/`, or a scaffolded module boundary in your fork via `airstack module create --in-tree <name>` (lands under `robot/ros_ws/src/modules/<name>`).
- **Module repo** — shareable, version-pinned, with its own CI and Docker dependency layer: a thin external repo added with `airstack module add <url> --version <pin>`. See [AirStack Modules](../../../development/modules.md) (especially [the researcher workflow](../../../development/modules.md#the-researcher-workflow-fork-module)) and the [create-module skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/create-module/SKILL.md). The precedent for a state estimator shipped this way is [asm_macvo](../../../modules/macvo.md) — MAC-VO learned stereo visual odometry, consumed by the [full_macvo](../../../../stacks/full_macvo/README.md) stack.

The steps below are the same either way; only step 5 differs.

## Steps

### 1. Create the package

Follow the [add-ros2-package skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/add-ros2-package/SKILL.md) and the [Module Integration Checklist](../integration_checklist.md). Put it under `robot/ros_ws/src/perception/` (or use the module scaffold above). Declare every topic endpoint as a launch argument defaulting to its canonical name — that is what lets a stack include it with zero remaps.

**Verify:** it builds inside the robot container — `docker exec airstack-robot-desktop-1 bash -c "bws --packages-select <your_package>"` exits cleanly.

### 2. Conform to the spec §2 surface

Publish `nav_msgs/Odometry` with a RELIABLE publisher; link the [spec table](../interface_conventions.md#2-odometry-primary-state-estimate) from your README rather than restating it. Frames per the spec's [TF table](../interface_conventions.md#tf-frames-and-units): `pose` in `map` (ENU, meters), `twist` in the body frame named by `child_frame_id`, yaw right-handed about +Z.

The recommended integration is to publish your estimate on your own namespaced topic (e.g. `perception/<your_estimator>/odometry`) and route it *through* `odometry_conversion` — `interface.launch.py` declares the `interface_odometry_in_topic` launch argument exactly for this. You then keep frame normalization and the `map → base_link` TF broadcast for free. If you instead bypass `odometry_conversion` and publish the canonical topic directly, **you** must broadcast `map → base_link` — `odometry_conversion` is the node that publishes it in the default graph, and it is launched unconditionally by `interface.launch.py`, so bypassing also means forking that launch file to avoid two publishers on the same surface. Route through it.

**Verify:** with your node running, `docker exec airstack-robot-desktop-1 bash -c "sws && ros2 topic echo /$ROBOT_NAME/perception/<your_estimator>/odometry --once"` shows a sane pose and the frame ids you expect.

### 3. Wire it into a stack

The [single-locus rule](../../../development/stacks.md#the-single-locus-rule-and-its-lint): all wiring deviations live in one place — the stack entry launch file. Never edit `interface.launch.py` or another module's launch file to point at your estimator. Copy a reference stack and change the include lines:

```bash
airstack stack new full_default full_my_estimator
```

Then in `stacks/full_my_estimator/launch/stack.launch.xml`, (a) include your estimator under the `perception` namespace, and (b) pass the interface's odometry-input arg. This is the same pattern [full_macvo](../../../../stacks/full_macvo/README.md) uses to add MAC-VO under `perception/` and rewire one consumer with a single include arg (`stacks/full_macvo/launch/stack.launch.xml` — there the deviation is DROAN's disparity input; here it is the interface's odometry input):

```xml
<!-- My estimator: publishes /$ROBOT_NAME/perception/my_estimator/odometry -->
<group>
  <push-ros-namespace namespace="perception" />
  <include file="$(find-pkg-share my_estimator)/launch/my_estimator.launch.xml" />
</group>

<!-- Interface: odometry_conversion consumes my estimator instead of MAVROS EKF -->
<include file="$(find-pkg-share interface_bringup)/launch/interface.launch.py">
  <arg name="interface_odometry_in_topic"
    value="/$(env ROBOT_NAME)/perception/my_estimator/odometry" />
</include>
```

**Verify:** `airstack up --stack full_my_estimator --sim isaac --robots 1`, then `airstack ready`, then `docker exec airstack-robot-desktop-1 bash -c "ros2 node list"` shows your estimator node alongside `odometry_conversion`.

### 4. Verify the running graph

1. Rate and content: `docker exec airstack-robot-desktop-1 bash -c "sws && ros2 topic hz /robot_1/odometry_conversion/odometry"` reports a `state`-class rate (~10–100 Hz), and `ros2 topic echo ... --once` shows `frame_id: map`, `child_frame_id: base_link`.
2. Snapshot the wiring and diff it: `airstack test -m wiring --stack full_my_estimator` regenerates `stacks/full_my_estimator/wiring.md`; review that the only deviations from `full_default`'s wiring are your estimator and the odometry input.
3. Live drift check: `airstack doctor --live --stack full_my_estimator` reports the running graph against the committed `wiring.md` with no unexplained diffs.

A best-effort/reliable QoS mismatch here fails *silently* (consumers receive nothing) — the spec calls this out; `ros2 topic hz` from step 1 is the check that catches it.

### 5. Optional: package it as a module

If the estimator should be shareable and pinnable outside your fork, graduate it to a module repo following [AirStack Modules](../../../development/modules.md) and the [create-module skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/create-module/SKILL.md), with [asm_macvo](../../../modules/macvo.md) as the worked precedent (heavy deps in `Dockerfile.module`, a consuming reference stack, module CI). Your stack's `modules.repos` then pins it, exactly as `stacks/full_macvo/modules.repos` pins `asm_macvo`.

**Verify:** `airstack module add <your-repo-url> --version <tag>` followed by `airstack module doctor` passes, and `airstack up --stack full_my_estimator` brings the graph up from a clean checkout.

## See also

- [Interface Conventions Specification](../interface_conventions.md) — the citable contract (§2 odometry, TF frames)
- [Module Integration Checklist](../integration_checklist.md) — package-level integration steps
- [Perception Packages](index.md) — where estimators live in the layer
- [AirStack Stacks](../../../development/stacks.md) — stack anatomy, wiring.md, doctor
