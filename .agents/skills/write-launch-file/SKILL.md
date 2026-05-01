---
name: write-launch-file
description: Author a ROS 2 launch file for AirStack with the correct conventions. Use when creating or editing any .launch.xml/.launch.py for the robot autonomy stack — covers ROBOT_NAME namespacing, topic remapping, allow_substs parameter loading, conditional launch, and layer bringup composition.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Write a ROS 2 Launch File for AirStack

## When to Use

- Creating a launch file for a brand-new module package (paired with [add-ros2-package](../add-ros2-package))
- Editing a layer's bringup launch file to wire in a new module (paired with [integrate-module-into-layer](../integrate-module-into-layer))
- Adding optional/conditional behavior (e.g. `enable_rviz`, `use_alt_planner`) to an existing launch file
- Composing multiple module launch files into a higher-level launch
- Any time a module's topic names, parameters, or remappings need to change

If you only need to tweak runtime parameters and the launch file already exists with sensible launch arguments, override via `<arg>` rather than editing the file.

## Core Conventions (Read First)

These are non-negotiable in AirStack:

1. **Every robot-side topic is under `/$(env ROBOT_NAME)/...`.** Never hardcode `/drone1`, `/robot`, etc. Multi-robot scenarios depend on this.
2. **Module nodes use *relative* topic names internally** (e.g. `odometry`, `global_plan`). The launch file **remaps** them to the correct global topic.
3. **YAML config files load with `allow_substs="true"`.** Without it, `$(env ...)` and `$(var ...)` inside the YAML are not expanded.
4. **XML is the AirStack default.** Use `*.launch.xml`. Use Python (`*.launch.py`) only when you need real control flow (loops, conditional graph construction, computed values).
5. **Launch files are installed at build time.** After editing, you must rebuild the package (`bws --packages-select <pkg>`) before the change takes effect.
6. **ROS 2 does NOT scope launch arguments.** A `<arg name="odometry_topic">` declared in a child launch file leaks to the parent. Prefix args with the layer/module name (e.g. `local_odometry_in_topic`) — see the warning at the top of `local.launch.xml`.

## XML vs. Python Launch Files

| Use XML when… | Use Python when… |
|---------------|------------------|
| Launching a fixed set of nodes | You need to loop (e.g. spawn N robots from a list) |
| Static remappings, parameter files, conditional groups | You need to compute values at launch time |
| Including other launch files | You need conditional logic that XML's `if`/`unless` cannot express |
| You want a config that is easy to diff and read | You need to read JSON/YAML and generate nodes from it |

Almost every module launch file and every layer bringup launch file in `robot/ros_ws/src/` is XML. Defaulting to XML keeps the codebase consistent and reviewable. If you find yourself reaching for Python, first check whether environment variables + `<arg>` + `<group if=...>` can express the same thing.

## Steps (Module Launch File)

This is the typical "I just made a new package" workflow. The result is a single `<package>/launch/<package>.launch.xml`.

### 1. Locate the Launch Directory

```
robot/ros_ws/src/<layer>/<category>/<package_name>/launch/<package_name>.launch.xml
```

Make sure your `CMakeLists.txt` (C++) or `setup.py` (Python) installs the `launch/` directory — see [add-ros2-package](../add-ros2-package). A launch file that is not installed does not exist as far as `ros2 launch` is concerned.

### 2. Declare Launch Arguments

Put every topic name, every config path, and every toggleable feature behind an `<arg>` with a sensible default. This is what lets the layer bringup file remap your module without editing your launch file.

```xml
<launch>
  <!-- Topic args: prefix with module name to avoid collisions in parent launches -->
  <arg name="my_planner_odometry_in_topic"
       default="/$(env ROBOT_NAME)/odometry_conversion/odometry" />
  <arg name="my_planner_global_plan_in_topic"
       default="/$(env ROBOT_NAME)/global_plan" />
  <arg name="my_planner_trajectory_out_topic"
       default="/$(env ROBOT_NAME)/trajectory_controller/trajectory_segment_to_add" />

  <!-- Config -->
  <arg name="my_planner_config"
       default="$(find-pkg-share my_planner)/config/my_planner.yaml" />

  <!-- Toggles -->
  <arg name="enable_my_planner_debug" default="false" />
```

### 3. Launch the Node with Param + Remap

```xml
  <node pkg="my_planner" exec="my_planner_node" name="my_planner" output="screen">
    <!-- allow_substs="true" is required if the YAML uses $(env ...) or $(var ...) -->
    <param from="$(var my_planner_config)" allow_substs="true" />

    <!-- Remap relative topic names from the node's code to global AirStack topics -->
    <remap from="odometry"            to="$(var my_planner_odometry_in_topic)" />
    <remap from="global_plan"         to="$(var my_planner_global_plan_in_topic)" />
    <remap from="trajectory_segment"  to="$(var my_planner_trajectory_out_topic)" />
  </node>
</launch>
```

The `<remap from="X" to="Y"/>` direction always reads as: "the topic the node calls **X** in its source code should resolve to **Y** at runtime."

### 4. (Optional) Add Conditional Sub-Components

Anything that should only sometimes run goes in a `<group if=...>`:

```xml
  <group if="$(var enable_my_planner_debug)">
    <node pkg="rviz2" exec="rviz2"
          args="-d $(find-pkg-share my_planner)/rviz/debug.rviz" output="screen" />
  </group>
```

Use `<group unless="$(var ...)">` for the inverse. For mutually exclusive alternatives, pair an `if` group with an `unless` group on the same arg.

### 5. (Optional) Push a Sub-Namespace

If your module spawns several supporting nodes, group them under a namespace so all of their topics get a clean prefix:

```xml
  <group>
    <push-ros-namespace namespace="my_planner" />
    <node pkg="my_planner" exec="planner_node"   output="screen"> ... </node>
    <node pkg="my_planner" exec="visualizer"     output="screen"> ... </node>
  </group>
```

This is exactly the pattern `local.launch.xml` uses for the `droan` group and `px4_interface.launch.xml` uses for the `fmu` group.

## Steps (Layer Bringup Launch File)

A layer bringup file (e.g. `local_bringup/launch/local.launch.xml`) is a *composition* — it does not start a single node, it starts every node the layer needs and wires them together with shared topic args.

### 1. Define Shared Topic Args at the Top

Use prefixed argument names so they cannot collide with sibling layers' args:

```xml
<launch>
    <!-- WARNING: ROS2 does NOT scope launch arguments. Use unique names. -->
    <arg name="local_odometry_in_topic"
         default="/$(env ROBOT_NAME)/odometry_conversion/odometry" />
    <arg name="local_disparity_in_topic"
         default="/$(env ROBOT_NAME)/perception/stereo_image_proc/disparity" />
    <arg name="local_camera_info_in_topic"
         default="/$(env ROBOT_NAME)/sensors/front_stereo/right/camera_info" />
```

### 2. For Each Module: Add a `<node>` (or `<include>`) Block

Two valid styles:

**Style A — direct `<node>`** (when the module's launch file is small or you need to override most of its params):

```xml
    <node pkg="trajectory_controller" exec="trajectory_controller"
          namespace="trajectory_controller" output="screen">
        <param name="target_frame" value="map" />
        <param name="look_ahead_time" value="1.0" />
        <!-- ... -->
        <remap from="odometry" to="$(var local_odometry_in_topic)" />
    </node>
```

**Style B — `<include>` the module's own launch file** (preferred when the module already exposes good launch args):

```xml
    <include file="$(find-pkg-share vdb_mapping_ros2)/launch/vdb_mapping_ros2.py">
        <arg name="config" value="$(find-pkg-share global_bringup)/config/vdb_params.yaml" />
    </include>
```

Style B keeps wiring concerns in the bringup file and parameter concerns in the module — that's the goal.

### 3. Wire Cross-Module Topics Using AirStack Standard Names

Use the canonical names from AGENTS.md → "Standard Topic Patterns":

| Topic | Where it comes from | Where it goes to |
|-------|---------------------|------------------|
| `/$(env ROBOT_NAME)/odometry_conversion/odometry` | `px4_interface` (or other interface) | every consumer (planners, controllers) |
| `/$(env ROBOT_NAME)/global_plan` | global planner | local planner |
| `/$(env ROBOT_NAME)/trajectory_controller/trajectory_segment_to_add` | local planner | trajectory controller |
| `/$(env ROBOT_NAME)/trajectory_controller/look_ahead` | trajectory controller | local planner |
| `/$(env ROBOT_NAME)/trajectory_controller/tracking_point` | trajectory controller | controllers, action servers |
| `/$(env ROBOT_NAME)/trajectory_controller/trajectory_override` | takeoff/land/fixed-traj action servers | trajectory controller |
| `/$(env ROBOT_NAME)/tasks/<task_name>` | behavior tree / GCS | task executor action server |

### 4. Add the Module to the Bringup `package.xml`

`ros2 launch` does not need this, but `colcon build --packages-up-to <bringup>` and the install dependency tracking do. Add `<depend>my_planner</depend>` to `<bringup>/package.xml`. See [integrate-module-into-layer](../integrate-module-into-layer) step 5.

### 5. Rebuild

```bash
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select <bringup_pkg> my_planner"
```

## Loading YAML Config Files

```xml
<param from="$(find-pkg-share my_planner)/config/my_planner.yaml" allow_substs="true" />
```

`allow_substs="true"` enables `$(env VAR)` and `$(var arg)` substitution **inside the YAML file itself**. Without it, a YAML line like:

```yaml
/**:
  ros__parameters:
    frame_id: $(env ROBOT_NAME)/base_link
```

…will load the literal string `"$(env ROBOT_NAME)/base_link"` and downstream code will fail in confusing ways. **When in doubt, set `allow_substs="true"` — there is no downside.**

YAML structure for ROS 2 parameters (this is what your `config/<package>.yaml` should look like):

```yaml
/**:
  ros__parameters:
    update_rate: 10.0
    target_frame: map
    enable_visualization: true
```

The `/**:` wildcard matches any node name, which is the most portable form when the launch file may rename the node.

## Including Child Launch Files

```xml
<include file="$(find-pkg-share other_pkg)/launch/other.launch.xml">
  <arg name="some_arg"  value="some_value" />
  <arg name="namespace" value="$(env ROBOT_NAME)/foo" />
</include>
```

Notes:
- Use `$(find-pkg-share <pkg>)` to locate launch files — never hardcode absolute paths.
- `<include>` runs the child launch file in the parent's argument scope (remember: ROS 2 does NOT scope args). Set every argument you depend on explicitly.
- You can include `.launch.xml` from a `.launch.xml`, and either format from a `.launch.py`.

## Conditional Launch Patterns

### Toggle a node on/off

```xml
<arg name="enable_logger" default="false" />

<group if="$(var enable_logger)">
  <node pkg="rosbag2" exec="record" args="-a" output="screen" />
</group>
```

### Pick one of two implementations

```xml
<arg name="use_droan" default="true" />

<group if="$(var use_droan)">
  <node pkg="droan_local_planner" exec="droan_local_planner" output="screen"> ... </node>
</group>
<group unless="$(var use_droan)">
  <node pkg="alt_local_planner"   exec="alt_local_planner"   output="screen"> ... </node>
</group>
```

### Read a default from an env var

```xml
<arg name="enable_rviz" default="$(env ENABLE_RVIZ false)" />
```

The second positional value to `$(env VAR default)` is the fallback when the env var is unset.

## Common Pitfalls

This list also appears in AGENTS.md → "Critical Pitfalls #5"; this is the expanded version.

- **Hardcoded topic names in node code.**
  - Bad: `create_subscription<Odometry>("/drone1/odometry", ...)`
  - Good: `create_subscription<Odometry>("odometry", ...)` and remap in the launch file.
- **Hardcoded topic prefixes in launch files.**
  - Bad: `<remap from="odometry" to="/drone1/odometry"/>`
  - Good: `<remap from="odometry" to="/$(env ROBOT_NAME)/odometry"/>`
- **Forgetting `allow_substs="true"` on a `<param from=...>`.** YAML substitutions silently fail to expand. Symptom: parameters look like literal `$(env ROBOT_NAME)` strings at runtime.
- **`ROBOT_NAME` env var not set.** All `$(env ROBOT_NAME)` substitutions become an empty string, producing topic names like `//odometry`. Set `ROBOT_NAME` in the container env (`robot/docker/.env`) or pass through compose.
- **`<arg>` name collisions across layers.** Two child launch files that both define `<arg name="odometry_topic">` will silently fight. Always prefix: `local_odometry_in_topic`, `global_odometry_in_topic`, etc.
- **Missing `install(DIRECTORY launch …)` in CMakeLists.txt.** The launch file builds fine but `ros2 launch` cannot find it. Run `ls install/<pkg>/share/<pkg>/launch/` to verify after build.
- **Editing a launch file but not rebuilding.** Launch files are *installed* by `colcon build`. The source `launch/` directory is NOT what `ros2 launch` reads. Always rebuild after editing.
- **Forgetting `output="screen"`.** The node runs but its logs go to a file you have to hunt for. Use `output="screen"` on every node during development.
- **Wrong `<remap from/to>` direction.** `from` is what the *node's source code* calls the topic; `to` is what it should resolve to at runtime. Swapping these is one of the most common silent failures.
- **Using `~/topic` without understanding it.** `~/foo` resolves to `<node_name>/foo`. Useful for action server private namespaces (see `random_walk_planner` and `takeoff_landing_planner` examples) but confusing if you don't expect it.
- **Pushing a namespace and then absolute-path remapping anyway.** `<push-ros-namespace>` only affects *relative* topic names. `<remap from="odometry" to="/drone1/odometry">` ignores the pushed namespace entirely. That's usually what you want, but be aware of the interaction.

## Verification

After writing or editing the launch file, run through this checklist:

```bash
# 1. Rebuild
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select <pkg>"

# 2. Confirm the file was installed
docker exec airstack-robot-desktop-1 bash -c "ls install/<pkg>/share/<pkg>/launch/"

# 3. Dry-run launch to catch XML/syntax errors
docker exec airstack-robot-desktop-1 bash -c "sws && ros2 launch <pkg> <pkg>.launch.xml --print"

# 4. Actually launch and watch logs
docker exec airstack-robot-desktop-1 bash -c "sws && ros2 launch <pkg> <pkg>.launch.xml"

# 5. Verify the node and its topics resolved correctly
docker exec airstack-robot-desktop-1 bash -c "ros2 node list"
docker exec airstack-robot-desktop-1 bash -c "ros2 node info /<robot_name>/<namespace>/<node>"
docker exec airstack-robot-desktop-1 bash -c "ros2 topic info /<robot_name>/<expected_topic>"
```

If `ros2 node info` shows a node subscribing to `/odometry` instead of `/<robot_name>/odometry_conversion/odometry`, your remap is wrong or `ROBOT_NAME` is unset.

## Skeleton Template

Copy-paste this into `robot/ros_ws/src/<layer>/<category>/<my_package>/launch/<my_package>.launch.xml` and replace the `MY_*` / `my_*` tokens.

```xml
<launch>
  <!-- =========================================================
       MY_MODULE_NAME launch file
       Wires MY_MODULE into the AirStack autonomy stack.
       Topic args use the `my_module_*` prefix to avoid collision
       when this file is included from a layer bringup.
       ========================================================= -->

  <!-- ── Topic arguments ─────────────────────────────────────── -->
  <arg name="my_module_odometry_in_topic"
       default="/$(env ROBOT_NAME)/odometry_conversion/odometry" />
  <arg name="my_module_global_plan_in_topic"
       default="/$(env ROBOT_NAME)/global_plan" />
  <arg name="my_module_output_topic"
       default="/$(env ROBOT_NAME)/my_module/output" />

  <!-- ── Config + toggles ────────────────────────────────────── -->
  <arg name="my_module_config"
       default="$(find-pkg-share my_package)/config/my_package.yaml" />
  <arg name="enable_my_module_debug" default="false" />

  <!-- ── Main node ───────────────────────────────────────────── -->
  <group>
    <push-ros-namespace namespace="my_module" />

    <node pkg="my_package" exec="my_module_node"
          name="my_module" output="screen">
      <!-- allow_substs lets the YAML use $(env ROBOT_NAME) etc. -->
      <param from="$(var my_module_config)" allow_substs="true" />
      <param name="enable_debug" value="$(var enable_my_module_debug)" />

      <!-- Inputs -->
      <remap from="odometry"    to="$(var my_module_odometry_in_topic)" />
      <remap from="global_plan" to="$(var my_module_global_plan_in_topic)" />

      <!-- Outputs -->
      <remap from="output"      to="$(var my_module_output_topic)" />
    </node>
  </group>

  <!-- ── Optional debug visualization ────────────────────────── -->
  <group if="$(var enable_my_module_debug)">
    <node pkg="rviz2" exec="rviz2"
          args="-d $(find-pkg-share my_package)/rviz/debug.rviz"
          output="screen" />
  </group>
</launch>
```

## References

- **Real launch files to study:**
  - Module: `robot/ros_ws/src/interface/px4_interface/launch/px4_interface.launch.xml`
  - Module: `robot/ros_ws/src/local/planners/droan_local_planner/launch/droan_local_planner.launch.xml`
  - Layer bringup (global): `robot/ros_ws/src/global/global_bringup/launch/global.launch.xml`
  - Layer bringup (local): `robot/ros_ws/src/local/local_bringup/launch/local.launch.xml`
  - Template: `.agents/skills/add-ros2-package/assets/package_template/launch/template.launch.xml`

- **ROS 2 docs:**
  - [ROS 2 Launch Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)
  - [Using ROS 2 Launch For Large Projects](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
  - [Launch XML format reference](https://design.ros2.org/articles/roslaunch_xml.html)

- **Related skills:**
  - [add-ros2-package](../add-ros2-package) — full package creation flow that includes a launch file
  - [integrate-module-into-layer](../integrate-module-into-layer) — wiring your launch file into a layer bringup
  - [debug-module](../debug-module) — diagnosing launch and topic issues
  - [test-in-simulation](../test-in-simulation) — verifying the launched stack end-to-end
