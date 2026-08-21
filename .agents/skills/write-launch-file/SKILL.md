---
name: write-launch-file
description: Author a ROS 2 launch file for AirStack with the correct conventions. Use when creating or editing any .launch.xml/.launch.py — covers the single-locus wiring rule (remaps live ONLY in stack launch files), canonical-default topic args with descriptions, ROBOT_NAME namespacing, allow_substs parameter loading, and stack-folder composition.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Write a ROS 2 Launch File for AirStack

## When to Use

- Creating a launch file for a brand-new module package (paired with [add-ros2-package](../add-ros2-package))
- Wiring a module into a stack's entry launch file (`stacks/<name>/launch/*.launch.xml`)
- Adding optional/conditional behavior (e.g. `enable_rviz`) to an existing launch file
- Any time a module's topic names, parameters, or connections need to change

If you only need different runtime wiring and the module already declares topic args, override the args from the stack launch file — never edit the module.

## The Two Kinds of Launch File (Read First)

AirStack separates launch files into exactly two roles (RFC #379 §4 — the **single-locus wiring rule**):

| | Module launch file | Stack launch file |
|---|---|---|
| Lives in | `<package>/launch/` | `stacks/<name>/launch/` |
| Declares topic `<arg>`s | YES — with **canonical defaults** and a `description=` on every arg | May declare args (each needs `description=`) |
| `<remap>` / `remappings=` | **NEVER** | YES — this is the only place cross-module wiring lives |
| Hardcodes cross-module topics | NEVER | N/A — the stack file IS the wiring document |
| Format | XML default; Python only for real control flow | **XML only** — declarative, greppable, statically parseable |

Consequences:

- **All cross-module wiring lives in the stack's entry launch file(s).** One flat file where every `<include>` block reads "this module, these connections." `grep -r global_plan stacks/my_stack/` answers "who touches this."
- **A CI lint enforces it** ([`tests/meta/test_launch_single_locus.py`](../../../tests/meta/test_launch_single_locus.py)): any `<remap>`/`remappings=` outside `stacks/*/launch/` fails unless the file is in the frozen [`launch_lint_allowlist.txt`](../../../tests/meta/launch_lint_allowlist.txt). The allowlist only shrinks — never add to it. Legacy layer-bringup files still carry remaps (wrap form); they flatten into the stack files over time, deleting their allowlist lines as they go.
- **Module topic args default to the canonical names** from [docs/robot/autonomy/integration_checklist.md](../../../docs/robot/autonomy/integration_checklist.md). A module launched with zero overrides connects to a default stack correctly.

Current status: **wrap form**. Reference stacks (`stacks/full_default/` etc.) include the existing layer bringups; flattening the nodes and remaps into the stack files arrives in a later phase. New wiring you write goes in the stack file, not in a module or bringup file. See [docs/development/stacks.md](../../../docs/development/stacks.md).

## Core Conventions

1. **Every robot-side topic is under `/$(env ROBOT_NAME)/...`.** Never hardcode `/drone1`, `/robot`, etc. — multi-robot depends on this.
2. **Module nodes use *relative* topic names internally** (e.g. `odometry`, `global_plan`); the module launch file exposes them as prefixed `<arg>`s; the **stack** launch file overrides those args (or remaps) to wire modules together.
3. **Every `<arg>` gets a `description=`.** The lint requires it for stack launch files; treat it as mandatory everywhere — an undescribed arg is unwireable by the next person.
4. **YAML config files load with `allow_substs="true"`.** Without it, `$(env ...)`/`$(var ...)` inside the YAML are not expanded.
5. **ROS 2 does NOT scope launch arguments.** A child's `<arg name="odometry_topic">` collides with a sibling's. Prefix args with the module name: `my_planner_odometry_in_topic`.
6. **Launch files are installed at build time.** Rebuild (`bws --packages-select <pkg>`) after editing — except stack launch files, which are read from the bind-mounted `stacks/` folder and need no build.

## Steps (Module Launch File)

Result: a single `<package>/launch/<package>.launch.xml` that declares its interface and starts its nodes — **no remaps, no cross-module topic knowledge beyond canonical defaults**.

```xml
<launch>
  <!-- Topic args: module-name prefix, canonical defaults, described. -->
  <arg name="my_planner_odometry_in_topic"
       default="/$(env ROBOT_NAME)/odometry_conversion/odometry"
       description="Input odometry (nav_msgs/Odometry)" />
  <arg name="my_planner_global_plan_in_topic"
       default="/$(env ROBOT_NAME)/global_plan"
       description="Input global waypoint path (nav_msgs/Path)" />
  <arg name="my_planner_trajectory_out_topic"
       default="/$(env ROBOT_NAME)/trajectory_controller/trajectory_segment_to_add"
       description="Output planned trajectory segment (airstack_msgs/TrajectorySegment)" />

  <arg name="my_planner_config"
       default="$(find-pkg-share my_planner)/config/my_planner.yaml"
       description="Node parameter YAML" />

  <node pkg="my_planner" exec="my_planner_node" name="my_planner" output="screen">
    <param from="$(var my_planner_config)" allow_substs="true" />
    <!-- Topics come from the args: pass them as parameters or use the args
         directly in code via declared parameters. Do NOT add <remap> here —
         wiring is the stack file's job. -->
    <param name="odometry_topic" value="$(var my_planner_odometry_in_topic)" />
    <param name="global_plan_topic" value="$(var my_planner_global_plan_in_topic)" />
    <param name="trajectory_topic" value="$(var my_planner_trajectory_out_topic)" />
  </node>
</launch>
```

If the node subscribes by relative name in code (the common ROS pattern), keep the relative names and let the **stack** launch file remap them — the prohibition is on remaps *in module files*, not on nodes using relative names.

Sub-namespace grouping is still fine inside a module:

```xml
<group>
  <push-ros-namespace namespace="my_planner" />
  <node pkg="my_planner" exec="planner_node" output="screen" />
  <node pkg="my_planner" exec="visualizer" output="screen" />
</group>
```

## Steps (Stack Launch File)

A stack folder (`stacks/<name>/`) is the unit of topology — see [docs/development/stacks.md](../../../docs/development/stacks.md) for the full anatomy. Its `launch/stack.launch.xml` is **THE wiring document**: a flat list of module `<include>`s, each block stating that module's connections.

1. **Start from a reference stack.** Copy `stacks/full_default/` and rename (`airstack stack new` arrives in a later phase).
2. **One `<include>` per module.** Pass the module's declared topic args to wire it:

```xml
<!-- Local planner: disparity from MAC-VO instead of stereo -->
<include file="$(find-pkg-share droan_gl)/launch/droan_gl.launch.xml">
  <arg name="droan_gl_disparity_topic"
       value="/$(env ROBOT_NAME)/perception/macvo/disparity" />
</include>
```

3. **Remaps (when a module exposes relative names) go here and only here.**
4. **Swap a module = edit one include.** Point the include at a different package/launch file and adjust the args — nothing else changes.
5. **XML only** in stack launch files. Python launch stays available *inside* modules where genuinely needed.
6. **No rebuild needed** — `stacks/` is bind-mounted into the robot container at `/root/AirStack/stacks`; re-launch to pick up edits.
7. Run it: `airstack up --stack <name> --sim isaac`.

The shared per-robot preamble (ROBOT_NAME namespace push, `use_sim_time`, `robot_state_publisher`, world→map static TF) runs in `autonomy_bringup/launch/robot.launch.xml` before your stack entry file is included — do not repeat it.

## Loading YAML Config Files

```xml
<param from="$(find-pkg-share my_planner)/config/my_planner.yaml" allow_substs="true" />
```

`allow_substs="true"` enables `$(env VAR)` / `$(var arg)` substitution **inside the YAML**. Without it a line like `frame_id: $(env ROBOT_NAME)/base_link` loads as a literal string. When in doubt, set it — there is no downside.

```yaml
/**:
  ros__parameters:
    update_rate: 10.0
    target_frame: map
```

The `/**:` wildcard matches any node name — the most portable form.

## Conditional Launch Patterns

```xml
<arg name="enable_logger" default="false" description="Record a rosbag of all topics" />
<group if="$(var enable_logger)">
  <node pkg="rosbag2" exec="record" args="-a" output="screen" />
</group>
```

- `<group unless="...">` for the inverse; pair `if`/`unless` on one arg for two-way alternatives.
- Env-var default: `<arg name="enable_rviz" default="$(env ENABLE_RVIZ false)" description="..."/>` — the second token is the fallback when unset (`''` for an empty-string fallback).
- String comparison in conditions uses the escaped-quote eval idiom:
  `<group if="$(eval '&quot;$(var mode)&quot; == &quot;sim&quot;')">`.

## Common Pitfalls

- **Adding a `<remap>` to a module or layer launch file.** The single-locus lint fails CI. Wiring belongs in `stacks/<name>/launch/`.
- **Hardcoded topic names in node code.** Bad: `create_subscription("/drone1/odometry", ...)`. Good: relative `"odometry"` (stack remaps it) or a declared parameter fed by a launch arg.
- **Hardcoded topic prefixes.** Always `/$(env ROBOT_NAME)/...`, never a literal robot name.
- **Missing `description=` on an `<arg>`.** Mandatory in stack files (lint rule 3); expected everywhere.
- **Forgetting `allow_substs="true"`.** Symptom: parameters contain literal `$(env ROBOT_NAME)` at runtime.
- **`ROBOT_NAME` unset** → topics like `//odometry`. It is resolved by `robot/docker/.bashrc` via `resolve_robot_name.py`; for ad-hoc `docker exec`, pass `-e ROBOT_NAME=robot_1`.
- **`<arg>` name collisions across included files.** ROS 2 does not scope args — prefix them (`local_odometry_in_topic`, not `odometry_in_topic`). Passing a wrongly-named arg to an include fails *silently* (the bug that broke the legacy macvo local-bringup variant — superseded by `stacks/full_macvo/`, which deleted it; see that stack's README).
- **Editing a module launch file but not rebuilding.** `ros2 launch` reads the *installed* copy. (Stack launch files are exempt — bind-mounted, no build.)
- **Missing `install(DIRECTORY launch …)` in CMakeLists.txt.** Builds fine, `ros2 launch` can't find it.
- **Wrong `<remap from/to>` direction** (stack files only): `from` = the name in the node's code, `to` = what it should resolve to.
- **`~/topic`** resolves to `<node_name>/topic` — used for action-server private namespaces.

## Verification

```bash
# Module files: rebuild + confirm installed
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select <pkg>"
docker exec airstack-robot-desktop-1 bash -c "ls install/<pkg>/share/<pkg>/launch/"

# Static parse (host): stack files are plain XML
xmllint --noout stacks/<name>/launch/stack.launch.xml

# Dry-run launch to catch errors
docker exec airstack-robot-desktop-1 bash -c "sws && ros2 launch <pkg> <file>.launch.xml --print"

# Verify actual connections
docker exec airstack-robot-desktop-1 bash -c "ros2 node info /<robot_name>/<node>"
docker exec airstack-robot-desktop-1 bash -c "ros2 topic info /<robot_name>/<topic>"

# The observed-truth check: snapshot the running graph and diff against the
# stack's committed wiring.md
airstack test -m wiring --stack <name> --sim isaacsim --num-robots 1

# The static lint (runs in CI as a unit test)
airstack test -m unit -v
```

If `ros2 node info` shows a node subscribing to `/odometry` instead of `/<robot_name>/...`, the stack wiring or `ROBOT_NAME` is wrong.

## References

- **Stacks:** [docs/development/stacks.md](../../../docs/development/stacks.md) — anatomy, wiring.md generation, the AUTONOMY_ROLE removal/migration table
- **Canonical topic names:** [docs/robot/autonomy/integration_checklist.md](../../../docs/robot/autonomy/integration_checklist.md)
- **Reference stack launch files:** `stacks/full_default/launch/stack.launch.xml`, `stacks/full_droan_cpu/`, `stacks/full_macvo/`
- **Lint:** `tests/meta/test_launch_single_locus.py` + `tests/meta/launch_lint_allowlist.txt`
- **ROS 2 docs:** [Launch tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html) · [Launch XML format](https://design.ros2.org/articles/roslaunch_xml.html)
- **Related skills:** [add-ros2-package](../add-ros2-package) · [integrate-module-into-layer](../integrate-module-into-layer) (legacy wrap-form path) · [debug-module](../debug-module) · [test-in-simulation](../test-in-simulation)
