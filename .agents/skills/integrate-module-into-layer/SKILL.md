---
name: integrate-module-into-layer
description: Integrate a ROS 2 module into a stack. Use after creating a package to add it to a running topology. Covers the stack entry launch file, canonical-default topic args (usually zero include args), the single-locus wiring rule, wiring.md regeneration, and the launch lint. The old layer-bringup workflow this skill used to teach is legacy.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Integrate a Module into a Stack

## When to Use

After creating a ROS 2 package (see [add-ros2-package](../add-ros2-package)),
integrate it into a **stack** — the self-contained folder under `stacks/`
whose entry launch file is the single wiring document for a running topology
(RFC #379 §3–4). This replaces the legacy layer-bringup workflow.

> **The layer-bringup workflow is LEGACY.** Editing
> `local_bringup/launch/*.launch.xml` (or any `*_bringup` launch file) to add
> modules is the old monolith pattern. Those files are frozen: the launch
> lint (`tests/meta/test_launch_single_locus.py`) forbids new `<remap>`s
> outside `stacks/*/launch/`, and the grandfather allowlist
> (`tests/meta/launch_lint_allowlist.txt`) only shrinks. Integrate into a
> stack instead.

## The model (read this first)

1. **A module's interface is its launch file.** Every topic endpoint is a
   declared `<arg>` with a `description=` and a **canonical default** from
   the [Interface Conventions Spec](../../../docs/robot/autonomy/interface_conventions.md)
   (e.g. `odometry_topic` defaults to `odometry_conversion/odometry`,
   `global_plan_topic` to `global_plan`). No `<remap>`, no hardcoded
   cross-module topics inside the module. See
   [write-launch-file](../write-launch-file).
2. **A stack composes modules with `<include>` blocks.** Because defaults are
   canonical, a conventional integration is a **bare include — usually zero
   args**. Only deviations from canonical appear as include args, which is
   what keeps the stack file skimmable.
3. **Single-locus rule:** all cross-module wiring overrides live in the
   stack's entry launch file(s) — one file for an unsplit stack, one per host
   role for a split stack. `grep -r global_plan stacks/my_stack/` answers
   "who touches this".
4. **The observed graph is the truth.** Each stack commits a generated
   `wiring.md` (snapshotted from the *running* system); changing the topology
   means regenerating it, so the PR diff shows the change visually.

## Prerequisites

- Module package builds: `docker exec airstack-robot-desktop-1 bash -c "bws --packages-select <pkg>"`
- The module has a launch file with canonical-default topic args
  (template: `.agents/skills/add-ros2-package/assets/package_template/launch/`)
- You know which stack to integrate into (`airstack stack list`); to make a
  new one, follow [create-stack](../create-stack)

## Steps

### 1. Pick the stack and read its entry file

```bash
airstack stack list
cat stacks/my_stack/launch/stack.launch.xml   # or onboard/offboard for splits
```

Every existing block reads "this module, these connections" — a comment
stating the module's inputs/outputs, then the include.

### 2. Add the module include

Canonical wiring — the common case — is a bare include plus an honest
comment:

```xml
<!-- My detector: image in from front_stereo left (canonical), detections
     out at my_detector/detections -->
<include file="$(find-pkg-share my_detector)/launch/my_detector.launch.xml" />
```

Deviation from canonical — pass ONLY the deviating args:

```xml
<!-- My detector, reading the RIGHT camera instead of canonical left -->
<include file="$(find-pkg-share my_detector)/launch/my_detector.launch.xml">
  <arg name="image_topic" value="sensors/front_stereo/right/image_rect" />
</include>
```

If you find yourself passing every arg, fix the module's defaults instead —
they should BE the canonical names.

Placement rules (RFC #380 §2): the trajectory controller, PID controller, and
safety executive are **onboard-only** — in a split stack they belong in the
onboard entry file, never offboard. A module that publishes
`trajectory_override` inherits the whole safety apparatus — that's the
intended integration point for new planners/behaviors.

### 3. Task executors: canonical action name

Action servers are exposed at `tasks/<task_name>` (see
[add-task-executor](../add-task-executor)). The module's launch file should
default its action arg to `tasks/<name>`; the stack passes nothing.

### 4. Split stacks only: does anything new cross the boundary?

If the module's topics must cross the onboard/offboard boundary, add them to
the stack's `bridge.yaml` and regenerate the router config:

```bash
python3 tools/gen_dds_router.py stacks/my_stack/bridge.yaml
```

**Never** list `control_setpoint` or trajectory-group names
(`trajectory_override`, `trajectory_segment_to_add`, `set_trajectory_mode`,
`tracking_point`, `look_ahead`) — `airstack doctor` hard-errors
(RFC #379 §4 / #380 §2). Details: [create-stack](../create-stack).

### 5. Run and verify

```bash
airstack up --stack my_stack --sim isaac --robots 1
airstack ready
docker exec airstack-robot-desktop-1 bash -c "ros2 node list | grep my_detector"
docker exec airstack-robot-desktop-1 bash -c "ros2 topic info /robot_1/my_detector/detections --verbose"
```

Stack launch files are read from the bind mount — edit and re-launch, no
rebuild. (The module package itself still needs `bws` after code changes.)
Debugging: [debug-module](../debug-module).

### 6. Regenerate wiring.md

The committed `wiring.md` no longer matches the graph you just changed — CI's
drift check will (correctly) fail until you regenerate:

```bash
airstack test -m wiring --stack my_stack --sim isaacsim --num-robots 1
# validate tests/results/<run>/wiring/observed_my_stack.md, then copy it:
cp tests/results/<run>/wiring/observed_my_stack.md stacks/my_stack/wiring.md
```

On hardware-only setups use `airstack doctor --snapshot --stack my_stack`
(writes wiring.md with an `unverified-in-CI` provenance line).

### 7. Lint and doctor

```bash
airstack doctor           # anatomy + bridge hard gate + module checks
airstack test -m unit -v  # single-locus lint, layout contract, bridge contract
```

The lint fails on: `<remap>` outside `stacks/*/launch/`, stack `<arg>`s
without `description=`, stale allowlist lines.

### 8. Update docs

Stack README (what changed, known limits) and the module README. For new
interchange points, propose an addition to the
[Interface Conventions Spec](../../../docs/robot/autonomy/interface_conventions.md)
(spec additions are semver-minor; renames are major + RFC — see its
deprecation policy).

## Common Pitfalls

- ❌ Adding the module to a `*_bringup` launch file → ✅ add the include to
  the stack entry file (the lint will catch you).
- ❌ `<remap>` inside the module launch file → ✅ declared topic args with
  canonical defaults; overrides only at the stack level.
- ❌ Passing every topic arg in the stack include → ✅ fix the module's
  defaults to the canonical names; pass only deviations.
- ❌ Forgetting to regenerate `wiring.md` → ✅ step 6; CI drift check fails
  otherwise.
- ❌ Bridging trajectory/control topics in a split stack → ✅ hard-gated;
  bridge `global_plan`/task goals instead.
- ❌ Hardcoded `/robot_1/...` topics → ✅ relative names resolve under the
  `$(env ROBOT_NAME)` namespace pushed by the preamble.

## References

- [create-stack](../create-stack) — making/copying stacks, split stacks, bridge.yaml
- [write-launch-file](../write-launch-file) — module launch authoring rules
- [Interface Conventions Spec](../../../docs/robot/autonomy/interface_conventions.md)
- [Stacks guide](../../../docs/development/stacks.md)
- [Integration Checklist](../../../docs/robot/autonomy/integration_checklist.md)
