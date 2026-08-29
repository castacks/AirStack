# Creating a Custom Stack Topology

Make a custom stack when you need a topology no reference stack provides: a different module mix, an experiment variant, or an onboard/offboard split. This is the how-to; the design rationale, folder anatomy, and single-locus rule are in [AirStack Stacks](stacks.md) — and if this is your first contact with stacks, walk the tutorial rendition first: [Modular AirStack Walkthrough §5](../getting_started/modular_airstack.md#5-make-your-own-stack).

## Step 1 — Copy a reference stack

Never build a stack folder by hand — copy the closest reference:

```bash
airstack stack list                            # see what exists (entry points, wiring.md, compat)
airstack stack new full_default my_experiment  # airstack stack new <source-stack> <new-stack>
```

`full_default` = everything onboard (the baseline); `lite_default` = no global/logging; `lite_offload_global` = the reference split. Names must be lowercase snake_case; `stack new` refuses to overwrite and deliberately does **not** copy `wiring.md` — that file is the *source* stack's observed graph and would lie about yours.

**Verify:** `stacks/my_experiment/` exists with `launch/`, `modules.repos`, `docker-compose.yaml`, `README.md` — and no `wiring.md`.

## Step 2 — Edit the entry launch file

`stacks/my_experiment/launch/stack.launch.xml` is **the** wiring document: a flat list of module `<include>`s. All cross-module remaps and topic-arg overrides live here and nowhere else (the single-locus rule, lint-enforced by `tests/meta/test_launch_single_locus.py`). Pass only deviations from canonical defaults — a real example from `stacks/full_macvo/launch/stack.launch.xml`, where DROAN's disparity source is rewired from stereo to MAC-VO:

```xml
<include file="$(find-pkg-share droan_gl)/launch/droan_gl.launch.xml">
  <arg name="droan_gl_disparity_topic"
    value="/$(env ROBOT_NAME)/perception/macvo/disparity" />
</include>
```

Rules the unit lints enforce: every declared `<arg>` needs a `description=`; never include `robot.launch.xml` (it is the dispatcher that includes *you* — infinite recursion); remaps are legal only under `stacks/*/launch/`. Update `README.md` too — the layout contract rejects READMEs under 200 characters. Canonical topic names/types are in the [Interface Conventions Spec](../robot/autonomy/interface_conventions.md); authoring details in the [create-stack skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/create-stack/SKILL.md).

**Verify:** `airstack test -m unit -v` passes the layout contract and single-locus lint.

## Step 3 — Declare module pins if the stack needs external modules

If your topology includes packages that don't live in trunk, pin them in the stack's `modules.repos` — vcstool format, tags/SHAs only (never branches), plus a top-level `airstack_compat:` key declaring the trunk semver range you tested against (see `stacks/full_default/modules.repos` for the annotated shape; its `repositories: {}` shows a trunk-only stack). `airstack module add <url> --version <tag>` pins and syncs a module at the checkout level; a stack commits its own pins. If the capability doesn't exist yet, scaffold it as a module rather than growing trunk — `airstack module create --in-tree <name>` scaffolds the boundary at `robot/ros_ws/src/modules/<name>/`; see [AirStack Modules](modules.md) and the [create-module skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/create-module/SKILL.md).

**Verify:** `airstack module doctor` reports valid manifests and overlay integrity.

## Step 4 — Launch it

```bash
airstack up --stack my_experiment --sim isaac --robots 1
airstack ready
```

`--stack <name>` exports `AIRSTACK_STACK_DIR`/`AIRSTACK_STACK_ENTRY`; the `--stack <name>:<entry>` form selects an alternate entry file (split stacks). Stack launch files are bind-mounted into the containers — edit and re-launch, no `colcon build` needed.

**Verify:** `airstack ready` reports flight-ready, and `docker exec airstack-robot-desktop-1 bash -c "ros2 node list"` shows your topology's nodes.

## Step 5 — Bootstrap and commit wiring.md

`wiring.md` is generated from the *running* graph — never hand-edited:

```bash
airstack test -m wiring --stack my_experiment --sim isaacsim --num-robots 1
```

With no committed `wiring.md`, the test PASSES and logs an INSTRUCTION pointing at the observed snapshot (`tests/results/<run>/wiring/observed_my_experiment.md`). Validate it, copy it to `stacks/my_experiment/wiring.md`, commit. From then on CI drift-checks the running graph against it, so any PR that changes wiring must regenerate the file. Needs a GPU + sim license; hardware-only stacks use `airstack doctor --snapshot --stack my_experiment` instead, which writes `wiring.md` with an `unverified-in-CI` provenance line. Details: [wiring.md generation and drift-checking](stacks.md#wiringmd-generation-and-drift-checking).

**Verify:** re-running the wiring test against the committed file passes with no drift.

## Step 6 — Validate

```bash
airstack doctor                               # anatomy, module manifests, overlay, both hard gates
airstack doctor --live --stack my_experiment  # RUNNING graph vs wiring.md — exit 1 on drift
airstack test -m unit -v                      # layout contract + single-locus lint + bridge contract
airstack stack diff full_default my_experiment  # topology delta vs the stack you copied
```

`stack diff` compares the two stacks' generated wiring graphs (nodes/edges/topics/QoS), never XML formatting noise — a quick sanity check that your intended change is the *only* change.

**Verify:** doctor exits 0, `--live` prints "graph matches wiring.md", and the diff shows exactly your intended delta.

## Step 7 — Split topologies (onboard/offboard)

A split is a stack *shape*: one entry file per host role plus a `bridge.yaml` listing every topic/service/action that crosses the machine boundary. Don't design one from scratch — copy the reference (`airstack stack new lite_offload_global my_split`), edit `bridge.yaml` first, regenerate the router config with `python3 tools/gen_dds_router.py stacks/my_split/bridge.yaml`, and run each half with `--stack my_split:onboard` / `:offboard`. Mind the hard gate: `control_setpoint` and the `trajectory_controller/*` group must never appear in a `bridge.yaml` — the controller and safety executive are onboard-only. The full recipe, including the bridge entry shape and fleet-driven placement, is in [Split stacks and bridge.yaml](stacks.md#split-stacks-and-bridgeyaml) and the [create-stack skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/create-stack/SKILL.md).

**Verify:** `airstack doctor` passes the bridge hard gate, and the layout contract accepts the multi-entry folder (≥2 entry points ⇒ `bridge.yaml` present).
