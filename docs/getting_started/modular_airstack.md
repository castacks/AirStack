# Modular AirStack Walkthrough

The end-to-end journey through Modular AirStack for a
developer new to the project: fly a **reference stack**, read its **wiring**,
pull in a **module**, make a **stack of your own**, scale to a **fleet**, and
let **doctor** check your work. Every command below is real and current.

Prerequisites: the base [Getting Started](index.md) setup (clone, install,
images pulled or built).

## 1. Clone and set up

```bash
git clone --recursive -j8 git@github.com:castacks/AirStack.git
cd AirStack
./airstack.sh install   # docker, compose, NVIDIA Container Toolkit
./airstack.sh setup     # enables the `airstack` command
source ~/.bashrc        # or ~/.zshrc
airstack images pull     # or: airstack images build
```

## 2. Fly a reference stack

A **stack** is a self-contained folder under [`stacks/`](../development/stacks.md)
— pinned module list, plain ROS 2 launch entry points, compose file, README.
Trunk ships five reference stacks; start from the baseline:

```bash
airstack up --stack full_default --sim isaac
airstack ready    # waits: containers → sim /clock → nodes → PX4 ready
```

`--stack full_default` launches
[`stacks/full_default/launch/stack.launch.xml`](../../stacks/full_default/README.md)
— a flat list of module includes where every connection is written down.
(Stacks are the only dispatch: with no `--stack`, `full_default` launches
anyway; this makes the choice explicit.) Command the drone from
Foxglove exactly as in [Getting Started](index.md#move-robot).

## 3. Read the wiring — the map of the system

Each stack commits a
[`wiring.md`](../../stacks/full_default/wiring.md) snapshotted **from the
running graph in CI** (nodes grouped by module, edges labeled
topic/type/QoS). It cannot lie or rot: CI fails when the running system
drifts from the committed diagram. When you wonder "who publishes this
topic?", the answer is two greps away:

```bash
grep -r global_plan stacks/full_default/   # every connection is in the source
```

Read `stacks/<name>/wiring.md` first whenever you meet a new stack — it is
the system diagram, observed rather than drawn.

## 4. Add a module

**Modules** are thin external repos, discovered in the
[Module & Stack Catalog](../modules/index.md) and pulled on demand — pinned
to a tag or SHA, never a branch. Example: the
[dfm2_disturbances](../modules/dfm2_disturbances.md) Isaac Sim disturbance
library (fans, vents, strobes, lens flare):

```bash
airstack module add https://github.com/castacks/asm_dfm2_disturbances \
  --version af3daa783248b07b82165833d419d322ab3137fe
airstack module list     # name, type, pin, targets, valid?
```

`module add` records the pin in `modules.repos`, syncs the repo into the
gitignored `modules/` dir, validates its `module.yaml`, places overlay
symlinks, and regenerates a compose override that mounts the module into the
right containers. Then bring the stack up with the module mounts, selecting
one of the module's Isaac scene scripts:

```bash
ISAAC_SIM_SCRIPT_NAME=modules/dfm2_disturbances/one_px4_pegasus_fan_force_field.py \
  airstack up --stack full_default --sim isaac \
  -f .airstack/generated/docker-compose.modules.yaml
```

Full CLI reference (sync, remove, hooks, the pinning rule):
[AirStack Modules](../development/modules.md).

## 5. Make your own stack

Never edit a reference stack — copy one and rewire it:

```bash
airstack stack new full_default my_stack
```

1. Edit `stacks/my_stack/launch/stack.launch.xml`: swap, add, or remove
   module `<include>`s and their topic args. All cross-module wiring lives in
   this one file (the single-locus rule), so the file *is* your topology.
2. Pin any external modules in `stacks/my_stack/modules.repos`.
3. Run it: `airstack up --stack my_stack --sim isaac`.
4. Snapshot its wiring: `airstack test -m wiring --stack my_stack`, validate
   the observed file, commit it as `stacks/my_stack/wiring.md`.

Guide: [AirStack Stacks](../development/stacks.md); agent workflow: the
[create-stack skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/create-stack/SKILL.md).

## 6. Scale to a fleet

A **fleet file** (`config/fleets/*.yaml`) declares a whole deployment: which
robots exist, which vehicle each flies, which stack each runs, and which
ground hosts run split-stack offboard halves. The reference heterogeneous
fleet flies three quads with three different brains:

```bash
airstack up --fleet sim_three_mixed --sim isaac
airstack ready
```

`robot_1` runs `full_default`, `robot_2` runs `lite_default`, and `robot_3`
runs the split `lite_offload_global` stack with its global layer placed on
the GCS host. Guide: [AirStack Fleets](../development/fleets.md).

## 7. Let doctor check your work

`airstack doctor` observes and reports — it never generates or edits wiring:

```bash
airstack doctor          # manifests, overlay, dep conflicts, stack anatomy,
                         # bridge safety placement (the two hard gates)
airstack doctor --live   # diff the RUNNING graph against the stack's wiring.md
airstack module doctor --drift   # fork research: module-contained vs trunk edits
```

Use `doctor` after every `module add`, stack edit, or fleet change; use
`doctor --live` when a running system misbehaves.

## Where to go next

- [Module & Stack Catalog](../modules/index.md) — what's registered today
- [AirStack Modules](../development/modules.md) ·
  [Stacks](../development/stacks.md) ·
  [Fleets](../development/fleets.md) ·
  [Module CI](../development/module_ci.md)
- [Interface Conventions Spec](../robot/autonomy/interface_conventions.md) —
  the canonical topic names/types/QoS that make bare includes "just wire"
