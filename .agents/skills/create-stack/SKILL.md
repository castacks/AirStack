---
name: create-stack
description: Create a new AirStack stack folder — copy a reference stack with `airstack stack new`, edit the entry launch file(s), bootstrap wiring.md, and validate with doctor and the unit lints. Covers stack anatomy, split stacks (multiple entry points + bridge.yaml), the control/trajectory placement hard gate, and gen_dds_router.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Create a Stack

## When to Use

When you need a topology different from an existing stack: a different module
mix, an experiment variant, an onboard/offboard split, or an unconventional
graph (an end-to-end planner is the same case, not a special one — RFC #379
§3). A **stack** is a self-contained folder; making one never touches
`autonomy_bringup` or any `*_bringup` package.

## Stack anatomy (the contract)

```text
stacks/<name>/
├── modules.repos        # pinned module list (vcstool) + top-level airstack_compat
├── launch/              # entry point(s): stack.launch.xml for unsplit;
│   └── ...              #   one <role>.launch.xml per host for split stacks
├── docker-compose.yaml  # per-stack image composition (stub until module pins)
├── wiring.md            # GENERATED from the running graph — never hand-edited
├── README.md            # purpose, how to run, known limits
└── bridge.yaml          # SPLIT STACKS ONLY — every crossing topic/service/action
```

Enforced by `tests/meta/test_stack_layout_contract.py` (unit mark): the four
files always; ≥2 entry points ⇒ `bridge.yaml` required; `wiring.md` optional
only until the first snapshot lands, and it must carry the machine-readable
trailer when present.

## Steps — unsplit stack

### 1. Copy a reference

```bash
airstack stack list                            # see what exists
airstack stack new full_default my_experiment  # refuses overwrite
```

Pick the closest starting point: `full_default` (everything onboard),
`lite_default` (no global/logging), `full_droan_cpu`, `full_macvo`.
`stack new` deliberately does **not** copy `wiring.md` — that file is the
*source* stack's observed graph and would lie about yours.

### 2. Edit the entry file

`stacks/my_experiment/launch/stack.launch.xml` is the wiring document: a flat
list of module `<include>`s, each with a comment reading "this module, these
connections". Swap/add/remove includes; pass ONLY deviations from canonical
defaults (see [integrate-module-into-layer](../integrate-module-into-layer)
and the [Interface Conventions Spec](../../../docs/robot/autonomy/interface_conventions.md)).

Rules the unit lint enforces (`test_launch_single_locus.py`,
`test_stack_layout_contract.py`):

- XML only at the stack level; every declared `<arg>` has a `description=`.
- Never include `robot.launch.xml` (it's the dispatcher that includes YOU —
  infinite recursion).
- Remaps are allowed *only* here (`stacks/*/launch/`).

External modules: pin them in `modules.repos` (`airstack module add <url>
--version <tag>` writes the checkout-level file; a stack commits its own
pins) and update `airstack_compat`.

### 3. Update README.md

Purpose, what it launches, how to run, known limits. The layout contract
rejects trivial READMEs (<200 chars).

### 4. Run it

```bash
airstack up --stack my_experiment --sim isaac --robots 1
airstack ready
```

Stack launch files are bind-mounted — edit and re-launch without rebuilding.

### 5. Bootstrap wiring.md

```bash
airstack test -m wiring --stack my_experiment --sim isaacsim --num-robots 1
```

With no committed `wiring.md` the test PASSES and logs an INSTRUCTION:
validate `tests/results/<run>/wiring/observed_my_experiment.md`, copy it to
`stacks/my_experiment/wiring.md`, commit. From then on CI drift-checks the
running graph against it. **Needs a GPU + sim license** (the ephemeral CI
runner, or a workstation). Hardware-only stacks instead run
`airstack doctor --snapshot --stack my_experiment`, which writes `wiring.md`
with a provenance line (`observed on <host>, <date>, <sha> —
unverified-in-CI`).

### 6. Validate

```bash
airstack doctor           # anatomy, module manifests, overlay, both hard gates
airstack test -m unit -v  # layout contract + single-locus lint + bridge contract
airstack stack diff full_default my_experiment   # see the topology delta
```

## Steps — split stack (RFC #380 §2)

A split is a stack **shape**: one entry file per host role + `bridge.yaml`.
Reference: `stacks/lite_offload_global/` (onboard lite vehicle, offboard
global planning).

### 1. Copy the reference split

```bash
airstack stack new lite_offload_global my_split
```

### 2. Decide the boundary — edit bridge.yaml FIRST

`bridge.yaml` is the authoritative, reviewable list of everything crossing
the machine boundary. Entry shape:

```yaml
bridge:
  - topic: global_plan                 # relative name (no /robot_1/, no $(env))
    type: nav_msgs/msg/Path
    direction: offboard_to_onboard     # or onboard_to_offboard
    qos: reliable                      # or best_effort (topics only)
  - service: interface/robot_command
    type: airstack_msgs/srv/RobotCommand
    direction: offboard_to_onboard
  - action: tasks/navigate
    type: task_msgs/action/NavigateTask
    direction: offboard_to_onboard
```

**THE HARD GATE (memorize this):** `control_setpoint` and trajectory-group
names — `trajectory_override`, `trajectory_segment_to_add`,
`set_trajectory_mode`, `tracking_point`, `look_ahead`, anything under
`trajectory_controller/*`, and `interface/cmd_*` — must NEVER appear in a
`bridge.yaml`. The controller and safety executive are onboard-only: link
loss must leave the vehicle able to failsafe. **`global_plan` crosses;
trajectory commands don't.** This is one of `doctor`'s two enumerated hard
gates (RFC #379 §4 / RFC #380 §2) — `gen_dds_router.py --check` exits 1
naming the offender; don't fight it, redesign the split (offload planning,
not control).

### 3. Generate the router config

```bash
python3 tools/gen_dds_router.py stacks/my_split/bridge.yaml
# validates + writes .airstack/generated/dds_router.my_split.yaml (deterministic)
python3 tools/gen_dds_router.py stacks/my_split/bridge.yaml --check   # gate only
```

The onboard entry file loads the generated config through
`interpolate_dds_router.launch.py` (args `dds_router_config_file` /
`dds_router_args`) — update the default path if you renamed the stack
(`dds_router.<stack>.yaml` matches the `stack:` key in bridge.yaml).

### 4. Edit the per-role entry files

- `launch/onboard.launch.xml` — everything the vehicle runs. The trajectory
  controller, PID controller, and safety executive stay HERE, always.
- `launch/offboard.launch.xml` — what the ground host runs.
- A third machine = a third entry file + its bridge sections; run each half
  with `airstack up --stack my_split:onboard` / `:offboard`.

### 5. Validate

Same as unsplit, plus:

```bash
airstack doctor                          # bridge hard gate runs on every bridge.yaml
airstack doctor --live --stack my_split  # running graph vs wiring.md + safety-floor scan
```

One `wiring.md` per stack, split or not — the snapshot run brings up every
entry point; nodes group by host, bridge edges render as boundary crossings.

## Common Pitfalls

- ❌ Hand-editing `wiring.md` → ✅ it is generated; regenerate via the wiring
  test or `doctor --snapshot`.
- ❌ Keeping the copied stack's `wiring.md` (stale baseline) → ✅ `stack new`
  already drops it; bootstrap your own.
- ❌ Two entry points, no `bridge.yaml` → ✅ layout contract fails; declare
  the boundary.
- ❌ Bridging `set_trajectory_mode` "because the GCS needs it" (the legacy
  allowlist did) → ✅ hard-gated; mode changes belong to onboard task servers.
- ❌ Absolute names or `$(env ...)` inside `bridge.yaml` → ✅ relative names;
  the generator adds the namespace tokens.
- ❌ Branch refs in `modules.repos` → ✅ pins only (tags/SHAs).

## References

- [Stacks guide](../../../docs/development/stacks.md) — anatomy, split stacks, doctor, CLI
- [integrate-module-into-layer](../integrate-module-into-layer) — adding modules to a stack
- [Interface Conventions Spec](../../../docs/robot/autonomy/interface_conventions.md)
- [create-module](../create-module) — module-side scaffolding
- `stacks/lite_offload_global/bridge.yaml` — the annotated reference bridge
