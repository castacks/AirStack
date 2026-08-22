# AirStack Stacks

A **stack** is a self-contained folder that defines a complete robot topology:
which modules run and how they are wired together. Stacks exist so wiring has
a **single locus** — one place to read, diff, and pin an entire topology —
instead of connections scattered across per-package launch files. Trunk ships
a small set of **reference stacks** under `stacks/`; custom stacks live with
their owners (a lab keeps a private stack repo wiring together public modules
— no fork of AirStack needed).

## Stack folder anatomy

Plain files, no schema beyond a required layout:

| File | Purpose |
|------|---------|
| `modules.repos` | vcstool format, **pinned** to tags/commits — never branches. A stack with a pinned `.repos` *is* a localized release set. Carries a top-level `airstack_compat:` key (sibling of `repositories:`; vcstool ignores it) declaring the trunk semver range the stack was tested against. |
| `launch/stack.launch.xml` | **THE wiring document**: a flat list of module `<include>`s. All cross-module remaps and topic-arg overrides live here — nowhere else (the single-locus rule). Unsplit stacks have exactly this one entry point; [split stacks](#split-stacks-and-bridgeyaml) carry one entry file per host role plus `bridge.yaml`. |
| `docker-compose.yaml` | Composes this stack's images from [module layers](modules.md#docker-layer-composition). A documented stub until the stack pins modules. |
| `wiring.md` | **Generated** from the *running* graph by the wiring-snapshot test — never hand-edited. Drift-checked in CI. |
| `README.md` | What this stack is for, how to run it, its known limits. |

Anatomy is enforced by a unit test: `tests/meta/test_stack_layout_contract.py`
(`wiring.md` is optional until the first snapshot run commits it).

## Reference stacks in trunk

| Stack | Topology |
|-------|----------|
| [`full_default`](https://github.com/castacks/AirStack/tree/develop/stacks/full_default) | The full-autonomy topology (GPU `droan_gl` planner) — the baseline, and what launches when no stack is selected. |
| [`full_droan_cpu`](https://github.com/castacks/AirStack/tree/develop/stacks/full_droan_cpu) | CPU DROAN planner + live `disparity_expansion` (for machines without the GPU planner). |
| [`full_macvo`](https://github.com/castacks/AirStack/tree/develop/stacks/full_macvo) | MAC-VO as the planner's disparity source. Requires the `asm_macvo` module (`airstack module add asm_macvo`). |
| [`lite_default`](https://github.com/castacks/AirStack/tree/develop/stacks/lite_default) | Onboard-lite topology, unsplit: interface, sensors, perception, flat Local layer, behavior; **no global, no logging**. |
| [`lite_offload_global`](https://github.com/castacks/AirStack/tree/develop/stacks/lite_offload_global) | A **split stack**: `onboard.launch.xml` (= lite topology) + `offboard.launch.xml` (global layer only) + `bridge.yaml`. |

## Flat includes, and the deliberate exceptions

Every reference stack composes its graph as **flat module-launch includes** —
one `<include>` per module, so the entry file reads as the topology. Two
blocks are wrapped **by design**: `interface.launch.py` (the safety boundary,
kept whole until the planned platform-module extraction) and the
`interpolate_dds_router` / gossip helpers (their wiring lives in YAML
configs). The lint allowlist (below) covers exactly that deliberate remainder.

The stack's `wiring.md` — snapshotted from the running system — is the
observed truth of the graph.

## Running a stack

```bash
airstack up --stack full_default --sim isaac --robots 1
airstack ready
```

Mechanics: `--stack <name>` validates `stacks/<name>/launch/stack.launch.xml`
exists, then exports `AIRSTACK_STACK_DIR=/root/AirStack/stacks/<name>` (the
*container* path — `stacks/` is bind-mounted into every robot container) and
`AIRSTACK_STACK_ENTRY=stack`. Inside the container,
`autonomy_bringup/launch/robot.launch.xml` runs the shared preamble
(ROBOT_NAME namespace, `use_sim_time`, `robot_state_publisher`, world→map TF),
then includes the stack entry file. With no stack selected anywhere
(`--stack` / env / `--env-file` / `.env`), the trunk reference stack
`full_default` launches — `AIRSTACK_STACK_DIR` is always set in the
effective config.

`--stack <name>:<entry>` selects an alternate entry file
(`launch/<entry>.launch.xml`) — reserved for
[split stacks](#split-stacks-and-bridgeyaml).

Stack launch files need no `colcon build` — they are read from the bind mount;
edit and re-launch.

### Why stacks don't launch standalone

It is tempting to `ros2 launch` a stack entry file directly and delete the
dispatcher. Three reasons the thin `robot.launch.xml` earns its ~50 lines:

1. **Namespace scoping is mechanical, not stylistic.** `push_ros_namespace`
   only scopes what sits *inside* its enclosing scope, so an included
   "preamble" file cannot namespace the sibling includes that follow it. The
   dispatcher pushes `/$ROBOT_NAME` and includes the stack entry *within*
   that scope — the one arrangement where every stack node lands namespaced
   without each stack author hand-rolling (and occasionally fumbling) a
   wrapper group. A stack that wraps the dispatcher instead recurses
   infinitely; the layout contract test enforces the direction.
2. **Stack files stay pure wiring documents.** The preamble — `use_sim_time`,
   `robot_state_publisher`/URDF plumbing, the world→map TF — is *platform*
   infrastructure, not topology. Keeping it out of stack entries preserves
   the "entry file *is* the wiring diagram" property, and keeps
   vehicle-driven URDF generation (planned) a one-file change instead of
   an every-stack (and every external stack repo) migration.
3. **It is the seed of the platform module.** Planned work extracts
   "interface + controller + safety + preamble" as the `px4_multirotor`
   platform module; this dispatcher is precisely the file that becomes that
   platform's bringup. `autonomy_bringup` *thins* over time — it does not
   disappear.

Practically it is also the single point where `AIRSTACK_STACK_DIR`/`_ENTRY`
resolution happens, so compose, the fleet resolver, and the CLI converge on
one contract.

## wiring.md: generation and drift-checking

The wiring-snapshot system test brings the stack up in sim, waits for the node
graph to settle, captures it (`ros2 node list` + `ros2 topic info --verbose`),
and renders a mermaid dataflow document:

```bash
airstack test -m wiring --stack full_default --sim isaacsim --num-robots 1
```

- **No `stacks/<name>/wiring.md` committed yet (bootstrap):** the test PASSES
  and logs an INSTRUCTION pointing at the observed snapshot
  (`tests/results/<run>/wiring/observed_<name>.md`). Validate it, copy it to
  `stacks/<name>/wiring.md`, commit.
- **Committed:** the test fails on any drift between the committed diagram and
  the observed graph — a PR that changes wiring must regenerate `wiring.md`,
  so the review diff shows the topology change.

Runs without `--stack` launch the default dispatch — `full_default` — and
drift-check against its `stacks/full_default/wiring.md` (there is no separate
goldens tree; each stack folder owns its baseline).

## The single-locus rule (and its lint)

All cross-module remaps and topic-arg overrides live in the stack's entry
launch file(s). Module launch files declare topic args (canonical defaults per
[the integration checklist](../robot/autonomy/integration_checklist.md), a
`description=` on every arg) but **never** `<remap>` and never hardcode
cross-module topics. The stack file *is* the wiring diagram; `grep -r
global_plan stacks/my_stack/` answers "who touches this".

Enforced by `tests/meta/test_launch_single_locus.py` (`unit` mark, runs in CI):

1. No `<remap>`/`remappings=` outside `stacks/*/launch/` — except files frozen
   in `tests/meta/launch_lint_allowlist.txt` (down to the deliberate remainder:
   a standalone utility, a vendored driver, one module launch awaiting its
   canonical rewrite, and the interface safety boundary).
2. The allowlist only shrinks: an entry whose file carries no remap
   fails the lint until its line is deleted.
3. Stack launch files must describe every `<arg>` they declare.

See the [write-launch-file skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/write-launch-file/SKILL.md)
for the authoring workflow.

## Making a new stack today

1. Copy a reference stack: `airstack stack new full_default my_stack`
   (refuses to overwrite; deliberately does **not** copy `wiring.md` — that
   is the source stack's observed graph).
2. Edit `launch/stack.launch.xml` — swap/add/remove module `<include>`s and
   their topic args. Update `README.md`.
3. Run it: `airstack up --stack my_stack --sim isaac`.
4. Snapshot the wiring: `airstack test -m wiring --stack my_stack`, validate
   the observed file, commit it as `stacks/my_stack/wiring.md`.
5. `airstack doctor` and `airstack test -m unit` — the layout contract, the
   bridge gate, and the launch lint must pass.

The full workflow (including split stacks) is the
[create-stack skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/create-stack/SKILL.md).

## Split stacks and `bridge.yaml`

**A split is a stack shape, not special machinery.** A split stack folder
carries **multiple launch entry points** — one per host role — plus a
`bridge.yaml` explicitly listing every topic/service/action that crosses the
machine boundary (name, type, direction, QoS). The bridge list feeds
DDS-router config generation and *is* the split, readable in source —
explicit beats derived. The reference split is
[`lite_offload_global`](https://github.com/castacks/AirStack/tree/develop/stacks/lite_offload_global):

```text
stacks/lite_offload_global/
├── launch/onboard.launch.xml    # the vehicle: lite topology + the DDS router
├── launch/offboard.launch.xml   # the ground host: global layer only
├── bridge.yaml                  # THE boundary document
└── (modules.repos, docker-compose.yaml, README.md, wiring.md as usual)
```

- **Run each half** with the `NAME:ENTRY` form:
  `airstack up --stack lite_offload_global:onboard` on the vehicle,
  `... :offboard` on the ground host. A host's role is simply "which entry
  point does this host run"; a third machine is just another entry file +
  bridge section.
- **Or declare the placement in a fleet**: a fleet entry with
  `stack: stacks/lite_offload_global` and `hosts: {offboard: gcs}` derives
  both halves — the robot's service gets the `onboard` entry, the named
  ground host's service gets the same stack with
  `AIRSTACK_STACK_ENTRY=offboard` (`airstack fleet generate`; the reference
  is `config/fleets/sim_three_mixed.yaml`, robot_3). See
  [AirStack Fleets](fleets.md).
- **Generate the router config** from the bridge list (never hand-edit the
  output):

  ```bash
  python3 tools/gen_dds_router.py stacks/lite_offload_global/bridge.yaml
  # -> .airstack/generated/dds_router.<stack>.yaml (deterministic)
  ```

- **Anatomy rule:** a stack with two or more entry points **must** carry a
  `bridge.yaml` (`tests/meta/test_stack_layout_contract.py`).
- **The placement hard gate:** `control_setpoint` and trajectory-group names
  (`trajectory_override`, `trajectory_segment_to_add`, `set_trajectory_mode`,
  `tracking_point`, `look_ahead` — the `trajectory_controller/*` group) must
  never appear in any `bridge.yaml`. The controller and safety executive are
  **onboard-only** (link loss must leave the vehicle able to failsafe):
  `global_plan` crosses, trajectory commands don't. `gen_dds_router.py
  --check` — run by `airstack doctor` and by
  `tests/meta/test_bridge_contract.py` — exits 1 naming any violation.
  This is one of doctor's **two** enumerated hard gates; the gate list
  grows only by deliberate project-wide design decision, never ad hoc.
- **One `wiring.md` per stack, split or not**: the snapshot groups nodes by
  host and draws bridge edges as boundary crossings.

Canonical names/types/QoS for everything crossing (or forbidden from
crossing) are in the
[Interface Conventions Spec](../robot/autonomy/interface_conventions.md).

## The `airstack stack` CLI

```bash
airstack stack list             # name, entry points (+bridge), wiring.md?, airstack_compat
airstack stack new <src> <dst>  # copy a reference stack (no wiring.md; refuses overwrite)
airstack stack diff <a> <b>     # topology diff of two stacks' generated wiring.md
```

`stack diff` (backed by `tools/stack_diff.py`) compares the **wiring-graph
trailers** of the two stacks' `wiring.md` files via the same
`tests/wiring_snapshot.py` machinery CI uses: nodes/edges/topics added or
removed, QoS and type mismatches — topology differences, never XML
formatting noise. That, plus shared includable sub-launches, is how
copy-drift between stack folders stays bounded.

## `airstack doctor`

Observe-and-report health checks. Doctor never generates or
edits wiring; in default mode it exits non-zero only on the **two enumerated
hard gates**:

```bash
airstack doctor                 # compose-time battery:
                                #  1. module manifests valid          (report)
                                #  2. overlay integrity               (report)
                                #  3. HARD GATE: module dep conflicts
                                #  4. stack anatomy incl. split=>bridge.yaml (report)
                                #  5. HARD GATE: control/trajectory names in a
                                #     bridge.yaml

airstack doctor --live --stack <name>
                                # capture the RUNNING graph (docker exec per
                                # robot, same capture as the wiring test) and
                                # diff it against stacks/<name>/wiring.md —
                                # exit 1 on drift, 'graph matches wiring.md'
                                # when identical. Also flags publishers of
                                # control-setpoint / controller-output topics
                                # outside the blessed controller chain
                                # (loud WARN; fatal only with --strict).

airstack doctor --snapshot --stack <name>
                                # the identical capture, WRITTEN as the
                                # stack's wiring.md with a provenance line:
                                # 'observed on <host>, <date>, <sha> —
                                # unverified-in-CI'. The hardware path for
                                # stacks that cannot run in CI: an observed
                                # artifact still can't lie, it is just
                                # refreshed by hand instead of by CI.
```

`--stack` is inferred from `AIRSTACK_STACK_DIR` when omitted (exported by
`airstack up` with the stack flag). `airstack module doctor` remains the
module-scoped subset (manifests + overlay + `--drift`).

## `AUTONOMY_ROLE` is not a launch input

Stacks are the only launch dispatch — there is no role env var. `airstack up`
hard-errors at preflight when it sees an explicitly set `AUTONOMY_ROLE`
(env / `.env` / `--env-file`), naming this page: a role variable would
silently compete with the stack selection, so it is rejected rather than
ignored. Select the topology with `--stack`:

| You want | Command |
|----------|---------|
| The full-autonomy baseline | Nothing — `full_default` launches by default (or be explicit: `--stack full_default`) |
| CPU DROAN topology | `--stack full_droan_cpu` |
| MAC-VO disparity topology | `--stack full_macvo` (requires the `asm_macvo` module) |
| Onboard-lite, unsplit | `--stack lite_default` |
| Split onboard/offboard | `--stack lite_offload_global:onboard` / `:offboard` (+ generate the router config from `bridge.yaml`) |
