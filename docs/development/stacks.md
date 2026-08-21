# AirStack Stacks

A **stack** is a self-contained folder that defines a complete robot topology:
which modules run and how they are wired together ([RFC #379 §3](https://github.com/castacks/AirStack/discussions/379)).
Trunk ships a small set of **reference stacks** under `stacks/`; custom stacks
live with their owners (a lab keeps a private stack repo wiring together
public modules — no fork of AirStack needed).

## Stack folder anatomy

Plain files, no schema beyond a required layout (from [RFC #385 §1](https://github.com/castacks/AirStack/discussions/385)):

| File | Purpose |
|------|---------|
| `modules.repos` | vcstool format, **pinned** to tags/commits — never branches. A stack with a pinned `.repos` *is* a localized release set. Carries a top-level `airstack_compat:` key (sibling of `repositories:`; vcstool ignores it) declaring the trunk semver range the stack was tested against. |
| `launch/stack.launch.xml` | **THE wiring document**: a flat list of module `<include>`s. All cross-module remaps and topic-arg overrides live here — nowhere else (the single-locus rule). Unsplit stacks have exactly this one entry point; split stacks (RFC #380 §2) carry one entry file per host role plus `bridge.yaml`. |
| `docker-compose.yaml` | Composes this stack's images from module layers (RFC #379 §6). A documented stub until the stack pins modules. |
| `wiring.md` | **Generated** from the *running* graph by the wiring-snapshot test — never hand-edited. Drift-checked in CI. |
| `README.md` | What this stack is for, how to run it, its known limits. |

Anatomy is enforced by a unit test: `tests/meta/test_stack_layout_contract.py`
(`wiring.md` is optional until the first snapshot run commits it).

## Reference stacks in trunk

| Stack | Topology |
|-------|----------|
| [`full_default`](https://github.com/castacks/AirStack/tree/develop/stacks/full_default) | The current full-autonomy topology (GPU `droan_gl` planner) — baseline; machine-proven graph-identical to the removed legacy `AUTONOMY_ROLE=full` dispatch, and what launches when no stack is selected. |
| [`full_droan_cpu`](https://github.com/castacks/AirStack/tree/develop/stacks/full_droan_cpu) | CPU DROAN planner + live `disparity_expansion` — absorbs `local_droan_cpu.launch.xml`. |
| [`full_macvo`](https://github.com/castacks/AirStack/tree/develop/stacks/full_macvo) | MAC-VO as the planner's disparity source — supersedes (and fixes) the broken `local_macvo_obstacle_avoidance.launch.xml` variant. Requires the `asm_macvo` module (`airstack module add asm_macvo`). |
| [`lite_default`](https://github.com/castacks/AirStack/tree/develop/stacks/lite_default) | Onboard-lite topology, unsplit — the equivalent of the removed `AUTONOMY_ROLE=onboard` role: interface, sensors, perception, flat Local layer, behavior; **no global, no logging**. |
| [`lite_offload_global`](https://github.com/castacks/AirStack/tree/develop/stacks/lite_offload_global) | The first **split stack** (RFC #380 §2): `onboard.launch.xml` (= lite topology) + `offboard.launch.xml` (global layer only) + `bridge.yaml`. Replaced the removed `onboard`/`offboard` role pair. |

## Wrap vs. flatten — current status

The wrap→flatten migration is COMPLETE: every reference stack composes its
graph as flat module-launch includes, the legacy layer bringup launch files
(`local/perception/sensors/global/behavior *.launch.xml`) are deleted, and
the AUTONOMY_ROLE dispatch is gone from `autonomy_bringup` — stacks are the
only dispatch. Two blocks remain wrapped **by design**: `interface.launch.py`
(the safety boundary, until RFC #380 Part 2) and the
`interpolate_dds_router` / gossip helpers (their wiring lives in YAML
configs). The lint allowlist (below) is down to that deliberate remainder.

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
(`launch/<entry>.launch.xml`) — reserved for split stacks (RFC #380 §2).

Stack launch files need no `colcon build` — they are read from the bind mount;
edit and re-launch.

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
2. The allowlist only shrinks: an entry whose file no longer carries a remap
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

## Split stacks and `bridge.yaml` (RFC #380 §2)

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
  `... :offboard` on the ground host. The old coarse `AUTONOMY_ROLE`
  trichotomy (removed) became "which entry point does this host run"; a
  third machine is just another entry file + bridge section.
- **Or declare the placement in a fleet** (RFC #380 §2): a fleet entry with
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
  `tests/meta/test_bridge_contract.py` — exits 1 naming any violation
  (RFC #379 §4 / RFC #380 §2). This is one of doctor's **two** enumerated
  hard gates; growing that list takes the RFC process.
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
copy-drift between stack folders stays bounded (RFC #379 §3).

## `airstack doctor`

Observe-and-report health checks (RFC #379 §4). Doctor never generates or
edits wiring; in default mode it exits non-zero only on the **two enumerated
hard gates**:

```bash
airstack doctor                 # compose-time battery:
                                #  1. module manifests valid          (report)
                                #  2. overlay integrity               (report)
                                #  3. HARD GATE: module dep conflicts (RFC #379 §6)
                                #  4. stack anatomy incl. split=>bridge.yaml (report)
                                #  5. HARD GATE: control/trajectory names in a
                                #     bridge.yaml                     (RFC #380 §2)

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

## AUTONOMY_ROLE: removed

`AUTONOMY_ROLE` was **removed on this branch** — stacks are the only launch
dispatch. `airstack up` hard-errors (preflight) when it sees an explicitly
set `AUTONOMY_ROLE` (env / `.env` / `--env-file`), naming this page.
Migration:

| You had | Now | Migration |
|---------|-----|-----------|
| No stack, no `AUTONOMY_ROLE` (compose default role `full`) | `full_default` launches by default — machine-proven graph-identical to the old `full` role | Nothing to do (or be explicit: `--stack full_default`) |
| Explicit `AUTONOMY_ROLE=full` | Preflight error | `--stack full_default` (or drop the variable — same stack launches) |
| `local_droan_cpu.launch.xml` variant | **Deleted in P5-E2** — the CPU-DROAN topology lives only in the stack | `--stack full_droan_cpu` |
| `local_macvo_obstacle_avoidance.launch.xml` | **Deleted in P5-E2** (was broken: wrong arg names, stale topic) | `--stack full_macvo` (fixed) |
| `AUTONOMY_ROLE=onboard` (lite, no split) | Preflight error. (On the desktop profile this role was unreachable anyway — `robot-desktop` hardcoded `AUTONOMY_ROLE=full`.) | `--stack lite_default` |
| `AUTONOMY_ROLE=onboard` / `offboard` (split) | Preflight error | `--stack lite_offload_global:onboard` / `:offboard` (+ `bridge.yaml`; generate the router config — the generated config deliberately drops the legacy split's `set_trajectory_mode` crossing, doctor hard gate #2) |
| `--stack X` **and** `AUTONOMY_ROLE` | Preflight error (no silent "stack wins" anymore) | Drop `AUTONOMY_ROLE` |
