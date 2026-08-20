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
| [`full_default`](https://github.com/castacks/AirStack/tree/develop/stacks/full_default) | The current full-autonomy topology (GPU `droan_gl` planner) — baseline, graph-identical to legacy `AUTONOMY_ROLE=full`. |
| [`full_droan_cpu`](https://github.com/castacks/AirStack/tree/develop/stacks/full_droan_cpu) | CPU DROAN planner + live `disparity_expansion` — absorbs `local_droan_cpu.launch.xml`. |
| [`full_macvo`](https://github.com/castacks/AirStack/tree/develop/stacks/full_macvo) | MAC-VO as the planner's disparity source — supersedes (and fixes) the broken `local_macvo_obstacle_avoidance.launch.xml` variant. |

## Wrap vs. flatten — current status

Stack adoption is a two-step migration:

- **Wrap form (now):** each reference stack's `stack.launch.xml` *includes*
  the existing layer bringup files (`interface_bringup`, `local_bringup`, …),
  capturing today's topology without moving any wiring. The remaps still live
  inside those layer bringups.
- **Flatten (next phases):** the layer bringups' nodes and remaps move into
  the stack entry files, the legacy files shrink, and their lines disappear
  from the lint allowlist (below). `autonomy_bringup` thins until the
  AUTONOMY_ROLE dispatch is gone.

In both forms, the stack's `wiring.md` — snapshotted from the running system —
is the observed truth of the graph.

## Running a stack

```bash
airstack up --stack full_default --sim isaac --robots 1
airstack ready
```

Mechanics: `--stack <name>` validates `stacks/<name>/launch/stack.launch.xml`
exists, then exports `AIRSTACK_STACK_DIR=/root/AirStack/stacks/<name>` (the
*container* path — `stacks/` is bind-mounted into every robot container) and
`AIRSTACK_STACK_ENTRY=stack`. Inside the container,
`autonomy_bringup/launch/robot.launch.xml` still runs the shared preamble
(ROBOT_NAME namespace, `use_sim_time`, `robot_state_publisher`, world→map TF),
then includes the stack entry file *instead of* the legacy role groups.

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

Legacy runs without `--stack` keep using the golden at
`tests/goldens/wiring/full_default.<sim>.<N>robot.md`.

## The single-locus rule (and its lint)

All cross-module remaps and topic-arg overrides live in the stack's entry
launch file(s). Module launch files declare topic args (canonical defaults per
[the integration checklist](../robot/autonomy/integration_checklist.md), a
`description=` on every arg) but **never** `<remap>` and never hardcode
cross-module topics. The stack file *is* the wiring diagram; `grep -r
global_plan stacks/my_stack/` answers "who touches this".

Enforced by `tests/meta/test_launch_single_locus.py` (`unit` mark, runs in CI):

1. No `<remap>`/`remappings=` outside `stacks/*/launch/` — except files frozen
   in `tests/meta/launch_lint_allowlist.txt` (the wrap-form legacy set).
2. The allowlist only shrinks: an entry whose file no longer carries a remap
   fails the lint until its line is deleted.
3. Stack launch files must describe every `<arg>` they declare.

See the [write-launch-file skill](https://github.com/castacks/AirStack/blob/develop/.agents/skills/write-launch-file/SKILL.md)
for the authoring workflow.

## Making a new stack today

1. Copy a reference stack: `cp -r stacks/full_default stacks/my_stack`
   (`airstack stack new` arrives in a later phase).
2. Edit `launch/stack.launch.xml` — swap/add/remove module `<include>`s and
   their topic args. Update `README.md`.
3. Run it: `airstack up --stack my_stack --sim isaac`.
4. Snapshot the wiring: `airstack test -m wiring --stack my_stack`, validate
   the observed file, commit it as `stacks/my_stack/wiring.md`.
5. `airstack test -m unit` — the layout contract and the launch lint must pass.

## AUTONOMY_ROLE deprecation

`AUTONOMY_ROLE` remains fully functional while stacks land, but it is the
legacy dispatch — stacks replace it in 0.21. `airstack up` warns when it sees
an explicitly set `AUTONOMY_ROLE`:

| You have | What happens | Migration |
|----------|--------------|-----------|
| No stack, no explicit `AUTONOMY_ROLE` | Legacy dispatch, compose default role (`full`) — unchanged, no warning | `airstack up --stack full_default` when ready |
| Explicit `AUTONOMY_ROLE=full` (env / `.env` / `--env-file`) | Legacy dispatch + deprecation warning | `--stack full_default` |
| `local_droan_cpu.launch.xml` variant | **Deleted in P5-E2** — the CPU-DROAN topology lives only in the stack | `--stack full_droan_cpu` |
| `local_macvo_obstacle_avoidance.launch.xml` | **Deleted in P5-E2** (was broken: wrong arg names, stale topic) | `--stack full_macvo` (fixed) |
| `AUTONOMY_ROLE=onboard` / `offboard` (split) | Legacy dispatch + warning | split stacks arrive with RFC #380 §2 (`lite_offload_global`) |
| `--stack X` **and** explicit `AUTONOMY_ROLE` | Stack wins — the role is ignored by the launch dispatch; warning says so | Drop `AUTONOMY_ROLE` |
