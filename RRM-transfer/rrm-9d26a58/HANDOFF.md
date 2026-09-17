# RRM remote Codex handoff

Prepared 2026-09-13. Read this before implementing changes. This file captures the
user's instructions and the findings from the local review; it does not claim that
the proposed architecture or SIL integration has been implemented or approved as a
completed baseline.

## Start here

You are continuing RRM work inside the user's remote AirStack workspace at AirLab,
using NVIDIA OSMO. The user wants software-in-the-loop (SIL) first. The Mac is for
source preparation and transfer; do not download model weights or install the GPU
stack there. This archive contains the RRM source, not AirStack or model weights.

1. Read this handoff and the applicable AGENTS.md in the remote AirStack checkout.
2. Inspect the actual remote branch, uncommitted changes, containers, and resource
   allocation. Preserve existing work. Do not infer remote state from local findings.
3. Read the Phase 1 sources below and inspect RRM's existing code. If connectors are
   unavailable, use the requirement transcription below and identify that limitation.
4. Complete the SCRUM-8 architecture allocation and logical interface contracts before
   adding components or making detailed model/transport/deployment commitments.
5. Use the capstone curriculum and metrics to define a traceable SIL evaluation plan.
6. Implement incrementally after allocation is clear, preserving the mock regression
   baseline and adding meaningful checks for capability, supervision, and traceability.

Do not mark SCRUM-8 Done based on this handoff or the five mock benchmark passes.
Do not send Jira comments or other messages without the user's authorization.

## User decisions and scope

- Phase 1 is complete in Jira/Confluence: SCRUM-6 / RRM-01 Needs & CONOPS and
  SCRUM-7 / RRM-02 Requirements & Traceability are Done.
- Next work: SCRUM-8 / RRM-03 System Architecture, last read as To Do.
- Map every system requirement to architecture responsibilities; define stable
  interfaces between reasoning, world/task state, planning, safety, execution
  monitoring, telemetry, and embodiment adapters.
- Keep task-level reasoning embodiment-independent. Declared capabilities and limits
  can cause different valid plans without requiring robot-specific reasoning behavior.
- Avoid detailed implementation choices until architecture allocation is clear.
- The current focus is SIL. AirStack/OSMO is the primary execution environment; PSC
  is a possible fallback if problems arise. PSC access and configuration are unknown.
- Select any future VLA, VLM, NLP, or other learned model for the best capability per
  available compute; record the exact model, license, resource needs, interface role,
  and evaluation plan before integration. Begin the drone-SIL integration with a
  model-free deterministic/shadow baseline; no model is needed for that increment.
- The capstone website, including Weeks 1–11, is an intended technical and evaluation
  reference. The full curriculum has NOT yet been reviewed in this session.
- The user intends to ZIP RRM locally and unzip it in the remote AirStack workspace.
  A local AirStack folder was found, but the user explicitly clarified that the actual
  integration target is remote. Do not require a local full AirStack setup.
- Reachy Mini was an aside/reference, not a selected embodiment or scope change.

## Authoritative Phase 1 sources

- CONOPS, page 1048599, version 4 when read:
  https://deboabolade.atlassian.net/wiki/spaces/SCRUM/pages/1048599
- Requirements, page 786434, version 3 when read:
  https://deboabolade.atlassian.net/wiki/spaces/SCRUM/pages/786434
- Architecture, page 1015810, version 1 when read (a short capability outline):
  https://deboabolade.atlassian.net/wiki/spaces/SCRUM/pages/1015810
- https://deboabolade.atlassian.net/browse/SCRUM-6
- https://deboabolade.atlassian.net/browse/SCRUM-7
- https://deboabolade.atlassian.net/browse/SCRUM-8
- Atlassian cloud ID used: 13ecd67f-0309-4e63-838e-f3e1da003e98.

The user's instructions and Phase 1 baseline supersede conflicting prototype choices
in CLAUDE.md, README.md, docs/architecture.md, requirements.txt comments, and setup
scripts. Preserve useful design history but distinguish it from current requirements.
The old repo's phase numbers are not the Jira engineering lifecycle's phase numbers.

### CONOPS summary

RRM interprets contextual task objectives, maintains relevant world/task state,
decomposes and plans permitted tasks, executes through declared embodiment interfaces,
monitors outcomes, and replans, requests assistance, or reaches a safe condition.

The representative scenario is a structured simulated workspace with multiple relevant
objects, a contextual multi-step instruction, at least one ambiguity or changed
condition, and explicit constraints. Variations include a displaced object, an
ambiguous target requiring clarification, unsupported embodiment actions, safety
rejection, and execution failure requiring recovery.

Autonomy is bounded by permissions, safety, capability, observability, and required
human approvals. Human override/stop remains available whenever execution can affect
the environment. Embodiment progression in CONOPS is dexterous hand → arm → bimanual
system → mobile manipulator → humanoid; not every embodiment supports every action.

### System requirements (transcribed from the baseline)

All IDs below have prefix RRM-SYS-REQ-. Parent IDs use RRM-STK-REQ-.

| ID | Requirement | Parent/source | Initial verification intent |
|---|---|---|---|
| 001 | The RRM shall interpret a task objective using available contextual information relevant to the current task. | STK-001 | Demonstration / test |
| 002 | The RRM shall maintain a task-relevant representation of current world and task state during execution. | STK-002 | Inspection + test |
| 003 | The RRM shall decompose a permitted multi-step task into executable subgoals or actions. | STK-002 | Test |
| 004 | The RRM shall evaluate planned actions against declared embodiment capabilities and limits before dispatch. | STK-004 | Test |
| 005 | The RRM shall monitor execution outcomes and compare observed state against expected action effects. | STK-002 | Test |
| 006 | The RRM shall replan, request assistance, or transition to a safe condition when observed execution invalidates the active plan. | STK-002, STK-003 | Off-nominal test |
| 007 | The RRM shall request clarification or approval when uncertainty materially affects task intent, safety, or successful execution. | STK-003 | Scenario test |
| 008 | The RRM shall evaluate applicable safety and permission constraints before dispatching an action to the robot embodiment. | STK-003 | Safety test |
| 009 | The RRM shall support interruption and externally commanded stop or override before and during physical execution where the embodiment can affect the environment. | STK-003 | Demonstration / test |
| 010 | The RRM shall support execution of task-level reasoning across multiple robot embodiments by consuming declared embodiment capabilities and limits without requiring embodiment-specific changes to the task-level reasoning behavior. | STK-004 | Demonstration + architecture inspection |
| 011 | The RRM shall record telemetry sufficient to reconstruct task objectives, relevant state, decisions, dispatched actions, observed outcomes, replanning events, interventions, and failures. | CONOPS Operational Success Criteria | Inspection + test |

Stakeholder requirements cover contextual objectives/constraints (001), multi-step
execution under change/failure (002), human oversight (003), and reusable reasoning
across embodiments (004). New architecture elements must identify the requirements
they satisfy; trace forward to tests and evidence as those are created.

## Current RRM implementation and verified state

Source revision: 9d26a58eb8516b6754c5d12b041cdc789950e047, repository
https://github.com/o-abolade/rrm.git. The source working tree was clean before adding
this handoff. No runtime code, Jira, or Confluence changes were made in this session.
The handoff is an additional uncommitted file included in the transfer archive.

| Path | Current responsibility |
|---|---|
| rrm/schema.py | Pydantic payloads, WorldBackend and ActionPolicy protocols |
| rrm/verbs.py | Authored preconditions/effects and predicate evaluation |
| rrm/reasoning.py | ScriptedOracle backward-chaining planner with predefined goal |
| rrm/loop.py | Outer planning/replanning and inner execution/effect checking |
| rrm/safety.py | Symbolic action and placeholder numeric trajectory safety gates |
| rrm/world.py, rrm/policy.py | Mock backend and policy |
| rrm/trace.py, rrm/benchmark.py | JSONL events and benchmark metrics/harness |
| simulation/isaac_backend.py | Relation inference, initial adapter renderer, Isaac/GR00T stubs |
| scripts/oracle_loop.py | CPU-only benchmark CLI |

Verified locally during review:

```text
python scripts/oracle_loop.py --suite    → 5/5 pass (T1, T2, T6, T8, T9)
python simulation/isaac_backend.py      → relation inference PASS
```

The local commands used the existing .venv, which is excluded from the archive.
Core dependency is pydantic>=2.0. Recreate the environment on the remote machine.
No Isaac/GR00T/learned-reasoner integration or GPU job has been run here.
Mock passes do not establish SIL integration or Phase 1 compliance.

### Gaps and conflicts found before any code changes

1. RobotState assumes one gripper, one held object, and radial reach. Core predicates
   depend on these assumptions. Numeric safety uses hardcoded joint/velocity limits.
   Declared capabilities/limits have no implemented contract.
2. ReasonerBackend combines reasoning and planning, receives no capability declaration,
   and the oracle is given its goal. Contextual interpretation is not implemented.
3. Clarification, approval, permission context, and external stop/override contracts
   are absent. Bounded abort is not confirmation of a physical safe state.
4. Safety rejection replans the same action until budget exhaustion. T6 starts with a
   human already present; it does not test human entry during motion or active stop.
5. World confidence/uncertainty fields do not drive escalation. Unknown predicates are
   treated as false, so negation can turn missing evidence into apparent satisfaction.
6. Telemetry lacks full relevant state, capabilities, permissions, and interventions
   needed for reconstruction. Action IDs are reused across plan versions; correlation
   semantics need definition.
7. Numeric path safety treats the first waypoint coordinates as Cartesian positions
   while also treating waypoint values as joints. It is a mock placeholder, not a
   verified embodiment safety model.
8. Isaac backend and GR00T policy are stubs. Adapter rendering has no declared support
   contract and defaults missing poses to zero; grounding failures need explicit handling.
9. The old docs commit to A10G, Panda, GR00T, Qwen/vLLM, ROS topology, and perception
   timing before requirement allocation. These are historical prototype choices,
   not current constraints. Panda-first differs from the stated CONOPS progression.

Preserve useful properties: abstract actions over object IDs, authored expected effects,
observed-state reconciliation, mandatory safety gates, bounded recovery, and a stable
deterministic oracle regression baseline. Do not assume the oracle is infallible.

## Proposed architecture allocation — not yet implemented

| Logical responsibility | Requirement allocation |
|---|---|
| Operator/task interface and reasoning | 001, 007; supports 003, 006, 009 |
| World/task state, observation ingestion, task memory | 002; supports 001, 005, 006, 011 |
| Planning | 003, 004; supports 006, 008, 010 |
| Safety/permission authority and execution supervision | 008, 009; supports 004, 006, 007 |
| Execution monitoring | 005, 006; supports 009, 011 |
| Embodiment declaration, adaptation, and execution | 004, 010; supports 005, 008, 009 |
| Telemetry/evidence | 011, with evidence from every boundary |

These are logical responsibilities, not a demand for separate processes or new packages.
Contracts to define before implementation:

- Task request: objective, context, constraints, approval/clarification state.
- World/task snapshot: version, observation times, provenance, uncertainty, progress.
- Capability declaration: supported actions/resources, limits, availability, feasibility.
- Reasoning result: interpreted goal, grounded entities, constraints, unresolved ambiguity.
- Plan: versioned actions, preconditions/effects, capability and state references.
- Safety decision: permission/safety result and context it applies to; dispatch must
  not reuse a stale authorization after relevant changes.
- Execution: acceptance, progress, completion/failure/interruption, observed outcomes,
  and explicit stop/cancel/override acknowledgment and safe-state reporting.
- Telemetry: correlated task/plan/action/dispatch/state/decision records sufficient
  for replay and assessment; numeric details and limits remain behind adapters.

## Remote AirStack / OSMO integration

User-supplied operating information (not verified against live cluster):

- 3 machines, 4 RTX PRO 5000 Blackwell GPUs each: 12 GPUs total.
- 48 CPU cores and 216 GB RAM per machine: 144 CPUs and 648 GB RAM total.
- Fair-use target per GPU: 12 CPUs + 48 GB RAM.
- Suggested maximum: 4 GPUs per person/project.
- LOW priority can use spare capacity and can be preempted.

Normal one-GPU request:

```yaml
resources:
  default:
    cpu: 12
    gpu: 1
    memory: 48Gi
    storage: 500Gi
```

500 GiB is the intended job request, NOT a verified pool maximum or persistent volume.
Actual GPU VRAM, driver/runtime compatibility, active workflow ID, remote branch, and
persistent result/cache location still need remote inspection. Do not assume the old
A10G model budget or precision restrictions apply to Blackwell.

### Current live Isaac / OSMO operating procedure (2026-09-13)

This later GPU workflow is live. The user started it through the normal AirStack
OSMO workflow procedure with the one-GPU fair-use request (1 GPU, 12 CPU, 48 GiB
RAM), opened the IDE tunnel with `./airstack.sh osmo ide`, and connected VS Code
to `airstack-osmo`. The AirStack Isaac livestream, robot desktop, and GCS
containers are running. This supersedes the earlier CPU-only transfer-workspace
observations in `docs/scrum-8/remote-state.md`; retain those only as historical
diagnosis of that separate `gpu: 0` workspace.

The desired manual Pegasus PX4 launch uses livestream and all of the following
runtime flags:

```sh
--/renderer/activeGpu=0
--/renderer/multiGpu/enabled=false
--/physics/cudaDevice=0
```

These flags are the deliberate single-GPU fix. In the nested OSMO runtime, Isaac
can enumerate all four physical GPUs even when the workflow requested one; without
the flags, its renderer can initialize allocations on all four. The flags constrain
rendering and physics to GPU 0.

Read-only process inspection on 2026-09-14 found that the *current* Isaac process was
instead started by the Compose `AUTOLAUNCH=true` command. Its command line includes
only `--/app/livestream/enabled=true`, not the three pinning flags above. Treat the
desired manual procedure as not applied to the current process. Do not alter Compose
or restart the live simulator solely to change this without user direction; apply the
pinned command at the next controlled launch.

The initial inspection occurred with the Pegasus timeline stopped, which stops the
PX4 backend while leaving the Isaac Python process alive. The user then pressed Play;
read-only checks confirmed PX4 running, MAVROS connected, and Isaac reporting its
first heartbeat and `Ready for takeoff!`. Leave Isaac open and playing while using the
robot stack.

The live robot stack initially had a MAVROS namespace duplication: MAVROS published
under `/robot_1/interface/mavros/mavros/*`, while AirStack's `robot_interface` and
`odometry_conversion` expected the canonical `/robot_1/interface/mavros/*` paths.
The source cause was `interface.launch.py` pushing `interface` while
`mavros_px4.launch.xml` also supplied `namespace="mavros"`; MAVROS's own relative
`mavros/*` topic names added the second level.

With user authorization, `interface.launch.py` was corrected to pass an empty MAVROS
namespace, and only `robot-desktop` was recreated (Isaac and GCS stayed running).
The workspace rebuilt successfully. MAVROS now lives at `/robot_1/interface/mavros`;
canonical MAVROS odometry has one publisher and feeds `odometry_conversion`, converted
odometry and `map -> base_link` TF stream, and the obsolete nested topic is absent.
A bounded `airstack ready --json` passed all gates: containers, `/clock`, autonomy
nodes, MAVROS-to-PX4 connectivity, and EKF odometry. No task or flight command was
sent. A model-free RRM shadow adapter may now consume the canonical state stream;
retain the separate safety gate before any execution integration.

The first adapter has been implemented and passively verified. Its deterministic core
is `rrm/airstack_shadow.py`; `scripts/airstack_shadow.py` is an `rclpy` observer that
subscribes only to canonical MAVROS odometry/state, `/tf`, and task-status topics. It
has no ROS action client, publisher, service client, trajectory, or PX4 command path.
Every snapshot remains `execution_inhibited=True`; the evidence manifest also declares
`execution_dispatch_enabled: false`. A 15-second live session emitted 15 snapshots
(14 fresh after one explicit startup-incomplete snapshot), with an observation-complete
final replay report. The shadow tests (5), existing safety/contract tests (14), and
the deterministic oracle suite (5/5) passed. The next safe increment is shadow
correlation of a separately user/GCS-initiated task, followed by offline RRM reasoning
evaluation; do not add dispatch authority as part of that work.

The first user-initiated task correlation is also complete. The user connected
Foxglove Desktop through the GCS websocket, then submitted one takeoff and one land;
the shadow observer recorded `EXECUTING → SUCCEEDED` for each. It recorded 160
snapshots (158 fresh after two explicit startup gaps), all execution-inhibited. Final
MAVROS/odometry evidence was connected, disarmed, near ground height, and effectively
stationary; its replay report was observation-complete and dispatch-disabled. GCS,
not RRM, issued the two task actions. Treat this as an evaluation baseline only; no
RRM action authority has been added.

For remote viewing, the user starts the patched Mac-side forwarder with the patched
OSMO binary first in `PATH`, then connects the AirLab Isaac Sim WebRTC Streaming
Client to `127.0.0.1`. The patch is required for the known stock OSMO 6.3.1 UDP
49099 forwarding failure.

OSMO workspace storage and Codex conversation memory are not durable across a
workflow replacement. Persist source and this handoff by committing and pushing;
export non-source evidence separately before workflow termination. In particular,
the AirStack feature `notebook/` is gitignored and is not a handoff mechanism.

Guide supplied by user:
https://docs.theairlab.org/0.20/docs/tutorials/airstack_on_osmo/

Guide architecture: OSMO workspace pod runs sshd and Docker-in-Docker; AirStack's
Isaac Sim, robot-desktop, and GCS run inside it. SSH alias airstack-osmo reaches
localhost:2200 through an authenticated OSMO port-forward. IDE workspace is normally
/root/AirStack. Files in the workspace pod are not automatically present at every path
inside the nested containers; inspect bind mounts and build in the correct container.

The guide's raw tunnel command is:

```bash
osmo workflow port-forward <workflow-id> workspace --port 2200:22 --connect-timeout 86400
```

AirStack branches are cloned remotely from GitHub on job startup; local unpushed edits
are not transferred automatically. ZIP transfer is the user's selected handoff path.
Unzipping stages the source; it does not integrate it into ROS or make it persistent.
Save/push source changes and export evidence before the ephemeral job is terminated.
Keep weights and datasets out of Git and the source archive; download pinned revisions
remotely and record their identifiers, licenses, precision, and checksums where available.

Local AirStack reference inspected earlier was on main, with modified
osmo/workflows/airstack-dev.yaml and untracked osmo/patch_osmo_udp.py. Those files are
NOT in this archive, and that state is NOT evidence of remote state. Remote AGENTS.md
is authoritative for AirStack editing/build conventions. The local reference required
a feature notebook before implementation and described external modules, stacks, and
fleets; inspect the remote version before choosing integration structure.

RRM should consume AirStack state and execute through explicit interfaces while keeping
its task-level core independent of AirStack's drone stack. Do not infer that using the
AirStack runtime changes the RRM scenario to aerial robotics. Select the SIL scene and
embodiment against CONOPS and document any deliberate change.

## Capstone and model assessment

Sources already inspected:

- https://001-physical-ai-book.vercel.app/docs/capstone/project-specification/capstone-project-specification-framework
- https://001-physical-ai-book.vercel.app/docs/capstone/implementation-guide/implementation-guide

Also inspect the evaluation page and the full Weeks 1–11 curriculum:
https://001-physical-ai-book.vercel.app/docs/capstone/evaluation-criteria/evaluation-criteria

The spec includes targets such as simple command processing <2 s, complex planning
<10 s, navigation planning <1 s, emergency stop response <0.1 s, perception >=30 FPS,
manipulation positioning within 1 cm, and average task completion within 30 s. It also
lists operational RAM <8 GB and models/data storage <100 GB. These are source targets
to map and assess, not proof that every target applies to the current SIL scenario or
that the larger OSMO allocation changes an operational requirement.

Next deliverable: source-linked metric → Phase 1 requirement → architecture owner →
scenario/test → measurement definition → acceptance target → evidence mapping.
Record applicability, measurement start/end boundaries, simulation versus wall-clock
time, trial counts, and unresolved conflicts. Keep system performance distinct from
individual model inference performance. No "best model" has been selected or verified.
Compare candidates remotely under the same scenes, inputs, resources, and measurement
conditions; separate reasoning, perception, and execution-policy contributions.

The implementation guide is a broad humanoid engineering checklist (layered design,
interfaces, incremental integration, safety, test and deployment practices), not a
drop-in RRM implementation. Its phase numbering differs from Jira Phase 1.

## Reachy Mini aside

The user asked for a separate architecture review. Reachy Mini has an expressive head,
rotating body, antennas, camera, microphones, and speaker. It uses an application SDK
and hardware daemon, with media streaming and a MuJoCo simulation option. It is useful
as a conceptual example of an embodiment that supports interaction but not grasping
or navigation. It has NOT been chosen as an RRM target.

- https://huggingface.co/docs/reachy_mini/en/SDK/core-concept
- https://huggingface.co/docs/reachy_mini/en/platforms/reachy_mini/hardware
- https://huggingface.co/docs/reachy_mini/en/SDK/media-architecture

An adapter must expose command adjustments and observed outcomes: e.g. a clamped pose
does not necessarily achieve the originally requested effect.

## Transfer status and next action

This archive includes the original RRM runtime code plus this handoff. It excludes
.git, .venv, model weights, caches, and experiment recordings. It is a starting source
snapshot, not a completed SCRUM-8 architecture or an integrated SIL deployment.

Begin with the architecture allocation and metrics mapping above; inspect the remote
environment in parallel with that read-only work. Then make the smallest requirement-
backed changes and verify them locally within the remote development environment,
before advancing to integrated GPU SIL trials.

## Continuation — 2026-09-17 UTC

The complete staged SIL integration plan is now in
[`docs/scrum-8/integration-plan.md`](docs/scrum-8/integration-plan.md). It translates
the logical C01–C09 allocation into gates: freeze the shadow baseline; select a
CONOPS-compatible hand workspace; make task/context/state/plan contracts executable in
shadow; build a dry-run adapter; implement independent supervision and safe-state proof;
then enable the narrow simulator-only command path and run the frozen S01–S10 campaign.
Do not skip directly from the existing read-only drone observer to PX4 or task-action
authority.

The current AirStack process was intentionally started by the user and is the existing
`example_one_px4_pegasus_launch_script.py` / Iris / PX4 drone launch. It remains an
aerial transport baseline, not the RRM manipulation scene. Do not modify or repurpose
it in place. The configured Isaac asset root is
`omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1`; the image enables
Franka and robot-motion/Lula infrastructure, but no local hand, Allegro, Shadow Hand,
or Franka USD asset is cached. Standalone Python does not expose `omni.client`, so no
Nucleus browse was attempted by opening a second Kit process. Select and record a
versioned dexterous-hand asset/controller, reset method, observation channels and safe
state before a separate controlled hand-scene launch. Foxglove is not required for the
contract work and was not started by this continuation.

`rrm/task_contracts.py` and `tests/test_task_contracts.py` now implement the first
proposal-only C01/C04/C05 foundation. `TaskRequest`, `Interaction`, `ReasoningResult`,
`PlannedAction` and `PlanProposal` are immutable Pydantic records. Plans require a
ready intent with matching task/state/capability revisions, authored verb arity, unique
action IDs and acyclic dependencies. Material ambiguity cannot become a plan. These
records have no ROS dependency and do **not** implement C06 admission, C07 dispatch, a
controller command, a publisher, an action client, a service client or any execution
authority.

Validation on 2026-09-17 used Pydantic in an isolated `/tmp/rrm-contract-deps` target
because host Python has neither the dependency nor `venv` support; no system packages,
AirStack image, simulator scene, or Foxglove client were changed. Results: 5/5 new
task-contract tests, 24/24 total shadow/contract/unit tests, and the original Oracle
suite 5/5 (T1, T2, T6, T8, T9). The local feature notebook is gitignored and is not a
handoff mechanism; durable source/doc changes still need commit and push, while runtime
evidence must be exported separately before workflow replacement.

`rrm/state_contracts.py` and `tests/test_state_contracts.py` now add the C02
evidence-backed state foundation. `FactEvidence` records semantic fact identity, truth,
provenance, source reference and freshness bounds; `StateSnapshot` resolves a fact only
from fresh, explicit, non-contradictory evidence. Missing, stale, UNKNOWN,
contradictory and negated-UNKNOWN evidence resolve to UNKNOWN. Coverage metadata does
not silently convert an absent fact into negative evidence. This deliberately remains
separate from the legacy mock `WorldState` and creates no ROS, adapter, safety-admission
or execution path. Five state-contract tests passed; the total unit suite is now 29/29
and the Oracle regression remains 5/5. The feature notebook is gitignored; commit/push
the source and documentation changes before workflow replacement.

The user clarified that “planning” means RRM output to the current drone, not merely
symbolic planning. The chosen integration seam is AirStack's public task-action boundary
used by Foxglove: `/{robot}/tasks/takeoff`, `/navigate`, and `/land`. Do not issue raw
PX4/MAVROS/service/trajectory commands. `rrm/airstack_drone.py` contains immutable
typed proposals; `scripts/airstack_drone_dispatch.py` maps them to the exact
`task_msgs` action goal shape. It prints a dry-run by default without importing ROS;
only `--execute` creates one ActionClient and sends one selected task goal. A dry-run
takeoff fixture verified `/robot_1/tasks/takeoff`, `TakeoffTask`, altitude 2.0 m and
velocity 1.0 m/s with `execution_requested: false`. No live task was sent. Four adapter
tests passed; the total unit suite is 33/33 and the Oracle regression is still 5/5.
This is an output adapter, not the complete C06 admission/C08 stop authority; require an
explicit user-approved simulation proposal before the first `--execute` invocation.

## First controlled drone-action SIL run — 2026-09-17 UTC

The user explicitly approved the bounded Isaac/PX4 sequence “take off to 2 m at 1 m/s,
then land.” This was treated as two separately completed public AirStack task actions,
not a direct PX4/MAVROS command or an autonomous multi-action chain. The live
`airstack-robot-desktop-1`, `isaac-sim-livestream`, and `airstack-gcs-1` containers were
up; `/robot_1/tasks/takeoff` and `/robot_1/tasks/land` each reported one task-action
server at `/robot_1/takeoff_landing_planner/takeoff_landing_task`.

The runner and its Pydantic dependency were staged only under the robot container's
temporary paths `/tmp/rrm-sim-adapter` and `/tmp/rrm-sim-deps`; neither the source
workspace nor the AirStack image is mounted or persistently modified. Both proposals
first completed dry run in that same container: `takeoff-1` mapped to
`TakeoffTask(target_altitude_m=2.0, velocity_m_s=1.0)` and `land-1` mapped to
`LandTask(velocity_m_s=1.0)`.

An initial execution attempt failed before an ActionClient was created because staging
overwrote the ROS `PYTHONPATH`, so no goal was sent. Preserving the existing ROS path
and adding the temporary adapter/dependency paths passed `import rclpy` and
`TakeoffTask` preflight. The subsequent takeoff returned
`success: true, message: "takeoff complete"`; after that terminal result, the land goal
returned `success: true, message: "landing complete"`. The task-server feedback was
received throughout (empty status for takeoff; `landing` for land). This establishes one
successful command-and-result path through RRM → AirStack task actions → existing
drone stack. It does **not** independently prove pose/visual observation agreement,
C06 admission, C08 stop authority, fault handling, repeatability, or autonomous
sequencing; those remain required before any flight-readiness claim.

## Outcome-verification increment — 2026-09-17 UTC

The next robustness increment adds `OdometryEvidence`, `VehicleStateEvidence` and
`verify_drone_outcome` to `rrm/airstack_drone.py`. It is a pure, fail-closed evaluator:
it verifies a successful takeoff only with fresh independent `map -> base_link`
odometry within the AirStack planner's 0.3 m absolute-altitude acceptance distance and
a fresh connected/armed vehicle state; it verifies landing only with fresh near-ground
odometry plus connected/disarmed vehicle state. Missing, stale, future, wrong-frame,
unsuccessful or mismatched evidence remains `UNCONFIRMED` or `MISMATCH`; navigation is
explicitly unconfirmed pending a defined endpoint/effect contract.

`scripts/airstack_drone_dispatch.py --verify-observation` now adds only read-only
subscriptions to `/{robot}/odometry_conversion/odometry` and
`/{robot}/interface/mavros/state`. It requires fresh pre-dispatch odometry, captures
post-result samples, writes a JSON outcome when `--outcome-json` is supplied and exits
nonzero unless the independent evidence verifies the action-server success. It retains
the explicit `--execute` gate and has no publisher, service client, direct PX4/MAVROS
command or trajectory interface. No task was dispatched while implementing this
increment. A separate eight-second observer-only run after the first SIL sequence
reported `observation_complete: true`, MAVROS connected/disarmed, fresh map/base-link
evidence and `z=0.0201 m`; it corroborates the final landed state but is not causally
attached to either earlier action.
