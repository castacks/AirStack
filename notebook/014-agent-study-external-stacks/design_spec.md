# Design Spec: Agent Proxy Study on External Stacks (UAS + Aerostack2) via OSMO

> Notebook entry: `notebook/014-agent-study-external-stacks/` · Date started: 2026-09-11 04:41 · Last updated: 2026-09-12 10:45 · Branch: `airstack-paper` · Commit: `4556d8233` (agent_study submodule `38be204`)
>
> Campaign: `Agent study (paper Sec. VI-C)` — extends
> [011](../011-agent-study-v6-trials/design_spec.md) (v6, AirStack arms)
> with two **external-platform arms** run on the lab's OSMO GPU pool.
>
> **Status: `DONE`** <!-- 20/20 trials scored 2026-09-12 10:28 EDT; results/results_summary.md -->
>
> **Canonical spec:** `agent_study/agent_study_protocol.md` (PRIVATE
> submodule). This entry is the execution record for the external-stack
> extension: design decisions, infrastructure (OSMO pods), per-trial
> dispositions, and the accumulating score matrix. Protocol amendment
> text (Amendment 4) is drafted in §2.6 for lead approval.

## 1. Problem Context

Campaign v6 (entry 011) measured four arms on AirStack: A1 scaffolded,
A2 scaffolding-ablated, A3 open-loop, A4 bare parts (post-relabel names).
With time before the ICRA deadline (2026-09-15), the lead (Andrew, session
2026-09-11 04:41 EDT) asked to run the *same* proxy-developer ladder on two
related, publicly released aerial autonomy stacks so the paper can situate
AirStack's agent-legibility result against comparable platforms rather than
only against its own ablation and a bare-parts counterfactual:

- **Unified Autonomy Stack (UAS)**, NTNU ARL —
  <https://github.com/ntnu-arl/unified_autonomy_stack>. ROS 1 Noetic +
  ROS 2 Humble hybrid (GBPlanner on ROS 1, NMPC/CBF/RL on ROS 2, bridged),
  Gazebo Harmonic with a forked `gz-sim` (multicopter control), 13 locally
  built Docker images (`make images`, `docker buildx bake`), workspaces
  imported with `vcstool` from SSH `git@github.com:` URLs, `make build`
  compiles ~17 workspaces in containers, `make launch DOCKER_COMPOSE_FILE=…`
  runs an example. Sim drone namespace `rmf`, odometry `/rmf/odom`
  (`nav_msgs/Odometry`, gz OdometryPublisher), velocity/acceleration
  commands on `/rmf/cmd/vel`. Default world `darpa_cave_01`, default spawn
  (40, 5, 0.5). Default `DOMAIN_ID=205`.
- **Aerostack2 (AS2)**, UPM CVAR — <https://github.com/aerostack2/aerostack2>.
  ROS 2 Humble (Ubuntu 22.04), Gazebo **Fortress** by default (Harmonic
  supported as an alternative build), published Docker images
  (`aerostack2/nightly-humble:<sha>`, `aerostack2/humble:1.1.3`), project
  template `project_gazebo` (tmuxinator launch, Python API missions:
  arm/offboard/takeoff/go_to/land). Drone namespace `drone0`; state via
  `self_localization/{pose,twist}`; ground-truth odometry bridged by a
  custom bridge (not `sensor_measurements/odom`); world defined by
  `config/world.yaml` (`world_name` → SDF in the assets resource path).

Constraints stated by the lead:

- **This box's RTX 5090 is reserved for another project.** Trials run on
  the lab's OSMO cluster (RTX PRO 5000 Blackwell, 48 GB) — pool `airstack`
  (shared, 12 GPUs total across the lab; 9 in use at 04:44 EDT). Use 3–4
  pods concurrently, checking availability at submit time.
- **Claude enterprise seat, not API billing:** the "Claude Code container
  trick" — a long-lived OAuth token from `claude setup-token`, injected into
  pods as an OSMO GENERIC credential → `CLAUDE_CODE_OAUTH_TOKEN`.
- The lab's OSMO docs (`~/Downloads/airlab_osmo_docs/*.pdf`) and AirStack's
  own OSMO tutorial (`docs/tutorials/airstack_on_osmo.md`, `osmo/`) are the
  references; the OSMO CLI is installed and logged in as `ajong`
  (Administrators role), harbor project `airstack` is public (pull needs no
  credential), this box is logged in to harbor for pushes.
- Study integrity rules from strategy.md choice #6 stay in force: frozen
  prompt (v4, sha `afad954d…` composed), frozen judge parameters, answer-key
  material (eval layout, reference solution) never staged into workspaces.
  Adding arms changes `config_sha256`, so this is a **new campaign
  (v7-external)** whose results are reported alongside — never pooled
  with — v6; the judge parameters are byte-identical to v6 (verifiable by
  `diff` of the two config files' `judge:` blocks).

Lead decisions received in-session (2026-09-11 04:46–05:20 EDT):

- OSMO pool `airstack`; **hard cap 4 GPUs at a time (official lab
  policy)** — a request to go to 7 was withdrawn.
- If a stack cannot be built/launched as intended despite best efforts,
  substitute a different platform from the paper's `tab:comparison`
  (MRS UAV, KR Flight, Agilicious, Crazyswarm2, AAS, XTDrone).
- Credentials registered in the lead's OSMO profile: `claude-oauth`
  (from `claude setup-token`), plus `airlab-docker-login` /
  `airlab-docker-registry` / `airlab-nucleus` via `./airstack.sh osmo
  setup`. The auto-mode classifier refused to let the agent read the
  harbor password from `~/.docker/config.json`; the lead registered it.
- OpenStack credentials exist at `~/.airlabcloud/openrc.sh` (fallback
  for S3 result storage; not used — results return via `osmo workflow
  rsync download`).

Decisions taken by the agent (lead asleep; flagged for review):

1. Models `claude-sonnet-5` + `claude-opus-5` (as v6); **5 trials per
   cell** → 2 stacks × 2 models × 5 = 20 trials; interleaved round-robin
   across pods per reproducibility rule 3.
2. The external arms are **closed-loop, host-mode arms** (judged exactly
   like the bare-parts arm A3: contract scripts `./bringup`, `./takeoff`,
   `./land`; ROS-graph-level checks from the pod against the agent's
   containers on host networking at `ROS_DOMAIN_ID=1`; Gazebo eval world
   staged over `provided/world_practice.sdf` at R7 per Amendment 3). This
   is the only arm-neutral judging path that needs no platform-specific
   code, so no judge logic is written per stack.
3. **Environment provisioning parity with A1:** A1 workspaces got the
   AirStack images pre-pulled and all submodules initialized; only the
   `colcon` workspace build ran during the session. External arms get the
   platform's published/built Docker images pre-pulled (UAS: the 13 images
   from `make images`, built once in a provisioning pod and mirrored to
   harbor; AS2: the pinned Docker Hub image plus a Harmonic-variant build
   if buildable) and the platform's source repos cloned at pinned commits
   (UAS: `./scripts/import_all_repos.sh --exact` with SSH→HTTPS URL rewrite
   since pods have no GitHub SSH key). Compiling the workspaces
   (`make build`, `colcon build`) is left to the agent, as in A1.
4. **Judge CLI distro:** both stacks are ROS 2 Humble. Humble↔Jazzy CLI
   interop works (§3(a); an initial failure was a stale daemon from
   another user on this box), but with host networking every process on a
   host shares ONE `ros2` daemon per domain, so a pod-native Jazzy CLI
   would hand the agent's Humble containers a Jazzy daemon (or vice
   versa). The pod therefore runs the judge CLI through a persistent
   Humble container (`ros:humble-ros-base`, host network), exposed as
   `/usr/local/bin/ros2` so the existing judges' `ros2 …` calls work
   unchanged (the `source /opt/ros/jazzy/setup.bash 2>/dev/null` prefix
   is a silent no-op) and the shared daemon is always Humble.
5. Raw arm ids stay short and stable in artifacts: **`E1` = UAS, `E2` =
   Aerostack2** (paper labels chosen at analysis time, per Amendment 2's
   presentation-only relabel rule).

## 2. Proposed Implementation

```mermaid
flowchart LR
  subgraph box [This box, no GPU use]
    orch[osmo_orchestrator.py<br/>submit ≤N pods, rsync study bundle in,<br/>poll, rsync results out, cancel]
    runs[agent_study/runs/E*_…]
  end
  subgraph osmo [OSMO pool airstack, 1 GPU each]
    pod1[trial pod: DinD + claude + judge-cli<br/>run_trial.py --config v7 --arm E1|E2]
    pod2[trial pod]
    pod3[trial pod]
  end
  harbor[(harbor airstack/agent-study-*<br/>pod image, UAS image mirror)]
  orch -- osmo workflow submit --> pod1 & pod2 & pod3
  orch -- rsync upload --> pod1
  pod1 -- rsync download --> runs
  harbor -. pull .-> pod1
```

### 2.1 Runner: external-platform arm transform — `DONE` (agent_study `93356dd`)

`agent_study/runner/run_trial.py`, `judge.sh`, `checks/r5_provenance.py`,
`prune_run.sh`:

- New transform `external` (config `arms.<id>.external: {name, repos: [{url, commit, dest}], images: [...], provision: <script>}`):
  fresh workspace = pinned clones of the platform repos + `ENVIRONMENT.md`
  (rendered from the arm's config, same wording skeleton as A3's) +
  `provided/` (planners, contract, ROUTE.txt, `world_practice.sdf`) +
  marker `.study_host_mode`.
- Host-mode generalization: `judge.sh` and `r5_provenance.py` branch on
  `STUDY_HOST_MODE=1` (exported by the shim for A3 and external arms)
  instead of `STUDY_ARM == A3`; A3 keeps identical behavior.
- `--arm` choices come from the config; trial-end container teardown for
  host-mode arms stops every container on the (single-purpose) pod.
- `prune_run.sh`: host-mode workspaces are kept minus large logs (they are
  the agent's whole solution), like A3.
- New config `config/study_config_v7_external.yaml` (campaign
  `2026-09-icra27-vic-v7-external`): `judge:` block copied verbatim from
  v6; `airstack:` block retained for provenance but unused by E arms.

### 2.2 OSMO trial pod image + entrypoint — `DONE` (agent-study-pod:v1; sshd transport, auth probe, CIFS optional)

`agent_study/osmo/pod/Dockerfile` → `airlab-docker.andrew.cmu.edu/airstack/agent-study-pod:v1`:
FROM `airstack-osmo-workspace:latest` (Ubuntu 24.04, docker-ce + compose,
nvidia-container-toolkit, DinD entrypoint pieces) + `claude` CLI (native
installer, pinned version) + `python3-yaml`, `python3-vcstool`, `tmux`,
`make`, `rsync` + `/usr/local/bin/ros2` judge-CLI wrapper + git
`url.insteadOf` SSH→HTTPS rewrite. Entrypoint (`pod_entrypoint.sh`):
start dockerd (data-root on the emptyDir, overlay2), `docker login`
harbor if credential present, start the `study-judge-cli` Humble container,
wait for the rsync'd study bundle marker, run the arm's provisioning
(pull/retag images, clone repos), run `run_trial.py`, write `DONE` marker,
sleep until cancelled (so results can be downloaded).

### 2.3 Provisioning workflow (one-off) — `DONE` (results/d-provisioning)

`agent_study/osmo/workflows/provision.yaml`: a CPU-heavy pod that clones
UAS at the pin, imports repos (`--exact`), runs `make images`, and pushes
each `unified_autonomy:<target>` to harbor as
`airstack/agent-study-uas:<target>-<uas_sha>`; builds the Aerostack2
Harmonic variant if the upstream Dockerfile exists and pushes it too.
Needs the `airlab-docker-login` OSMO credential (lead to register).

### 2.4 Trial workflow + local orchestrator — `DONE` (rsync-over-ssh transport; per-arm resources; HIGH priority; ENOSPC-tolerant)

`agent_study/osmo/workflows/trial.yaml` (privileged, gpu 1, cpu 16,
64Gi, 300Gi, `exec_timeout` 9h; env `STUDY_ARM/MODEL/TRIAL_INDEX`;
credentials `claude-oauth` → `CLAUDE_CODE_OAUTH_TOKEN`).
`agent_study/osmo/osmo_orchestrator.py`: takes a trial list, keeps ≤N
workflows in flight (N from free GPUs, cap 3–4), rsyncs the study bundle
(everything except `runs/`) into each pod, polls for the `DONE` marker,
downloads `runs/<trial_id>/` (pruned), cancels the workflow, records
dispositions in `results/c-trial-matrix/`.

### 2.5 Environment description per stack — `DONE` (in the v7 config, hash-covered)

`ENVIRONMENT.md` content per arm (kept parallel to A3's; no mechanism
hints beyond what the platform's own README gives):

- E1: "`unified_autonomy_stack/` — a clone of the Unified Autonomy Stack
  (NTNU ARL) at commit …, with its workspaces imported and its Docker
  images already built locally (`docker images`)." + `provided/`.
- E2: "`aerostack2/` — the Aerostack2 framework source at commit …;
  `project_gazebo/` — its Gazebo example project at commit …; the
  matching Docker image `aerostack2/nightly-humble:<sha>` is available
  locally" + `provided/`.
- Common: "The host has docker (NVIDIA runtime), the ROS 2 CLI tools
  (`ros2`), and network access; ROS 2 topics published on the host network
  are visible at `ROS_DOMAIN_ID=1`. You may install additional packages."

### 2.6 Protocol Amendment 4 (draft, needs lead approval) — `WIP` (text in agent_study_protocol.md; approval pending)

External-platform arms E1/E2: same frozen prompt, planners, ladder, caps,
judge parameters, host-mode judging as the bare-parts arm; provisioning
parity rule (§1 decision 3); campaign v7-external reported beside v6 (no
pooling); OSMO pods as the compute substrate (record pod GPU model).

### Affected packages

| Package | Change |
|---------|--------|
| `agent_study/runner/*` (private) | host-mode generalization, external transform, config-driven arms |
| `agent_study/config/study_config_v7_external.yaml` | new campaign config |
| `agent_study/osmo/` | pod image, workflows, orchestrator, provisioning |
| `agent_study/agent_study_protocol.md` | Amendment 4 (draft) |

## 3. Test Plan

### (a) Judge-CLI cross-distro interop

- **What is run:** Humble publisher container (`aerostack2/nightly-humble`) on
  host network; observers: Jazzy CLI container vs Humble CLI container
  (`topic list -t`, `echo --once --csv`, `hz`, `node list`, `node info`).
- **What is measured:** which observer sees topics/nodes/types.
- **Pass criteria:** the chosen judge CLI passes all five calls.

### (b) Runner smoke (no tokens, no GPU)

- **What is run:** `run_trial.py --config v7 --arm E1|E2 --agent mock:noop --smoke`
  on this box; A3 smoke unchanged (regression).
- **What is measured:** workspace layout, `ENVIRONMENT.md`, `provided/`,
  judge shim exports, results.json schema, config/prompt hashes.
- **Pass criteria:** prompt_sha256 equals v6's; both arms produce valid
  results.json; A3 smoke byte-identical on invariant fields to before.

### (c) Pod image + OSMO pilot pod

- **What is run:** build/push pod image; submit one pod with a mock agent;
  verify dockerd (overlay2), GPU visible to inner containers, judge-cli
  `ros2` wrapper, rsync in/out, DONE marker, cancel.
- **Pass criteria:** end-to-end mock trial archived on this box.

### (d) Provisioning

- **What is run:** provisioning workflow; UAS `make images` + push; AS2
  image mirror; measure image sizes and pull time in a trial pod.
- **Pass criteria:** trial pod has all UAS `unified_autonomy:*` tags and
  the AS2 image before the agent starts; provisioning time recorded.

### (e) Pilot real trial

- **What is run:** one E2 (AS2) sonnet-5 trial end to end on OSMO.
- **Pass criteria:** transcript, judge log, scoring pass complete; any
  runner/infra defects fixed BEFORE the batch (rule 4: infra failures are
  rerun, never scored).

### (f) Trial matrix

- **What is run:** 20 trials round-robin (E1:son, E2:opus, E1:opus, E2:son, …)
  across ≤3–4 pods.
- **What is measured:** highest rung, judge invocations, wall-clock,
  tokens/cost, per-trial disposition.
- **Pass criteria:** ≥5 scored trials per cell.

### (g) Analysis

- **What is run:** extend `011/results/d-analysis/analysis.py` to load v7
  results (separate campaign, own hashes) and render survival curves /
  table rows for E1/E2 beside v6 arms.
- **Pass criteria:** figures + table regenerated; numbers cross-checked
  against hand tallies.
