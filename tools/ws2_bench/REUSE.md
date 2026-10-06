# Reused implementation

`ravi_metrics.py` and `ravi_reporting.py` were initially copied from
FloareDor/AirStack `ravi/automated-testbench`, commit
`a8adef094872bddc40053e534a62d5f2cb74a4a1`, via the local `ws2_offline` snapshot.
The bridge disturbances and Office variant generator also originate there.
The original snapshot and repository license are preserved in the workspace.
Local reporting now recognizes the reactive planner's completed-horizon success,
excludes operator stops from performance rates, and includes duration, motion and
mean-clearance metrics. These reporting extensions are not unchanged upstream code.

The local runtime adapter reuses these metric/report contracts and the same
scenario → isolated execution → independent scoring boundary. Its lifecycle
targets the existing v0.18 containers, because Ravi's newer modular bringup is
not compatible with this checkout. The Office oracle extends evaluation to
all external PhysX contacts and scene-mesh clearance, replacing three-box-only
geometry scoring for this scene. WS1's existing scenario/flight/ROS bag workflow
is retained; planner policies and checkpoints are not retrained.

## Optional LLM integration (2026-10-06)

The saved-layout adaptive campaign, policy and action-schema components originate
from FloareDor/AirStack `mason/adv-ws2`, commit
`4366377557f984797e7f7b8cebe0f95f35ff8cc0`. This is a selective source integration
over castacks/AirStack `35fc5646`, not a merge of server deployment settings or
committed experiment artifacts. Local worker images, launch arguments, missions,
speeds and headless observer rendering are retained.

Local additions connect official Claude Code Team authentication, validate exact
LLM-selected actions and produce a separate evidence-linked final interpretation.
Existing Gemini-compatible intent selection remains optional. The viewer exposes
both these modes and the existing LLM-free feedback path. Runtime mounts are
checked before starting containers; no credentials or downloaded models are copied.
