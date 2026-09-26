# Learned embodiment architecture: RRM-EM

Status: **proposed research architecture**. This document describes the intended
evolution of RRM; it does not claim that online embodiment learning or cross-robot
transfer is implemented today. The current verified implementation scope remains in
[goal-to-finish status](scrum-8/end-to-end-status.md).

## Purpose

RRM reasoning remains independent of robot product names, controller coordinates and
motor commands, while still reasoning about the capabilities and uncertainty of the
body available for a task. The embodiment subsystem separates a reasoning error from
an adapter, controller, simulator or hardware error and returns structured evidence to
the reasoner.

```text
                              evidence and outcomes
                     +--------------------------------+
                     |                                |
                     v                                |
Goal -> RRM <-> RRM-EM -> RRM-Adapt -> control backend -> robot/world
          |        ^          |              |               |
          |        +----------+--------------+---------------+
          |             capability-model updates
          |
          +--------- safety and authorization --------->
```

`RRM-EM` and safety are not merely two more sequential pipeline stages. RRM-EM is a
queried, continuously updated body model. Safety is an independent authority across
reasoning, adaptation, control and hardware execution.

## Responsibilities

### RRM: embodiment-neutral task reasoning

RRM interprets goals, maintains task/world belief, selects semantic operations and
compares expected effects with observations. It may request capabilities such as
`NAVIGATE_TO`, `INSPECT`, `TAKEOFF`, `PICK` or `PLACE`; it does not emit rotor speeds,
joint targets or controller gains.

Embodiment-neutral does not mean embodiment-ignorant. Planning queries RRM-EM for the
current body's feasibility, uncertainty, cost and expected outcomes.

### RRM-EM: learned embodiment model

RRM-EM represents what this particular body can do under current conditions. A
capability record should include:

- semantic operation and parameter domain;
- preconditions, resources and hard limits;
- predicted outcome distribution and confidence;
- expected duration, accuracy and resource cost;
- controller/adapter binding and qualification revision;
- environment, payload, damage and configuration context;
- evidence counts, provenance and last update.

Rejected attempts update knowledge about admission constraints. Executed attempts add
physical outcome evidence. Independently verified effects—not an action server's
success flag alone—update capability accuracy and success estimates. Unknown outcomes
remain unknown rather than being converted into negative training labels.

Online learning may update calibrated predictions and task-level beliefs. It may not
silently rewrite hard safety limits, permissions, action semantics or qualified
controller bindings.

### RRM-Adapt: morphology and platform adapter

RRM-Adapt compiles semantic operations into typed actions for a concrete embodiment.
A library can supply templates such as aerial multirotor, wheeled base, quadruped,
humanoid, manipulator or mobile manipulator. A platform plugin then binds the template
to the actual ROS actions, frames, sensors, limits and controllers.

URDF/SRDF, the ROS graph and controller metadata can assist discovery, but discovery
does not grant execution authority. Imported or generated adapters require bounded
qualification evidence before safety may admit their actions.

### Control backend

The control backend owns real-time motion. It may be PX4/AirStack, Nav2, MoveIt, a
model-predictive controller, an RL policy, or a qualified hybrid. Isaac Sim supplies a
simulated physical plant; in deployment the physical robot is the plant. Where
possible, the same controller and action interface operate against both.

Classical controller structures can transfer within a robot family, but dynamics,
gains, geometry and limits remain platform-specific. A practical hybrid keeps a
qualified classical inner loop while using learning for system identification,
residual correction, skill selection or bounded policies.

### Safety and authorization

Safety is cross-cutting and must not depend solely on the model proposing an action:

1. semantic policy decides whether the task may be attempted;
2. capability admission checks whether this body may attempt this exact action now;
3. runtime monitors enforce position, velocity, force, altitude and timing envelopes;
4. controller/hardware failsafes preserve stop, hold or landing behavior when upstream
   reasoning or communication fails.

Learned risk estimates may make admission more conservative. They are not the only
barrier protecting motion.

## Evidence loop

Every attempt should retain the proposed semantic action, capability/model revision,
admission verdict and reason, exact adapter binding, controller revision, pre/post
observations, safety interventions, physical outcome, semantic effect verdict and
uncertainty. This supports both failure attribution and later model training.

The lifecycle is:

```text
understand goal -> observe world/body -> query capability model -> plan
-> compile for embodiment -> admit -> execute -> observe -> verify effect
-> update embodiment evidence -> continue, replan, recover or halt
```

## Current realization and missing work

| Concern | Current repository state | Needed for the proposed architecture |
| --- | --- | --- |
| Semantic goals | Narrow goal contracts and deterministic aerial command compiler | General learned reasoning over the same contracts |
| Capability declaration | Static profiles plus live ROS action discovery | Calibrated, context-dependent learned capability model |
| Adapter | AirStack task actions and bounded hand prototypes | Qualified morphology library and additional robot plugins |
| Outcome evidence | Fresh action/odometry/state verification and immutable artifacts | Training-ready normalized cross-embodiment evidence |
| Control | Existing AirStack/PX4 classical stack; isolated hand gateway work | Evaluated classical, adaptive and learned backend choices |
| Safety | Admission, stop/recovery and execution checks at several boundaries | Independent deployment and real-world qualification |
| Generalization | Contract-level/profile tests | Repeated cross-robot experiments and strong baselines |

The research hypothesis is therefore not that one controller fits every body. It is
that an embodiment-neutral reasoner can learn a calibrated body model from execution
evidence and reuse its reasoning through qualified robot-specific adapters and hybrid
control backends.

## Documentation authority

- This document is the proposed RRM-EM direction.
- [SCRUM-8 architecture](scrum-8/architecture.md) remains the current requirement and
  authority allocation.
- [Interfaces](scrum-8/interfaces.md) defines the current logical contracts.
- [Goal-to-finish status](scrum-8/end-to-end-status.md) records measured implementation
  status and must constrain all demo or publication claims.
- [Original RRM-1 architecture](architecture.md) is a historical prototype design and
  is retained for provenance rather than treated as current operational truth.
