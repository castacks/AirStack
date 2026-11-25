# Research roadmap mandate

- **Experiment** ORE-MS-EXP-PER-001
- **Project** ore-ms-research
- **Research** Predictive Multispectral Perception in Stochastic Degrading Environments
- **Project Director and Chief Systems Architect** ChatGPT
- **Software development and IV&V** Codex

Amendment A1 governs execution order, milestone authorization, and reporting. The original research specification remains the scientific authority. Future handoffs, implementation plans, milestone reports and acceptance records must reference this mandate.

### 2. Required milestones and phases

| Milestone | Title | Primary objective |
|---|---|---|
| M01 | Existing Capability Audit and Deterministic Physics Baseline | Audit existing software, establish mathematical foundations, and implement verified RGB/LWIR baseline models. |
| M02 | RGB Environmental Degradation Modeling | Model visible-light attenuation, scattering, illumination changes, noise, and RGB image degradation. |
| M03 | Thermal Radiative Transfer Modeling | Model LWIR radiation, emissivity, atmospheric transmission, thermal contrast, and sensor response. |
| M04 | Geometric and Dynamic Occlusion | Model static and moving obstacles, visibility masks, partial occlusion, and line-of-sight obstruction. |
| M05 | Stochastic Environmental Evolution | Model randomly evolving fog, dust, smoke, rain, temperature, and environmental disturbances using stochastic processes. |
| M06 | Combined Multispectral Simulation | Integrate RGB, thermal, occlusion, and environmental degradation into a unified simulation framework. |
| M07 | Environmental State Estimation | Estimate hidden environmental variables and degradation parameters from sensor observations. |
| M08 | Classical Probabilistic Prediction | Predict future environmental conditions and sensor degradation using Bayesian and statistical methods. |
| M09 | Physics-Informed Machine Learning | Develop learning models that combine physical equations with data-driven prediction. |
| M10 | Adaptive Multispectral Sensor Fusion | Dynamically combine RGB and thermal observations based on predicted sensor reliability and uncertainty. |
| M11 | Independent Simulator Verification | Verify mathematical correctness, physical accuracy, numerical stability, and reproducibility through independent simulation comparisons. |
| M12 | Physical Sensor Verification | Validate simulation predictions against controlled experiments using physical RGB and thermal sensors. |
| M13 | ORE-MS Predictive World-Model Integration | Integrate environmental observability, sensor-quality prediction, and uncertainty-aware reasoning into the ORE-MS predictive world model. |

### 3. Research execution phases

**Phase I — Scientific Foundations (M01–M04)**

Mathematics, optical physics, thermal radiation, and geometric/dynamic occlusion modeling.

**Phase II — Stochastic Environmental Simulation (M05–M06)**

Random environmental dynamics and combined multispectral simulation. Phase II is conditionally complete under M07-R1 (2026-11-25).

**Phase III — Predictive Intelligence (M07–M10)**

Environmental state estimation, probabilistic forecasting, physics-informed machine learning, and uncertainty-aware fusion.

**Phase IV — Verification and Physical Validation (M11–M12)**

Independent simulator comparison and physical sensor validation.

**Phase V — World-Model Integration (M13)**

Integration of predictive environmental observability and sensor-quality reasoning into the ORE-MS research architecture.

### 4. Work breakdown and traceability

The program retains 25 mathematics, 30 physics, 30 machine learning and 20 systems integration tasks. The [105-task catalog](14_WORK_BREAKDOWN_STRUCTURE.csv) links each task to a scientific objective, milestone, status, verification method and evidence. The [dependency table](17_TASK_DEPENDENCIES.csv) retains predecessor relationships. M01 planning tasks retain their planning scope. Future implementation requires explicit approval for the relevant milestone.

### 5. Milestone authorization policy

**Current execution authorization: M08 only.**

M09–M13 are pending.

Codex shall not begin implementation of a subsequent milestone without explicit authorization from the Project Director.

For every milestone, the required progression is:

1. Define requirements and scientific acceptance criteria.
2. Implement the authorized scope.
3. Execute numerical or physical experiments as appropriate.
4. Measure results against defined references and baselines.
5. Perform documented verification and validation.
6. Submit an acceptance package.
7. Await Project Director authorization before proceeding.

A milestone is not complete merely because code executes or automated tests pass.

### 6. Mandatory anti-drift rules

Codex shall:

- Follow the original scientific objectives and the approved milestone sequence.
- Avoid substituting existing AirStack research priorities for this experiment's objectives.
- Avoid introducing unrelated capabilities, algorithms, datasets, or integration requirements.
- Establish deterministic physics before stochastic environmental models.
- Establish classical statistical and probabilistic baselines before machine learning.
- Preserve the distinction between physical ground truth, sensor observations, estimated states, and predicted future sensor quality.
- Prefer mature scientific software where technically justified.
- Maintain explicit physical units, assumptions, validity domains, numerical tolerances, and uncertainty.
- Preserve reproducible experimental evidence and negative results.
- Clearly distinguish implemented, verified, planned, and experimentally unvalidated capabilities.
- Escalate proposed scope changes before implementation.

The governing research principles are:

**Approximate the solution, not the problem.**

**Execute → Measure → Verify → Improve → Scale.**

### 7. Milestone status register

| Milestone | Authorization | Execution status | Requirements | Deliverables | Verification | Open issues | Acceptance decision |
|---|---|---|---|---|---|---|---|
| M01 | Authorized | Accepted | [R01–R08](02_SYSTEM_REQUIREMENTS.md) | [M01 evidence and mapping](M01_ACCEPTANCE_REPORT.md) | [G0–G6](24_ACCEPTANCE_GATES.md) | External review and physical validation pending | Conditionally accepted under M03-R1, recorded 2026-11-25 |
| M02 | Authorized | Accepted | [M02 requirements](m02/REQUIREMENTS_TRACEABILITY.csv) | [RGB degradation model and evidence](m02/ACCEPTANCE_REPORT.md) | [V01–V06 passed](m02/VERIFICATION_REPORT.md) | Physical spectral response and multiple-scattering validity remain open | Conditionally accepted under M03-R1, recorded 2026-11-25 |
| M03 | Authorized | Accepted | [M03 requirements](m03/REQUIREMENTS_TRACEABILITY.csv) | [Thermal model and evidence](m03/ACCEPTANCE_REPORT.md) | [V01–V07 passed](m03/VERIFICATION_REPORT.md) | Physical response, atmospheric coefficients and external review open | Conditionally accepted under M04-R1, recorded 2026-11-25 |
| M04 | Authorized | Accepted | [M04 requirements](m04/REQUIREMENTS_TRACEABILITY.csv) | [Geometry and visibility evidence](m04/ACCEPTANCE_REPORT.md) | [V01–V08 passed](m04/VERIFICATION_REPORT.md) | Physical camera and complex geometry validation pending | Conditionally accepted under M05-R1, recorded 2026-11-25 |
| M05 | Authorized | Accepted | [M05 requirements](m05/REQUIREMENTS_TRACEABILITY.csv) | [Stochastic evolution evidence](m05/ACCEPTANCE_REPORT.md) | [V01–V09 passed](m05/VERIFICATION_REPORT.md) | [Classified acceptance conditions](ACCEPTANCE_CONDITIONS_REGISTER.md) | Conditionally accepted under M06-R1, recorded 2026-11-25 |
| M06 | Authorized | Accepted | [M06 requirements](m06/REQUIREMENTS_TRACEABILITY.csv) | [Paired dataset and simulation evidence](m06/ACCEPTANCE_REPORT.md) | [V01–V10 passed](m06/VERIFICATION_REPORT.md) | [Classified acceptance conditions](ACCEPTANCE_CONDITIONS_REGISTER.md) | Conditionally accepted under M07-R1, recorded 2026-11-25 |
| M07 | Authorized | Accepted | [M07 requirements](m07/REQUIREMENTS_TRACEABILITY.csv) | [Conditional estimation evidence](m07/ACCEPTANCE_REPORT.md) | [V01–V07 and V09–V10 passed, V08 not tested](m07/VERIFICATION_REPORT.md) | AC10 open for unconstrained nuisance identification | Conditionally accepted under M08-R1, recorded 2026-11-25 |
| M08 | Authorized | Verification | [M08 requirements](m08/REQUIREMENTS_TRACEABILITY.csv) | [Classical forecast evidence](m08/ACCEPTANCE_REPORT.md) | [V01–V10 passed](m08/VERIFICATION_REPORT.md) | [Classified acceptance conditions](ACCEPTANCE_CONDITIONS_REGISTER.md) | Director review pending |
| M09 | Pending | Not Started | Physics-informed learning | Matched-input learned and hybrid models | Planned: Ablations, leakage audits and interval coverage | Training data readiness | Pending authorization |
| M10 | Pending | Not Started | Reliability-adaptive fusion | Fusion policy and sensor baselines | Planned: Matched accuracy, availability and measured cost | Failure correlations and abstention | Pending authorization |
| M11 | Pending | Not Started | Independent simulator verification | Independent cross-simulator report | Planned: Numerical and physical consistency checks | Reference simulator and models | Pending authorization |
| M12 | Pending | Not Started | Physical sensor verification | Calibrated sensor recordings and transfer report | Planned: Controlled exposures and residual uncertainty | Sensor and physical reference availability | Pending authorization |
| M13 | Pending | Not Started | Predictive world-model integration | World-model interface and qualification packet | Planned: End-to-end trace and deployment checks | Interface and qualified evidence readiness | Pending authorization |

### 8. Mandatory pre-task alignment check

Before starting any future implementation task, Codex must confirm:

1. Which milestone authorizes the work?
2. Which original research requirement does it satisfy?
3. Which of the 105 tasks does it address?
4. What measurable scientific result is expected?
5. What verification method will establish correctness?
6. Does the work introduce any unapproved scope?

If authorization or traceability is missing, Codex must stop and request clarification.

### 9. Change control

The roadmap is a controlled research governance document.

Codex may update factual progress, artifact references, verification status, and unresolved issues.

Codex may not independently alter milestone objectives, scientific requirements, task scope, milestone ordering, or acceptance criteria.

Any proposed change must include:

- Proposed modification.
- Scientific or engineering justification.
- Affected milestones and tasks.
- Impact on dependencies, experiments, and verification.
- Project Director approval status.

Until approved, the original roadmap remains in force.

### 10. Acceptance conditions governance

**ACCEPT WITH CONDITIONS** means the authorized milestone passed its acceptance criteria and any outstanding conditions are classified and tracked. The [Acceptance Conditions Register](ACCEPTANCE_CONDITIONS_REGISTER.md) records classification, originating milestone, responsible owner, target milestone or gate, closure evidence and progression impact for each item.

- **Required acceptance obligation** applies before the named scientific claim, integration gate or milestone acceptance.
- **Deferred validation** is required verification assigned to an explicit later milestone. M11 covers independent simulator verification and M12 covers physical sensor verification.
- **Optional future extension** is outside accepted scope and applies only if separately authorized or needed for a later claim.

Organizationally independent assurance has its own review gate. Procedural separation within Codex is numerical verification evidence, with organizational independence still open. This amendment preserves all historical reports, numerical results, protocols and manifests.
