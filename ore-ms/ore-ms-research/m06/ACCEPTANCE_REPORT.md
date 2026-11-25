# M06 acceptance report

- **Experiment** ORE-MS-EXP-PER-001
- **Directive** M06-R1
- **Recommendation** ACCEPT WITH CONDITIONS. All ten frozen verification criteria passed. Project Director decision remains pending.

The generator joins M04 first-hit geometry and M05 environmental snapshots to the unchanged M02 and M03 radiometry. It maps heterogeneous paths into ordered layers and emits synchronized RGB and LWIR components, detector observations, ground truth and hashes. Both modalities share one ideal pinhole and frame timestamp. The [scientific specification](SCIENTIFIC_MODEL.md) and [requirements traceability](REQUIREMENTS_TRACEABILITY.csv) describe these assumptions and the qualification limits.

The corpus includes six layouts, 15 full-frame control cases, two probe counterfactual arms, 128 independent stochastic realizations, 793 frames, and 2,280 valid paired observations. The 63 no-hit rows are explicit. First-hit and timestamp labels had zero discrepancies. The largest 16-segment heterogeneous optical-depth error was 3.209e−5. Changing a fully occluded target left both spectral observations unchanged, and the thermal crossover preserved RGB contrast and geometric visibility. The [dataset qualification](DATASET_QUALIFICATION.md), [joint analysis](JOINT_ANALYSIS.md), and [verification](VERIFICATION_REPORT.md) report coverage, dependence and uncertainty.

All frozen M01–M05 replays passed. AC13 closed under M06-R1 while real-sensor, atmospheric, simulator and independent assurance gates remain open. See the [live acceptance register](../ACCEPTANCE_CONDITIONS_REGISTER.md) and [roadmap mandate](../RESEARCH_ROADMAP_MANDATE.md). M07 requires explicit authorization. No later capability was started.
