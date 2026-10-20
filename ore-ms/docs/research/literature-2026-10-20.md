# Literature grounding for EXP-016

Inspected 2026-10-20 UTC. Branch `ore_ms`, base `3ac07f047`. Focused methods reading; not a systematic novelty review.

## MAC-VO

Qiu et al., *MAC-VO: Metrics-aware Covariance for Learning-based Stereo Visual Odometry*, arXiv:2409.09479v2, March 12, 2025.
[Official paper](https://arxiv.org/html/2409.09479v2).

Sections III-A/Eq.1 train diagonal 2D flow covariance with a Gaussian negative-log-likelihood objective using ground-truth flow. III-B uses uncertainty in feature selection. III-C and Appendix A derive depth-dependent 3D covariance and cross-axis terms. Therefore, our raw 2D trace is not its full spatial covariance. Reprojection mixes depth and matching error; epipolar disagreement also includes reference/calibration error. Correlation does not establish covariance scale calibration. The current local disparity backend is documented in EXP-007/008 and must not be described as identical to the paper's original 3D objective. EXP-016's context controls are our diagnostic choice, not a protocol prescribed by this paper.

## MACThermal

Yifei Liu, *3D Thermal Perception for Autonomous Navigation in Visually Degraded, Unstructured Environments*, CMU-RI-TR-25-103, December 2025.
[Official thesis](https://publications.ri.cmu.edu/storage/publications/2025/12/YifeiLiu_CMU_MSR_Thesis_ThermalOffroadAutonomy.pdf).

Section2.1 (printed pp.7–8; PDF pp.20–21) specifies 1st/99th-percentile binding, CLAHE with 8×8 tiles and clip limit2, and bilateral filtering. Section3.1 adds training-time dynamic-range augmentation. Section3.3 (pp.26–27; PDF39–40) describes FlowFormer/covariance fine-tuning with TartanAirV2 forest and TartanDrive2.5T data. Section3.5 (pp.27–28; PDF40–41) reports optical-flow accuracy and one-standard-deviation coverage, a different target from our geometric proxies. These details provide a preprocessing audit checklist, not proof that supplied uint8 images or released weights reproduce the thesis recipe/split. Coverage alone is insufficient to establish calibration at all confidence levels. No cross-modal covariance comparability claim is inferred.

## Reproducibility and scope

The official85-page thesis was downloaded and parsed locally with pypdf6.1.3. PDF SHA256: `91d680477b21d2292e81877df8ed05dfef0dbe829784857baa1581add645d6a4`,17,420,576 bytes. Local ignored source metadata records retrieval time. Copyrighted full PDF/text are excluded from our Box snapshot; recover from the official URL. Methods sections above were inspected; no full-thesis replication is claimed.

Follow-up: trace the supplied thermal processed topic's converter to this recipe; obtain checkpoint split provenance. Keep original images unchanged during EXP-016. MSO and MAC-I² remain separate literature leads; neither was evaluated for novelty in this focused reading.
