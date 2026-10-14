# MS Research Handoff — Cross-Modal Geometric Uncertainty for RGB–Thermal Visual Odometry

**Status:** Research direction approved by Basti in October 2026. Previous integration and respiratory-rate work is complete.  
**Program:** CMU MS ECE, Intensive Project Option.  
**Research window:** Fall 2026 (12 units) and Spring 2027 (24 units); expected completion May 2027.  
**Document role:** Living research context for an AI coding/research assistant. Update the progress log, decisions, and experiments as evidence accumulates.  
**Last contextual update:** 2026-10-11.

> **Source boundaries:** The technical formulation below is based on the attached three-page Overleaf PDF, *Cross-Modal Uncertainty and Relative Geometric Reliability*, plus decisions and context stated in our conversations. The linked Google Doc has **not** been inspected. Literature descriptions and links are orientation pointers, **not** a completed or verified systematic literature review. Do not represent proposed hypotheses as established findings.

## 1. Project in one paragraph

I am investigating whether geometric uncertainty estimates from RGB and thermal sensing are meaningfully comparable across modalities and predictive of **relative actual geometric reliability**, especially under changing sensing conditions. Existing uncertainty-aware VO can already use measurement covariance for feature weighting and optimization; merely adding adaptive weighting is not my novelty claim. I will first study how predicted RGB/thermal uncertainties relate to each other **and** to their respective measured geometric errors. If this relationship contains predictive information beyond conventional modality-specific weighting, I will investigate whether explicit cross-modal reliability modeling/calibration improves VO pose accuracy and robustness. The direction is approved, but specific implementation choices remain experimental.

## 2. Core research question and hypothesis

**Question:** Are modality-specific geometric uncertainty estimates cross-modally comparable and predictive of relative geometric reliability between RGB and thermal, and can exploiting that information improve visual odometry?

Define:
- `U_R`, `U_T`: predicted RGB and thermal geometric uncertainties, respectively.
- `E_R`, `E_T`: corresponding observed geometric errors, measured using ground truth or a justified reference.
- `C`: optional environmental/temporal context.

The core test is whether a relationship `f(U_R, U_T)` predicts relative error `g(E_R, E_T)`, for example whether `U_R < U_T` predicts `E_R < E_T` **when uncertainty quantities are comparable in meaning, coordinates, and units**. Ratios such as `U_R/U_T` versus `E_R/E_T` are illustrative, not a required method; full covariance matrices cannot generally be compared by naive scalar division.

Possible probabilistic formulation:

`h(U_R, U_T, C) -> P(E_R < E_T)`

**Hypothesis, not fact:** A calibrated/context-aware cross-modal reliability model may predict which modality supplies better geometric constraints and improve VO when used for measurement selection/weighting.

## 3. How this differs from related work

- **MAC-VO:** learned metrics-aware covariance for stereo visual odometry; uncertainty informs geometric estimation and downstream VO.
- **Thermal MAC-VO / MACThermal:** thermal perception/VO building on related geometric-uncertainty ideas.
- **MSO (CMU MRSD Team G):** multispectral-inertial odometry with uncertainty-driven RGB/thermal measurement use. Public summaries discussed suggest per-feature covariance and adaptive factor-graph weighting. **Do not assert that MSO never examines cross-modal comparability without reading its implementation/paper closely.**
- **MAC-I² / visual–inertial work:** learned metrics-aware covariance for visual–inertial fusion; reinforces that uncertainty-aware multimodal weighting is established.

**Novelty boundary:** Not “using uncertainty to weight modalities.” Potentially novel is a systematic, grounded test of cross-modal **calibration, comparability, relative-error prediction**, and an adaptive method that improves upon a strong existing uncertainty-weighted baseline. Whether this is novel must be established through literature review and experiments.

## 4. Research stages and decision gates

### Fall 2026 — characterization and feasibility (12 units)

1. **Baseline reproduction and instrumentation.** Identify runnable MAC-VO / thermal or RGB–thermal pipelines; inspect uncertainty definitions, frame alignment, outputs, calibration, dataset licensing/access, and available reference poses/errors.
2. **Define comparable measurements.** Choose geometric targets and coordinate systems (e.g., feature reprojection, correspondence error, depth, relative pose). Decide how covariance is propagated/normalized; avoid comparing unlike quantities.
3. **Modality-specific calibration.** Evaluate `U_R` vs `E_R` and `U_T` vs `E_T` using appropriate calibration and predictive metrics.
4. **Cross-modal analysis.** Test whether `f(U_R,U_T)` predicts relative errors or reliable modality rankings across conditions and time; distinguish co-variation from useful predictive signal.
5. **Fall decision gate.** Decide whether (a) raw uncertainty is already comparable, (b) cross-modal calibration is needed, or (c) current uncertainty is weakly informative and the research approach needs refinement.

**Fall deliverable:** Reproducible baseline, aligned uncertainty/error dataset, initial plots and quantitative evidence, failure analysis, and a defensible Spring plan. Avoid promising an adaptive VO implementation before the signal is validated.

### Spring 2027 — exploitation and evaluation (24 units; contingent)

1. Design an interpretable or learned cross-modal reliability model only if Fall evidence supports it.
2. Integrate the reliability signal into measurement selection/weighting in the VO pipeline.
3. Compare against RGB-only, thermal-only, fixed fusion, and strong existing uncertainty-weighted multimodal fusion where reproducible.
4. Evaluate pose accuracy and robustness (e.g., ATE, RPE, drift, tracking failures, recovery, latency) under diverse conditions.
5. Run ablations to isolate the contribution of calibration, cross-modal inputs, temporal context, and adaptation.
6. Produce final MS report, reproducible code/results, and optionally a paper draft if findings justify it.

**Fallback:** If relative uncertainty is not predictive, document the negative result and reassess whether a better uncertainty representation, calibration strategy, or narrower reliability question is justified. Do not force a positive fusion result.

## 5. First experiment — draft, to refine with Basti

**Objective:** Test whether the uncertainty ranking of RGB and thermal measurements predicts the ranking of their actual geometric errors.

**Inputs needed:** Synchronized and calibrated RGB/thermal sequences; model uncertainty outputs; matched geometric measurement definitions; reference data adequate to estimate error; condition labels if available.

**Procedure:**
1. Run baseline(s) on a small set of sequences with diverse lighting/thermal conditions.
2. Record per-measurement or per-frame uncertainty, geometric error, timestamp, modality, and condition.
3. Validate that uncertainties are in commensurate coordinates/units or transform/calibrate them before direct comparison.
4. Evaluate each modality's uncertainty-to-error calibration.
5. Measure relative-error ranking performance from raw uncertainty and simple calibrated variants; stratify by condition.
6. Assess temporal stability, failure regimes, and whether the relationship adds predictive value beyond modality-specific uncertainty alone.

**Candidate measurements:** Rank accuracy/AUROC for predicting `E_R < E_T`, reliability/calibration curves, negative log likelihood or Mahalanobis consistency when justified by the uncertainty model, and condition-specific error analyses. The exact metric depends on the form of uncertainty and geometric target. Prevent leakage by splitting by sequence/environment rather than adjacent frames only.

**Important cautions:** Pose-level error is not the same as feature-level error. RGB and thermal features may not have one-to-one correspondence. A smaller covariance is not inherently better unless estimates share meaning/units and are calibrated. Association and ground-truth quality can dominate conclusions.

## 6. Codebase operating instructions for an LLM collaborator

- Treat this file as a **living project brief**, not proof that an experiment has been run.
- Inspect repository structure, READMEs, configs, and existing experiments before proposing code changes.
- Never invent datasets, GPU access, baselines, experiment results, advisor decisions, or paper claims.
- Keep research hypotheses separate from measured findings.
- Favor minimal, reproducible experiments over broad architecture changes.
- Record exact commands, commits, configs, seeds, dataset versions/splits, hardware, and output paths for every result.
- Do not assume the full AirStack robotics simulation stack is required for VO research. AirLab AirStack runs on OSMO; PSC is an optional compute alternative. Select infrastructure based on accessible GPUs, dependencies, data locality, storage persistence, and scheduling.
- Ask before committing to a major methodological pivot or extensive compute job.

### Suggested repository structure (adapt to existing layout)

```text
research/
  MS_RESEARCH_HANDOFF.md       # this living brief
  literature.md               # verified paper notes and novelty matrix
  decisions.md                # dated decisions / advisor feedback
  experiments/
    README.md                 # index of experiments and reproducibility
    EXP-001.md                # hypothesis, setup, metrics, results, next steps
  progress.md                 # weekly accomplishments and blockers
  datasets.md                 # datasets, access, calibration, splits, provenance
```

## 7. Initial reference list (verify bibliographic details)

**Directly relevant**
- MAC-VO — *Metrics-aware Covariance for Learning-based Stereo Visual Odometry*: https://arxiv.org/abs/2409.09479 ; project https://mac-vo.github.io/
- MACThermal — Yifei Liu, *3D Thermal Perception for Autonomous Navigation in Visually Degraded, Unstructured Environments* (CMU thesis, 2025). Locate/verify official thesis PDF.
- MSO — CMU MRSD 2025 Team G, system implementation: https://mrsdprojects.ri.cmu.edu/2025teamg/system-implementation/
- MAC-I² — *Learned Metrics-Aware Covariance for Robust Visual-Inertial Fusion in Initialization and Calibration*: https://arxiv.org/abs/2609.07116
- CMU visual–inertial thesis/publication: https://publications.ri.cmu.edu/learned-metrics-aware-covariance-for-visual-inertial-fusion

**Related research leads (not all verified as formal papers)**
- TartanMatch — *Towards Universal Dense Correspondence Across Modalities*: https://caijiting.github.io/ (previously described as in submission)
- AnyThermal / TartanRGBT: https://www.parvmaheshwari.com/
- Super Odometry: https://theairlab.org/superodom/
- Resilient Odometry via Hierarchical Adaptation (Science Robotics, 2025): verify publication record and full citation.
- External adaptive RGB–thermal and uncertainty-aware sensor-fusion papers discussed previously: exact bibliographic metadata needs verification.

**Historical proposal context:** Original research interest was passive multimodal depth estimation (RGB/NIR/temporal information, adverse conditions, theoretical precision limits). This is background, **not the currently approved MS scope**.

## 8. Tentative longer-term PhD direction — not a commitment

If MS findings justify it, the broader question could be how heterogeneous sensors (RGB, thermal, event, depth, LiDAR, IMU) can express and compare reliability in a principled way for adaptive state estimation, SLAM, and autonomous navigation. This is a **possible extension only**, not an MS requirement or established PhD plan.

## 9. Current state and next actions

**Confirmed:** Basti approved the MS direction in October 2026. Earlier integration/respiratory-rate work is complete. The research question is established; experimental details may still be refined.

**Not yet confirmed in this handoff:** Specific repo, available code, datasets, synchronized camera calibration, access credentials, baseline reproducibility, initial results, hardware quotas, or precise advisor-approved experimental design. The Google Doc shared in chat was not inspected.

**Resource inventory confirmed 2026-10-13 (from Geethika Hemkumar and Jerry Hou):**
- Processed TartanDrive 3 data is at AirLab storage path `/offroad/datasets/tartandrive3`; prioritize sequences `20251027` and `20251106` for currently usable stereo-RGB plus stereo-thermal data. Raw ROS bags are at `/offroad/rosbags`.
- The public `castacks/ros_torch_converter` provides ROS-bag conversion and a `get_slam_data.py --depth --odom` path for extracting projected depth and camera poses. Extracted directories contain `timestamps.txt`; camera intrinsics are in `_camera_info` / `_intrinsics` directories. Extrinsics are supplied through AirLab's static-TF YAML and should be obtained through the authorized AirLab source rather than recreated manually.
- MACThermal is reported to use the MAC-VO FlowFormer architecture with a fine-tuned checkpoint, not a distinct architecture. The known checkpoint bundle is `macslam.zip` in the shared CMU Box folder. Jerry reports no known updates after Yifei Liu's thesis; its exact training subset remains unverified. On one indoor dataset, the released MACThermal weight showed little improvement over default MAC-VO, and separate in-house fine-tuning is underway. Treat these as operational reports, not benchmark conclusions.

**Next practical steps:**
1. Inventory codebases, datasets, compute environments, and baseline reproducibility.
2. Agree on the geometric measurement/error definition and covariance comparability protocol.
3. Draft and run a small baseline extraction/analysis experiment.
4. Record actual findings below, then update the Fall decision gate.

## 10. Living progress log

| Date | Work completed | Evidence / links | Blockers | Next step |
|---|---|---|---|---|
| 2026-10-11 | Created initial living research handoff; direction already approved | This file and Overleaf research formulation | Baseline/data access not yet documented here | Repository inventory and first experiment definition |
| 2026-10-13 | Inventoried and unpacked `MAC-VO-main.zip` to `MAC-VO-main`; inspected the pipeline, configs, and runtime prerequisites | Local source archive; `README.md`, `Odometry/MACVO.py`, and `Module/Covariance/Project2to3.py` | No Python environment (including PyTorch), pretrained models, demo data, or initialized `FlowFormer` / `DPVO` submodules; exact MACThermal reproducibility bundle not yet located | Obtain the MACThermal code/model/data-access pointer, then reproduce the RGB MAC-VO demo in an isolated environment before adapting or comparing modalities |
| 2026-10-13 | Added verified operational pointers for TartanDrive 3, ROS-bag conversion, calibration, and MACThermal checkpoints | Messages from Geethika Hemkumar and Jerry Hou; `castacks/ros_torch_converter` README and scripts | AirLab storage is not mounted in this environment; its reachable CMU-SAML web endpoint is a Synology NAS console, so a lab-approved non-browser transfer method (mount, SFTP/rsync, or WebDAV) is still needed; Box bundle has not been accessed; MACThermal training split/provenance remains unknown | Obtain the approved AirLab-to-compute transfer method or a persistent compute mount, inspect `20251027`/`20251106` schemas and calibration, then validate a short MACThermal inference run with the released weights |
| 2026-10-13 | Prepared a disposable OSMO/Box staging path: Docker image `macvo:box` includes rclone; added secret-driven bootstrap and operator guide; pinned the Torch/xFormers/torchvision versions needed for a healthy MAC-VO import | `MAC-VO-main/Docker/Dockerfile`; `MAC-VO-main/Scripts/Setup/bootstrap_osmo.sh`; `MAC-VO-main/Scripts/Setup/README.md`; verified in the built image: rclone plus Torch 2.4.0+cu121, torchvision 0.19.0+cu121, and xFormers 0.0.27.post2 import successfully with CUDA available | One-time CMU Box SSO authorization must be completed privately by the account holder; workflow secret/volume behavior must be configured in the particular OSMO deployment; no AirLab dataset or Box file has been copied yet | Create the encrypted OSMO secret from a private Box OAuth flow, pull only `macslam.zip` first, verify its checksum/contents, then stage one selected TartanDrive 3 sequence |
| 2026-10-13 | Downloaded, validated, and extracted the shared `macslam.zip` checkpoint bundle | `/root/ore-ms/macslam.zip` SHA-256 `77ad168672019f94d8e48ebb087cb109b938bcf8784e3df480070cd5f437f00c`; extracted files: `MACT_Offroad.pth` (76.8 MB), `MACVO_FrontendCov.pth` (77.4 MB), and `MACVO_posenet.pkl` (189.4 MB) | The checkout still lacks the `FlowFormer` submodule required to instantiate the frontend. The thermal checkpoint contains weights only, not its preprocessing/training configuration. | Recover the public FlowFormer submodule, run an actual checkpoint load, then identify/reproduce thermal frame normalization and stereo data adapters before staging a TD3 sequence |

## 11. Decision log

### Git location correction

The MS source and notes are now tracked under `ore-ms/` in the AirStack
repository on the user's `ore_ms` branch. The earlier standalone repository
at `/root/ore-ms` remains a preserved session copy. Use
`/root/AirStack/ore-ms` for future edits and commit/push through AirStack.
FlowFormer and DPVO are registered in AirStack's root `.gitmodules` at their
verified revisions. Weights, archives, credentials, and local Docker images
are excluded from this migration and must be restored separately. The user
will push the AirStack branch before shutting down.

### Blackwell environment verified (2026-10-13 America/New_York)

- Added `MAC-VO-main/Docker/Dockerfile.blackwell`, using the official
  `pytorch/pytorch:2.7.1-cuda12.8-cudnn9-runtime` base pinned by digest.
  Built image `macvo:blackwell` ID:
  `sha256:19f24ed5ee40d0e4e00b74aac1a463a26572a88b04536b3b24bb9b10e193d93f`.
  Framework versions are Torch 2.7.1+cu128 and torchvision 0.22.1+cu128;
  no xFormers requirement is needed for the recovered native-attention frontend.
- Actual GPU execution now passes on NVIDIA RTX PRO 5000 Blackwell: strict
  load of all 428 MACT entries, inference at 128x128 (0.423s) and 480x640
  (0.404s), expected flow/covariance shapes, all finite outputs and positive
  covariance. Timings include first-use overhead and are not benchmarks.
  `python MACVO.py --help` passes. Full VO, accuracy, and DPVO remain untested.
- Reproduce from the repository root with:
  `docker run --rm --gpus all -v /root/ore-ms:/workspace -w /workspace/MAC-VO-main macvo:blackwell python Scripts/Setup/verify_mact_checkpoint.py --forward --height 480 --width 640`.
- No TD3 sample or `/offroad` mount is present. Box `ore_ms:` authenticates;
  its root has no TD3 staging folder. Next required data is a small synchronized
  stereo-RGB / stereo-thermal sample from `20251027` or `20251106`, timestamps,
  intrinsics, extrinsics/baseline, and any rectification metadata. Preserve raw
  thermal encoding when transferring it. The original normalization recipe
  remains unknown. Current local Docker images are ephemeral and have not
  been published to a registry.

### MACT frontend verification (2026-10-13)

- Recovered empty submodule directories using public upstream HEADs:
  `Module/Network/FlowFormer` at `c836f66e597620a2ffaaffb60693201050b93b7a`
  and `Baseline/DPVO` at `f7266f7b0b3c212001afe260d391539376df8283`.
  These are recorded upstream revisions; the ZIP has no parent Git metadata
  to establish its original gitlink revisions. DPVO's compiled extensions and
  runtime have not been tested.
- Added `MAC-VO-main/Scripts/Setup/verify_mact_checkpoint.py`. It disables
  pretrained downloads, loads with `weights_only=True`, strips the DDP
  `module.` prefix, and requires a strict state-dictionary match.
- Actual verification: `MACT_Offroad.pth` loads all 428 entries with zero
  missing/unexpected keys into a 19,157,555-parameter FlowFormerCov. Synthetic
  CPU inference at 128x128 returns finite flow and positive covariance, both
  shaped `(1, 2, 128, 128)`. This establishes code/checkpoint execution, not
  thermal accuracy or reproduction of the original preprocessing.
- GPU smoke test failed: the current Torch 2.4.0+cu121 build cannot execute
  on the RTX PRO 5000 Blackwell (`sm_120`). Earlier `cuda_available=True` was
  only device discovery, not proof of runnable GPU kernels. Use a Blackwell
  compatible build, such as Torch 2.7.x with CUDA 12.8, and rerun the script.
  The recovered FlowFormer uses native PyTorch attention and does not import
  xFormers. Reference: https://pytorch.org/blog/pytorch-2-7/ .
- Frontend expects three-channel float images in `[0,1]` and internally maps
  them to `[-1,1]`. The generic stereo loader uses `cv2.IMREAD_COLOR` and
  divides by 255; it must not be assumed to preserve raw 16-bit thermal data.
  The released bundle has no normalization recipe or dataset configuration.
  Confirm thermal encoding, normalization, rectification, timestamps,
  intrinsics, and stereo baseline on a small actual TD3 sample before writing
  its production adapter. Do not label guessed normalization as reproduced
  MACThermal preprocessing.

| Date | Decision | Reason | Revisit when |
|---|---|---|---|
| Oct 2026 | Focus MS work on cross-modal RGB–thermal geometric uncertainty and relative reliability | Approved research direction, focused two-semester scope | Evidence or advisor feedback changes scope |
| Oct 2026 | Characterize before building a new adaptive VO method | Avoid assuming uncertainty relationship is useful | Fall experimental results |

## 12. Open questions

- Which precise geometric target (feature correspondence, reprojection, depth, pose) should uncertainty predict?
- Are RGB and thermal covariances defined in the same coordinate frame and units? If not, how should they be mapped?
- What exactly do available AirLab baselines implement, and which claims about MSO are supported by code/paper?
- Which synchronized RGB–thermal datasets have adequate ground truth and diverse degradation regimes?
- What predictive value remains after controlling for each modality's own uncertainty and environmental context?
- Which compute environment (OSMO/AirStack vs PSC) minimizes engineering overhead for each workload?

---

**Handoff instruction:** When resuming this project, read this file first, then inspect repository state and recent experiment/decision logs. Summarize verified progress, unresolved questions, and the smallest useful next experiment. Do not revert the project to a pre-approval stage or treat the prior respiratory-rate/integration tasks as outstanding.
