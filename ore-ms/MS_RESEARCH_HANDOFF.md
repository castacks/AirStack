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

### Workspace resume audit — 2026-10-17 (America/New_York)

- Resumed `/root/AirStack` on branch `ore_ms`, with a clean working tree at
  `af553955e` (`Add MS RGB-thermal uncertainty project and verified MACT setup`,
  October 13 local time). The preceding project-setup commit `d9b343619`
  changed only the root ignore rules. No later experiment commits are present.
- Both project submodules are initialized at the revisions recorded below.
  NVIDIA RTX PRO 5000 Blackwell is visible, but `macvo:blackwell` is absent;
  host Python has no Torch, NumPy, or OpenCV. The checkpoint bundle and extracted
  weights are absent, as are `/offroad` and the old `/root/ore-ms` session copy.
  Historical inference results below were not rerun in this workspace.
- `rclone` is installed. A private config exists but cannot be parsed as INI
  or validated as a complete base64-encoded Box config, including after
  whitespace/padding normalization. No valid decoded config was created and
  the original was left untouched. `RCLONE_CONFIG_B64` is unset and the OSMO CLI
  is absent, so the saved OSMO credential is not available to this process.
  Restore the existing `ore-ms-box` credential through the workflow's documented
  environment mapping; do not place credentials in chat or version control.
- The active `Module/Covariance/Project2to3.py` constructs covariance in
  `(depth, horizontal, vertical)` order: its first matrix diagonal is
  `sigma_dd`, followed by the horizontal and vertical projected variances.
  Preserve and verify this convention when moving covariances to a shared
  RGB/thermal frame; raw pixel variances from different cameras are not yet
  a comparable physical uncertainty measure.

**Next smallest experiment:** restore the weights and Blackwell image, rerun
`verify_mact_checkpoint.py --forward --height 480 --width 640`, then inspect
about 10 synchronized stereo frames per modality from `20251027` or `20251106`.
Record raw image dtype/range, timestamps, intrinsics, stereo baseline/extrinsics,
and rectification metadata before choosing thermal normalization or writing a
production adapter. Run one paired frontend inference only after those input
checks, retaining preprocessing and output provenance. This is a baseline
execution/input audit; uncertainty calibration and accuracy claims require a
justified geometric error reference and a common frame/units protocol.

### Blocker recovery — 2026-10-17 (America/New_York)

- Recovered the local Box configuration: only the base64 tail/expiry was
  truncated; the Box backend and complete authentication fields were intact.
  Forced an OAuth refresh using those fields in a private alternate config;
  authentication and root listing succeeded. Installed the refreshed config
  at the default private rclone path, preserving the truncated original in
  a private backup. No credentials are stored in this repository.
- Root listing found no names matching `macslam`, `ore-ms`, `ore_ms`,
  `tartandrive`, `thermal`, or `airlab`. The shared-folder link/path for the
  checkpoint archive and the approved dataset transfer/staging endpoint
  have been requested from the user. Root-name absence does not establish
  that the files are absent from nested or shared folders.
- The refresh changes local OAuth state. Before ending this workspace,
  update the existing private OSMO `ore-ms-box` credential from the current
  config using the setup guide; the OSMO CLI is not installed here.

### Runtime and checkpoint restoration verified — 2026-10-17 (America/New_York)

- The supplied Box link resolved through a Box web-link item to the
  `tartandriver_models_storage` shared folder (folder ID `303805616544`).
  Downloaded only `macslam.zip` (file ID `2037543542268`, 318,804,129 bytes).
  SHA-256 matches the earlier session:
  `77ad168672019f94d8e48ebb087cb109b938bcf8784e3df480070cd5f437f00c`.
  ZIP CRC checks pass; extracted all three weights under `ore-ms/macslam/`.
- Rebuilt `macvo:blackwell` from the committed Dockerfile. New image ID:
  `sha256:90fe85f1a9c139a8f53533e2420992273648267206eec214952e0b0b18f57b99`.
  Torch 2.7.1+cu128 / torchvision 0.22.1+cu128 run on the RTX PRO 5000 Blackwell.
  CUDA matrix multiplication, random-weight frontend inference at 128x128
  and 480x640, and `python MACVO.py --help` pass.
- Actual restored MACT checkpoint verification passes: strict load of all
  428 entries into 19,157,555 parameters; GPU inference at 480x640 returns
  finite flow and finite positive covariance, both `(1, 2, 480, 640)`.
  Single forward elapsed 0.397s, including first-use overhead; this is an
  execution check, not an accuracy result or throughput benchmark.
- Exact checkpoint verification command, from the AirStack root:

  ```bash
  docker run --rm --gpus all \
    -v /root/AirStack/ore-ms:/workspace -w /workspace/MAC-VO-main \
    macvo:blackwell python Scripts/Setup/verify_mact_checkpoint.py \
    --forward --height 480 --width 640
  ```

- Local ignored artifacts are under `ore-ms/results/runtime-restore/`, including
  the MACT verification log, package freeze, image ID, and archive source IDs.
  Weights and runtime images are restored locally; persistent storage of the
  refreshed Box config and runtime image still requires the documented
  OSMO/registry steps before shutdown.
- Remaining data blocker: no approved transfer endpoint or sample has been
  supplied. The user knows the folders in AirLab cloud storage and has been
  asked for the sequence and calibration folder URLs. No dataset has been
  downloaded; thermal preprocessing, full VO, and accuracy remain unverified.

### Dataset access check — 2026-10-17 (America/New_York)

- User supplied `https://airlab-storage.andrew.cmu.edu:4001/?#/SAMLResp`.
  An unauthenticated HTTPS request returns the Synology NAS landing page
  (HTTP 200), not dataset files. No browser SAML session or dataset mount
  is available to this process; no authenticated file download was performed.
- User supplied `castacks/static_tf_publisher/config/offroad/yamaha.yaml`
  pinned at `688c34472166f93b5129173030acccc96499b8fc`. Public GitHub,
  raw-file access, and the connected GitHub file tool all return 404.
  This does not distinguish missing files from inaccessible private content.
  Contents remain uninspected; the user's description as intrinsics is
  not yet verified against the actual YAML.
- Added `DATA_STAGING.md` with the smallest useful browser-to-Box transfer:
  about 10 contiguous synchronized captures from four camera streams,
  timestamps, camera-info/intrinsics, pinned YAML, and rectification metadata.
  The user must download via their authorized sessions and stage this subset
  in their Box account or provide an accessible shared-folder link.

### Extraction workflow inspected — 2026-10-17 (America/New_York)

- Collaborator guidance confirms extrinsics in the pinned `yamaha.yaml`,
  intrinsics in extracted `_camera_info` / `_intrinsics` directories, and
  per-directory `timestamps.txt`. The extrinsics YAML remains inaccessible
  here, but `castacks/ros_torch_converter` files are readable.
- Inspected README, `scripts/get_slam_data.py`, `tf_manager.py`, and the RGB
  and thermal extraction configs on `main`. File blob SHAs respectively:
  `b71ff349cd07a3a0880f325f6a0e415f118dbc11`,
  `7d3de8debd7c989a0c86053a5056926b67283c26`,
  `3cdd9a69eb081374dac98008581b49a281537385`,
  `778167ad1843e6f44e6bb662acfad9c6e5ffad9d`,
  `1a56e2cefcfd773ee4f60f958769c6581ebdadad`.
- `get_slam_data.py --depth --odom` processes extracted data, not raw bags.
  It requires `tf/`; depth additionally uses SLAM point clouds under
  `super_odometry/pointcloud_in_odom`. Configs target left RGB rectified and
  left thermal processed images. Keep right streams for stereo separately.
- The script does not automatically apply the external extrinsics YAML or
  read camera-info directories to replace config intrinsics. Validate config
  calibration and explicitly verify TF updates before extraction. The thermal
  config's `use_chain: True` comment scopes it to data extracted before
  October 1, 2025; applicability to the selected recording is unverified.
- `TfManager.update_from_calib_config` reads `transform_params` with
  `from_frame`, `to_frame`, `translation`, and `quaternion`. It skips existing
  dynamic TFs and conflicting non-root parents rather than replacing all
  transforms unconditionally. The demo documents `get_transform(a, b)` as
  mapping points from b into a; verify frame names and conventions on real data.
- These outputs are SLAM-projected depth and recorded TF/odometry poses;
  independent ground-truth status has not been established. Updated
  `DATA_STAGING.md` to include TF/SLAM inputs and the bounded extraction command.
  No data extraction or accuracy experiment has been run.

### First TD3 clip staged and thermal real-image inference verified — 2026-10-17

- Downloaded the user's `ore_ms:ore-ms/1_salon_mac_cov_day.zip` (8,856,044,347
  bytes); SHA-256 `02dd584bda8455487a4b6b4763b876e4c6bdeffbb8b5c0cf3ee3b3c4af693bd2`.
  Recording audit confirms parent sequence `20251027`. Extracted with CRC/path
  checks to `data/tartandrive3/20251027/1_salon_mac_cov_day/`.
- Clip contains stereo RGB, raw uint16 and processed uint8 thermal, calibration,
  per-stream timestamps, TF, SLAM point clouds, and odometry. No further data
  download is needed for initial engineering checks.
- Frames 0–9 are complete across four cameras, with approximately 23.12 ms
  RGB/thermal offset and 1 ms thermal stereo offset. Full clip has missing and
  repeated thermal samples and larger same-index offsets; pair by timestamps
  and explicitly handle duplicates rather than assuming index synchronization.
- Actual MACT temporal inference on processed thermal frames 0 and 1 passes:
  strict 428-entry load, native 512x640, finite flow and positive covariance,
  each `(1, 2, 512, 640)`. Used provided uint8 images, BGR→RGB, float32 /255,
  no resizing; equivalence to training preprocessing remains unverified.
- New RGB blocker: `MACVO_FrontendCov.pth` fails strict loading against recovered
  frontend (one missing `memory_decoder.delta`, seven unexpected encoder-norm /
  positional-embedding entries). No permissive fallback or RGB result claimed.
- New extraction issues: archived TF metadata schema differs from the inspected
  converter reader and includes repeated child-frame entries. Raw K and
  rectified P differ, and thermal/RGB right projection translation signs differ.
  Resolve geometry/schema compatibility before running full VO or reporting error.
- Detailed evidence, limitations, and next experiment:
  `experiments/EXP-001-input-audit.md`. Local artifacts: `results/data-access/`.

### Baseline readiness implementation — 2026-10-17

- Resolved RGB state mismatch via an exact audited legacy migration, justified
  by pinned FlowFormer cleanup history. All active state remains strictly
  validated; native MACT is unchanged. Unknown/missing learned state is rejected.
- RGB and thermal both pass real-image temporal inference at native resolution;
  full active parameter/buffer state exactly matches the old runtime wrapper.
  11 focused regression tests pass.
- New timestamp tool produces 1320 unique four-camera groups with explicit
  2 ms stereo and 30 ms cross-modal exploratory limits. Paths, increasing times,
  no acquisition reuse, and every within-group timing bound verified.
- Geometry audit treats old TF serialization as graph constraints and checks
  both stored-transform directions without selecting one. Thermal projection
  and TF disagree in axis/sign under both hypotheses. Static graph has no
  independent cycles, so its internal residuals cannot certify calibration.
- User was asked to stage the pinned `yamaha.yaml` in Box; supplied its original
  GitHub link again when they asked where to find it. Until reconciled, reference
  extraction, full VO, and geometric/covariance accuracy claims remain pending.
- Reproducible tool commands: `MAC-VO-main/Scripts/Setup/README.md`. Detailed
  measured results: `experiments/EXP-002-baseline-readiness.md`. No commits/push
  performed in this continuation.

### Supplied extrinsics inspected and asset layout cleaned — 2026-10-17

- User supplied `yamaha.yaml` and authorized reorganizing downloaded artifacts.
  Current paths: `models/macslam/` weights; `data/archives/` ZIPs;
  `data/calibration/yamaha.yaml` extrinsics; `docs/research-proposal.pdf` proposal.
  Updated checkpoint default and active commands; historical logs retain original
  paths. Local `results/data-access/layout-migration.json` maps the moves/hashes.
- Supplied YAML SHA-256:
  `031c199619916bf1466a807d9ed91896d58b1428f0bdf74951dabc3c1b59e27d`.
  All 13 transforms match recorded values under reversed frame-pair labels.
  Numeric matrices are identical, not inverted. Do not blindly feed the archive
  into the inspected current TF reader or invert matrices solely from labels.
- Inspected thermal producer/config and saved text snapshots under
  `data/calibration/upstream/`. Its inverse-matrix OpenCV rectification reproduces
  saved left/right R/P exactly. This reproduces generation, not physical validity.
- Three-frame image diagnostic: provided/published-inverse thermal inputs yield
  median vertical feature residuals around 0.58–0.63 px, direct-matrix variants
  around 1.89–1.94 px. Provided image matches have positive median disparity while
  thermal P encodes negative baseline. No source files or calibration changed.
  Signed metric geometry/reference poses remain unverified; no full VO run.
- 12 focused tests pass, and the relocated default checkpoint strictly loads all
  428 entries. Detailed sources/findings/commands are in `docs/calibration.md`.

### Bounded RGB and thermal camera-local odometry — 2026-10-18

- Added a manifest-backed provisional TD3 adapter preserving source acquisition
  timestamps, rectified P intrinsics, native uint8 imagery, and explicit grayscale
  replication. It rejects uint16 rather than silently normalizing raw thermal.
- Both 10-frame runs pass full camera-local execution through stereo/depth,
  temporal correspondence, 3D covariance, and synchronous pose optimization.
  RGB retains 1800 matches; thermal 1470. Neither has interpolated frames.
  Retained 3D covariances are finite, with positive minimum eigenvalues.
  Trajectories contain exactly 10 finite unit-quaternion poses with original
  int64 acquisition times. 15 focused regression tests pass.
- Explicit experimental geometry: absolute projection baseline / absolute
  disparity; identity body/sensor transform, trajectories in each rectified
  left camera's NED frame. Thermal signed geometry remains unresolved. No
  body/world alignment, reference poses, accuracy, or cross-modal reliability
  claim follows from this execution result.
- Runtime uses regular fp32 frontend (no CUDA capture), mapping disabled,
  synchronous CPU optimizer, seed 0. Complete checkpoint loading now avoids
  unrelated pretrained encoder downloads. Optional compile helpers fall back
  to eager math because the image lacks a C compiler; timings are not benchmarks.
- Final artifacts: `results/bounded-vo/run-002/{rgb,thermal}/`, including graph
  observations/covariances, depth, trajectories, effective configs, and provenance.
  Reproduction and next research step: `experiments/EXP-003-bounded-vo.md`.

### First moving-window proxy-reference comparison — 2026-10-18

- Recorded odometry metadata identifies vehicle poses in sensor_init. High-rate
  odometry interpolated at dynamic TF timestamps matches its numeric transforms
  to floating-point precision, confirming export direction/source consistency;
  it is not independent ground truth.
- Added linear-translation / SLERP reference interpolation, no extrapolation,
  camera lever-arm + rectification + optical/NED conversion, first-pose-only
  anchoring, and consecutive relative-pose diagnostics. 20 focused tests pass.
- A reference-only maximum-displacement selector chose 20 complete paired
  frames at manifest start 348 (source indices 452–471): 1.899992 s and about
  8.873 m recorded body displacement. Both VO runs complete without interpolation
  fallback; RGB retains 3734 matches and thermal 3325.
- Conditional disagreement against the SLAM proxy, no scale fitting:
  RGB anchored translation RMS 0.1277 m / rotation RMS 0.3108 deg;
  thermal 1.5171 m / 5.6879 deg. Consecutive translation RMS: RGB 0.0309 m,
  thermal 0.1415 m. This is one moving window, with provisional thermal geometry
  and unknown reference error, not an independent accuracy benchmark or ranking.
- Outputs: `results/moving-vo/run-001/` and `results/reference/` (metrics, reference
  poses, plot, provenance). Methods and next measurement:
  `experiments/EXP-004-moving-reference.md`. No model/calibration files changed.

### Retained-feature uncertainty/consistency diagnostic — 2026-10-18

- Analyzed 3734 RGB + 3325 thermal retained temporal observations using proxy
  motion: r=p1-(R p0+t), with approximate C=C1+R C0 R.T. All observations have
  finite positive depth and positive-definite combined covariance. This omits
  cross-correlations and reference/calibration uncertainty; no NIS/calibration
  claim. Targets are two-view consistency, not independent per-point error.
- Pooled trace/residual Spearman: RGB 0.621, thermal 0.404; depth-partial ranks
  0.413 / 0.180. Associations vary strongly across 10/20/50/100 m depth caps;
  thermal near-range association is negative. No RGB/thermal feature equivalence
  or significance/ranking claim follows from these selected observations.
- 75–89% of depth-variance components sit at configured 0.05 m² minimum. These
  propagated matrices include priors, floors, and local depth statistics, not
  solely learned uncertainty. Extreme distant/near-zero-disparity depths make
  all-retained unweighted depth-scale fits collapse; do not apply them.
- Sequential trajectory scale diagnostic: scalar fitted on first 10 moving
  frames is 0.7683 thermal; applied without refitting to the next 10, proxy
  translation RMS drops 2.0115→0.1669 m. RGB 0.9752, 0.1654→0.0823 m. This is
  within one selected dependent segment, not independent calibration validation.
  Original VO trajectories/weights/calibration remain unchanged.
- 25 focused tests pass. Final artifacts: `results/feature-consistency/run-004/`.
  Exact scope, methods, sensitivity tables, and next measurement are in
  `experiments/EXP-005-feature-consistency.md`. Next: raw covariance instrumentation
  before floors/selection and independent-window scale/preprocessing checks.

### 2026-10-18: raw variance and separate-window check (EXP-006)

- Opt-in raw temporal variance/grid/selection/floor recording is implemented.
  Both original-window graphs are bitwise unchanged with recording enabled.
- Predeclared disjoint 20-frame starts 306 and 1132 by reference motion only;
  froze first 10 original-window translation scales (RGB 0.97515, thermal 0.76826).
  Thermal proxy translation RMS: early 0.703→0.545 m; late 2.897→3.391 m.
  RGB: early 0.090→0.127 m; late 0.100→0.096 m. Do not adopt a global scalar.
- Raw flow trace median within-pair depth-partial rank (≤50 m), grid/retained:
  RGB original 0.409/0.268, early 0.377/0.276, late 0.436/0.311;
  thermal original 0.273/0.147, early 0.200/0.199, late 0.076/0.081.
  No calibrated or comparable modality confidence established.
- Six runs pass bounded execution; 27 tests pass. Full evidence and limits:
  `experiments/EXP-006-raw-uncertainty.md`; artifacts `results/uncertainty/run-001/`.
  Same clip, recorded SLAM proxy, provisional geometry; no corrected outputs.
- Next: diagnose late thermal failure with reference-conditioned reprojection,
  stereo disparity/depth distributions, and region/range strata, preserving
  calibration/preprocessing. Do not fit another evaluation-window scalar.

### 2026-10-18: late thermal reprojection diagnosis (EXP-007)

- Reused all six EXP-006 runs offline; no inference, correction or fitted scale.
  Thermal late estimated/reference net displacement is 3.540/8.747 m, versus
  original 11.377/8.857 m. Yet retained median reference reprojection improves
  6.655→5.688 px and Sampson distance 1.055→0.468 px (both depths ≤50 m).
  A larger aggregate match residual does not explain the trajectory failure.
- Late thermal primary features shift upward: upper third 59.1% versus 46.7%
  original; bottom third 5.6% versus 13.3%. Counts and ranges differ; this is
  descriptive evidence, not isolated cause. Fixed-grid results agree in trend.
- Important backend clarification: saved graph_type=disp/autodiff=false uses
  analytic image+disparity residuals and target pixel/disparity covariance.
  Projected 3D covariance/depth-variance floor is not a direct pose weight here.
  Pixel flooring affects the active covariance through the existing in-place
  clamp. Prior 3D floor measurements remain valid diagnostics, not evidence for
  a direct 3D-weight cause of these pose errors.
- Five new geometry/backend tests pass. Final artifacts:
  `results/thermal-failure/run-002/`; source/input hashes and representative
  images included. Methods/limits: `experiments/EXP-007-thermal-failure.md`.
- Next: controlled reprojection-only versus disparity backend replay on the
  same saved matches across all three windows, keeping initialization fixed
  and fitting no scale. Isolate objective effects before changing calibration,
  preprocessing, masks or fusion. Same-clip/provisional/SLAM-proxy limits remain.

### 2026-10-18: fixed-match backend ablation (EXP-008)

- Replayed the six EXP-006 graphs using analytic disparity and reprojection-only
  objectives. Identical world points, observations/covariances and static prior
  initialization; per-arm input fingerprints match and inputs are unchanged.
  Native LM/Huber/stopping settings preserved; no inference or fitted scale.
- All disparity controls reproduce original local motion within 4.56e-7 m and
  3.48e-6 degrees. Independent fixed-prior increments are composed for diagnosis;
  this is not a fresh end-to-end VO run or independent dataset evaluation.
- Thermal translation RMS (disp→reproj): early 0.703→0.590 m;
  original 1.517→1.446 m; late 2.897→2.948 m. Late net travel 3.540→3.482 m
  versus 8.747 m recorded reference. Removing target disparity does not fix it.
- Median weighted local projection-Jacobian condition number, thermal:
  early 23.49/23.83, original 22.34/22.79, late 52.48/53.46 (disp/reproj).
  These meters/radians diagnostics are before Huber, not solver Hessians or
  calibrated uncertainty; they do not isolate the cause.
- Two new synthetic/replay tests pass; 228 pair solves complete. Final evidence:
  `experiments/EXP-008-backend-ablation.md`; `results/backend-ablation/run-002/`.
  One CPU thread avoids matrix-thread overhead; original replay controls pass.
  Interrupted run-001 preserved. No backend default or source trajectory changed.
- Next: fixed-match/depth/initialization learned-versus-fixed covariance weight
  ablation; predefine per-channel weights and relative units before evaluation.
  Then assess feature geometry before changing calibration, masks or fusion.

### 2026-10-18: learned versus fixed covariance weights (EXP-009)

- Frozen per-modality u/v/disparity variance medians from original frames 0–9
  before replay. Thermal constants [0.08894412,0.0625,0.28550828] px²;
  RGB [0.06561724,0.0625,0.03216033] px². No reference data in this rule.
- Four disparity-backend arms on identical nonweight inputs: learned,
  fixed_uv, fixed_disp, fixed_both. All retained matches, including depth
  tails; no 50 m cap. No inference, scale fit or source trajectory changes.
- Late thermal translation RMS: learned 2.8966, fixed_uv 0.7898,
  fixed_disp 2.8955, fixed_both 0.7897 m. Pixel weighting strongly affects the
  fixed-input failure; fixing disparity alone barely changes it.
- Critical tradeoff: late rotation RMS 1.713→4.375° under fixed_uv. Early
  thermal accumulated translation improves 0.703→0.452 m, while local
  consecutive translation worsens 0.117→0.182 m. No default replacement.
- Seven new test cases pass; 456 pair solves complete. Learned replay controls
  match originals; nonweight fingerprints agree and all finite/unit-pose checks
  pass. Results/provenance: `results/weight-ablation/run-001/`.
  Full tables/methods: `experiments/EXP-009-weight-ablation.md`.
- Next: audit spatial/range concentration of pixel precision and local pose
  influence for learned versus fixed pixel weights, preserving matches/depths
  and including both controls. Design a weighting/selection hypothesis only
  after assessing the translation/rotation tradeoff. A separate clip is needed
  before changing defaults or claiming comparable modality confidence.

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
