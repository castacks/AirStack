# MS Research Handoff — Cross-Modal Geometric Uncertainty for RGB–Thermal Visual Odometry

2026-10-28 — [EXP068 training semantics audit](experiments/EXP-068-ms2-training-semantics.md): available inference exp(2*raw) conflicts with current covariance trainer raw+epsilon; cov_loss ignores mask/max_cov. Exact CPU probes and independent88-check audit pass; relevant upstream code matches. RGB checkpoint byte-identical to official release; thermal exact training recipe remains unverified. No training/fix/neural inference/default change or claim that this caused prior failures. Next empirical scale/coverage check on saved data; holdouts reserved.

2026-10-28 — [EXP067 distribution-risk assumption audit](experiments/EXP-067-ms2-distribution-risk.md): same mean/variance with lognormal/Gamma assumptions gives different inverse-depth risk; Gamma infinite when variance/disparity²>=.5. Four reused captures/80frames/106562pairs: lognormal mildly improves some first-order risks but worsens day19/20; Gamma worsens day20/20, with1920 both-infinite pairs explicitly mixed. No replacement adopted. Mathematical quadratures/counterexample and independent1589390-check audit pass. No fit/inference/default change; holdouts reserved. Next trace training objective/checkpoint semantics.

2026-10-27 — [EXP066 disparity-domain diagnostics](experiments/EXP-066-ms2-disparity-diagnostics.md): four complete saved-frontend captures/80frames/106562 both-valid pairs. EXP064 day RGB: median0.684% tiny-relative-disparity points contribute99.05% axial-depth squared error. Raw uncertainty ranking changes with error domain; thermal night variance/depth-error correlation median-0.563 despite positive disparity-error ranking. Independent4512464-check native-PNG audit passes. No fitting/inference/clipping/default change; holdouts untouched.

2026-10-27 — [EXP065 signed residual accounting](experiments/EXP-065-ms2-signed-residuals.md): all7 examined captures/140frames/240003 both-valid pairs,3anchors,fixed range/grid groups. Global mean subtraction retains RGB lower squared error20/20 in both night windows; thermal has larger spatial mean components. This is reference-assisted descriptive accounting, not physical bias identification or a deployable correction. Independent502138-check audit passes; no fit/inference/default changes; two remaining recordings untouched.

**Status:** Research direction approved by Basti in October 2026. Previous integration and respiratory-rate work is complete.  
**Program:** CMU MS ECE, Intensive Project Option.  
**Research window:** Fall 2026 (12 units) and Spring 2027 (24 units); expected completion May 2027.  
**Document role:** Living research context for an AI coding/research assistant. Update the progress log, decisions, and experiments as evidence accumulates.  
**Last contextual update:** 2026-10-11.

2026-10-27 — [EXP064 prospective frozen suite](experiments/EXP-064-ms2-frozen-suite.md): freeze063numericalmodels unchanged; declare U-only primary before new3captures (day15-46-56,night22-03-03,rain16-59-13),first20IDs each,no refit/newtraininglabels. Criterion eachcapture medianmeanL2 U below bothfixed+scalar not met. day:U/RGB 0.1920,U/THR 1.0355,U/scalar 0.7979; night:U/RGB 1.0709,U/THR 0.5828,U/scalar 0.9822; rainy:U/RGB 0.8692,U/THR 0.6253,U/scalar 0.9149. Allmodels/anchors/losses/ties and availability retained,independent source/frontend/rawgeometry/frozen/summary audits PASS; models/scalers128developmenthashes unchanged. Separate pre-outcome Boxdeclaration, exactparentparity. New240camera hashes no prior320/crosssuite overlap; sourcefullarchives local excludedrecovery. SameGPUruntime/config/weights, no default/calibratedprobability/physicalGT/VO/weathercause/trainingheldout claim. Threecaptures nowexamined,twoofficialtestcaptures remainunused; consolidate evidence before further fits. Recoverytarget backups/2026-10-27-exp-064; parents063/057/058/060. See the [transfer synthesis](docs/research/ms2-transfer-findings-2026-10-27.md) for the supported claims and next signed-residual/mean-versus-spread diagnostic; empirical spatial mean is not automatically physical/stochastic bias.

2026-10-27 — [EXP063 frozen new-capture test](experiments/EXP-063-ms2-unexamined-recording.md): select firstunusedofficialtest seq_2021-08-06-16-45-28,rainlabel,20openingIDs beforepixels. Fit unchanged062fourfamilies+060scalar once on all3development60frames133441pairs,freezecoeffs/scalers19:42:15NY SHA51b2b9a15d1951c2902e6ef8a0a5e45dcae3d1713d428e36d9545b530e0ae377; no testfit. Acquire27.61GB exact413ranges; bounded80images40singledepth,6672fulltimestamps/stream,zerooverlap240oldcamerahashes. RebuildGPUruntime; oldRGB/THR firstpairs+tensors/config/checkpoints exactbitwise;new40fields+2repeats/80producerinputs PASS. Support22277allbothvalid,199–2036/frame; noframesremoved. Primarycombined/U medianrisk1.0385L2/1.4012MSE,17worse3ties; combined/RGB1.00235/1.00018,15worse5ties;allanchorszero wins. U-only/RGBmed0.9696L2/0.7143MSE12/20wins,controlnotadopted. Combined beatsoriginal19/20L2,14/20MSE but doesnotvalidategeometryincrement. Independent raw44554predictions/frozenmodel89kdecisions/sourceaudits PASS; fixedartifact unchanged. Thiscapture nowexamined;no retune/default/physicalGT/VO/trainingheldoutclaim. Additionalpredeclaredrecording/referencecoverage before morecapacity;existingofficialpointers suffice,no upload needed. Recoverytarget backups/2026-10-27-exp-063; development057/058/060, pointers040,parity048.

2026-10-27 — [EXP062 prediction-geometry selector](experiments/EXP-062-ms2-geometry-risk-selection.md): predeclared constant/U-only/G-only/combined linearcost-sensitiveLS, fixedridge0.01, train-only equalrecording/frame normalization, three excludedrecordings. Features logoriginalU_R/T, logpredictedrange_R/T, RGBx/1224,y/384; no reference/error features though supportreference-selected. Fit midpointmeanL2 signedwinner with costweights abs(E_T−E_R)/framerandom, margins not probabilities/calibratedrisks. Night combined beats U/G/bothfixed/original/camerascalars/random20/20 bothlosses/allanchors; midpointcombined/U0.9162,combined/RGB0.9162L2/0.8981MSE. Daycombined/U1.1341L2(0/20),1.2379MSE(1/20); losesRGB20/20both. Raincombined/original4.643L2/314.270MSE,losesthermal20/20both despite combined/U0.981L2. No generaltransfer/defaultadoption. Independent3213914 recordedchecks inclalternatesolverdecisions and128hashes PASS; syntheticpurity/ties/mutation controls pass. run002 zero-cost synthetichandling only, realdatareportbyteidentical001. Existingwindows inspecteddevelopment,notuntouched/trainingheldout. Next independentrecording/referenceevidence before morecapacity; checkexistingcoveragefirst. Recoverytarget backups/2026-10-27-exp-062, prerequisites057/058/060.

2026-10-27 — [EXP061 range/spatial/tail structure](experiments/EXP-061-ms2-error-structure.md): frozen6 midpointreference-range bands/3x3RGBgrid/54jointcells, all3 existing20IDwindows/allanchors/bothlosses, no fit/inference. Night thermal lower meanL2 everyframe below20m (threebands), RGB everyframe20–80m (two bands); >=80m only2–8pairs/frame. Night overallRGBmean6.022m vsTHR7.396m, but medianpoint3.118vs2.733m andmeanrelative0.372vs0.231 favorTHR; targetloss matters. Sharedreference-assisted worst1/5% removals retainnightRGBadvantage20/20 bothlosses; rainRGB/THRmeanL2ratio4.079→0.993→0.773 (0/10/20 RGBwins), RGBtop1% contains70.3%L2/99.5%MSE. No badlabel/outlierdiscard/causalnight claim. Lit night overlays visibly lit road, thermal surrounds clearer; contextonly. Independent946023 recordedchecks incl800646 reconstructederrorvalues/126hashes PASS, boundary/tie/empty controls PASS. Paper note Levi/Chen/Ovadia publicprimaryversions read, no PDF needed. Next predictedgeometry/context risk information with declared model/loss/recordingprotocol; referencerange not deployable. Defaults unchanged. Recoverytarget backups/2026-10-27-exp-061; inputs EXP057/058.

2026-10-27 — [EXP060 relative camera-score transfer](experiments/EXP-060-ms2-camera-scale-transfer.md): fixed pre-fit243-policy grid, one RGB propagated-score multiplier, three recording-excluded fits on existing20-ID rain/day/night windows. Fit midpointmeanL2 per-image/random risk, equalrecording weights; freeze for allanchors/bothlosses. Selected log10c rain/day−2.45,night−0.8. Rain worsens original20/20 bothlosses (L2median1.416); day improves original20/20 (0.849) but beatsRGB only10/20; night improves original20/20 (0.952) yet losesRGB20/20 bothlosses (L2 1.191/MSE1.503) andrandom19/20L2,20/20MSE. No universal constant-scale correction adopted. Previously inspected windows are exploratory recording-excluded evidence, not untouched/training-heldout. Independent7923 recordedchecks PASS,63 frozeninputs unchanged;273 recoverymembers verified,231 resultfiles restored. No inference/default/VO/fusion change. New richer calibration would need declared model/development/evaluation; stronger independentrecording/reference helpful, check existingcoverage before uploads. Box target backups/2026-10-27-exp-060; prerequisites EXP057/058/059.

2026-10-27 — [EXP059 camera-scale/depth-score diagnosis](experiments/EXP-059-ms2-score-failure-diagnosis.md): all60 existing rain/day/night image pairs, unchanged bothvalid support/predictions/errors/anchors/losses/depth sensitivities. Exact U=||q||²/B²*Zpred^4*vangular, camera-median/localvariance/raybaseline/depth log terms verified. Frozen2×2 original versus per-camera image-median normalization and/or supplied-reference-depth Jacobian. Allfour remain worse than RGB/random20/20 night under bothlosses, both endpoints; combined reference-assisted normalized mean-distance ratios1.216/1.088. Reference-depth intervention helps dayvsRGB0.972(19/20) but hurts rainvsrandom1.526(0/20); never a deployable policy. Median RGB/THR angularvariance scale0.041rain/0.072day/1.607night; night medianselected underhalf reference depth13.9%, wrong+underhalf13.0%, allwrong56.2%. Underdepth deterministically suppresses fixed-ray metric score by fourthpower, but neither scale removal nor reference-Jacobian substitutions explain away wholefailure; no single cause/globalcorrection established. Independent raw-factor audit266882 raw camera factors,1440 primary+1920 subset risks/40232 scalarfields,240 rawmodel/depth hashes and126 analysis-source hashes PASS; original scores reproduce; no inference/fit/default/sourcepixel/clock/calibration change. Outputs and explicitly nondeployable reference-assisted controls preserved. Next method question is whether calibrated camera risk can transfer across recordings versus fixedcamera/camera-scale controls, with declared recording-level evaluation before fitting; do not call these inspected windows untouched holdout. No new access/compute blocker; CATS/trainingmembership pending separately.

2026-10-27 — [EXP058 third-recording night replication](experiments/EXP-058-ms2-night-cross-modal-replication.md): recover original EXP043/044-complete/045 archives;277/109/32 member hashes pass and244/85/23 missing scientific payloads restored. Byte-preserving adapter stages124 required files with original archive/member/hash provenance; no copied calibration from another recording. Unchanged EXP057 runner/independent validator and EXP056 math, fixed20 opening nightIDs, same mutual/frontmost angular support/controls/anchors/availability/losses/depth sensitivities; fixed3 overlays inspected before scores.85,170 candidates allbothvalid,4251 median/image, median image reference3D separation0.01282m. Learned loses alwaysRGB/equalrandom20/20 under bothlosses (mean-distance medianratios1.253/1.121; squared1.653/1.285), despite camera-scale gain20/20 (0.978 mean-distance/0.979 squared). AlwaysTHR wins17/20 mean-distance and19/20 squared; fixedpixel geometry wins20/20 both. Endpointanchors change60/85170 pointwinnerlabels and retain failures; strict2/256m and1/2/5% depth sensitivities retain all20 mean-distance RGB/random failures. Independent audit170340 raw predictions,120 nearest/zbuffer samples,80 masks,600 primary+800 subset risks/16800 scalarfields,40 timestampjoins/230 repeated frozenhashchecks PASS;124 source/copy members/3 original manifests and historicalhelpers separately preserved. No fit/inference/default/timing/calibration changes, physical correspondence/trainingheldout unverified. No new access/compute block. Empirical roadblock is systematic cross-camera score failure; next diagnose camera-scale versus nonlinear depth-propagation effects before calibration/fusion, with explicit recording-level evaluation before any fit. Papers on crosssensor uncertainty/modelselection and nonlinear stereo-depth uncertainty useful; CATS/trainingmembership still separate pending items.

2026-10-26 — [EXP057 frozen day cross-modal replication](experiments/EXP-057-ms2-day-cross-modal-replication.md): restore EXP048 bounded archive777347065B SHA589b47fb91bd8bf01c905bcea581238f4f160633b6e231fecba494984010a51b, all524 member hashes pass;501 day files selected/500 missing restored without overwrites. New dataset-parameterized runner reuses unchanged EXP056 math and first reproduces320 rain support arrays bitwise,280 metric arrays/7200 risk fields numerically. Fixed day opening20IDs, same angular/mutual/frontmost support, depth sensitivities, all metric/camera-scale/fixed-camera controls and anchors.22,717 candidates all bothvalid; median image reference separation0.1298m; original left recorded separation26.019–36.503ms retained as diagnostic, not exposure/gate. Learned beats camera-scale20/20 squared (medianratio0.744),19/20 mean-distance(0.904), unlike rainparity0.988/1.002. Crucial counterexample: alwaysRGB beats learned all20 mean-distance (learned/RGB1.157),13/20 squared(1.318); unpropagated learnedangular comparator beats propagated all20 bothlosses. Endpointanchors preserve results (380/22717 pointwinner changes). Thus within-image variance benefit differs between tested windows; no universal modalityselector/fusion justified. Independent raw-data audit45,434 model predictions,120 nearest/zbuffer samples,80 depth masks,600 primary+800 subset risks/16800 scalarfields,40 timestamps and230 repeated frozen hashes PASS. All20frames and counters retained, no fit/inference/timing/calibration/default changes. No new access/compute block; exact sparse physical point/visibility and nonlinear covariance limitations remain. Any method next needs recording-level development/evaluation split, camera-scale and fixedcamera baselines; do not tune these already scored windows and call it heldout. CATS/trainingmembership pending.

2026-10-26 — [EXP056 fixed cross-modal support/metric pilot](experiments/EXP-056-ms2-cross-modal-support.md): all20 inherited rain IDs, normalized angular radius1/fxRGB in both directions, unique mutual nearest labels and both sparse frontmost zbuffers, no primary depth-threshold selection.25,754 candidates (52/1221.5/2429 perframe), reference3D separation medians0.0084–0.6333m; fixed3 overlays inspected before winner scoring.25,554 both-valid,200 thermal-only candidates. Common nominal RGB-frame midpoint primary, both endpoint anchors retained; horizontal depth-only rank-one covariance propagation. Learned beats equal-angular geometry20/20 (median MSEratio0.802/mean-distance0.855), but image-wide per-camera median angular-variance control largely matches full pointwise learned:0.988 MSE11/20 and1.002 mean-distance9/20. No consistent additional spatial-variance benefit established. Preserve fixed-pixel geometry counterexamples,284/25554 endpoint-winner changes, strict quantization subset17/19 mean-distance improvements and missing-frame sensitivity. Full independent audit reconstructs51,508 raw predictions,480 selector risks/80 depthmasks/160 subset risks; scale audit51,108 valid variance/geometry contributions+120 risks PASS;counts repeat dependent samples. No new inference/model/default/calibration/clock/data changes. Roadblocks beyond pending people: exact sparse cross-modal surface/visibility and uncompensated acquisition motion; first-order inverse-disparity uncertainty is not calibrated metric confidence. Papers on occlusion-aware multimodal/LiDAR correspondence and cross-camera uncertainty-scale calibration would help, not block current results. Next: frozen distinct-recording replication with camera-scale baseline, not fusion or more owner calibration questions. CATS pending/training membership unknown.

2026-10-26 — [EXP055 cross-modal feasibility](experiments/EXP-055-ms2-cross-modal-feasibility.md): audit all20 existing rain IDs, single-scan depths and documented NIR-to-camera calibration. Composed RGB→thermal separation0.65709m; median RGB source fraction with nearby thermal depth85.4% at1 target pixel, reverse20.8%; target-Z discrepancy medians~0.058m. These are candidate geometry/label-consistency statistics, not verified same points, calibration accuracy or modality ranking.46 inputs match recovery hashes and remain unchanged;40 directional image cases pass roundtrips,120 independent scalar projections+120 brute-force nearest distances pass,40 left timestamps match original manifest (absolute separation median4.252ms/max61.537ms). Initialrun001 time-unit bug retained/invalidated;run002 authoritative. No model outputs/inference, calibration fits, timing gates or new imagery. Existing data supports a bounded frozen correspondence/visibility pilot; if credible, use common-frame metric point error with explicitly propagated depth-only uncertainty, not direct normalized-flow-trace camera ranking. CATS still pending; no new Geethika questions needed.

**Current synthesis — 2026-10-26:** [MS² findings and research decision](docs/research/ms2-findings-2026-10-26.md) consolidates EXP046–054. Controlled mean-L2 selection improves over matched random in all tested primary image/modality cases, with group median reductions about9–37%; central-pixel thermal counterexamples remain. This supports within-camera ranking against supplied references, not cross-modal comparability, covariance calibration or VO/fusion improvement. User priority: focus on necessary work; CATS access remains pending, and no further TD3 calibration questions are needed now. The next design task for the original research question is a defensible RGB/thermal comparison target and support, beginning with a feasibility audit of existing MS² data. Training membership is needed only before training-held-out claims. This synthesis recomputes30 group-level scalar checks from restored reports, not historical array validation or new inference.

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

### 2026-10-20: calibration reconciliation attempts rejected (EXP-020)

- User authorized addressing calibration, without requiring camera identity.
  Adapter correctly uses rectified P. Raw K/D match supplied stereo config;
  metadata consistency does not establish physical validity. Current TfManager
  composition matches T_first<-second. Pinned publisher source still404.
- Producer inverses a field declared by Kalibr as cam0-to-cam1. Direct source
  interpretation improves cheirality but worsens vertical alignment; neither
  label change nor translation-sign flip validates the full geometry.
- Stage A: two fixed training pairs,ten original held-out pairs. Free essential
  fit fails: single-training translation directions differ73.10deg; four
  held-out residual-improvement failures. Attempt and explicit failed export kept.
- Stage B: same training pairs,body centers fixed,rotation-only robust fit;
  preregistered fresh positions4/8 across all six windows,12 pairs/1631 matches.
  Vertical medians improve0.317–1.659px to0.223–0.435px, but four cheirality
  failures and1.254deg single-training rotation disagreement reject it. Eleven
  of12 meet the fixed residual criterion. No per-recording correction selected.
- 20 tests and228/269 frozen hashes pass; independent24 pair-set/20513
  candidate-point checks pass. These are technical controls, not calibration
  success. Original calibration,images,depths,model files and VO defaults unchanged.
- Artifacts: EXP-020, `results/calibration-reconcile/`, Box snapshot
  `backups/2026-10-20-exp-020` plus inherited inputs. Both fitted candidates
  explicitly failed; direct-source export explicitly unvalidated. No new VO/GPU.
- Asked for raw thermal stereo calibration-target images with known target
  geometry, or an independently measured stereo calibration for these recordings.
  Existing scene evidence has not yielded a stable physical replacement;
  serial numbers are not required. Metric/cross-modal calibration remains open.

### 2026-10-20: fixed-grid spatial stereo consistency (EXP-019)

- User authorized the spatial audit. No new images/inference/VO: all12 EXP-018
  first stereo pairs, fixed border32/stride16 grid,16848 points retained.
- Thermal spatial signs agree across day/night: upper-third positive0–15.7%,
  lower-third100% positive (all also>1px). Grid positives60.3–67.2%; night
  feature-location positives0–2.5%, with98.1–100% of those features in the upper
  third. Earlier feature-median contrast is spatially selection-dependent,
  not evidence of a recording-wide sign reversal.
- Lower thermal thirds have larger model vertical displacement and poorer
  model-target patch NCC than upper thirds in all six inputs. Inbound/vertical
  sensitivity leaves55.1–62.9% positive; NCC-defined grids60.2–66.5%. Conditional
  patch agreement and predicted vertical flow are not independent geometry.
- Night RGB source std<=1 covers1784/1800,1800/1800,1800/1800 points; NCC
  defined14/1800,0,0. All three absent fitted feature populations retained.
- 17 tests pass;190 frozen hashes,16848 grid points,24 populations and900
  summaries independently validated. Report EXP-019; `results/spatial-stereo/`;
  Box `backups/2026-10-20-exp-019`, with EXP-018 prerequisites. Defaults unchanged.
- User offered more data; requested authoritative physical left/right camera
  identity, mounting history and per-recording stereo/body calibration pointers.
  User supplied the existing converter/get_slam_data, pinned Yamaha extrinsics,
  tf_manager/demo, intrinsics and timestamp pointers already incorporated by
  earlier audits. No additional recording-specific mounting evidence is known.
  Existing images suffice for this audit. No blanket sign flip or region mask.
- A subsequent Google summary conflated public TD2.0 with later thermal rigs
  and mischaracterized GO. Primary sources confirm later physical AirLab
  thermal integration; thesis nominal hardware does not establish our exact
  recording provenance. See [dataset source boundaries](docs/research/dataset-provenance-2026-10-20.md).

### 2026-10-20: signed stereo and camera-topic ordering (EXP-018)

- User authorized the next bounded geometry check after Box recovery. Original
  model bundle SHA/CRCs verified; Blackwell runtime restored. Six fixed first
  stereo pairs per modality, from the existing day/night reports, no reselection.
- Saved positive depth erases model flow sign. Native signed stereo inference
  now retains it; all12 saved depth arrays reproduce bitwise, all strict
  checkpoint/finite covariance controls pass. No VO or defaults changed.
- Thermal feature signs agree99.4–100% on |observed| and |model| >1px; median
  signed horizontal errors0.160–0.364px. Day medians positive, selected night
  starts negative, while all six dense fields remain60.5–67.2% positive. Feature
  agreement is not whole-image or physical-depth validation. Night RGB has no
  usable fixed-protocol stereo SIFT fit in all three cases; empty results kept.
- All12 thermal processed images reproduce exactly from labeled raw topics;
  swapped raw inputs reproduce none. Day/night raw frame labels/camera-info
  and processed matrices are identical. No software topic-swap evidence;
  physical identity/mounting and calibration applicability remain unknown.
- 13 tests pass,112/174 frozen-plan hashes pass; independent interpolation,
  signed statistics and depth-byte controls pass. Evidence: EXP-018 and
  `results/stereo-sign/`; Box `backups/2026-10-20-exp-018` plus inherited snapshots.
- Keep defaults and no blanket sign flip. Next: authoritative physical camera
  and per-recording calibration provenance; preregister fixed-grid spatial
  sign/vertical-consistency diagnostics before extending beyond selected matches.

### 2026-10-20: thermal preprocessing and signed geometry audit (EXP-017)

- Fixed9 night stereo captures (window positions0/9/19). Staged18 raw PNGs +6 metadata files, exact raw/processed timestamp matches and ZIP-member CRCs.
- OpenCV4.10.0.84 replays the original producer functions/config to18/18 pixel-exact supplied images, including3 replicated channels. OpenCV5 differences MAE0.053–0.151 disappear under the single declared version sensitivity. This verifies sampled computational compatibility; historical runtime, all frames and checkpoint split remain unverified.
- Run-001's OpenCV5 strided-R/T failure retained; run-002 adds only contiguous layout/column-vector adapter, with original AST bodies/numeric values. Run-003 uses the same source under pinned OpenCV4.10; no source images/defaults changed.
- Saved R/P reproduce exactly. Night signed disparity supports saved negative baseline in7/9 sample medians;906 fitted inliers give778 points in front with saved P vs128 after sign flip. Middle is mixed; physical calibration is not validated. This differs from the earlier day diagnostic; do not apply one sign correction universally.
- Body/stereo inconsistency0.670113m horizontal and0.617019deg remains. Kalibr and OpenCV both document first-to-second transform convention, while producer inverts config. This is a convention discrepancy, not sufficient evidence to rewrite inputs.
- 67 tests pass under each OpenCV version;783 hashes pass in both successful plans. Exact pixel/triangle/sign invariance independently checked. Evidence: `experiments/EXP-017-thermal-provenance.md`, `results/thermal-provenance/run-003/` plus run-001/002 and inputs. Box `backups/2026-10-20-exp-017`, prerequisites EXP-013..016.
- Next bounded check: day/night raw-camera ordering and model stereo disparity signs versus observed signed correspondences, using saved model outputs first. Authoritative physical camera/calibration provenance and independent reference remain needed before cross-modal fusion claims.

### 2026-10-20: within-window temporal/context characterization (EXP-016)

- User authorized continuation and paper references. Analysis uses all114 EXP-015 grid pairs, all/inbound populations;19 consecutive observations per camera/window (~1.9s).
- Context-adjusted temporal rank correlations have mixed signs. Late RGB flow/error rho0.981/0.921 vs uncertainty adjusted0.120/0.100 (reprojection/epipolar); flow/error share prediction errors, so this is diagnostic.
- Limited positives remain (early RGB reprojection adjusted0.414; late thermal epipolar0.290). Five of12 all-grid raw associations reverse sign across chronological9/10 halves. Inbound filtering flips adjusted reprojection sign for late RGB and middle thermal. No general additional temporal signal or fusion rule established.
- 60 tests pass;750 hashes unchanged;24 independent numeric partial-correlation equivalence checks pass. Source data/pipeline unchanged. Full context/median/rank-residual arrays saved.
- Official MAC-VO v2 methods and MACThermal thesis sections inspected; see `docs/research/literature-2026-10-20.md`. Published thermal preprocessing recipe is now a concrete audit target, not proof of our input equivalence. No MSO/MAC-I² novelty review claimed.
- Evidence: `experiments/EXP-016-temporal-context.md`, `results/temporal-context/run-001/`; Box `backups/2026-10-20-exp-016` requires EXP-013/014/015.
- Next gate before fusion training: trace thermal processed-topic conversion to published recipe and verify signed geometry, then evaluate longer recordings with independent reference. Current phase remains characterization with validation in parallel.

### 2026-10-20: night fixed-grid geometric consistency (EXP-015)

- Completed all114 saved pairs including failed late RGB without using VO trajectories.
- RGB early/middle/late median reprojection5.67/232.42/139.06px; depth-independent Sampson0.776/47.578/35.687px. Inbound/depth sensitivities preserve the contrast.
- Within-pair rank associations vary: RGB middle Sampson rho=-0.134; thermal late=-0.163. Window warning evidence strengthens, pointwise calibrated reliability is unproven.
- 54 tests pass, all631 hashes unchanged,160056 grid samples; full point arrays and masks saved. No inference or defaults changed.
- Evidence: `experiments/EXP-015-night-geometry.md`, `results/night-geometry/run-001/`. Box snapshot `backups/2026-10-20-exp-015` with EXP-013/014 prerequisites.
- Next bounded analysis: within-window temporal covariation and repeatability against flow/motion/depth context, with no fitted fusion rule. Recorded SLAM and provisional calibration still limit accuracy claims.

### 2026-10-20: night raw-uncertainty/retention characterization (EXP-014)

- User authorized the EXP-013 follow-up. Enabled only the existing recorder on
  the same six night cases/runtime/settings. All five available graphs/poses
  and six initial depth arrays are bitwise unchanged; configs/status/error and
  input/model provenance match. Late RGB still fails after saving diagnostics.
  All 114 pair files, 45 tests and 514 frozen file checks pass.
- RGB median raw fixed-grid trace (px²), early/middle/late: 5.74/6499.89/6435.59.
  Selected/inbound/retained totals: early 3800/3791/3791, middle 4/4/4, late 0/0/0.
  Starvation occurs at selection before inbound/outlier filtering. Grid q<100
  fraction median 91.83%/0%/0%; middle at most 0.111%, late always zero. High raw
  uncertainty spans all image thirds; no region mask or downstream weight fix
  is justified by this availability diagnostic.
- Raw `<10 retained` labels mark all 19 middle and 19 late RGB pairs, thermal
  0/57. Native MotionInterpolate clears first/last two motion flags, explaining
  middle's saved 15 flags; cleared endpoint flags do not prove tracking. No
  native code changed. Late RGB has no graph, so no graph equality/export claim.
- RGB contemporaneous AUROC 1.0 and Spearman -0.8255 on 57 dependent pairs are
  descriptive/selection-coupled, not independent error calibration or early
  prediction. Thermal AUROC undefined (one class), Spearman +0.4195. Camera-local
  variances are not physically comparable confidence estimates.
- Evidence: EXP-014 and `results/night-uncertainty/run-001/`. Keep defaults.
  Next proposed bounded measurement: fixed-grid reference-conditioned
  reprojection/epipolar consistency including rejected points, with motion/depth
  strata and no fitted corrections; SLAM/calibration limits remain explicit.

### 2026-10-20: separate night recording validation completed (EXP-013)

- User added `night_thermal_so_salon2.zip` to Box folder `428559630945`.
  Recording audit identifies `20251027/night_thermal_so_salon2`, distinct from
  day, with nonoverlapping source timestamps. Staged metadata and 240 images
  by ZIP ranges/CRCs. Original checkpoints restored with matching archive/model
  hashes; Blackwell runtime rebuilt and strict GPU checkpoint inference passes.
- Frozen protocol unchanged: 1,149 paired groups, starts 123/619/789 within
  thirds, 20 groups each, body displacements 1.7623/6.1707/0.2613 m. Same
  provisional thermal geometry; all 13 supplied transforms match the archive.
  Frozen 481 source/input files and runtime settings before six baselines.
- Thermal tracks all three windows (3748/3677/3313 matches; no interpolation).
  RGB early tracks (3791 matches); middle retains four matches with 15
  interpolated frames; late fails with no retained matches. Both RGB failures
  remain reported and are unavailable for valid fixed-prior replay metrics.
- All four eligible learned replay controls pass (max 1.82e-7 m / 7.51e-7 deg),
  all 228 pair solves and invariants pass, precision mass error <=5.62e-8,
  37 tests pass, and all frozen/baseline hashes remain unchanged.
- U-only transfer is mixed: night thermal middle accumulated T/R worsens
  0.7670 m/2.6308 deg → 0.9822 m/4.3995 deg; early T improves but R worsens;
  late changes little. No u-only default change. The technical controls pass
  while scientific transfer is not consistent. Recorded SLAM remains a proxy.
- Evidence: `experiments/EXP-013-separate-clip-validation.md`,
  `results/separate-clip/night-run-001/` and `night-audit/`. Next proposed step:
  unchanged opt-in raw-uncertainty recording on the fixed night windows,
  graph-preservation checks, and uncertainty/retention characterization around
  RGB failures; no further weight tuning or fusion claim from these windows.

### 2026-10-20: separate-clip validation protocol prepared; data required

- User authorized the next validation gate after EXP-012. Authenticated Box
  listing of folder `428559630945` shows only the previously evaluated
  `1_salon_mac_cov_day.zip` and backups; the account root's `ore-ms` entry is
  the same folder. No second dataset was found there. No NAS mount is available.
- Asked the user for a different clip's authorized Box path/link or staging
  from `20251027`/`20251106`, including all four processed streams, calibration,
  acquisition timestamps, TF and recorded odometry. A renamed copy or another
  window of the original recording does not satisfy separate-clip validation.
- EXP-013 preregisters unchanged learned/u-only/both-channel native replay,
  three disjoint reference-motion-selected 20-group windows across clip thirds,
  original pairing/gap bounds, and both local/accumulated translation/rotation.
  Source/protocol fingerprints saved before new outcomes. No new clip, selected
  windows, inference or validation results exist yet; default weighting unchanged.

### 2026-10-19: channel-specific native replay completed (EXP-012)

- User requested the next experiment. Frozen four mass-preserving arms before
  evaluation: learned, u-only, v-only and both-channel redistribution. Selected
  channels use per-pair harmonic-mean variance; unselected channels remain
  bitwise unchanged. Disparity, geometry, priors and native solver stay fixed.
- All 456 pair solves, 31 focused tests and 63 frozen file checks pass. Learned
  and both-channel controls reproduce EXP-011 within 2.23e-16 m / 3.74e-16 deg.
  Precision mass maximum relative change 5.69e-8; all channel/nonpixel checks pass.
- Late thermal accumulated translation/rotation RMS: learned 2.8966 m/1.7129 deg;
  u-only 1.4812 m/2.0122 deg; v-only 2.4742 m/4.0743 deg; both 0.7979 m/4.2469 deg.
  U-only provides a more favorable late-window tradeoff than both here, but
  early local translation and late local rotation still worsen versus learned.
  V-only worsens accumulated thermal rotation in all three windows. Effects are
  conditional and not additive; no region or physical failure cause identified.
- Results: `results/channel-mass/run-001/`; report: EXP-012. Existing CPU image
  reused; no new inference/data download. EXP-011 inputs/source/outputs unchanged.
- Keep defaults. Next gate is a separate-clip test of the unchanged u-only rule
  with learned and both-channel controls, plus thermal geometry/reference
  validation. Do not treat further same-clip tuning as independent validation.

### 2026-10-19: native precision-mass replay completed (EXP-011)

- User authorized the EXP-010 follow-up. Frozen three arms before replay:
  learned, original fixed_uv, and fixed_uv_mass (per-pair/channel harmonic-mean
  variance, preserving u/v total inverse variance before Huber). Native solver,
  disparity, matches, depth, points and initialization remain unchanged.
- Restored five camera/reference metadata files via 8.7 MB of authenticated
  ZIP range reads; selected-member CRCs and all three historical odometry hashes
  pass. Built `macvo:replay-cpu` with Torch 2.7.1+cpu/PyPose 0.9.5; no GPU image,
  checkpoint or full dataset download required. Runtime is reproducible from
  `Docker/Dockerfile.replay`; package freeze/image ID archived with the results.
- All 342 pair solves, 21 focused test cases, source/nonpixel fingerprints and
  48 frozen file hashes pass. Controls match EXP-009 local motions within
  2.23e-16 m / 3.74e-16 degrees. Mass relative change is at most 5.69e-8.
- Late thermal accumulated translation RMS, learned/fixed/mass-preserved:
  2.8966/0.7898/0.7979 m; rotation 1.7129/4.3745/4.2469 degrees. Translation
  benefit survives without increasing total pixel precision; rotation penalty
  persists. This is fixed-input same-clip SLAM-proxy evidence, not new
  end-to-end VO or grounds for replacing defaults. Results and full tables:
  `results/precision-mass/run-001/`, `experiments/EXP-011-precision-mass.md`.
- Keep defaults. Next proposed bounded test: u-only versus v-only redistribution
  with channel mass preserved and both-channel/learned controls, reporting
  local and accumulated translation AND rotation. Separate-clip evidence,
  thermal geometry/preprocessing and comparable confidence remain unresolved.

### 2026-10-19: Box restored and EXP-010 completed

- User reauthorized rclone. Folder `428559630945` contains the preserved
  `backups/2026-10-18-3ac07f047/` snapshot. Artifact archive SHA-256 matches
  its manifest; selected graphs, weight outputs, audits and calibration were
  restored without overwriting existing files. All 18 frozen EXP-009 source
  hashes match before/after the new analysis. The active private config is
  `/root/.config/rclone/ore-ms-recovery.conf`; use `--config` explicitly.
- All six common-pose cases complete (114 pairs, two arms each). Full and
  group-deleted Jacobians have rank six; precision/count/leverage partition
  checks pass. Results: `results/weight-influence/run-001/`, with per-pair JSON,
  summary tables, figure and provenance. No GPU/image/weight download needed.
- Late thermal upper third: median feature share 56.6%, learned u/v precision
  74.0%/73.5%; lower third 10.9% features but only 1.4%/0.4% precision. Fixed_uv
  increases lower-third leverage 9.4%→25.4%. Upper-third linear deletion
  sensitivity changes 0.343→0.157 m and 0.623→0.283 degrees. These are
  pre-Huber local step differences, not new pose errors or causal proof.
- Keep VO defaults. Proposed next test: native replay with fixed u/v precision
  rescaled per pair to preserve each learned channel's total, retaining learned
  and fixed_uv controls. This separates precision mass from redistribution
  before masks/fusion. EXP-009's rotation penalty and geometry/SLAM-proxy limits
  still apply. Full protocol, control tables and evidence: EXP-010.

### 2026-10-19: next diagnostic prepared; real-data run pending

- Added `Scripts/Setup/td3_weight_influence.py`: NumPy-only common-original-pose
  learned/fixed_uv precision shares, pre-Huber leverage, and linearized group
  deletion sensitivity, with separate translation/rotation summaries. Uses
  frozen EXP-009 constants and verifies source hashes; no model or VO defaults
  changed. Four synthetic tests pass. Protocol/limits: EXP-010.
- Historical ignored data, checkpoints and results and `macvo:blackwell` are
  absent in this workspace. Both submodules remain initialized. A private Box
  recovery config was prepared while preserving the original; listing failed.
  No restored access or real-data result is claimed. User was asked where
  `results/uncertainty/run-001` and `results/weight-ablation/run-001` were saved.
- Restore those graphs and the frozen plan first; the diagnostic does not need
  GPUs, weights, dataset images or independent reference poses. Run all six
  window/modality cases with verified image sizes before interpreting results.

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

## EXP-021: MSA source calibration (2026-10-21)

User supplied msa_cal.zip from NAS offroad/calibration/msa_cal. Six text files,
no camera JPEGs. January26,2025 MSA source uses equidistant intrinsics and
common-LiDAR-parent extrinsics; baseline0.326186422m. All24 fixed EXP-020 pair
sets/3147 inliers pass inherited per-image checks without fitting/rematching.
10 relevant tests and independent validation pass. See
[EXP-021](experiments/EXP-021-msa-calibration-audit.md). This is a credible source
for a separate raw-image replay, not an adopted VO calibration. Source-export
status explicitly leaves October applicability unconfirmed. Do not reuse old
processed R/P/depths as if generated under MSA. No vehicle-body anchor inferred.
Next source: separately listed thermal JPEG pair/applicability notes.

MSA source-image follow-up: [EXP-022](experiments/EXP-022-msa-source-images.md)
checks the subsequently supplied thermal JPEG pair. Both all242 mutual matches
and214 F inliers pass the retained image criteria without calibration fitting.
This supports separate raw/depth/VO replay; metric/body/timing accuracy and
October applicability remain unconfirmed. Existing inputs/defaults unchanged.

## EXP-023: Raw-first MSA bounded VO (2026-10-21)

Completed six20-frame thermal replays, fixed old windows/config/checkpoint/seed.
All240 old images reproduce exactly from selected raw data; one duplicate raw
timestamp resolved by source index1518 AND exact image replay. New MSA data
saved separately. Day translation proxy RMSE1.517/0.703/2.897m becomes
0.076/0.160/0.179m. Night0.301/0.767/0.142m becomes0.176/0.742/0.141m;
night early/late rotation worsen and late motion remains nearly zero. Same
original nominal body anchor, each own rectification; SLAM is still a proxy.
All25 tests,788/50 frozen hashes and independent controls pass. See
[EXP-023](experiments/EXP-023-msa-preprocessing-vo.md). Use MSA as experimental
thermal baseline; recompute uncertainty/error diagnostics before fusion claims.
No production defaults or old outputs replaced.

[EXP-024](experiments/EXP-024-msa-uncertainty-geometry.md) recomputes
original/MSA thermal uncertainty against the same SLAM proxy across all six
windows. Day reprojection errors fall, but spatial ranking does not become a
consistent temporal reliability score. Late-night epipolar association remains
negative, and depth/visibility sensitivity weakens late reprojection ranking.
No new inference, weights or fusion rule. MSA remains the experimental baseline.

[EXP-025](experiments/EXP-025-salon-so-day-validation.md) stages the added
`20251027/0_salon_so_day`:1335 synchronized groups/~163s, no timestamp overlap
with earlier two recordings. Six RGB/MSA-thermal20-frame runs complete. Thermal
translation proxy RMSE0.185/0.198/0.084m; RGB0.381/0.122/0.130m, with RGB lower
rotation error in all3. Source camera metadata matches;120 old thermal replays
exact. GNSS-labelled data are present, but public converter maps their name
to `/odometry/filtered_odom`; upstream sensor/fusion/accuracy/timing provenance
requires confirmation. Keep SLAM-proxy interpretation and unchanged defaults.

[EXP-026](experiments/EXP-026-long-salon-so-day.md) extends the added
day capture to3 fixed100-frame RGB/MSA-thermal windows, excluding old samples.
Strict150ms continuity failed middle/late; preregistered250ms revision retained
actual gaps (9.9/10.2/10.0s). All6 attempts complete with no interpolation.
Near-static early spatial uncertainty/reprojection ranking is negative; moving
windows positive. Image/translation/rotation uncertainty associations differ,
and accumulated drift versus per-step errors can rank cameras differently.
SLAM/body anchor provisional;21 tests pass. No defaults/weights changed.

[EXP-027](experiments/EXP-027-night-low-motion.md) checks late-night
thermal low motion with independent LK tracking on fixed saved source pixels.
LK0.053px/model0.083px,model-LKerror0.103px on95.54% valid grid tracks;
LK-SLAM/predicted-deptherror0.925px. Texture/region sensitivity persists; no
exact decoded frame duplication. The0.0085m VO vs0.261m SLAM discrepancy remains,
but network-alone visible-motion suppression is unsupported. Reference/timing/
depth/calibration causes unresolved; no tuning. Exact bag interval
2025-10-27 22:13:40.207–22:13:42.107 EDT.7 tests/222 source hashes pass.

## EXP-028: Reference/timestamp audit (2026-10-21)

[EXP-028](experiments/EXP-028-reference-timing.md) checks the same five windows
without inference. All120 night raw/processed acquisition joins pass;1,952
binding-grid SLAM samples and1,916 supported TF samples reproduce the source-rate
series. All55 prescribed window/offset cases are retained without fitting; late-night
LK–SLAM/depth disagreement remains. Auxiliary `/odometry/filtered_odom` reports
0.0161m net displacement at gps_frame versus SLAM vehicle0.2608m over the exact
late interval; median supplied speeds0.0097 vs0.1030m/s. All57 mux and38 teleop
command samples are zero, while auto requests remain nonzero. These are distinct
frames/streams, not independent GT or a verified standstill.52 selected ZIP
metadata members pass CRC/SHA;11 tests and402 frozen hashes pass; independent
controls check1,045 pair/offset cases and882,684 projection/Sampson scores each.
No VO/settings/corrections changed. Source question: vehicle stopped/creeping
at2025-10-27 22:13:40.207–22:13:42.107 EDT; filtered-odom generator/sensors/SLAM
inputs, timing/quality/covariance and gps_frame extrinsic. Box recovery
`backups/2026-10-21-exp-028` requires prior EXP-023/026/027 context. Do not label
late-night thermal-network failure or calibrate fusion against an unvalidated
physical error target. MSA remains the experimental thermal baseline.

## Filtered-odometry source audit (2026-10-21)

The [dated source note](docs/research/filtered-odometry-provenance-2026-10-21.md)
records a stronger historical lead: TartanDrive2.0 describes NovAtel GPS/IMU
poses, and a2022 CMU report attributes `/odometry/filtered_odom` to legacy
`physics_atv_ws/src/novatel_span`. It also lists NovAtel IMU as an SO input.
Neither establishes the2025 graph or independence. Pinned public candidate
SPAN code accepts external odometry aiding and stamps with ROS time; those
behaviors are questions for the actual driver, not findings about our recording.
Another public EKF uses the same topic with different inputs/frames, so the name
alone does not identify an estimator. Nine public files pass remote SHA checks;
all402 EXP-028 scientific hashes remain unchanged. No new inference/tests or
corrections. Connected mail searches returned no matches; GitHub integration
was offered but remains uninstalled/unconnected. Exact source/launch/config,
gps_frame/lever arm and original covariance/receiver quality for the late interval
are still required. A local request is prepared; no messages sent. Box snapshot:
`backups/2026-10-21-filtered-odom-provenance`, with prior EXP-028 prerequisites.

User follow-up: a supplied Slack-search summary reports a February2024 ATV
launch/mesh discussion and `/home/physics_atv/physics_atv_ws/` as a workspace
lead, with Matthew Sivaprakasam or Wenshan in `#learningphysics` suggested as
contacts. Original messages were not independently retrieved; no2025 driver
or recording-specific source was confirmed. That path and candidate packages
are not present in this compute environment. The source note includes read-only
file searches for the actual ATV computer; present-day HEAD alone is not proof
of the recording revision. A refined local request is prepared; nothing sent.

## EXP-029: Stereo-image correspondence consistency (2026-10-21)

[EXP-029](experiments/EXP-029-stereo-image-consistency.md) progresses while
reference replies are pending:18 saved first stereo pairs (12 MSA-thermal,
6 day RGB controls) compared with an independent exhaustive/reciprocal NCC
matcher on22,896 fixed source pixels. No pose reference or inference. Thermal
primary support6.75–30.26% versus RGB41.11–70.22%, with different views/native
resolutions; these are matcher availability, not physical modality rankings.
Late-night thermal supports68/1,008 points, median model/matcher difference
0.423px and64/64 eligible signs agreeing;940 locations remain unvalidated.
Stereo-variance/discrepancy association is weak/mixed and selection-dependent,
not a temporal-flow or pose-reliability calibration. Run-001 revealed float32
NCC gate sensitivity; preserved fully. Mean-centering before NCC stabilizes
run-002 with identical scientific thresholds/inputs.9 tests/97 frozen hashes
pass,17,046 accepted scalar NCC checks/108 summaries and1,008 independent
night-late exhaustive searches pass at the original1e-4 NCC tolerance, with
zero remaining float64 forward gate changes. All402 EXP-028 hashes unchanged.
Keep MSA/VO defaults, no depth/scale/sign/fusion correction. Box recovery
`backups/2026-10-21-exp-029` requires prior EXP-023/025/026 input/signed-output
context; no weights/GPU required for this saved-output diagnostic. Exact
recording odometry/quality/timing and physical frame provenance still pending.

## EXP-030: Temporal angular image consistency (2026-10-21)

[EXP-030](experiments/EXP-030-temporal-angular-consistency.md) checks342 saved temporal pairs in nominal angular units without a pose reference. Median spatial uncertainty/LK-disagreement rho ranges−0.187–0.572 for thermal and0.361–0.594 for RGB. Late-night thermal remains negative despite94–97% LK support; normalization is not physical confidence calibration. Seven tests,2,394 independent ray/covariance checks,95 exactly reproduced prior LK pairs and729 frozen hashes pass;402 earlier audit hashes unchanged. Keep defaults and pending provenance gates. Box recovery: `backups/2026-10-21-exp-030`.

## EXP-031: Fresh night recording validation (2026-10-22)

[EXP-031](experiments/EXP-031-new-night-validation.md) stages the new `night_mac_salon_warehousefig8`:1,500 paired groups, no overlap with three prior recordings. Three fixed first-eligible20-frame windows,751 prepared hashes. All three MSA-thermal attempts and early RGB complete without interpolation; middle/late RGB retain0 matches and fail, with0 independent LK support. Preserve failures and unavailable errors. Thermal median spatial angular uncertainty/disagreement rho0.005/−0.223/0.529; operational availability differs from calibrated reliability.24 tests,600 identical image decodes,798 scalar/finite-difference checks,114 pairs/160,056 points and267 analysis hashes pass; prior729/402/97 hashes unchanged. Rebuilt GPU image f429af4afa23 has recorded transitive version differences; CPU analysis unchanged. Keep defaults/provisional calibration/reference gates. No more upload needed now: unused intervals of this archive suffice for a next bounded study. Box recovery `backups/2026-10-22-exp-031`.

## EXP-032: Longer night windows and tracking-state audit (2026-10-22)

[EXP-032](experiments/EXP-032-long-night-validation.md) extends the new night recording to three nonoverlapping100-group midpoint blocks (14.9/9.9/11.0s), preserving30/0/1 gaps>150ms. All thermal attempts have0 pre-refinement lost pairs; RGB has99/97/99 out of99. Middle/late RGB raw execution passes are interpolation-dominated; existing MotionInterpolate aliases/clears boundary flags, understating original losses (97→94,99→95). Preserve reports/settings; audit raw counts/logs. Thermal median spatial uncertainty/LK-disagreement rho0.407/0.397/0.537; temporal/gap effects vary, not physical confidence calibration.25 tests,4,158 scalar/finite-difference checks,594 pairs/833,976 points,54 strata and3,041/1,229 hashes pass; prior751/267/729/402/97 unchanged. Box recovery `backups/2026-10-22-exp-032`. Next software task: pre-refinement diagnostics with unchanged poses. For future numerical RGB/thermal error comparison, request another night recording with visible RGB detail; names first. No more data needed to reproduce this audit.

## Owner replies: calibration, GNSS and checkpoint training (2026-10-22)

[Attributed source update](docs/research/source-replies-2026-10-22.md): user-pasted Geethika confirms MSA applicability; Matthew identifies direct GNSS-system odometry with approximate UTM axis flip, occasional jitter and uncertain RTK membership. Geethika tentatively believes GNSS odometry is not consumed by SO. Use MSA as owner-supported working baseline and proceed with separately labelled GNSS translation checks, retaining independence/quality/axis/origin/timing qualifications. Do not call GNSS centimetre GT or assume no onboard filtering/shared IMU. Existing export13 fields omit covariance/solution status. Need exact axis conversion and applicable gps_frame/lever arm; original covariance/fix/RTK logs if available. No new inference/reference/default changes. Day MSA-vs-dataset translation proxy RMSE improved77–95%, night mixed. Yifei Liu is the suggested source for MACT_Offroad.pth training recordings;20251027 training/finetuning membership unknown, so “new to our analysis” is not verified training-held-out. Draft follow-ups preserved; nothing sent. Box recovery `backups/2026-10-22-source-replies`.

## EXP-033: GNSS comparison and tracking diagnostics (2026-10-22)

[EXP-033](experiments/EXP-033-gnss-and-tracking-reporting.md) completes GNSS scalar translation cross-checks on15 fixed cases x3 offsets and a versioned reporting fix. New `run_td3_execution_v2.py` preserves pre-refinement support/flags even on zero-observation failure; original runner/postprocessor unchanged. Three GPU controls reproduce5,753 arrays bitwise; middle RGB correctly exposes97 original unsupported pairs vs94 stored flags.13 tests pass. GNSS original late-night0.0161m vs VO0.0085m/camera SLAM0.2610m; canonical early and new short late also show reference-dependent disagreement. Longer thermal GNSS31.71/32.52/31.47m vs VO26.45/31.72/26.74m retains early/late shortfall. Magnitudes from distinct physical origins are not camera-pose errors or calibrated GT; no axes/latency/scale/lever arm fitted.45 comparisons/2,340 queries validated, max bracket41.56ms;828 final hashes,291 reporting-control hashes and previous scientific hashes unchanged. GNSS phase001 source preserved after scope-label correction; phase002 arrays bitwise identical. Owner axes/RTK/origin/timing and checkpoint train split still pending. No more image upload needed for these checks. Box `backups/2026-10-22-exp-033`.

## EXP-034: Fixed-block thermal/GNSS diagnostic (2026-10-22)

[EXP-034](experiments/EXP-034-gnss-block-localization.md) partitions all three EXP032 thermal windows at fixed frames0/20/40/60/80/99.45 block/offset cases,90 independently verified magnitudes. Largest disagreement localizes to gap-containing blocks: late1.2s-gap block GNSS10.22m vs VO5.70m; other late blocks differ−0.33m to+0.03m. Early gap-heavy blocks differ−4.04/−0.97m. Different origins/motion/texture/turning and GNSS quality prevent causal gap or camera-error claims; no scale/time/axis/default correction. Prior828/3041/1229 hashes unchanged. Latest attributed Matthew reply tentatively recalls no RTK; satellite-map use does not certify accuracy. No new upload needed for this diagnostic; visible-RGB night data would help actual cross-modal numerical comparison. Box target backups/2026-10-22-exp-034.

## EXP-035: New waypoint-skip2 night recording readiness (2026-10-22)

[EXP-035](experiments/EXP-035-waypoint-skip2-readiness.md): Box night_so_waypoint_skip2.zip20.33GB,54,855entries;103metadata+360selectedimages+12RGBscout verified by member CRC/SHA, no fullZIP hash.549s capture. Strict2ms stereo/30ms cross pairing184groups/no middle20-frame eligible window; separately labelled5ms diagnostic281groups, starts0/98/195. Nearest thermal−RGBmedian−43.46ms, no time correction. Source RGB LK median4.06%/.056%/0%, thermal95.9%/69.5%/95.3%; prior-dark controls and12even sourceRGBframes still mostly dark. Sourceprocessed thermal only, no MSA or inference.320,112scalar gates/228pairs,3tests,728analysis hashes; prior828/3041/1229/751/267 unchanged. Useful thermal capture, limited sampledRGB comparison. Faint late scout intervals remain possible targets; visibleRGB night preview/names preferred before another large upload. Box target backups/2026-10-22-exp-035.
## Resumption context: remaining night recordings (2026-10-22)

The user supplied the closing conversation from the previous session: after
EXP-035, they requested screening all remaining `20251027` night-session
folders together because folder names do not identify brighter RGB intervals.
This supersedes earlier requests to have the user select a bright clip first.
Screen representative previews, RGB detail/tracking support and stereo/cross-modal
timing before deciding which recordings support further VO comparisons.

On resumption, authenticated Box folder `428559630945` contains four additional
night ZIPs beyond the three already inspected (`night_thermal_so_salon2`,
`night_mac_salon_warehousefig8`, `night_so_waypoint_skip2`):
`night_macthermal_salon_redcourse_waypointskip1`,
`night_teleop_so_salon_fig8`, `night_thermal_so_salon`, and
`night_thermal_so_salon3`. Presence is verified; their image readiness is not.
Screen these available archives before requesting another upload. A public
dataset is a fallback if the available session lacks usable RGB; the previously
suggested MS2 access and suitability have not been reverified in this session.

The Box incremental bundles recover eleven commits after `46784eb6e`, through
`9e40ba724` (EXP-035), including source replies and EXP-028–034. The research
branch was fast-forwarded to that preserved commit. Archive SHA256 and all listed
member hashes passed for the eleven recovered backup snapshots. This restores
historical evidence, not a new execution of the experiments. Earlier model/data
snapshots remain prerequisites for full replay.

## EXP-036: Remaining night archive batch screen (2026-10-22)

[EXP-036](experiments/EXP-036-night-batch-screen.md) screens all four additionally
available night archives using fixed twelve-image scouts and three independent
20-frame windows per modality.569 selected images,456 pairs,640224 grid points;
1985521 computational checks pass. RGB is mostly dark. Redcourse-waypointskip1
late RGB frames4173–4192 show illuminated vegetation,10.56% median LK support
(2.78–19.78% pair range),8.72% tighter-gate/4.33% texture-restricted support;
other fixed RGB windows0–4.06%. Thermal window medians69.2–96.9%. Redcourse
nearest thermal-minus-RGB median−41.48ms; thermal-salon3 has large timing holes
(abs cross-modal p99~22s), despite near-zero median. No time correction, model
inference or VO/calibration/weight change. Prefer bounded sustained-support and
pairing checks around redcourse's illuminated interval before seeking a public
fallback. Outcome-selected follow-up is exploratory, not independent evaluation.
No additional upload needed for that candidate. Full plans, images, masks,
source/logs and validation preserved at Box target backups/2026-10-22-exp-036.

## EXP-037: Redcourse illuminated-interval persistence (2026-10-22)

[EXP-037](experiments/EXP-037-redcourse-lit-interval.md) expands fixed redcourse
RGB4173–4192 by100 acquisitions each side:220frames/21.9s,874 unique images
including both stereo views/nearby thermal. Original190 RGB arrays reproduce
exactly;1088 previous hashes unchanged. RGB-left adjacent median support
before/original/after5.78/10.56/4.22%, but only6/1800 identities survive original
20frames (0 initially textured). Precursor4153–4172 is stronger:74/1800 survive,
43 initially textured; retain as exploratory candidate, not sustained VO proof.
Thermal original20 retains308/1008. Three nearest thermal frame reuses are
explicitly removed from tracking. Joint2ms stereo/30ms cross timing admits
16/220rows;50ms cross sensitivity admits84, both longest continuous unique
run1frame. Median cross offset−40.819ms; no correction/threshold adoption.
150ms thermal-boundary excesses are only95–96ns, not major dropout claims.
2984570 computational checks pass; no learned inference or calibration/weights/
pose/default changes. Current limits yield no continuous paired window; justify
pairing separately before learned comparisons, or use a synchronized public-data
alternative. Recovery target backups/2026-10-22-exp-037; no new upload needed.

## EXP-038: Direct MAC relative covariance ranking (2026-10-22)

[EXP-038](experiments/EXP-038-mac-relative-ranking.md) returns to MAC uncertainty
using saved outputs, with no inference/training/new images. Six paired day
windows from the same previously inspected recording,114 paired transitions;
all existing stereo/cross timing gates pass. Primary raw temporal covariance
trace in nominal ray units selects lower LK angular disagreement on108/114,
versus112/114 alwaysRGB; six thermal switches help1/hurt5. Mean selected
D0.8575mrad vsRGB0.8212mrad(+4.42%). Native-pixel rule99/114. Only2 targets
favor thermal, so high agreement/balanced recall is not strong general evidence.
Predeclared full-grid-U sensitivity gives113/114 with one helpful switch and
−0.86% meanD vsRGB; preserve as population-sensitive hypothesis,not tuned method.
Tight/textured rules remain worse than their RGB baselines. Pooled log-ratio
rho0.499 contrasts with within-window−0.418–0.711; no significance/held-out
claim. LK disagreement is not physical error; views/features differ. This tests
raw temporal matching covariance,not full3D/pose covariance.231 frozen hashes/
11452 independent controls pass. Float32-median discrepancy fixed in run002
using prior float64 convention; no policy choices changed; old attempt preserved.
Keep weights/defaults; no fusion adopted. Recovery target backups/2026-10-22-exp-038
contains saved inputs for standalone CPU replay; no models/GPU needed.

## EXP-039: Frozen rules on a different day capture (2026-10-22)

[EXP-039](experiments/EXP-039-mac-recording-replication.md) applies EXP038's
unchanged primary/full-grid angular covariance rules to three historical
1_salon_mac_cov_day windows,57 paired transitions,disjoint from0_salon_so_day.
Recover saved RGB predictions+60 source images;derive angular/LK quantities with
unchanged EXP030 helpers;reuse MSA-thermal arrays. No model inference/training.
Both angular rules chooseRGB57/57,and all targets favorRGB:equal to alwaysRGB,
no switches/no added benefit. There are no thermal-favored examples,so EXP038's
one helpful full-grid switch is neither validated nor refuted as a general gain.
Native-pixel primary makes3 harmful switches(full-grid9). Primary ratio rho
−0.728 here vs+0.499 previously;within-window−0.221/−0.005/−0.416. Keep rules/
weights/defaults unchanged; no stable ratio mapping established.314020 controls
pass,including399 scalar geometry checks. Initial validator precision mismatch
corrected to honor historical float32 inverseK before float64 ray algebra;no
output changes. Different recording is previously examined/same-date and not
network-training-held-out. Matching disagreement remains a proxy,not physical
accuracy or full3D/pose covariance. Need relative-error variation while both
cameras remain observable and a stronger error target. Recovery target
backups/2026-10-22-exp-039;all prepared CPU inputs preserved,no models/GPU needed.

2026-10-23 — EXP040 public-data audit: MS² user-supplied download scripts verified.
Pinned official loader flags uint16 thermal, mm calibration translation, depth/256
and network-filtered depth provenance; actual pose schema must be inspected.
Fixed pilot candidates: first lexical official test sequence per day/night/rain.
STheReO Valley Evening metadata downloaded; pose gaps max7.4s, thermal projected
vs raw baseline differs0.564%; no fixes or camera-pose conversion made. Images
return Drive quota-exceeded HTML; bounded scout failed safely. TartanRGBT raw
MACVO poses are circular as independent MAC labels. 2753 STheReO plus222538 MS² schema/hash controls pass;22 source rotations flagged;
no inference/training/fusion. See EXP040 and backup2026-10-23-exp-040.

2026-10-23 — EXP041 MS² night _2021-08-13-21-18-04: acquired39.732GB sync/depth,
small odometry already staged; preserve first20 IDs without outcome selection.
80 stereo source images+120 single/multi/filtered depthPNG+40poses,calibration,
full timestamps; raw thermaluint16 and metric depth/256 confirmed. All pilot
poses pass1e-5 rigidity screen; reference translation changes~2.4cm/1.93s.
5355 source+37 independent timing controls pass,but full10131 thermal stereo
recorded offsets15.219–49.754ms (median33.766ms),nearest-right minimum also15.219ms;
0 frames satisfy inherited2ms-stereo/30ms-cross gate. Official setup says thermal
sync uses left camera signal; timestamp semantics unresolved,not proof of physical
async. Do not fit clock shift or relax gate by default. No inference/training.
Full archives remain local; Box2026-10-23-exp-041 contains boundedpilot replay.
Next:ground timestamp semantics/protocol before selecting movinginterval/MAC test.

2026-10-23 — EXP042:timestamp meaning remains unresolved by public papers,
supplement,archive notes,official loaders/issues or accessreply(user says no notes).
Both official loaders use sorted same-ID stereo pairs without timestamp reads;
sourceprotocol differs from exposure verification. Thermal signedR−L spans−49.754
to+47.091ms,4572negative/5559positive and8810signchanges;no constant can put all
recorded residuals within2ms(48.422ms worst-case lowerbound),nor fixed±1rightindex.
No timestamps/pairing/gates changed.10174 controls pass;EXP041 inputs unchanged.
Prepared20-ID inputmanifest with rawimageSHA/dtype,all4times,K/B,depthprovenance,
pose direction explicitlyunverified. ExistingGeneralStereo fabricates timestamps
and uint8COLOR decoding collapses firstthermal619values→5;TD3 rejectsuint16.
Next implement model-compatible MS² adapter;do not use displaycontrast as network
preprocessing. Publishedsourceprotocol permits labelled preliminary diagnostics,
not physical same-time calibration claims. Authorquestion drafted in EXP042,not
sent. Box2026-10-23-exp-042 preserves sources/timestamps/manifest;restoreEXP041
for referenced pilot pixels/poses/depth. No newmodels/inference/training.

2026-10-23 — EXP043:MS2ManifestSequence adapter implemented/tested on fixed20IDs.
Direct import DataLoader.Dataset.MS2;no globalregistration/defaultchange. Native
uint16 raw_pair access;thermalmodel frames require explicit
THERMAL_RECIPE=td3-firestereo-v2-clahe-bilateral-v1,exact archivedproducer replay
with1/99orderstatistics,CLAHE2/8×8,bilateral5/20/15. Rawcopiesnotmutated;no repeat
rectification of MS² pixels. Thesis recipe evidence/sourceSHA reconfirmed,but
releasedcheckpointtrainingrecipe/split unverified. Bothcamera integerns retained
via MS2StereoData.right_time_ns;frame time usesexactleft time,not same-timeclaim.
Source-ID clip/collate/resize metadata pass;depthvariants explicit meter/maskAPI;
no inferredgt_pose or auto-selectedgt_depth.14tests+1087 independentcontrols,
80images/120depthmaps/205frozenhashes pass with realCPU Torch2.9.1,PyPose0.7.3,
NP2.2.6,CV4.10. Pytestfirstfailedmissingplugin,thenpassedwithupstreamwarnings.
No GPU/models/inference/default/calibration/timestampchanges. Full bounded replay
preserved Box2026-10-23-exp-043. Next:bounded originalfrontend execution smoke
check;that doesnotresolve physicaltimestamp/ref/trainingprovenance.

2026-10-24 — EXP044: restored self-contained EXP043 archive;277 member hashes
pass. Rebuilt Blackwell runtime with pinned MS² input libraries;14 adapter tests
pass;independent producer replay matches80 input tensors and4 saved first-input
arrays. Official-release RGB checkpoint SHA256
bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a loads using
audited legacy migration.20 stereo+19 temporal native384×1224 calls pass shapes,
float32,finiteness and positive covariance. First stereo/temporal repeats bitwise
identical;39 saved files independently validated;81 input hashes unchanged.
Arrays/metadata:results/ms2-frontend-smoke/rgb-run-001. Thermal still pending:
original Box folder303805616544/file2037543542268 return404;sharedURL or archive
in project folder requested. Do not claim all-modality completion. Added opt-in
smoke runner/independent validator;no defaults,pixels,times,geometry,VO or fusion
changes. Runtime compatibility only;physical timing/reference/training provenance
open. Recovery target backups/2026-10-24-exp-044 excludes models/full archives.

2026-10-24 — EXP044 completed after user supplied original shared Box link
https://cmu.app.box.com/s/daz2l4ei8m864vbhkbk3k2igzz6kxc69 . Resolve via shared_items
and retain BoxApi shared_link header for original file content;direct ID access
alone returns404.318804129-byte macslam archive SHA/CRC verify. RGB weights are
byte-identical to upstream release;MACT_Offroad SHA256
eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a
loads428 entries strictly,without migration. Thermal20 stereo+19 temporal calls
at native256x640 pass shapes,float32,finiteness,positive covariance;independent
saved-array validation passes39/39. Thermal first stereo/temporal repeats both
bitwise identical. Combined RGB+thermal78/78 calls PASS,same runner/inputmanifest;
267 historical scientific/source files unchanged. Full arrays/metadata and combined
validation:results/ms2-frontend-smoke. Runtime compatibility only;timing/reference/
training provenance and uncertainty calibration remain unresolved. No defaults,
VO,fusion or source changes. New complete snapshot backups/2026-10-24-exp-044-complete
preserves both runs;earlier RGB-only snapshot unchanged. Original source URL/hash
now recorded in models/README.md to avoid repeating the shared-access blocker.

2026-10-25 — [Recovery audit](docs/research/recovery-checkpoint-2026-10-25.md): restored newer Box EXP044-complete and fast-forwarded ef6ca6f57→018e7dbcf (two commits). EXP043277/EXP044109 archive-member hashes verified; original checkpoint archive SHA/CRC and both model hashes pass. Rebuilt Blackwell/smoke images;14 adapter tests and independent audits of all78 saved pairs pass,with80 producer-input tensors exact per audit. CUDA matmul passes;startup diagnostics retained. No new model inference or repeat execution;historical repeat flags remain historical. Full source archives not restored. Scientific next step remains justified timing/reference protocol. Session audit backup:backups/2026-10-25-recovery-audit.

2026-10-25 — [EXP045 reference/protocol audit](experiments/EXP-045-ms2-reference-protocol.md): official loader supports intended sequence-initial absolute camera-pose interpretation,but full export/rectification/time association remains unverified. First20 RGB/thermal reference endpoint~2.4cm and rotations~0.00052rad;both direction hypotheses retained,not chosen by output agreement. Calibration identityR/horizontal baseline supports conditional supplied-pair depth disparity target. Frozen primary=single-scan depth,all nonzero/right-in-bounds pixels;multi/filtered sensitivities separate. Coverage141472RGB/184792thermal primary pixels;not physical labels or independent samples.0/20 unchanged joint timing eligibility;raw nav quality samples absent from boundedpilot.958 independentcontrols incl350 historical hashes pass;166 frozen inputs unchanged. No model inference/error scores/GT assignment/default change. Next bounded work can characterize saved stereo outputs against supplied-depth proxy under this protocol;physical temporal/cross-modal claims still require source/owner timing/frame/nav provenance. Box target backups/2026-10-25-exp-045;no extra image upload needed for conditional diagnostic.

2026-10-25 — [EXP046 stereo/depth proxy](experiments/EXP-046-ms2-stereo-depth-proxy.md): executed EXP045 unchanged on40 saved stereo fields,120 image/depth-variant cases. Primary single-scan within-image normalized covariance-trace/error rho median0.353RGB/0.453thermal;positive20/20 each. Common support of all3 label products reduces primary rho0.120RGB/0.385thermal;RGB merged0.091 vsfiltered0.339 at same common coordinates demonstrates reference/population sensitivity. Primary normalized residual medians0.002455RGB/0.003985thermal refer to different scene supports,not camera-winner scores.16,380 independentcontrols pass across1,507,332 selectedpoints/1,920 scalar checks;292 frozen inputs+350 historical hash checks unchanged.0/20 timing eligibility retained. No new inference/physical calibration/temporal-ranking validation/fusion/default change. Box target backups/2026-10-25-exp-046. Next distinguish covariance association from depth/spatial scene/reference effects under fixed strata;physical cross-modal claims still need justified shared labels/timing and independent conditions. No more upload needed for same-pilot controls.

2026-10-25 — [EXP047 depth/spatial controls](experiments/EXP-047-ms2-depth-spatial-controls.md): fixed6 depth bands/3×3grid/min30 on EXP046 saved arrays,240 case/support combinations/720 controls. Primary own-support joint within-bin coefficient0.179RGB/0.388thermal,positive20/20 each,~99.5% retained. Common-support joint0.044RGB/0.365thermal;RGB raw0.120→same-retained0.010 before conditioning,82.04% retained,so maskloss and different rank weighting must be separated. All reference sensitivities retained;RGB filteredcommon0.199 vsprimary0.044 at fixed bin assignments.41,416 independentchecks across16,560 strata/1,643,598 pointmemberships pass;416 frozen inputs/350 historical hashes unchanged.9428 strata sparse,7132 eligible;no gates relaxed.0/20 physical timing unchanged,no inference/fusion/default change. Positive proxy association survives coarsecontrols but physical calibration/cross-modal reliability/generalization unestablished. Next prioritize fixed independent MS² day/rain condition from EXP040 over bin tuning;check existing access/coverage before asking upload. Box target backups/2026-10-25-exp-047.

2026-10-25 — [EXP048 fixed MS² day replication](experiments/EXP-048-ms2-day-replication.md): restored EXP040 pointers/odom; exact HTTP ranges acquired24.905GB day sync/depth archives and staged fixed first20 IDs from_2021-08-06-11-23-45.80 images/120 depth/40poses+20nav records;no nightpixelhash overlap. Decoded calibration/runtime/config/checkpoint/recipe exact to night;78 frontend fields and4 repeats pass,80 input tensors exact. Day reference translation columns change~8m/1.9s;3/20 inherited joint gates pass (night0),no exposure claim or gate change. Nav state2/flags0 all20 does not certify accuracy/RTK. Primary own rawrho0.252RGB/0.392thermal;unchanged jointcoeff0.130/0.149 vsnight0.179/0.388;thermal positive18/20,RGB20/20. Common dayrho0.209/0.449,joint0.087/0.219. All variants retained.18,719 independentsource/frontend/proxy checks across5,774,978points and41,580 controlchecks across16,560strata pass;321/442 frozen inputs and488 historical hashes unchanged. No training/VO/fusion/default/physical calibration claim. Box target backups/2026-10-25-exp-048. Full day archives local;no additional upload needed for fixed later intervals;rain pointers already available. Distinct captures with documented exposure/reference quality would strengthen physical cross-modal study more than images alone.

2026-10-25 — User says current students may not know acquisition/reference details; still waiting for Yifei and Matthew replies. Pursue external dataset validation in parallel. [Source-based options](docs/research/external-validation-options-2026-10-25.md): prioritize CATS for complementary static RGB/thermal stereo error checks with laser-scanned scenes (current access request required);STheReO remains sequential candidate but explicitly includes thermal NUC/asynchrony and fused LIO-SAM/RTK references;NGCLAB offers documented hardware-sync/MoCap but lacks the current same-modality stereo configuration. No new data, access request, author message or method pivot in this shortlist. External validation cannot retroactively certify TD3/MS² timing/calibration/training split.

2026-10-25 — [EXP049 later day windows](experiments/EXP-049-ms2-day-later-windows.md): CATS not yet available per user. Fixed MS² day startsfloor(5810/3)=1936 andfloor(2N/3)=3873,20IDs each,no outcome selection. Cached-index extraction only;original stems/all4 times preserved. New stereo-only runner baseline10 saved arrays bitwiseexact toEXP048;80 new fields/160 producer tensors pass. Proxy primary raw mediansmiddle0.137RGB/0.420thermal,late0.196/0.477. Joint own coefficientsmiddle0.149/0.209,late0.131/0.307 vsopening0.130/0.149;allnewprimaryjointpositive20/20. Same recording,not independent captures;support/reference variation retained.2/20 and9/20 inherited timing gates;no correction/subsetselection.19,604/19,582 source/frontend/proxy checks across10,330,339 selectedpoints and42,083 controlchecks perwindow pass;323/444 frozen inputs and989 historical hashes perwindow unchanged. No training/VO/fusion/default/physicalcalibration claim. Box target backups/2026-10-25-exp-049;no new data download/upload needed. External geometric-label validation and source replies remain the path to stronger claims.

2026-10-25 — [EXP050 timing metadata/scope clarification](experiments/EXP-050-ms2-timing-metadata.md): user asks whether timing is really needed. Correct earlier overemphasis: exact exposure semantics/2ms gate are not universal prerequisites for provided-pair benchmarking;MS² documents synchronization,intended pairing/reference protocol suffices for scoped benchmark work. Keep all old flags/scores,not a veto or thresholdrelaxation. Motion-sensitive physical/temporal/same-point cross-modal claims need appropriate alignment,not necessarily identical sensor acquisition times if compensated. Existing4windows/160 image-modality cases,all variants/supports yield960rows/576 descriptive metadata correlations;8 primary stereo-offset/L2 rhos change signs/magnitude (thermalnight+0.481,day−0.257/−0.018/−0.307). No causal/exposure/sync verdict.18,916 full independentchecks incl1,902 historicalhashes pass;33 inputs unchanged. Isolated scalar/timestamp-only replay17,014checks passes with explicit historyskip,no model/images/GPU. No inference,data,clockfits or default changes. Box target backups/2026-10-25-exp-050. Continue benchmark uncertainty/error study without waiting for perfect source documentation;CATS and source replies remain parallel stronger evidence.

2026-10-25 — [EXP051 selective reference risk](experiments/EXP-051-ms2-selective-risk.md): all4 windows/3 labels/2 supports,960 cases×4 coverages,lowest normalized trace ranked against mean squared normalized residual. Equal fractional cutoff ties preserve exact retained mass/order independence;random expectation full mean,oracle lowest loss,reverse highest trace. Primary50% own-support ratios RGB/thermal:night0.080/0.128,dayopening0.245/0.549,middle0.513/0.459,late0.535/0.320;all160 primary cases belowrandom,also160 repeated commonsupport. No pixelpooling/fits/inference/fusion or camera winner.49,871 independentchecks,485frozen inputs,1,944historical hashes pass. Initial slow loop interrupted without outputs;final sorted-cutoff independently checked. Next compare simple scene/depth selection at matched mass;more frames not needed yet,CATS pending. Box target backups/2026-10-25-exp-051;historical NPZs referenced from046/048/049 rather than duplicated.

2026-10-25 — [EXP052 selection controls](experiments/EXP-052-ms2-selection-controls.md): new scripts retain four windows/all references/supports/coverages,960 cases/3840 points. Fixed far/near reference depth and central/peripheral location baselines;no outcome-selected direction. Also same coverage within54 fixed depth×grid bins,no support dropped;fractional ties include singleton/constant bins (new estimand,not EXP047 rank-min30 eligibility). Primary50% own stratified/random medians RGB/thermal:night0.937/0.217,dayopening0.386/0.779,middle0.645/0.750,late0.778/0.597;156/160 primary belowrandom,151/160 repeatedcommonsupport. NightRGB original0.080 advantage greatly attenuated under composition constraint;not causal decomposition. Central baseline beats thermal U19/20 dayopening and12/20 middle,ratios1.267/1.116;no universal best selector.180,158 independentchecks pass,3840 EXP051 globalpoints reproduced,492frozeninputs/1,949historical hashes unchanged. No model/data/fits/fusion/default/physicalcalibration claim. Next fixed-selector robustloss/tail sensitivity;no new upload needed,CATS pending. Box target backups/2026-10-25-exp-052;historical NPZs referenced,not duplicated.

2026-10-25 — [EXP053 loss/tail sensitivity](experiments/EXP-053-ms2-loss-tail-sensitivity.md): unchanged four windows/labels/supports/coverages/selectors/54-bin budgets;mean L2 magnitude replaces squared loss,no clipping/trimming/new inference.960 cases,3840 curves,2880 tail records. Primary50% stratified/random medians RGB/thermal:night0.880/0.628,dayopening0.787/0.910,middle0.861/0.889,late0.880/0.810;160/160 primary belowrandom,157/160 repeatedcommon. Global median reductions21–59%,strat9–37%;nightRGB strat12% vsMSE6% shows loss effect not uniform. Central baseline still beats thermal20/20 openingday and12/20 middle,ratios1.135/1.043. Worst1% accounts12–39% MSE vs5–12% L2 acrossgroupmedians;primarymaximum source001942RGB MSE67%,L217.8%,no bad-label verdict.229,331 checks,3840 EXP052 MSE curves reproduced,493frozeninputs/1,963historical hashes unchanged. No fit/fusion/default/physicalcalibration or camera winner. Keep bothlosses;next fixed independent recording/condition replication,inspect existing rain pointers/coverage before downloads/uploadrequests,CATS pending. Box target backups/2026-10-25-exp-053;historical NPZs referenced,not duplicated.

2026-10-25 — [EXP054 fixed rain replication](experiments/EXP-054-ms2-rain-replication.md): before outcomes choose first20 IDs of_2021-08-06-16-19-00 fromEXP040 pointers. Acquire only rain sync24,906,100,204bytes SHA b6bd4e5ad17cfc9adcce5d6b52fccfe254609919c7ddc5c7601a9de43666cd54 anddepth10,943,011,305 SHA ab2f7344ed29040f04babe3d2200cb882b76109afc5cc6a26f2b0c44c3fd4f90;fullarchives+indices local,excludedbackup. Selected80images/120depth/40cachedposes/20nav;11868fulltimestamps/calibration exact/zero prior imagehashoverlap.1/20 inheritedjointmetadataflag,notphysicaleligibility despite legacy copied logger phrase. Reference translationcolumns~21.36m/1.93s,nav2/2/0 notaccuracy. Originalstereorunner40fields/80inputs/2bitwise repeats,config/runtime/models/recipe exact. Proxy rawprimaryrho0.322RGB/0.630thermal,common0.212/0.520. Frozen50% proportional54bin primarymeanL2 ratios0.827/0.739,40/40 belowrandom;MSE0.623/0.505,39/40. CommonmeanL20.878/0.784,40/40;MSE0.673/0.552. Globalbaselinecounterexamples retained,newrain medians favorU butnotallimages;no camera winner,weathercause,physicalcalibration,VO or fusionclaim.18,714 fullarraychecks/2,999,714points;50,203 selectionchecks perloss,119,120 finalchecks;source-only7,887;1,977 historicalhashes pervalidator. Proxy321frozeninputs,eachloss127 unchanged. Newms2_rain* andvalidators preservehistoricalscripts;selection readsfixedrainproxy path. Box target backups/2026-10-25-exp-054 includesbounded source+allnewoutputs,excludesfullarchives/checkpoints. No newupload needed;CATS pending,stronger independentreference next useful evidence;cachedrain/day can supportfuturepredeclaredintervals.
