2026-11-12 America/New_York — [Final longer unweighted test](docs/research/unweighted-longer-findings-2026-11-12.md): fixed first100frames per day/night/rain, added80intervals primary, nativeRGB and thermal controls. All three gates fail; CLOSE_UNWEIGHTED_ROUTE_NO_RETUNING. Day loses to RGB, night to both, rain mean slightly better thanRGB but RMSE/coverage fail. All600native frames/297candidate solves complete,40tests and independent882mapped-error/297strict-cost audits pass. Exposed development data; route closed without fitting, window/threshold search or full-motion integration.

2026-11-12 America/New_York — [Fixed day/night bearing-rotation transfer](docs/research/bearing-rotation-transfer-findings-2026-11-12.md): day passes, night fails; STOP_TRANSFER_PROMOTION_NO_RETUNING. Day primary rotation mean/RMSE improve34.33%/34.35%against native thermal; night worsens104.57%/147.53%, with19/19primary/NEC/native coverage each. All228attempts retained,39tests and independent228cost/227candidate-mapping audit pass. Exposed stereo pilots, not later whole-record cohort. Secondary replacement-flow NEC motivated the later bounded test above; no training/default/full-VO gain.

2026-11-12 America/New_York — [Frozen bearing-rotation assay](docs/research/bearing-rotation-assay-findings-2026-11-12.md): primary original-flow PNEC passes rain development gate. Mean/RMSE0.05658/0.05952deg versus NEC0.07670/0.08009 and native thermal0.07527/0.09538, unchanged18/19eligible slots; priorRGBstillbetter. All114solves converge,31tests and independent114cost/108motion/108aggregate audit pass. Minimum-eight SVD correction precedes execution. Rotation-only exposed-prefix result; later day/night transfer linked above.

2026-11-12 America/New_York — [Bearing-rotation preparation](docs/research/bearing-rotation-preflight-findings-2026-11-12.md): equation-level NEC/PNEC prototype passes21synthetic numerical/mapping tests. Bounded covariance extraction reproduces all19original flows bitwise;10,198common grid observations and protocol/input hashes locked. No real candidate pose solves or accuracy scores in that preparation step; later execution is linked above.

2026-11-12 America/New_York — [Updated progress brief](docs/research/progress-and-next-steps-2026-11-12.md) and [12-source literature decision](docs/research/research-direction-literature-2026-11-12.md): close the two mixed-result configurations; recommend a controlled PNEC/NEC bearing-space rotation assay before any pose-objective covariance learning. Thermal-specific hybrid tracking remains an alternative. Primary-source reading levels, prior-work overlap and implementation limits are explicit; no new experiment in this synthesis. Research continuity guidance is in [AGENTS.md](AGENTS.md).

2026-11-12 America/New_York — [Frozen thermal temporal matcher](docs/research/thermal-matcher-pilot-findings-2026-11-12.md): four 20-frame rain arms complete. Pretrained replacement flow improves thermal translation mean/RMSE 6.59%/7.32% but worsens rotation 16.01%/4.08%, with unchanged coverage; RGB remains lower on mean/RMSE. Frozen advancement rule fails: close this configuration without tuning. Three tests, exact passthrough parity and independent 186-value metric audit pass. No direct correspondence-error or generalization claim.

2026-11-10 America/New_York — [Final-output residual fit](docs/research/final-residual-fit-findings-2026-11-10.md): fixed 40-update day/night fit descends 17.65%. Rain VO translation/rotation means improve 3.04%/10.15% against thermal, but depth AbsRel worsens 15.47%, translation p95 worsens and RGB remains better. Predeclared advancement rule fails; close this recipe without retuning or promotion. Nine tests and amended independent audit of 358 hashes, 200 stereo metrics and 73 motions pass; zero-control poses/times exact. [Weekly brief](docs/research/progress-and-next-steps-2026-11-10.md).

2026-11-10 America/New_York — [Mean descent contract](docs/research/mean-descent-contract-findings-2026-11-10.md): strict-FP32 training probes confirm a shared recurrent-head gradient/forward mismatch. A separate zero-initialized final-output residual passes FP64 finite differences and three full-batch descent updates, loss 1.717747→1.690568 (−1.58%). Five tests and independent analytic-gradient audit pass. Training-only result; no rain/VO evaluation or default change. Old checkpoint unchanged. Next: fixed-budget corrected-adapter fit and existing evaluation controls.

2026-11-10 America/New_York — [Thermal stereo mean adaptation](docs/research/stereo-mean-adaptation-findings-2026-11-10.md): completed40updates on day/night with rain excluded from fitting, then stereo and four native VO controls. Only2,305horizontal mean scalars change. Excluded stereo AbsRel worsens1.96%; VO changes are tiny/mixed, and training loss itself worsens0.335%. No model promotion. Six tests/independent305hash audit pass; exact20pose passthrough parity. Next prerequisite is verified recurrent mean-head descent, not another confidence variant. [Weekly brief](docs/research/progress-and-next-steps-2026-11-10.md).

2026-11-09 America/New_York — [Exact rain depth recovery](docs/research/depth-label-retrieval-findings-2026-11-09.md): restored 40 matching EXP054 maps and verified 80 actual image hashes. All 38 map-complete native pair problems lack sufficient jointly supported depth factors (RGB max0, thermal max1); four-arm experiment remains unexecuted. Three tests and independent 9,846-endpoint audit pass. Park this assay for the recovered packet; stop further acquisition/support tuning. No new frontend/solver run or runtime benchmark.

2026-11-09 America/New_York — [Depth-mean ablation preflight](docs/research/depth-mean-ablation-preflight-findings-2026-11-09.md): all120native replay poses exactly reproduce cached parameters. Four-arm test NOT EXECUTED:126required own-camera maps have0matches in two inspected prior depth-reference backups (different sequences/sparse indices). Two tests and independent32hash/120event audit pass. No large archive download/new inference/model or runtime study. Next requires exact listed source/target depth maps; retain replay rather than rerun qualification.

2026-11-09 America/New_York — [Repetition and error-mechanism review](docs/research/fusion-mechanism-review-2026-11-09.md): substantial prior coverage confirmed; further directional/confidence variant deferred. Thermal translation MSE is89–98%along reference motion, but this does not identify the physical cause. Next is a matched depth-mean × covariance diagnostic; design only, no new solver/inference. Four tests/60independent scalar checks pass; runtime excluded. Supersedes the prior directional-score next step.

2026-11-09 America/New_York — [Translation-specific fusion](docs/research/translation-fusion-findings-2026-11-09.md): RGB rotation retained exactly; primary blend improves day but worsens night/rain, aggregate translation +0.46% vsRGB. Constant/selector controls also give no aggregate gain. Reference-dependent convex oracle shows19.47% translation headroom with unchanged RGB rotation. Seven tests and independent model/geometry/metric audit pass. Next: directional correction geometry and residual consistency; runtime excluded.

2026-11-09 America/New_York — [Thermal accuracy opportunities and cue test](docs/research/thermal-opportunities-findings-2026-11-09.md): 361 translation wins / 103 rotation wins / 21 joint wins on 1,735 paired native intervals. Translation oracle reduces translation error 9.97% but raises rotation error 21.35%. One frozen recording-excluded rotation cue model fails to improve RGB; availability preserved. Seven tests and independent audit pass. Next: translation-specific fusion with RGB orientation; runtime excluded.

2026-11-09 America/New_York — [Fixed-native information weighting](docs/research/native-information-weights-findings-2026-11-09.md): native means preserved exactly; raw/calibrated scalar/full controls completed on day/night/rain. Primary calibration increases equal-record common-support rotation/translation error by 8.95%/3.75% against RGB, with unchanged coverage. 39 tests and three independent audits pass; runtime excluded. Next: measure thermal accuracy opportunities and test cues that predict them.

2026-11-09 America/New_York — [Three-record accuracy comparison](docs/research/pose-covariance-accuracy-findings-2026-11-09.md): completed day/night/rain600-group comparison,1200new rain expert-frame calls, ten prediction methods locked before references. Replay/full covariance fails to improve native RGB on all three; new backend and missing-covariance equal fallback account for regressions. Native backend retained.31tests and independent audits pass; runtime excluded. Next: hold native means fixed to isolate weighting/calibration.

2026-11-09 America/New_York — [Pose-covariance prototype and preflight](docs/research/pose-covariance-prototype-findings-2026-11-09.md): implemented two-endpoint Gaussian registration, full covariance derivatives and local covariance primitives. Fifteen numerical tests /160synthetic trials pass; amended20-group smoke converges38/38. Planned day/night600-group prefixes:2292converged/58capped/46missing-native of2396; independent13752direction checks pass. No new inference or reference scoring; full fusion/rain/calibration comparison remains pending.

2026-11-09 America/New_York — [Next experiment: geometry-aware motion uncertainty](docs/research/next-experiment-pose-covariance-2026-11-09.md): user clarified the brief is a weekly progress update, not the research endpoint. Defined a MAC-I²-inspired pose-covariance bridge for frozen RGB/thermal frontends, with equal/scalar/full covariance controls and a three-recording development comparison. Design only; no new inference/results. Supersedes the recommendation to defer all method development; previous scalar-test outcomes remain intact.

2026-11-09 America/New_York — [Concise progress and next steps](docs/research/progress-and-next-steps-2026-11-09.md): shareable brief of completed work, positive within-camera evidence, final evaluation limits and the next research decision. No new results or experiments.

2026-11-09 America/New_York — [Shared-representation scope decision](docs/research/shared-representation-feasibility-2026-11-09.md): TartanMatch code/checkpoint metadata accessible; CMRW repository currently README-only. No demonstrated link from matcher replacement to the original relative-error-prediction gap. Keep as related work; no matcher integration/training now. Next main deliverable is the existing characterization and supervisor-facing research decision. Source hashes recorded; no new inference or data/model payload download.

2026-11-09 America/New_York — [Shared sensor representations: reading note and proposed direction](docs/research/shared-sensor-representations-2026-11-09.md): verified references for joint embeddings, shared/private features and RGB–thermal dense correspondence (ImageBind, CMC, DSN, CVPR2025 spatial correspondence and the October2026 TartanMatch preprint). Proposed next step is implementation/data compatibility assessment; no training or evaluation executed. The prior angular-rule stop remains unchanged.

2026-11-09 America/New_York — [Characterization and feasibility report](docs/research/characterization-report-2026-11-09.md): consolidated within-camera ranking, recording-dependent cross-camera results, availability/accuracy distinctions and the completed whole-record evaluation. Primary outcome remains **STOP_INSUFFICIENT_EVALUATION**, with no validated adaptive benefit. Current fixed-rule route closed; patch-gap closed and thermal-bias diagnostics parked. Synthesis only; no new inference, scoring or tuning.

2026-11-06 America/New_York — [Decisive-test qualification](docs/research/decisive-test-gates-2026-11-06.md): unchanged nominal target compatible on 18/19 existing development edges; termination-only edge retained unavailable. No prior candidate payload/evaluation found in scoped recorded inventories; two reserve candidates eligible under that explicit project-lineage limitation, not assigned/opened. Next is an actual whole-record execution/scoring lock and one evaluation; no new diagnostics, inference or scores in this step.

2026-11-06 America/New_York — [MAC/MACT asset recovery and development smoke](docs/research/bounded-asset-recovery-2026-11-06.md): exact original checkpoints and compiler runtime restored from verified backups. RGB and thermal each complete 20/20 frames, 19/19 covariance/motion receipts, zero observer errors; about 21s/15s native runtime. Existing source/recipe unchanged; no accuracy scoring or reserve access. Missing-local-assets block resolved; prospective common-target/partition execution gate remains pending. Patch-gap branch stays closed.

2026-11-05 America/New_York — [Research stop/go decision](docs/research/research-decision-2026-11-05.md): patch-gap confidence branch CLOSED; thermal-bias micro-diagnostics PARKED. Existing evidence supports a characterization/feasibility report, not a validated adaptive method. One unchanged-rule test has fixed headroom/benefit/translation/coverage stopping rules; it remains UNEXECUTED_NOT_READY because compatible fresh evaluation inputs and execution-pinned uncertainty assets are not currently staged. No new scores, inference, reserve access or retrospective rebranding.

2026-11-05 America/New_York — [EXP152 fixed patch-gap controls](docs/research/stereo-ambiguity-controls-findings-2026-11-05.md): freeze 7d6941e1a; primary 30-point joint controls unsupported in all images. At the predeclared 10-point sensitivity, thermal all-primary joint coefficients day −0.066791 (8/15 negative) / night −0.043283 (8/14); median retained points 32.1%/33.3% in defined images. Cue remains weak and support-sensitive; no confidence policy, correction or default promotion. Four tests, 115,338 audit checks / six corruptions pass; all 80 source cases / 960 controlled rows retained. Backup backups/2026-11-05-stereo-ambiguity-controls-exp152.

2026-11-06 UTC — [EXP151 initialized day/night depth](docs/research/cross-recording-stereo-depth-findings-2026-11-06.md): freeze 14159db76; thermal native/projected depth ratios day 0.865507 / night 0.826784, consistent 0.871988 / 0.874161. Thermal patch-gap/AbsRel median rho day −0.114880 (15/20 negative), night −0.191699 (17/20); descriptor cues weak/inconsistent. All 80pairs/two-pass geometry exact, 2,938 native-cost targets exact, 160 pair calls; no trajectories/fit/selector/default/physical promotion. Three tests, 1,434,047 audit checks / 7 corruptions pass. Backup backups/2026-11-06-cross-recording-stereo-depth-exp151.

2026-11-05 — [EXP150 stereo baseline initialization repair](docs/research/stereo-baseline-initialization-findings-2026-11-05.md): freeze 5a41bc175; original/rebuild native libraries byte-identical. Storage probe original/rebuild accepted 512/512/0, repaired 512/512/512 with identical all-feature geometry. Six full captures/12,000 native calls and 80 integration calls complete. Repaired thermal common final travel/reference 0.613906, translation RPE 2.8326 m (RGB 0.6435 m). Three tests, 40,093,761 audit checks/4 corruptions pass. No causal accuracy claim, physical qualification, selector, reserve/default/EXP133 promotion. Backup backups/2026-11-05-stereo-baseline-initialization-exp150.

2026-11-05 — [EXP149 initialized native match ambiguity](docs/research/native-match-ambiguity-findings-2026-11-05.md): freeze 498fa12e7; all 1,398 retained points reproduce historical and native-cost geometry exactly; two other thermal source0 features differ. Thermal patch relative-gap/error median rho −0.191699 (17/20 negative), consistent subset −0.232517 (15/19); descriptor margins weak/inconsistent. Source constructor assigns baseline after matcher reads it; initialized replay is qualified, no repair/adoption or causal bias claim. Six tests, 73,953 audit checks/4 corruptions pass; no new trajectories, fit, selector, reserve/default/EXP133 promotion. Next isolate initialization repair with original/patched controls, then cross-recording cue validation. Recovery paths EXP146–148 corrected; actual packets unchanged. Backup backups/2026-11-05-native-match-ambiguity-exp149.

2026-11-05 — [EXP148 local projection/depth sensitivity](docs/research/depth-projection-sensitivity-findings-2026-11-05.md): freeze aa0b84505; same 40maps/1398primary points, fixed 3×3/5×5reference windows, no new inference/acquisition/fit. Thermal5×5locally consistent subset 672 points/20 images retains original median depth ratio 0.874161 (all 0.826784);663/1089shorter than every eligible local label, including 99insufficient-support points. RGB consistency support 0 points at 3×3/one point at 5×5, no comparable camera subset. All primary errors unchanged; all strata/empties retained.6 tests,84738independent checks/4corruptions pass. Engineering window/consistency rules do not qualify visibility or physical timing; no correction/reserve/default/EXP133 promotion. Next native stereo-match ambiguity at retained points. Backup backups/2026-11-05-depth-projection-sensitivity-exp148.

2026-11-05 — [EXP147 native depth / single-scan LiDAR proxy](docs/research/native-depth-proxy-findings-2026-11-05.md): freeze 568b46473; same fixed 20sample frames per camera, no new native inference. Median of per-image median native/reference depth ratios RGB1.003929, thermal 0.826784 (17/20 thermal below 1); absolute relative errors 10.54%/26.59%. Compared only 309RGB/1,089thermal features,3–39/26–90 perimage; no clipping/interpolation/fit. Forty maps acquired via 48HTTP206ranges/192MiB,2archived overlaps exact. Independent sensing modality, but projection/visibility/physical timing qualification unresolved: conditional proxy agreement, not GT or causal VO-gap explanation.6 tests,406,384 checks/4corruptions pass. All cases/unsupported features retained; no reserve/default/EXP133 promotion. Next projection/visibility sensitivity and acquisition/export timing evidence. Backup backups/2026-11-05-native-depth-proxy-exp147.

2026-11-05 — [EXP146 native stereo history](docs/research/native-stereo-history-findings-2026-11-05.md): fixed 2,000night groups. Thermal median 1,008accepted native stereo matches (min 459), pooled 55.70%, yet finalized median travel/reference 0.60956 and 99.31% translation error parallel. Count rho 0.0192, signed recorded-time rho−0.0726; both timing signs retain shortfall, no causal timing/depth claim. Initial observer RGB stops at 179 on cleared-frame scalars; all 6,180 initial calls/outcomes retained. Guard amendment c8e5922cb then four-arm 8,000-call recapture complete; RGB stereo observations 1,998/2,000 (179/887missing), thermal 2,000.8 tests,120integration calls/311,100 initial audit checks,13,394,957 full independent checks/4corruptions pass. Native library/source/settings/inputs unchanged; no re-pair/fit/reserve/default/physical/adaptive/EXP133 promotion. Next independently qualified depth evidence and timestamp provenance. Backup backups/2026-11-05-native-stereo-history-exp146.

2026-11-05 — [EXP145 stereo/input/reference diagnostic](docs/research/stereo-reference-diagnostic-findings-2026-11-05.md): same2,000 exposed night groups, no new SLAM inference. Thermal finalized ORB median1s translation magnitude/reference0.6140; direction cosine0.999039;99.28% squared translation error parallel to travel. DROID ratio0.8942. Thermal recorded stereo |delta|median33.6835ms vsRGB0.0199ms, exposure meaning unresolved. Supplied units/K/baseline consistent;40 fixed stereo samples retain all generic matches,80 original producer replays exact. Initial scoring-runtime replay mismatch retained; amendment0ecad88cf uses original OpenCV4.6 producer, unchanged canonical inputs.9tests,154,236independent checks/5corruptions pass. No fit/re-pair/reserve/default/physical/adaptive/EXP133 promotion. Next native accepted-stereo disparity/depth history, with timing/calibration provenance still needed. Backup backups/2026-11-05-stereo-reference-diagnostic-exp145.

2026-11-05 — [EXP144 ORB output-stage control](docs/research/orb-finalization-findings-2026-11-05.md): fixed first2,000 exposed night groups, eight single attempts complete. Received→final translation RPE RGB0.661770→0.635020m, thermal2.842827→2.843425m; fresh same-prefix DROID0.358476/0.842099m on identical masks/maps. Finalization explains little of this bounded gap. Original-repeat variability retained.52tests,265integration,107,179factor/export/metric checks/18corruptions and84,068native DROID checks pass. No extraGBA/fit/retry/reserve/default/physical/online/adaptive claim; EXP133 unready. Next stereo/input/reference geometry diagnostic. Backup backups/2026-11-05-orb-finalization-exp144.

EXP143: [Full-record ORB support and matched errors](docs/research/orb-full-record-findings-2026-11-05.md).
All six native streams complete. Thermal has higher pose availability/fewer map
segments in all three conditions, but higher translation RPE. All ORB trajectories
break; matched-support DROID has lower translation/rotation RPE in all six cases.
48 tests, 818,793 native/metric and 153,030 matched metric checks pass. Conditional
development evidence; CPU received versus GPU offline scope remains explicit.

EXP142: [Full-record provided-reference scores](docs/research/provided-reference-full-record-findings-2026-11-05.md).
Six saved DROID streams / 55,618 poses; RGB SE3 ATE is lower in day/night/rain,
while thermal slightly lowers day translation RPE. Fifteen tests and 379,949 audit
checks pass. Provided-camera development proxy only; no physical/adaptive claim.

EXP141: [Complete-record bounded DROID engineering](docs/research/bounded-full-record-findings-2026-11-04.md).
All 111,236 raw images acquired; latest declared attempts complete 6/6 streams,
with 55,618/55,618 finite offline poses. 139 tests pass; prior night capacity/backend
failures are retained. Physical timing and trajectory accuracy remain unverified.

EXP140: [Whole-record source and capacity preflight](docs/research/whole-record-preflight-findings-2026-11-04.md).
27,809 original groups inventoried; 26,906 still need image acquisition after prior recovery.
Unchanged DROID night/rain allocation exceeds GPU memory. 252,888 audit checks pass; full runs pending.

EXP139: [MS² source-clock evidence audit](docs/research/ms2-clock-evidence-findings-2026-11-04.md).
Hardware synchronization architecture is documented; export timestamp semantics remain unresolved.
2,698 scalar checks pass. Clock assumptions remain explicit; benchmark stays unready.

EXP138: [Conditional common-time motion model](docs/research/common-time-model-findings-2026-11-04.md).
105 tests pass; explicit clock/motion assumptions preserve original times and horizon gaps.
Model accuracy and physical synchronization remain unestablished; benchmark stays unready.

EXP137: [DROID longer exposed prefixes](docs/research/droid-longer-prefix-findings-2026-11-04.md).
All six fixed101-frame runs initialize, returning606/606finite offline poses;
first20inputs/frontend traces stay exact. 82tests/13,874audit checks pass. No
accuracy claim; next common-target-time qualification and recording-level comparison.

EXP136: [DROID stereo metric contract](docs/research/droid-ms2-development-findings-2026-11-04.md).
Pinned baseline builds; 72 tests/23 CPU-CUDA checks and 1,818 independent checks pass.
All six bounded cases process20/20 frames, but only daythermal initializes; other
five retain null outputs. Next longer exposed prefixes and common-time qualification.

EXP135: [selected-motion trajectory publisher](docs/research/selected-motion-publisher-findings-2026-11-04.md).
Exact source-time/frame integration and segment breaks implemented; 50 tests,
2,505 independent checks/24 mutations and two analytic scorer checks pass.
Provided frame convention verified; unequal RGB/thermal intervals remain refused.
Cross-stream clock/time transport and full benchmark remain pending.

EXP134: [ORB stereo/MS² development adapter](docs/research/orb-ms2-development-findings-2026-11-04.md).
Six day/night/rain RGB/thermal runs each complete 20/20 frames; 27 tests and 2,628
independent checks pass. Three failed startup/ABI attempts remain retained.
Camera-local engineering smoke only; common-frame publisher and full benchmark remain pending.

EXP133: [independent benchmark preparation](docs/research/independent-benchmark-readiness-2026-11-04.md).
Protocol and common scorer prepared; 41 analytic checks/22 mutations pass.
Actual benchmark execution awaits external stereo adapters/builds and a verified
selected-motion trajectory publisher. Final reserves remain closed. Next: pinned
ORB-SLAM3 development adapter, then common-frame/scale proofs and DROID.

EXP132: [unchanged signal transfer](docs/research/signal-transfer-findings-2026-11-04.md).
All 171 joint intervals scored across four additional exposed recordings.
Pooled rotation gains are dominated by the known middle/96 failure; neither
uncertainty rule beats the best fixed camera within a recording. No refitting or
policy adoption. Sufficiency and independent audits pass; the initial audit
import-path failure and its corrected execution are retained.

EXP131: [rotation signal audit](docs/research/rotation-signal-audit-findings-2026-11-04.md).
Primary recording-disjoint combinedridge selects thermal323/323:0learnedbenefit.
Fixeduncertainty rules reduce rotation~5–6% but worsen translation~62–77%;
retention signalconstant.5744checks/10semanticmutations pass. Completed-output
arbitration only,both experts executed;developmentSLAMproxy,noadoption.

EXP130: [provided-export quality headroom](docs/research/provided-quality-headroom-findings-2026-11-04.md).
All323matchedintervals score against recorded SLAM. Retrospective rotation oracle:
28.71% below pooledbestfixed,21.45% below per-windowbestfixed. Translation gain
0.134% (0.170mm/interval). No causal selector or physicalaccuracy established.
Next freeze rotation-focused uncertainty/health signal audit with explicit timing.

EXP129 outcome: [provided-export survey](docs/research/provided-export-survey-findings-2026-11-04.md)
finds323jointly supported intervals among380;thermalcovers380,so0availability
headroom. All8nativeexperts return. Next source-backed provided-reference quality
headroom on thisdevelopmentpool. EXP128MSAinputcontrolfailure remainsfailed.

EXP128/129 (2026-11-04): [five-recording survey](docs/research/provided-export-survey-findings-2026-11-04.md).
4/15thirds metadataqualify;strict MSAproducer inputcontrolfails0/800exact,noMSAnative.
Separateprovidedexport arm:380guardedslots,0windows positiveconditionalheadroom,
0strictnativezero-matchfailures retained.1600sourceimages/16camera-info exact.
No candidatechange/poseaccuracy/reserveaccess. Source questions drafted,not sent.

EXP127 (2026-11-03): [exposed-data adequacy](docs/research/exposed-data-adequacy-findings-2026-11-03.md)
finds0natural availability headroom in all18windows/7records:thermal295/295TD3,
RGB855/855MS2. No sustained3phase recovery. Reference/rig/quality qualifications
remain;MS2native adapter verified. Minimal evidence request drafted,not sent.
8tests/35,873checks/9semanticmutations. No new policy/inference/reserve access.

EXP126 (2026-11-03): [decision timing/headroom audit](docs/research/decision-headroom-findings-2026-11-03.md)
finds0night availability headroom above thermal on retained histories; day candidate
85/95 versusconditional ceiling87/95. Preselection predicts before target motion
exists; final completed-motion selection is a different contract.8tests/33,874
checks/8mutations; no new policy/inference. Next exposed-data/target adequacy audit.

EXP125 (2026-11-03): [night-condition transfer](docs/research/night-transfer-findings-2026-11-03.md)
executionPASS, research superiorityFAIL. Unchanged candidate
67/95selected with137calls; native fixedthermal
71/95 with83calls.
Previously inspected night recording; conditional exact-cost readouts separate
from native fixed controls. No pose accuracy/generalization/default adoption.

EXP124 (2026-11-03): [leased native handover](docs/research/leased-handover-findings-2026-11-03.md)
passes the frozen synthetic contract: 85/95 selected deliveries,
141 vs158 full-read calls;
explicit4MiB store+2MiB shared lease.65tests/62,697auditchecks.
[Research connection](docs/research/research-position-2026-11-03.md): health-driven
scheduling feasibility; uncertainty calibration/general accuracy remain unproven.

EXP123 (2026-11-03): [retention capacity replay](docs/research/retention-capacity-findings-2026-11-03.md)
passes 30 actual-byte storage cases: 2 MiB loses the recorded catch-up predecessor,
4/8 MiB retain it. 58 tests / 60,342 audit checks. No native delivery or total
memory claim; next worker leases and bounded IPC. Production defaults unchanged.

# MS project: RGB–thermal geometric uncertainty

2026-11-03 — [EXP122 buffered handover](experiments/EXP-122-buffered-handover.md): one retained RGB input repairs target51 in both the paired failing trace and live candidate. The frozen contract passes: 85/95 deliveries, adjacent output after all four returns, 141 native pair calls versus full-read158. Actual encoded retention, decode/call accounting and original deadlines are explicit. All 52 tests, 48,655 audit checks and 8,918 byte-level array controls pass. Experimental synthetic buffer model; production defaults remain unchanged.

2026-11-03 — [EXP121 readiness continuation](experiments/EXP-121-readiness-continuation.md): the candidate recovers adjacent output in all four return episodes within its request caps, but FAILS the overall availability gate: 84/95 deliveries versus 85/95 for original bounded probing. A completed inactive probe does not ensure current-source continuity at handover. All 43 tests, 28,412 audit checks and 5,226 unperturbed-array comparisons pass; no candidate is adopted. The next boundary is explicit handover continuity/catch-up, not a larger unbounded probe budget.

2026-11-03 — [EXP120 synthetic availability transitions](experiments/EXP-120-availability-transitions.md): an audit of 36 exposed traces leads to an explicit synthetic input-outage test. Adaptive policies deliver 85/95 slots versus 71/95 for fixed-camera controls; active-camera switching produces alternate delivery two source groups after onset. Bounded probing saves 15.8% of pair calls versus perturbed full-read. Fresh inactive cache state does not guarantee adjacent return output. All 33 tests, 34,779 independent checks and 5,226 unperturbed-array comparisons pass. No defaults adopted.

2026-11-03 — [EXP119 native sparse feedback](experiments/EXP-119-closed-loop-sparse.md): seven dual-camera captures use their own returned observations and controller clock. Pair calls fall 434 → 334 (23.0%), measured native processing falls 27.5%, and guarded deliveries remain 204/205 with identical matched selected-motion proxy errors. All 12,584 control/repeat arrays are exact; 25 tests and 32,671 independent checks pass. Each window retains one active camera, so this validates offline feedback/skipping without establishing recovery or adaptive superiority. No defaults adopted.

2026-11-03 — [EXP118 native gap replay](experiments/EXP-118-native-gap-replay.md): fourteen fresh captures execute the frozen request schedule with actual omitted frames. Native pair calls fall 15.7% and measured processing falls 18.0%, while day RGB's matched translation proxy RMSE increases from 0.0432 to 0.0884 m. Middle RGB has no supported gap intervals. Sixteen tests and 12,631 independent checks pass; all-read controls and early dense repeats are bitwise exact. This is open-loop engineering, with no adaptive controller or default adoption.

2026-10-30 — [EXP087 flow-closure quality probe](experiments/EXP-087-flow-cycle-quality.md):434forward grid controls bitwise/434reverse predictions,609336points/529144valid, guarded217label-free choices saved before scoring. Closure matches thermal-only both night windows but day93RGB/6thermal gives0.043965m/0.198670deg versus RGB0.031673m/0.123630deg (+38.81%/+60.70%). Five of six thermal choices worsen translation; all six worsen rotation.9tests, independent868choice/complete-matched loss audit,8mutations PASS. No fit/default/physical/online claim; next reference/frame and recording-level evaluation readiness before a richer gate.

2026-10-30 UTC — [Solver decision and selection headroom](docs/research/solver-decision-and-selection-headroom-2026-10-30.md): close the three-window solver replication without default adoption. Original-arm217slots/120both-supported; perfect label-assisted translation selection improves best fixed camera only0.762%early,0%middle,9.950%day. Day strict translation winner repeats53/98adjacent comparisons. Independent908population values/7208binary policies and6semantic mutations pass. Preserves EXP086 audit refinement/exceptions. Next declared, unexecuted: frozen forward/backward flow-closure quality probe on same windows, no fitting/new scene/reserved access.

2026-10-29 — [EXP086 supported day solver replication](experiments/EXP-086-solver-day-replication.md): six100-frame runs, both cameras99/99supported. Original/passthrough2613arrays/camera bitwise; repair frontend2577unchanged. Translation proxy RMSE RGB0.031673→0.031589m, thermal0.077469→0.076313m; rotation slightly worse.198solves capped,1980accepted steps.33tests,2982events/594scores,9mutations; exact native cost/H/g replay1980states. Post-inference gradient audit refinement explicitly retains2near-cancellation component failures; no inference/settings changed. Across three known conditions effects mixed; keep experimental/defaults unchanged.

2026-10-29 — [EXP085 frozen solver collapse replication](experiments/EXP-085-solver-collapse-replication.md): six100-frame runs on known middle-night collapse. Original/passthrough2613arrays per camera bitwise; repair frontend2577unchanged. RGB remains97/99unsupported; two supported proxy RMSEs lower but still3.002m/26.023deg. Thermal99/99, translation RMSE0.042872→0.041549m, rotation0.676590→0.676750deg. All101repair solves cap10;1013attempts/1010accepts independently checked.33tests,2112events,594slots/303scores; qualified SLAM proxy only, no tracking rescue/convergence/default switch.

2026-10-29 — [EXP084 end-to-end solver control](experiments/EXP-084-end-to-end-solver-control.md): six fresh GPU runs on existing early-night20frames. Original/passthrough533arrays bitwise per camera; repair frontend497arrays unchanged. All19intervals/camera supported, including terminal write. Repair translation proxy RMSE slightly worse (RGB0.009971→0.010003m; thermal0.009260→0.009297m), rotation slightly better; all38solves hit10-step cap.33tests, independent582event/380attempt audit and9mutations PASS. Qualified SLAM proxy only; no convergence/general accuracy/default switch. Next separately declare unchanged-parameter harder-interval replication.

2026-10-29 — [EXP083 isolated weighted-Huber repair](experiments/EXP-083-weighted-solver-repair.md):research-only whiten-before-Huber objective/acceptance,best-state restoration and explicit termination.6chronological saved-factor controls reproduce original native pose vectors exactly;all6repair new-objective values lower,all hit10iterationcap (not convergence).Original earlyRGB/thermal replays hit16rejections and retain slightly worse rawloss.18tests,14affine controls,independent88attempt/63accept/SE3 audit,9mutations,label-read guard and FD-scale check PASS. No core/default/trajectory/physicalaccuracy change or claim. Next separately declare bounded end-to-end original/repair VO control on an existing interval.

2026-10-29 — [EXP082 native solver contract](experiments/EXP-082-robust-solver-contract.md):14CPU affine fixtures verify raw-Huber acceptance vs covariance-weighted/raw-FastTriggs linearization mismatch;native PINV/TR conflict uses17attempts/16rejections and retains worse rawloss0.027419 vs0.0225,while positive ratio of two negative reductions shrinks damping. Constructor-only weight ignored (activeVO explicitly passes weights);injected solver failure printed,finite unchangedloss returned.19tests,2bitwise wrapper/native controls,independent33solve/32strategy/7method audit and9mutations PASS. No core/default/trajectory changes or historical-cause claim. Next separately declare consistent objective/best-state/failure-status repair and frozen controls.

2026-10-29 — [EXP081 factor geometry/objective audit](experiments/EXP-081-factor-geometry-audit.md): all217deadlines/434slots retained;331eligible motions/56636factors split into UV/disparity weighted energy,4depthbins/9cells,conditional leverage and separate translation/rotation information. Low component energies occur in both better/worse proxy classes;daypair0 thermal lower energies yet0.04493m vsRGB0.02651m,nextpair same ordering but thermalbetter. All936classifications/24groups and chronological examples retained;no single cause/separator established.17CPUtests,independent finite-difference/QR/partition audit,9mutations and label-free guard PASS. No fit/default/physicalconfidence claim. Next inspect actual robust/weighted objective and solver contract on saved factors.

2026-10-29 — [EXP080 fixed factor/proxy comparison](experiments/EXP-080-factor-proxy-comparison.md): four fixed lower-diagnostic preferences,217deadlines/117both-available;all improve known middle-night RGBoutlier, residuals improve earlynight, but all worsen translation/rotation proxyRMSE vsfixedRGB in inspectedday. Day translation RMSE reproj0.07426/disparity0.04559/whitened0.07482/condition0.04414m vsRGB0.03146m. Matched history/coverage,all60 comparisons to5old signals retained.15CPUtests,independent choice/population/loss/rank audit,9mutations and byte-identical label-free guard PASS. No fit/adoptedgate/defaultchange or independentphysical/generalization claim. Next audit saved disagreement/objective-geometry mechanisms before another selector.

2026-10-29 — [EXP079 optimizer-factor audit](experiments/EXP-079-optimizer-factor-audit.md): all434intervals retained;337firstwritebacks/57594factorblocks support residual and conditional unrobust undamped camera-tangent information reconstruction;97RGB no-writes and6post-deadline terminal results explicit. Allmatrices numericalrank6 does not establish accuracy.15CPUtests, independent finite-difference/landmark/raw-factor audit,7mutations and label/model-read guard pass. Extreme inferred depths retained;no convergence/covariance/quality-gate claim or defaultchange. Next predeclare matched-support proxy evaluation before choosing any diagnostic.

2026-10-29 — [EXP078 causal quality-signal characterization](experiments/EXP-078-arbiter-quality-signals.md): five fixed diagnostics,217deadlines/434slots/117both-eligible comparisons, no fitting. Day native/angular trace preferences58/98 correct but~0.0440m proxy RMSE versus fixedRGB0.03146m; retained fraction selects known RGBoutlier despite18/18survival. Coverage ties and prior-support constants retained. All331eligible first motions lack convergence/pose covariance.14tests,609336point independent audit,8mutations and label-free guard pass. No scalar adopted; next inspect saved optimizer factors for pose-information/residual summaries.

2026-10-29 — [EXP077 unchanged history day replication](experiments/EXP-077-history-day-replication.md): fixed100-group EXP026 middle from separate0_salon_so_day recording. Both experts supported99/99; fresh controls preserve2613arrays each. History97RGB/2ABSTAIN; on matched97 intervals it equalsfixedRGB0.03151m proxy RMSE versus thermal0.07805m, with one cold-start coverage cost.76tests, three independent audits,6mutations and label-read guard pass. No rule tuning, new scene/reserved MS², independent physical/heldout claim or default change. Next quality-input characterization when both experts track.

2026-10-29 — [EXP076 causal two-interval history rule](experiments/EXP-076-arbiter-history-rule.md): symmetric confirmation length2, fixed before scoring, changes middle isolated RGB recovery to thermal. On matched97 intervals, proxy translation RMSE0.04282m versus fixed RGB preference0.11041m, matchingthermal-only. One extra cold-start abstention per window;early17 matched RGB choices unchanged.20tests,118-row independent audit,7mutations and label-read guard pass. Retrospective known-window control, no general/physical accuracy claim or default change. Next unchanged-rule replication under a separately declared condition.

2026-10-29 — [EXP075 conditional pose-quality benchmark](experiments/EXP-075-arbiter-pose-proxy.md): nominal-body/reference-assisted interval comparison now scores all118 frozen decisions, with missing results retained. Fixed RGB preference worsens middle SLAM-proxy translation RMSE0.10985m versus thermal-only0.04262m at equal98/99 availability; one RGB choice1.00308m versus thermal0.04029m. Early thermal also lower aggregate proxy loss, despite5/18 RGB wins. 13math tests, independent7020-component audit and5mutations pass. No independent GT/physical body verification, causal reference bridge or gate fitting. Next predeclare causal history/recovery baseline.

2026-10-29 — [EXP074 known-collapse thermal fallback](experiments/EXP-074-arbiter-support-collapse.md): unchanged advisory rule on the existing100-group middle window produces97THERMAL/1RGB/1ABSTAIN, zero unsupported choices. Deadline availability98/99 versusRGB-only1/99, matchingthermal-only98/99. Both experts preserve2613arrays/103NPZ bitwise against fresh controls;43tests,4158-field independent audit and7semantic challenges pass. Retrospective availability diagnostic, no VO accuracy advantage shown. Next: verified pose-comparison target/reference/loss before quality gating.

2026-10-29 — [EXP073 shared chronological replay](experiments/EXP-073-shared-arbiter-replay.md): 40 sensor-ordered frame calls, 19 prefix-only advisory decisions (18 RGB by fixed preference with both experts available; 1 final pre-drain abstention). Both estimators preserve 533 arrays bitwise. 29 tests, independent deadline audit and five semantic mutation checks pass. Offline serial replay only; no VO gain or real-time claim. Next: an already examined support-collapse segment.

2026-10-29 — [EXP072 paired arbiter events](experiments/EXP-072-paired-arbiter-events.md): the same existing20-row early-night window now has19 paired RGB/thermal event records. Fresh thermal v2/v3 preserves533 arrays/23NPZ bitwise;verified RGB output reused. Complete camera-local health/motion for both;actual sensor endpoint offsets8.014917–8.046151ms retained. Fifteen contract tests and independent1387-field pairing audit pass. Separate replay clocks are not concurrent availability;no frame conversion,pose labels or gate fitting. Next:single chronological dual-expert replay and advisory availability baseline.

2026-10-29 — [EXP071 causal arbiter event recorder](experiments/EXP-071-causal-arbiter-events.md): opt-in v3 logging now captures pair health and pre-refinement camera motion writebacks with submission IDs and observation stages. 20 focused tests pass; fresh 20-frame RGB GPU v2/v3 control preserves all533 arrays/23NPZ files bitwise, and all97 events pass independent audit. No gate fit or default change; next capture paired RGB/thermal events and define interval/frame comparison before arbitration.

2026-10-28 — [Arbiter input audit](docs/research/arbiter-input-audit-2026-10-28.md): CATS left-mask support is evidenced in the inspected scene, but signed-reference anchor/direction remains ambiguous. Recovered EXP033 controls yield 217 pair health records; middle RGB has97/99 unsupported transitions versus thermal0/99. Post-refinement poses are unsuitable as unqualified online inputs; optimization telemetry/common-frame target still missing. Next: read-only pre-refinement event recorder, without gate fitting or reserved captures.

2026-10-28 — [CATS access and sample inspection](docs/research/cats-intake-2026-10-28.md): user approval received; six ZIP metadata reads, two v2 indices and one preselected outdoor scene inspected. Native thermal is uint16; disparity has mixed signs/NaNs; conventions/calibration still need audit. [Temporal selector proposal](docs/research/temporal-selector-proposal-2026-10-28.md) distinguishes published MAC/thermal VO gains from failed cross-camera score corrections and proposes an estimator-health/history gate. No new inference, training or reserved-capture access.

2026-10-28 — [Fall research assessment through EXP070](docs/research/ms2-fall-decision-2026-10-28.md): consolidates useful within-camera ranking, failed cross-camera transfer and insufficient constant calibration. Includes a report-ready narrative, an evidence/claim map and requirements for a distinct prospective method. Synthesis only; no new experiment, inference or reserved-capture access.

2026-10-28 — [EXP070 retrospective bias/scale calibration](experiments/EXP-070-ms2-bias-scale-calibration.md): four recording-exclusion folds on the same 80 frames/106562 pairs. Both fitted selectors worsen primary mean-distance medians versus the original in all four captures. Bias plus scale improves thermal likelihood versus scale only but worsens selection in three captures; no method adopted. Independent 5234111-check native-source audit and 10 fixtures pass; defaults and two reserved captures unchanged.

2026-10-28 — [Calibration target and evaluation protocol](docs/research/ms2-calibration-protocol-2026-10-28.md): specifies per-camera scale-only versus additive-bias-plus-scale controls, four retrospective recording-exclusion folds, conditional interval coverage and fixed-point camera-selection losses. Declaration only; no fitting, new results or reserved-capture access.

2026-10-28 — [EXP069 empirical disparity coverage](experiments/EXP-069-ms2-disparity-coverage.md): four reused captures/80frames/106562pairs. At nominal95%, median RGB coverage43.18%–91.61%,thermal64.01%–99.12%. Newer-rain thermal95.11% and meanz²1.025 coexist with positive signed bias/asymmetric tails and variance-bin differences. Independent698988-check raw/native-reference audit passes;9fixtures PASS. No fit/inference/clipping/default change; no probability/physical-calibration claim, holdouts reserved.

2026-10-28 — [EXP068 training semantics audit](experiments/EXP-068-ms2-training-semantics.md): available inference exp(2*raw) conflicts with current covariance trainer raw+epsilon; cov_loss ignores mask/max_cov. Exact CPU probes and independent88-check audit pass; relevant upstream code matches. RGB checkpoint byte-identical to official release; thermal exact training recipe remains unverified. No training/fix/neural inference/default change or claim that this caused prior failures. Next empirical scale/coverage check on saved data; holdouts reserved.

2026-10-28 — [EXP067 distribution-risk assumption audit](experiments/EXP-067-ms2-distribution-risk.md): same mean/variance with lognormal/Gamma assumptions gives different inverse-depth risk; Gamma infinite when variance/disparity²>=.5. Four reused captures/80frames/106562pairs: lognormal mildly improves some first-order risks but worsens day19/20; Gamma worsens day20/20, with1920 both-infinite pairs explicitly mixed. No replacement adopted. Mathematical quadratures/counterexample and independent1589390-check audit pass. No fit/inference/default change; holdouts reserved. Next trace training objective/checkpoint semantics.

2026-10-27 — [EXP066 disparity-domain diagnostics](experiments/EXP-066-ms2-disparity-diagnostics.md): four complete saved-frontend captures/80frames/106562 both-valid pairs. EXP064 day RGB: median0.684% tiny-relative-disparity points contribute99.05% axial-depth squared error. Raw uncertainty ranking changes with error domain; thermal night variance/depth-error correlation median-0.563 despite positive disparity-error ranking. Independent4512464-check native-PNG audit passes. No fitting/inference/clipping/default change; holdouts untouched.

2026-10-27 — [EXP065 signed residual accounting](experiments/EXP-065-ms2-signed-residuals.md): all7 examined captures/140frames/240003 both-valid pairs,3anchors,fixed range/grid groups. Global mean subtraction retains RGB lower squared error20/20 in both night windows; thermal has larger spatial mean components. This is reference-assisted descriptive accounting, not physical bias identification or a deployable correction. Independent502138-check audit passes; no fit/inference/default changes; two remaining recordings untouched.

Study whether RGB and thermal uncertainty estimates are comparable and predict
which modality supplies more reliable geometric measurements. The first stage
is baseline reproduction and uncertainty/error characterization; adaptive
fusion follows only if the measurements support it.

Start with [MS_RESEARCH_HANDOFF.md](MS_RESEARCH_HANDOFF.md) for the approved
research scope, decisions, resource pointers, and verified progress.

For the consolidated MS² result and current research priorities, see
[MS² findings and research decision — 2026-10-26](docs/research/ms2-findings-2026-10-26.md).
The completed experiments support within-camera uncertainty ranking; cross-modal
reliability comparison still needs a defined comparison target and support.
The [EXP-055 feasibility audit](experiments/EXP-055-ms2-cross-modal-feasibility.md)
finds promising overlap in existing MS² rain data. The subsequent
[EXP-056 candidate-support comparison](experiments/EXP-056-ms2-cross-modal-support.md)
finds that image-wide per-camera variance scales largely match pointwise learned
selection; a consistent additional spatial-uncertainty benefit is not established.
Exact physical correspondence and cross-recording transfer remain unverified.
The frozen [EXP-057 day replication](experiments/EXP-057-ms2-day-cross-modal-replication.md)
shows a pointwise advantage over camera-scale control in 19/20 mean-distance
comparisons, while always RGB beats the learned selector in all 20. Results vary
between the tested recordings; no universal camera-selection rule is adopted.
The third-recording [EXP-058 night replication](experiments/EXP-058-ms2-night-cross-modal-replication.md)
finds the learned selector worse than always RGB and equal-random selection in
every frame under both losses, despite a small gain over camera-scale control.
The next research priority is diagnosing cross-camera score failure.
[EXP-059](experiments/EXP-059-ms2-score-failure-diagnosis.md) separates camera
median scale from predicted-depth propagation: neither median normalization nor
reference-depth diagnostics repair night selection, and effects vary across
day/rain. No blanket rescaling or propagation replacement is adopted.

[EXP-060](experiments/EXP-060-ms2-camera-scale-transfer.md) tests a fitted relative score multiplier with recording exclusion. It improves day and partly improves night, but worsens rain and leaves night worse than always RGB in all20 frames under both losses. These inspected recordings provide exploratory transfer evidence; no global scale correction is adopted.

[EXP-061](experiments/EXP-061-ms2-error-structure.md) separates fixed range, image-region and tail effects: thermal has lower mean error below20m and RGB at20–80m in the tested night window. Night RGB’s absolute-risk advantage survives tail diagnostics; rain’s ordering is tail-sensitive. Reference-supported diagnostics do not establish a universal night-camera winner.

[EXP-062](experiments/EXP-062-ms2-geometry-risk-selection.md) fits a declared small camera selector using predicted geometry/position and uncertainty with recording exclusion. Combined inputs improve night beyond both fixed cameras and single-family controls, but worsen day versus uncertainty-only and retain severe rain failures. No general transfer or calibrated-confidence claim; defaults unchanged.

[EXP-063](experiments/EXP-063-ms2-unexamined-recording.md) freezes all parameters before acquiring a previously unexamined MS² recording. Combined geometry/uncertainty never beats uncertainty-only (17 worse,3 ties) and never beats always RGB (15 worse,5 ties) under either loss. The night-specific gain does not establish general transfer; no retuning/default change.

[EXP-064](experiments/EXP-064-ms2-frozen-suite.md) prospectively tests the unchanged uncertainty-only model on three additional day/night/rain captures. The declared consistent gain over both fixed cameras and the scalar-fit control is **not met**. All model families,anchors,losses,ties and failures are preserved; no retuning or default change.

The [cross-camera transfer synthesis](docs/research/ms2-transfer-findings-2026-10-27.md) consolidates the method tests throughEXP064. Current evidence supports capture-specific gains,while general transfer remains unsupported; the next saved-array diagnostic is signed residual structure and mean-versus-spread accounting before further fitting.

## New bounded execution checks

For causal arbiter inputs, use [run_td3_execution_v3.py](MAC-VO-main/Scripts/Setup/run_td3_execution_v3.py) with `--record-arbiter-events`. [EXP-071](experiments/EXP-071-causal-arbiter-events.md) defines the journal, timing semantics, opt-in behavior and numerical-preservation evidence. Recording is disabled by default; events do not imply calibrated pose confidence or a gate decision.

Use [run_td3_execution_v2.py](MAC-VO-main/Scripts/Setup/run_td3_execution_v2.py) for new runs. Inspect `tracking_diagnostics.support_state` and original unsupported-pair counts as well as raw execution status. These preserve tracking failures before interpolation changes stored flags. [EXP-033](experiments/EXP-033-gnss-and-tracking-reporting.md) documents usage and bitwise pose-equivalence controls; earlier runners remain available for historical reproduction.

## Repository layout

- `MAC-VO-main/`: extracted MAC-VO source, recovered FlowFormer/DPVO dependencies,
  and environment/verification scripts. See its upstream README for the pipeline.
- `models/macslam/`: extracted checkpoints, excluded from Git.
- `data/archives/`: original model and dataset ZIPs, excluded from Git.
- `data/calibration/yamaha.yaml`: supplied extrinsics source, excluded from Git.
- `data/tartandrive3/`: extracted recordings, excluded from Git.
- `experiments/`: reproducibility notes and measured findings.
- `results/`: generated manifests, arrays, plots, and logs, excluded from Git.
- `docs/research-proposal.pdf`: research proposal.
- `MAC-VO-main/Scripts/Setup/README.md`: environment and Box staging instructions.

## Verified starting point

`MACT_Offroad.pth` strictly loads all 428 checkpoint entries into FlowFormerCov.
The MACT frontend runs on the RTX PRO 5000 Blackwell GPU using Torch 2.7.1 and
CUDA 12.8. Synthetic inference at 128x128 and 480x640 produces finite flow and
positive covariance. Bounded 10-frame camera-local RGB and thermal odometry
execution is now exercised; calibration validity and dataset accuracy remain
unverified. See [EXP-003](experiments/EXP-003-bounded-vo.md) for execution evidence
and [EXP-004](experiments/EXP-004-moving-reference.md) for the first moving-window
comparison against recorded SLAM. Raw uncertainty recording and separate-window
scale checks are complete in [EXP-006](experiments/EXP-006-raw-uncertainty.md):
a single thermal scale does not transfer uniformly, so no correction is adopted.
[EXP-007](experiments/EXP-007-thermal-failure.md) narrows the late thermal failure
and confirms that this backend directly weights image/disparity residuals.
[EXP-008](experiments/EXP-008-backend-ablation.md) replays both objectives on
identical inputs: removing target disparity does not resolve the late failure.
[EXP-009](experiments/EXP-009-weight-ablation.md) finds strong pixel-weight
sensitivity: fixed weights improve late thermal translation but worsen rotation.
[EXP-010](experiments/EXP-010-weight-influence.md) locates strong upper-image
precision concentration in late thermal. Native replay in
[EXP-011](experiments/EXP-011-precision-mass.md) retains most of the translation
gain with total u/v precision preserved, alongside the rotation penalty.
[EXP-012](experiments/EXP-012-channel-mass.md) isolates the channels: u-only
redistribution gives a smaller late thermal rotation penalty than redistributing
both, but still needs separate-clip and geometry validation.
[EXP-013](experiments/EXP-013-separate-clip-validation.md) tests the frozen rule
on a separate night recording: benefits do not transfer consistently, and RGB
tracking fails in two selected windows. Thermal completes all three windows.
[EXP-014](experiments/EXP-014-night-uncertainty.md) records raw uncertainty
without changing those outputs: RGB uncertainty rises strongly in failed
windows and candidate selection collapses. This is a selection-coupled
availability signal, not independent uncertainty calibration.
[EXP-015](experiments/EXP-015-night-geometry.md) adds fixed-grid geometric
residuals: RGB disagreement rises strongly in both failed windows, while
within-pair uncertainty/error ranking remains inconsistent. Keep defaults.
[EXP-016](experiments/EXP-016-temporal-context.md) tests within-window
association after motion/flow/depth controls: signs vary and temporal
repeatability is weak. No general additional temporal signal is established.
[Verified paper notes](docs/research/literature-2026-10-20.md) ground target
definitions and the next thermal preprocessing provenance audit.
[EXP-017](experiments/EXP-017-thermal-provenance.md) reproduces all18 sampled
night thermal images exactly under OpenCV4.10. Saved stereo matrices also
reproduce, but physical body/stereo geometry remains inconsistent; night
cheirality does not justify a blanket baseline-sign flip.
[EXP-018](experiments/EXP-018-stereo-sign.md) retains signed stereo flow on
12 fixed first-frame inputs and reproduces every saved depth array bitwise.
Thermal model signs agree with feature displacements across day/night;
raw-to-processed topic ordering reproduces exactly, while physical calibration
and dense-field validity remain unresolved.
[EXP-019](experiments/EXP-019-spatial-stereo.md) audits a fixed grid: both
recordings share upper-negative/lower-positive thermal predictions, while
night fitted features concentrate in the upper image. This makes the apparent
feature sign contrast spatially selection-dependent. Physical calibration
remains unresolved. No weighting default has changed.
[EXP-020](experiments/EXP-020-calibration-reconciliation.md) verifies our
camera-info usage and tests concrete stereo corrections. Both a free-pose fit
and a body-center-constrained rotation fit fail their validation criteria.
Rejected candidates and source-convention exports are preserved separately;
a known-target thermal stereo sequence or independent applicable calibration
was requested to resolve the remaining geometry. Serial numbers are not required.
[EXP-021](experiments/EXP-021-msa-calibration-audit.md) audits the subsequently
supplied MSA source calibration: all24 fixed image checks pass without fitting.
Its fisheye model differs from the current producer; applicability and metric/body
accuracy remain unconfirmed. No original calibration or VO output is replaced.

The older `macvo:box` image cannot execute on Blackwell. Use `macvo:blackwell`:

```bash
cd /root/AirStack/ore-ms/MAC-VO-main
docker build --progress=plain --network=host -t macvo:blackwell \
  -f Docker/Dockerfile.blackwell .
docker run --rm --gpus all \
  -v /root/AirStack/ore-ms:/workspace -w /workspace/MAC-VO-main \
  macvo:blackwell python Scripts/Setup/verify_mact_checkpoint.py \
  --forward --height 480 --width 640
```

Replace the host mount path if the project is restored elsewhere. Avoid
installing the legacy `requirements.txt` into this image: its xFormers pin
would downgrade the framework. The recovered frontend uses native attention.

## Checkpoints and data

The shared CMU Box bundle is `macslam.zip`. Keep it in `data/archives/` and
extract it into `models/` to produce `models/macslam/MACT_Offroad.pth`, `MACVO_FrontendCov.pth`, and
`MACVO_posenet.pkl`. Archive SHA-256:

```text
77ad168672019f94d8e48ebb087cb109b938bcf8784e3df480070cd5f437f00c
```

AirLab processed data: `/offroad/datasets/tartandrive3`; raw bags:
`/offroad/rosbags`. Prioritize `20251027` or `20251106` for RGB-stereo and
thermal-stereo. These paths are not mounted in the current workspace.

The `20251027/1_salon_mac_cov_day` clip is staged locally, with camera streams,
timestamps, intrinsics, recorded TF, and SLAM inputs. Both RGB and thermal pass
real-image frontend execution. The supplied `yamaha.yaml` is staged under
`data/calibration/`; camera calibration reconciliation is in progress. See the
latest notes under `experiments/` before running geometry evaluation.
Thermal normalization and the checkpoint's training split are
still unknown; three-channel input compatibility alone does not establish the
original preprocessing recipe.

## Storage and shutdown

OSMO workspace files and local Docker images are temporary. Keep model/data
archives and outputs in persistent storage. Box remote `ore_ms` is configured;
the user has saved its config in the OSMO generic credential `ore-ms-box`.
Future workflows must map that credential and restore the private rclone
config; see the setup guide. Tokens and checkpoint/data files are excluded
from Git. Base64 encoding is not encryption.

Commit code and notes before ending a session, then push to an authorized
repository or download a Git bundle to persistent storage. A local commit
does not survive deletion of the workspace.

This project is tracked inside AirStack on the `ore_ms` branch. From the
AirStack root, recover its pinned dependencies with:

```bash
git submodule update --init --recursive -- \
  ore-ms/MAC-VO-main/Module/Network/FlowFormer \
  ore-ms/MAC-VO-main/Baseline/DPVO
```

To restore a downloaded `ore-ms.bundle`:

```bash
git clone ore-ms.bundle ore-ms
cd ore-ms
git submodule update --init --recursive
```

Then restore the checkpoint bundle and rebuild the GPU image. DPVO source is
present, but its compiled extensions and execution have not been verified.

MSA source-image follow-up: [EXP-022](experiments/EXP-022-msa-source-images.md)
checks the subsequently supplied thermal JPEG pair. Both all242 mutual matches
and214 F inliers pass the retained image criteria without calibration fitting.
This supports separate raw/depth/VO replay; metric/body/timing accuracy and
October applicability remain unconfirmed. Existing inputs/defaults unchanged.

[EXP-023](experiments/EXP-023-msa-preprocessing-vo.md) regenerates raw thermal
images with MSA calibration and runs the same six20-frame windows. All240
old-image replay controls pass. Day VO proxy errors improve substantially;
night results remain mixed. MSA is the experimental baseline for subsequent
geometry diagnostics; production defaults and previous results are unchanged.

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

[EXP-028](experiments/EXP-028-reference-timing.md) finds no indexed timestamp
or SLAM-export mismatch on the fixed windows. Late-night filtered odometry
reports0.0161m displacement versus SLAM vehicle0.2608m, with zero mux/teleop
commands; its upstream sensors, independence and accuracy remain unconfirmed.
A55-case fixed reference-time sweep retains the late-night discrepancy; no
offset is fitted or adopted.11 tests/402 hashes and independent checks pass.
Keep MSA and all VO defaults; resolve reference provenance before using this
window as a physical pose-error or fusion-calibration target.

The follow-up [filtered-odometry source audit](docs/research/filtered-odometry-provenance-2026-10-21.md)
identifies a historical NovAtel GPS/IMU origin and the legacy `novatel_span`
package lead. Exact October2025 producer, aiding inputs, clock behavior,
quality and gps_frame geometry remain unverified. Public candidate drivers
must not be substituted for recording-specific source; no reference is promoted
to ground truth and no new VO experiment is claimed.

[EXP-029](experiments/EXP-029-stereo-image-consistency.md) checks18 saved stereo
pairs without a pose reference or new inference. Independent patch matching
supports68/1,008 late-night thermal grid points with0.423px median model
disagreement; most locations remain unvalidated. Stereo variance/discrepancy
associations are weak and selection-dependent. A recorded NCC precision issue
is stabilized without threshold changes;9 tests/97 hashes and independent
numeric controls pass. Keep MSA/defaults and the pending reference-source gate.


[EXP-030](experiments/EXP-030-temporal-angular-consistency.md) checks342 saved temporal pairs in nominal angular units without a pose reference. Median spatial uncertainty/LK-disagreement rho ranges−0.187–0.572 for thermal and0.361–0.594 for RGB. Late-night thermal remains negative despite94–97% LK support; normalization is not physical confidence calibration. Seven tests,2,394 independent ray/covariance checks,95 exactly reproduced prior LK pairs and729 frozen hashes pass;402 earlier audit hashes unchanged. Keep defaults and pending provenance gates. Box recovery: `backups/2026-10-21-exp-030`.


[EXP-031](experiments/EXP-031-new-night-validation.md) stages the new `night_mac_salon_warehousefig8`:1,500 paired groups, no overlap with three prior recordings. Three fixed first-eligible20-frame windows,751 prepared hashes. All three MSA-thermal attempts and early RGB complete without interpolation; middle/late RGB retain0 matches and fail, with0 independent LK support. Preserve failures and unavailable errors. Thermal median spatial angular uncertainty/disagreement rho0.005/−0.223/0.529; operational availability differs from calibrated reliability.24 tests,600 identical image decodes,798 scalar/finite-difference checks,114 pairs/160,056 points and267 analysis hashes pass; prior729/402/97 hashes unchanged. Rebuilt GPU image f429af4afa23 has recorded transitive version differences; CPU analysis unchanged. Keep defaults/provisional calibration/reference gates. No more upload needed now: unused intervals of this archive suffice for a next bounded study. Box recovery `backups/2026-10-22-exp-031`.


[EXP-032](experiments/EXP-032-long-night-validation.md) extends the new night recording to three nonoverlapping100-group midpoint blocks (14.9/9.9/11.0s), preserving30/0/1 gaps>150ms. All thermal attempts have0 pre-refinement lost pairs; RGB has99/97/99 out of99. Middle/late RGB raw execution passes are interpolation-dominated; existing MotionInterpolate aliases/clears boundary flags, understating original losses (97→94,99→95). Preserve reports/settings; audit raw counts/logs. Thermal median spatial uncertainty/LK-disagreement rho0.407/0.397/0.537; temporal/gap effects vary, not physical confidence calibration.25 tests,4,158 scalar/finite-difference checks,594 pairs/833,976 points,54 strata and3,041/1,229 hashes pass; prior751/267/729/402/97 unchanged. Box recovery `backups/2026-10-22-exp-032`. Next software task: pre-refinement diagnostics with unchanged poses. For future numerical RGB/thermal error comparison, request another night recording with visible RGB detail; names first. No more data needed to reproduce this audit.

[EXP-033](experiments/EXP-033-gnss-and-tracking-reporting.md) completes GNSS scalar translation cross-checks on15 fixed cases x3 offsets and a versioned reporting fix. New `run_td3_execution_v2.py` preserves pre-refinement support/flags even on zero-observation failure; original runner/postprocessor unchanged. Three GPU controls reproduce5,753 arrays bitwise; middle RGB correctly exposes97 original unsupported pairs vs94 stored flags.13 tests pass. GNSS original late-night0.0161m vs VO0.0085m/camera SLAM0.2610m; canonical early and new short late also show reference-dependent disagreement. Longer thermal GNSS31.71/32.52/31.47m vs VO26.45/31.72/26.74m retains early/late shortfall. Magnitudes from distinct physical origins are not camera-pose errors or calibrated GT; no axes/latency/scale/lever arm fitted.45 comparisons/2,340 queries validated, max bracket41.56ms;828 final hashes,291 reporting-control hashes and previous scientific hashes unchanged. GNSS phase001 source preserved after scope-label correction; phase002 arrays bitwise identical. Owner axes/RTK/origin/timing and checkpoint train split still pending. No more image upload needed for these checks. Box `backups/2026-10-22-exp-033`.

[EXP-036](experiments/EXP-036-night-batch-screen.md) batch-screens the four
remaining available October27 night archives. Most sampled RGB is very dark;
redcourse-waypointskip1 has a brief illuminated late candidate (10.56% median
LK support), requiring sustained-support and timing checks before VO. The
source-image screen includes569 selected images and456 pairs; no inference,
calibration or weighting change. No further upload is needed for this candidate.

[EXP-037](experiments/EXP-037-redcourse-lit-interval.md) extends that redcourse
candidate by100 RGB acquisitions on each side. Only6/1800 tracks survive the
original20-frame interval; a short preceding block retains74. Current stereo/
cross-modal timing gates yield no continuous paired window. No learned VO was
run; temporal pairing needs separate justification before a numerical comparison.

[EXP-038](experiments/EXP-038-mac-relative-ranking.md) directly tests saved MAC
raw temporal covariance ranking on114 paired day transitions. Nominal-angular
ranking agrees with lower MAC–LK disagreement on108/114 versus112/114 for
alwaysRGB; six thermal switches help1/hurt5. A full-grid uncertainty sensitivity
helps on one switch. This is population-sensitive matching-disagreement evidence,
not physical reliability calibration or a full3D/pose covariance test. No training,
new inference or fusion change.

[EXP-039](experiments/EXP-039-mac-recording-replication.md) repeats those frozen
rules on57 saved pairs from the different1_salon_mac_cov_day capture. Both
angular rules selectRGB throughout;all matching-disagreement targets also favor
RGB,so they equal the fixed baseline without testing successful thermal switches.
Ratio association changes sign across recordings. No inference/training/tuning.

[EXP-040](experiments/EXP-040-public-data-audit.md) audits public paired-data
access and independent reference suitability. MS² is preferred; user-supplied scripts now verify access. STheReO calibration/poses are preserved, but images hit Drive quota;
reference gaps reach7.4s and thermal baseline conventions need review. TartanRGBT's
MACVO poses cannot independently validate MAC. 2,753 STheReO and222,538 MS² schema/hash controls pass;22 source rotations flagged;
no model run or dataset-readiness claim. Box `backups/2026-10-23-exp-040`.

[EXP-041](experiments/EXP-041-ms2-night-pilot.md) acquires the fixed MS² night
sequence and preserves20 IDs with stereo RGB/raw16-bit thermal, three depth
variants and camera poses.5355 source controls plus37 independent timing controls
pass. Recorded thermal stereo offsets span15.2–49.8ms throughout10131 frames;
no pair passes the inherited2ms gate,including nearest-right pairing. Official
hardware-sync description does not resolve recorded timestamp semantics. No MAC
inference; resolve timing before a motion-sensitive comparison. Box
`backups/2026-10-23-exp-041` preserves the bounded pilot,not40GB archives.

[EXP-042](experiments/EXP-042-ms2-timestamp-semantics.md) checks primary acquisition
descriptions and both official MS² loaders. They use same-ID stereo pairing but
do not define exported exposure-time semantics. Thermal offsets flip sign8810
times;constant clock shifts and fixed±1-index shifts cannot satisfy the inherited
2ms diagnostic throughout.10174 controls pass,no correction/inference. Prepared
20-ID source-input manifest preserves all four times and raw pixels. Generic
loader would collapse619 thermal levels to5 and fabricate timestamps; implement
MS²-specific/model-compatible input handling next. Box2026-10-23-exp-042.

[EXP-043](experiments/EXP-043-ms2-mac-adapter.md) implements a real MS² MAC
sequence adapter. Rawuint16 thermal and both recorded camera times are retained;
model frames require an explicit audited percentile/CLAHE/bilateral recipe.
All80 source images and120 depth maps pass independent full-array checks;
14 tests/1087 controls pass with real CPU Torch/PyPose. Clip,batch and resize
preserve metadata;reference labels remain explicit. No model inference,timing
correction or default changes. Checkpoint training preprocessing remains
unconfirmed. Box `backups/2026-10-23-exp-043`.

[EXP-044](experiments/EXP-044-ms2-frontend-smoke.md) runs pretrained RGB on the
fixed MS² pilot:39 calls each for RGB384×1224 and thermal256×640 all pass shape,
finiteness,positive covariance and independent saved-array validation. All80
model-input tensors match archived producer replay;14 adapter tests pass. The
original checkpoint archive is hash-verified after shared-link recovery. Runtime
compatibility only,not physical accuracy. Full Box recovery:2026-10-24-exp-044-complete.

[EXP-045](experiments/EXP-045-ms2-reference-protocol.md) audits MS² pose/timing/calibration metadata and freezes a provided-pair single-scan-depth diagnostic before scoring. Both pose-direction interpretations remain explicit; physical timing/reference gates remain unresolved. Existing stereo outputs can support separately labelled per-modality projected-reference diagnostics, without a physical cross-modal accuracy claim.

[EXP-046](experiments/EXP-046-ms2-stereo-depth-proxy.md) executes the fixed MS² stereo/depth diagnostic: primary within-image uncertainty/proxy-error rho medians0.353RGB/0.453thermal, falling to0.120/0.385 on common depth-product support. All120 cases and16,380 independent checks pass. The result is sensitive to support/reference choice and establishes neither physical covariance calibration nor a cross-modal winner; no inference or defaults change.

[EXP-047](experiments/EXP-047-ms2-depth-spatial-controls.md) adds fixed depth/spatial controls: primary own-support joint within-bin coefficients0.179RGB/0.388thermal with~99.5% retained. Common-support RGB is weaker and eligibility-sensitive. All720 controls and41,416 independent checks pass; no physical calibration, cross-modal winner or default change follows.

[EXP-048](experiments/EXP-048-ms2-day-replication.md) replicates the fixed proxy protocol on a distinct moving MS² day pilot. Primary rawrho0.252RGB/0.392thermal; joint within-bin coefficients0.130/0.149 (night0.179/0.388). All78 frontend fields,120 proxy cases and720 controls validate. Association is positive but varies across captures; no physical calibration/camera-winner claim. Full day archives are now local, so further fixed intervals need no new upload.

[EXP-049](experiments/EXP-049-ms2-day-later-windows.md) tests fixed middle/later intervals of the existing day recording while CATS is unavailable. Primary joint coefficients are0.149RGB/0.209thermal and0.131/0.307; positive median proxy association persists with variable magnitude. All80 new stereo fields,240 proxy cases and1440 controls validate, with original source IDs/times preserved and no inference/analysis default changes.

[EXP-050](experiments/EXP-050-ms2-timing-metadata.md) clarifies timing scope: the inherited2ms gate is a diagnostic, not a validated universal MS² acceptance rule. Provided-pair benchmark work can proceed with documented assumptions. Recorded-offset associations vary across windows and do not identify exposure delays;18,916 validation checks pass, with no gate/source/model changes.

[EXP-051](experiments/EXP-051-ms2-selective-risk.md) tests fixed-coverage selection: retaining the lowest covariance-trace half lowers single-scan reference MSE in160/160 primary image/modality cases. Group median ratios to random retention range0.080–0.549;common-support sensitivity also improves.49,871 checks pass,485 inputs/1,944 historical hashes unchanged. Selection utility against provided labels does not establish covariance calibration or cross-modal weighting;simple scene/depth baselines are the next control.

[EXP-052](experiments/EXP-052-ms2-selection-controls.md) adds fixed depth/location selection controls. At50% retention,stratified uncertainty beats matched random in156/160 primary cases,but nightRGB median ratio changes0.080→0.937 under fixed depth/location composition. Central pixels beat uncertainty in19/20 opening-day and12/20 middle-day thermal images. Random-baseline gains do not establish superiority to simple selectors.180,158 checks pass;492 inputs/1,949 historical hashes unchanged. Next:fixed-selector loss/tail sensitivity before weighting.

[EXP-053](experiments/EXP-053-ms2-loss-tail-sensitivity.md) changes loss to mean Euclidean normalized residual with all selectors/coverage unchanged. Stratified50% selection improves in160/160 primary cases;group median reductions9–37%,with the central-pixel thermal baseline still stronger in opening/middle day. Worst1% contributes12–39% of MSE across group medians.229,331 checks pass;493 frozen inputs/1,963 historical hashes unchanged. Keep both loss diagnostics;next prioritize fixed independent-condition replication,not selector tuning.

[EXP-054](experiments/EXP-054-ms2-rain-replication.md) replicates on fixed rain recording `_2021-08-06-16-19-00`,first20 original IDs. Original40 stereo fields/80 producer inputs/two bitwise repeats pass. At50% within-bin retention,mean-L2 ratios0.827RGB/0.739thermal improve40/40 primary cases;MSE0.623/0.505 improves39/40. Global/simple-baseline and reference/support/tail sensitivities retained.119,120 final checks pass;1,977 historical hashes per audit unchanged. Distinct recording,not causal weather/physical covariance/camera winner evidence;CATS pending. No new upload needed;rain/day archives now cached.
