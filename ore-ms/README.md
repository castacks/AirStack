# RGB–thermal geometric uncertainty research

## Longer continuation limits the thermal benefit claim

The [next600intervalcheck](docs/research/fixed-rotation-continuation-findings-2026-11-15.md)
on originalcandidate recordings shows qualifiednightthermal31.276695m vsRGB-only
31.131610m(+0.466%),despitebeatingnative33.482921m. Dayhasone2.944mmnative-reproduction
failureagainstfrozen1mmbound; itsnumbersarediagnosticonly andthermalalso trails
RGB-onlyonnewspan. Samecandidate/prefixstates,no retuning.28946structuralchecks
pass,butdayaccuracygatefails. Shorttransfergainsremain; no persistentthermal
improvementovermatchedRGBcontrol established. Notlongertransferprefixrecords.

## Unchanged recording transfer: day and night gains

The [two-recording transfer](docs/research/fixed-rotation-recording-transfer-findings-2026-11-15.md)
keeps the candidate unchanged. On other100-group day/night records, positionRMSE
improves8.507%/1.031%versusnative and6.401%/2.857%versusRGB-onlyrefinement.
Rotation exact,99/99coverageallarms;2453auditchecks pass. Bothprefixes previously
exposed and≈10seconds; no blind/long-run generalization claim. Prior rainthermal
regression remains. Preservecandidatewithoutretuning; transfer adds evidence,
notnewR/Tdecouplingnovelty or fullfrontend/runtime qualification.

## Fixed-rotation translation baseline: scoped day improvement

The [measurement-level translation test](docs/research/component-constrained-native-findings-2026-11-15.md)
keeps native RGB rotation exactly. Day position RMSE11.115217→11.037059m(-0.703%);
RGB-only reoptimization reaches11.074827m, so thermal adds0.341%improvement beyond
thatcontrol. All6dayblocks beatbothcontrols;4/5night and1/6raincompleteblocks do.
Rain's fullpath gain againstnative comesfromRGBreoptimization:thermal is0.247%
worse thanthatcontrol.10922checks pass. Existing-family applicationablation,
notnewconfidence/decouplingnovelty; SFVO/Buczkoprimarysources added toregistry.
Preserve this candidate for separately frozen validation; no weight/window tuning.

## Native pose feedback completed

The [feedback replay](docs/research/native-pose-feedback-findings-2026-11-15.md)
now carries each arm's own float32 pose through source-point registration and
next initialization. Day position improves0.91% with orientation cost7.32%;rain
worsens7.91%/2.74%;nightgap247persists. Same4of17complete fixedblocks improveboth.
10884checks pass. This closes the fixed-weight recipe's state-integration check;
no weight tuning or default replacement. Cached measurements, not new inference.

## Native optimizer thermal baseline executed

The [native thermal-factor comparison](docs/research/native-thermal-constraints-findings-2026-11-15.md)
preserves RGB graph/solver bodies and reproduces all1796native motions. Day
position RMSE improves0.91% with orientation cost7.32%; rain worsens7.90%/2.95%.
Four of17complete fixed blocks improve both metrics; all18blocks retained.
3697independent checks pass. Saved-input optimizer replay, not frontend/feedback
VO integration or validated replacement. No weight search or timing correction.

## Native pipeline control completed

The [longer native control](docs/research/longer-native-control-findings-2026-11-15.md)
shows native MAC-VO beats the current fused replay on both day path metrics.
Rain position improvement over native is already present in RGB-only replay;
thermal slightly worsens it. No overall pipeline improvement is established.
Keep native MAC-VO as the application control; retain same-backend comparisons
for attributing thermal effects. No new inference, training or timing correction.

## Measurement-time review completed

The [source review](docs/research/measurement-time-contract-review-2026-11-15.md)
confirms the official loaders use supplied indices and the repository revisions
are unchanged. Physical exposure/clock semantics remain unresolved. Recorded
interval differences and the duration probe do not prove exposure mismatch.
Continue only with an explicit supplied-pair assumption for nominal benchmark
claims; do not apply a duration correction or repeat the closed clock search.


This project studies geometric uncertainty in visual and thermal estimation,
including matching supervision, selection, continuity and sensor transfer. Its
initial RGB–thermal question asks whether the modalities' uncertainties are
comparable and predict relative geometric reliability. A distinct contribution and consistent adaptive pose improvement
have not yet been established.

The [architecture and contribution reset](docs/research/rgb-thermal-architecture-and-contribution-reset-2026-11-14.md)
clarifies the starting point: MAC-VO processes visible-light stereo; MACThermal
processes thermal stereo. Their original systems do not jointly fuse RGB and
thermal. Our proposed condition-aware relationship layer would combine evidence
from these branches to improve motion accuracy or preserve it at lower measured
computation. It is a research hypothesis, not a validated model. The
[local relationship-layer contract](docs/research/local-relationship-layer-contract-2026-11-14.md)
specifies the shared-backend candidate and qualified input cache. The
[completed learning comparison](docs/research/local-relationship-learning-findings-2026-11-14.md)
retains scoped fusion gains but finds no added value over simple weighting. The
[consolidated decision](docs/research/rgb-thermal-weighting-consolidation-2026-11-14.md)
records overlap across the completed weighting studies and their remaining claims.

Start with the [research handoff](MS_RESEARCH_HANDOFF.md) for current context
and the original research question. The [scope decision](docs/research/research-scope-decision-2026-11-13.md)
records the requirement for a bounded, evidence-backed direction before further
experiments. The [contribution draft](docs/research/contribution-statement-2026-11-13.md)
separates supported findings from claims that remain unverified.

The [earlier contribution synthesis](docs/research/contribution-synthesis-decision-2026-11-14.md)
connects the completed studies to the original papers' remaining gaps. The
[updated technical report](docs/research/uncertainty-interfaces-report-draft-2026-11-13.md)
collects supported findings and their limits.

The [sensor-transfer inventory](docs/research/sensor-transfer-readiness-2026-11-14.md)
records current input availability, preprocessing and the claims supported by
nominal references versus qualified physical sensor evidence.

The [active-perception motion decision](docs/research/active-perception-motion-decision-2026-11-14.md)
selects matched acquisition histories as a controlled question. The
[completed estimator comparison](docs/research/active-motion-estimator-findings-2026-11-15.md)
finds small, mixed timing effects; no planning policy is validated.

The [cross-match covariance study](docs/research/cross-match-covariance-findings-2026-11-15.md)
uses saved trials to separate within-match variation from cross-match dependence,
with no new inference or covariance correction.

The [recording-excluded covariance comparison](docs/research/covariance-transfer-findings-2026-11-15.md)
finds partial predictive gains beyond a fitted scalar, with mixed transfer and
containment; it changes no pose estimate.

The [shape/scale diagnostic](docs/research/covariance-shape-scale-findings-2026-11-15.md)
interprets those fixed predictions using an explicitly test-informed oracle;
no predictive result or uncertainty interval is replaced.

Raw experiment outputs are kept outside Git. The
[verified backup index](docs/research/artifact-backup-index-2026-11-15.json)
records Box artifact locations and readback checksums.

The [fixed-group diagnosis](docs/research/fixed-group-diagnosis-findings-2026-11-15.md)
identifies a temporal-comparison confound in the nominal same-ID fusion studies.
Their numerical results remain unchanged, but do not establish physical thermal
sensor quality or environmental predictability. A qualified measurement-time
contract is the prerequisite for further physically interpreted fusion tests.

## Where to find things

| Location | Purpose |
|---|---|
| [Research handoff](MS_RESEARCH_HANDOFF.md) | Current research context, original scope and historical decisions |
| [Research notes](docs/research/) | Dated protocols, findings and detailed analyses |
| [Paper registry](docs/research/references/README.md) | Primary sources, reading depth, implementation pins and limitations |
| [Experiments](experiments/) | Experiment declarations and reproducibility records |
| [Research tools](tools/) | Instrumentation, analysis and audit scripts |
| [MAC-VO source](MAC-VO-main/) | Extracted upstream pipeline and setup instructions |
| [Agent instructions](AGENTS.md) | Research continuity and operating constraints |

Large datasets, checkpoints and generated results are stored outside Git.
See the [setup instructions](MAC-VO-main/Scripts/Setup/README.md) and the dated
experiment records for their required environments and recovery artifacts.
Historical runtime verification does not establish the current machine's state.

## Documentation convention

Keep this README as a project entry point. Put detailed results in dated research
notes and link current decisions from the handoff; do not copy the progress log
into both files. Older proposals describe their publication-time status and may
have been superseded by later executions.

The former README is preserved in [README_HISTORY.md](README_HISTORY.md).
It is an archive, not the current action list.
