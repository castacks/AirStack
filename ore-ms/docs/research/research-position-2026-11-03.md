# Research position: uncertainty and bounded RGB–thermal arbitration

2026-11-03, branch ore_ms. Original scope remains RGB–thermal geometric uncertainty.

The original research question asks whether learned RGB and thermal uncertainty
is comparable and can identify which camera supplies more reliable geometry.
Earlier experiments support useful within-camera ranking on inspected data, but
cross-camera transfer and simple calibration have not established a general
selector. See the [fall assessment](ms2-fall-decision-2026-10-28.md) and
[transfer findings](ms2-transfer-findings-2026-10-27.md). Successful engineering
checks below do not turn that unresolved premise into a demonstrated result.

The current branch studies a narrower prospective method: estimator-health and
history-based arbitration with sparse inactive-camera probes, explicit
availability and output-readiness semantics. Its research question is whether
useful native motion can be maintained while avoiding unnecessary expert calls,
including through an availability transition. This is a method feasibility and
resource-accounting branch, not yet a calibrated-uncertainty fusion result.

| Evidence | What it establishes | What remains open |
|---|---|---|
| [EXP119](../../experiments/EXP-119-closed-loop-sparse.md) | Actual own-feedback skipping reduces calls 434 to 334 with 204/205 guarded deliveries on the exposed windows | Active camera stays fixed within each window; no adaptive superiority |
| [EXP120](../../experiments/EXP-120-availability-transitions.md) | Explicit synthetic outages can make switching useful: 85/95 vs 71/95 fixed-camera deliveries | Natural failures, silent-fault detection, independent conditions |
| [EXP121](../../experiments/EXP-121-readiness-continuation.md) | Output-ready probing can restore inactive outputs but lose a handover interval | Fresh health alone does not ensure contiguous native inputs |
| [EXP122](../../experiments/EXP-122-buffered-handover.md) | One real retained input repairs the observed continuity break; live synthetic candidate gives 85/95 with 141 vs 158 full-read calls | Real acquisition deadlines, bounded worker ownership and broader replication |
| [EXP123](../../experiments/EXP-123-retention-capacity.md) | Storage capacity affects whether that exact predecessor is available: 2 MiB misses, 4/8 MiB hit in fixed traces | Native behavior with worker leases and bounded control transport |
| [EXP124](../../experiments/EXP-124-leased-handover.md) | 4 MiB store + 2 MiB shared arena preserves 85/95 deliveries and all four returns, 141 vs 158 calls; 2 MiB diagnostic drops to 84/95 | Total process/GPU memory, acquisition cadence, natural recovery and physical accuracy remain separate |

The immediate contribution being tested is a causal, resource-accounted
scheduling and handover mechanism. A positive bounded replay is evidence that the
mechanism can execute under those stated limits. It is not evidence of improved
pose accuracy, calibrated probability, independent generalization, or universal
advantage over the best fixed camera. Byte-exact controls and independent audits
support implementation validity; they are not independent scientific replications.

After this mechanism boundary, the next research evidence should include a
separately predeclared condition/recording, simple fixed-camera and matched-cost
controls, and justified reference/deadline semantics. More engineering iterations
on the same synthetic day interval cannot establish that broader claim. Existing
exposed inputs suffice for bounded software checks; sustained natural returns with
a useful alternate and independent pose references would strengthen the study.

EXP124 makes the tradeoff explicit: primary restores4/4 returns versus original
sparse1/4, at141 versus133 calls and the same85/95 selected-delivery count. Full
read uses158 calls. This is readiness at measured cost, not evidence of selected
delivery or accuracy superiority over the original sparse baseline. The2MiB
native diagnostic loses the handover frame and drops to84/95. Serial controller
processing is51.592 seconds for a9.900-second source span; live cadence is not
established. Next research tests should address independent-condition utility
with fixed-camera and matched-cost controls rather than treating more checks on
this synthetic interval as generalization evidence.

[EXP125 night transfer](../../experiments/EXP-125-night-transfer.md) does not show
adaptive superiority. Unchanged candidate67/95deliveries with137calls versus
fixed thermal71/95with83calls; conditional always-thermal71/95on the same137calls.
Original sparse67/95with130calls and the same3/4return-episode readiness. Four
candidate abstentions have eligible thermal output by the original deadline;
that later output is not assumed available at decision time. This separates
implementation feasibility from missing research utility. Night recording was
previously inspected and has0imagehash overlap with developmentday; not blindholdout
or illumination causality. No policy tuning/adoption follows this failure.

[EXP126 decision-clock/headroom audit](../../experiments/EXP-126-decision-headroom.md)
sharpens the decision after failed transfer. Night's full-read and sparse executed
histories have no RGB-only eligible interval: union71=thermal71. No camera-choice
rule on those same outputs can strictly beat thermal for availability. Daycandidate
has only2remaining readout slots (85vs87), and its constant readout48/53counts are
constrained by its switching schedule. The current controller predicts an expert
before target motion exists; the original completed-motion selection proposal
uses a different decision clock/information set. Counts of unavailable preselection
are not measured erroneous pose publication or downstream risk. Keep mechanism
frozen; prioritize exposed-data/target adequacy and explicit final-output contract
before another selector fit. No new inference/policy, no change to EXP125 failure.

[EXP127 exposed-data adequacy](../../experiments/EXP-127-exposed-data-adequacy.md)
confirms the limit across18natural dense windows from7exposed recordings:allhave
zeroavailability headroom overtheirbestfixedcamera;thermalcovers295/295TD3 and
RGB855/855MS2.18healthreturns exist,none has>=3healthy/failed/returned phases.
The reference audit/restoredBoxevidence preservesnativeMS2adapterverification,
qualifiedprovided-referencecomparison feasibility,unresolvedphysicalrig/GNSSquality/
time andunknowntraining,withoutturningunknowntraininginto a universalbenchmark
blocker. Reserves remainclosed. The useful nextinputs are targeted reference
provenance/quality or separate complementary temporalrecordingmetadata;request
is drafted,not sent. No availabilityselectorfit is justified bythissaturated
cohort;the original accuracy/uncertainty question still needs its own declared
reference/interval/final-output contract and recording-level test evidence.

EXP127 also indexes five additional already exposed Box TD3archives outside the
native cohort, using40.9MBof bounded directory metadata with no image/pose-member
payload. All haveGNSS/IMU/TFexports butno explicitquality-namedstreams under the
fixedterms. Their nativecomplementarity remainsunknown; this is a concrete existing
source pool for a predeclared development survey,not evidence that all existing
data are exhausted. No further imageupload or reservedrecordopening is necessary
to begin that narrower survey. Referenceprovenance/quality is still a separate gap.

2026-11-04 follow-up: [EXP128/129](../../experiments/EXP-129-provided-export-survey.md)
retain the5recordingsurvey's4/15metadataeligiblewindows. The plannedMSAinputcontract
fails exactproducer replay onall800thermalinputs;noMSAnativeexecution orcriterion
relaxation. A separately frozen exactprovided-exportbenchmark runs4dualcaptures
withunchangedestimatorweights,380core slots,0windows positiveconditionalheadroom,
and0strictnativezero-matchfailures retained. This separates sourceprovenance
failurefromnativeasprovidedavailability anddoesnotvalidateMSAcandidatetransfer,
qualityselection orphysicalaccuracy. Three unqualifiedrecordings remainunknown.
No source questions sent, newinputrecipefit,policychange orreserveaccess.

EXP129 adds323jointly supported native intervals from4fixed windows/2sampled
recordings (providedarchive inputs,notMSAregeneration). Thermal covers380/380,
soavailabilityheadroomremains0;all8expertsreturn. Source directory listsSO/GNSS
reference exports buttheir payload/quality/coverage wasnotread here. This opens
an existing-data path to a separatelyfrozenqualifiedquality-headroom study with
source-backed frame/time/reference interpretation,not an availabilityselector
win or physical accuracy assertion. EXP128strictinputfailure remainsfailed.


EXP130 (2026-11-04), pre-posefreeze6324a366f:
[qualified provided-SLAM quality headroom](provided-quality-headroom-findings-2026-11-04.md).
All323jointEXP129rows score,0excluded,380coveragepopulation preserved. Nominal
body transforms/actualprovidedR/stationarytwist predictions locked before pose
payload. Bothsourceframes sensor_init->vehicle; large reference gaps elsewhere,
all scoredendpoint brackets<=150ms. Source posesZIPCRC/SHA;nofit/alignment.
Rotation RGBmean0.25021140deg,T0.23224070,oracle0.16556426:28.710% belowpooled
bestfixed;21.447% belowper-windowbestfixed0.21076723. Translation gain0.134%,
0.00017044m/interval,RGBwins59/T264;rotationRGB108/T215. Separate metric targets.
5245independentchecks+8nominalgraphchecks+5semanticcases pass;2780pins unchanged.
No native inference/policy/reservechange. RecordedSLAM dependencies,physical
mounting/clock and trainingoverlap unresolved;MSAproducer failure remainsfailed.
This supports rotation-complementarity investigation,not learnedselection,
sparsecompute savings or independentphysicalaccuracy. Next freeze causal
uncertainty/healthsignal audit with explicit completed-output vs pre-input timing.
