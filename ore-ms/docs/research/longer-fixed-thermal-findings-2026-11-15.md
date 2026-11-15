# Longer saved-sequence validation: limited simple thermal contribution

**Interpretation update:** [Fixed-group diagnosis](fixed-group-diagnosis-findings-2026-11-15.md)
quantifies sensitivity to mismatched recorded RGB/thermal intervals. The same-ID
nominal results below remain unchanged; they do not isolate physical sensorquality
or environmental contribution. No corrected fusion result is claimed.


15 November2026 America/New_York; ore-ms-orb-development baseline53afa19d1.
Fixedthermalprecision0.1 and originalconditional12stepsolver unchanged. No new
training,frontends,calibration or weight selection. Allthreepredictionfiles locked
before referencevalues were parsed for new scoring. No failed numerical pose solve.

## Result

The stronger short-prefix position gains do not carry over at the same magnitude.
On another dayrecording, positionRMSE improves0.69%but orientationRMSE worsens3.74%.
On the extended rainrecording, positionRMSE worsens0.42%and orientation0.28%.
Night has one common missingincrement, so a continuous fullpath is not reported.
Its fixed100groupblocks show mixed small position changes and rotationtradeoffs.
This is limited and mixed evidence for the existing simplebaseline, not support
for another learnedweight variant or a robust general RGB–thermal improvement.

## Data and exact method boundary

First600same-IDgroups from _2021-08-13-16-31-10(day),
_2021-08-13-22-16-02(night),_2021-08-06-16-19-00(rain). Day/night differ from the
original100group experiments; rain extends the same source. Thus both duration
and some recordings changed, and their effects cannot be separated causally.
Allthree records previously development-exposed, not a fresh blindtest.
Durations60.526/61.193/60.397seconds. SourceK,timestamps,native firstwrite events,
pre-interpolation support,original calibration and every599expectedslot retained.

Recovered wholearchives against historical SHA and selectedmembers individually:
decisive16members,accuracy8,fusion1800referencefiles. Day/night nativegraphs contain
full5695/5234groups; rain600. The initialpreparation incorrectly required exactly600
graphframes and stopped before any pose predictions. The preserved correction
binds first600sourcegroups against originalgraph IDs/times; no scientificweight,
solver,reference or tolerance changed. Retrospective final savedgraph fields and
original firstwrite metadata are inputs; this is not a new causal online replay
or proof of acquisition cadence.

Controls: RGB-only,thermal-only,gate0.1joint and fulljoint(gate1), allcovariance-
weighted conditional fixedsource-landmark solve. Joint uses solecamera fallback
if exactlyone native packet available; missingboth remainsnull. Purethermal
precision scaling in a solecamera fallback does not create new observations.
Allmethods preserve missingoutputs, no hold/interpolation or change to admission.
No learnedgate rerun because its addedvalue overfixedweights was unestablished.

## Continuous600pose paths

Initialgaugeonly: rightcompose599increments into600poses, reference
absoluteprovidedRGBposes transformed only by their firstpose. No fittedSE3/Sim3,
scale,yaw,time or offset alignment. Existing1e-5rotationserialization screen and
nearestproperSO3roundoff policy unchanged. Fullpath refused if ANYincrementmissing.

| Condition | RGB position RMSE m | Gate0.1 position RMSE m | RGB orientation RMSE deg | Gate0.1 orientation RMSE deg |
|---|---:|---:|---:|---:|
|day|12.043942|11.960719|6.870079|7.126708|
|night|Incomplete|Incomplete|Incomplete|Incomplete|
|rain|56.367367|56.605124|4.916814|4.930709|

Night pair247 has no availableRGBor thermal packet. RGB/gate0.1/fulljoint each
produce598/599increments; no mechanism here recovers that gap. Thermal-only
has595day,558night,598rain outputs and lacks a continuous600posepath in every
condition. No missingerrors are assigned0or stitched across the gaps.
RGB/gate0.1/fulljoint each have599outputs on day/rain. Allpointwise/trajectory
comparisons preserve their actual populations; absence of solverfailures is not
absence of missinginput. Large rainreference-relative errors remain explicit.

## Predeclared100group blocks

Six nonoverlapping windows start0,100,200,300,400,500; each has99within-window
increments and its own initialgauge. Betweenblockincrements are not scored as
part of either block; blocks do NOT form a stitchedfulltrajectory. One nightblock
(start200) is incomplete duepair247 and is retained. The other17blockcomparisons
are allreported, not selected for a favorable error sign.

| Condition | Block start | RGB position RMSE m | Gate0.1 position RMSE m | Position change % | Orientation change % |
|---|---:|---:|---:|---:|---:|
|day|0|3.750509|3.743598|-0.184|+0.528|
|day|100|1.502396|1.492505|-0.658|+3.437|
|day|200|1.899613|1.880528|-1.005|-1.969|
|day|300|1.892034|1.886328|-0.302|-3.192|
|day|400|0.803148|0.797536|-0.699|+17.520|
|day|500|2.580261|2.561761|-0.717|+4.110|
|night|0|1.694175|1.692479|-0.100|+0.887|
|night|100|3.342661|3.341600|-0.032|+3.692|
|night|200|Incomplete|Incomplete|—|—|
|night|300|4.602699|4.454071|-3.229|+24.790|
|night|400|4.335249|4.380417|+1.042|+0.308|
|night|500|3.559535|3.561153|+0.045|-0.863|
|rain|0|13.675134|13.717521|+0.310|+0.001|
|rain|100|0.973180|1.007841|+3.562|-0.631|
|rain|200|0.162172|0.163856|+1.039|+2.122|
|rain|300|14.686736|14.661588|-0.171|+2.251|
|rain|400|22.226387|22.353778|+0.573|-0.507|
|rain|500|16.694081|17.024094|+1.977|-0.416|

Negative percentage favorsgate0.1. Daypositionimproves in6/6blocks, but byonly
0.18–1.01%; two dayblocks(start200/300) improvebothpositionandorientationRMSE.
Nightpositionimproves3/5completeblocks and worsens2/5; the3.23%positiongain at
start300 carries24.79%orientationRMSEcost. Rainpositionimprovesonly1/6blocks,
by0.17%; other5worsen. Small scoped gains are retained without pretending every
condition must succeed. No significance,practicaleffect threshold or independent
samplecount is inferred from adjacentposes.

## Secondary controls

Fulljoint continuouspath position/orientationRMSE: day11.769936m/8.019051deg,
rain58.074646m/5.025671deg; nightincomplete. Morethermal influence has a larger
position/orientation tradeoff on day and greater rainregression. It is not adopted
as a new winner after scoring. Thermal-only errors and availableblocks retained
in scores.json. No nativefullpipeline comparator rerun or new author-reproduction
claim is made; primarydifference is thermal use within the samebackend.

## Verification and interpretation

Independent scalartranslation errors/componentR,tcomposition match savedpath
arithmetic. Alloriginalpredictionhashes unchanged and solverhash matches the
previouslearningfreeze. Extendedrain first100block reproduces the previousshort
RGB/gate0.1/fulljoint results to1e-8bound acrosspositionRMSE,finalpositionerror,
orientationRMSE andfinalorientationerror. Thus that positive/negative shortresult
was preserved, not overwritten by a changedmethod. Input/preparationfailure and
correction retained separately from scientificpredictions.

The comparison uses qualified providedRGBreferenceposes and nominal same-IDrig
geometry. It does not establish physicalthermaltruth,sensorclockequivalence,
long-runoutdoor reliability,unseenconditiontransfer,livecadence or computation
saving. Fixedsourcepoints and uncertainty assumptions remainunchanged; no causal
reason for the regression is established. Reference/geometrylimitations and pose
bias must not be silently reinterpreted as thermalentropy or corrected by fitting.

## Decision

The simplethermalbaseline has real scoped translation cases, but evidence does
not support promoting it as a broadlybetterVOarchitecture. It is materially less
impressive on the longer/differentrecordingcheck and retains rotationcosts/missing
output. The richerlearnedlayer remains unjustified over it. Preservecompleted
work and these source/endpointdistinctions; do not automatically launch another
weight,head,capacity,threshold or chosenwindow variant. No nextmethod selected.

Artifacts: results/longer-fixed-thermal/{recovery,preparation-attempt-001,run-001}.
Parentarchives:2026-11-06-decisive-uncertainty-evaluation,
2026-11-09-pose-covariance-accuracy,2026-11-09-translation-fusion onBox.
