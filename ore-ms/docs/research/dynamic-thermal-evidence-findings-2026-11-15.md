# Dynamic thermal contribution: time-resolved evidence and spatial interventions

**Interpretation update:** [Fixed-group diagnosis](fixed-group-diagnosis-findings-2026-11-15.md)
quantifies sensitivity to mismatched recorded RGB/thermal intervals. The same-ID
nominal results below remain unchanged; they do not isolate physical sensorquality
or environmental contribution. No corrected fusion result is claimed.


15 November2026 America/New_York; ore-ms-orb-development baseline53afa19d1.
Evidence study requested to corroborate changing thermal usefulness rather than
assume environmental complexity or randomness. No new fusionmodel or trainedgate.
There ARE new diagnostic pose solves after removing specified measurements.

## What is now corroborated

Thermal's effect is spatially structured within the tested estimator: removing
particular observationgroups changes the benefit/harm more than equal-count
random removal in the illustrated cases. Some useful groups occupy source-image
regions sparsely represented by ACCEPTED RGB matches. That is evidence of
measurement/support complementarity in this implementation, not proof that those
surfaces are physically invisible toRGB or of a universal environmentalrule.

The strongest harmfulday group has high local thermalcontrast and relatively
low predicted UVcovariance; a usefulday group has lower localcontrast. Thus
neither weakthermalcontrast nor highuncertainty is an earned general explanation
for the observedregressions. Most raw descriptor associations weaken or change
when examined on differentrecords, adjacentchanges, or estimatedmotion/time
controls. Do not turn this into a claim that thermal is random or unlearnable.

## Frozen evidence selection and controls

Primary: originalmatched100group3recordings,297slots,296pairedfactorpopulations,
600verifiedRGB/thermalleftimages and593nativecamera packets. ExistingRGBcov and
fixedthermalprecision0.1outputs. Gain=RGBtranslationerror−fixedjointtranslationerror;
rotationreportedseparately. Helps>=1mm,hurts<=−1mm,neutralotherwise. The1mmmargin
was declared before this analysis; it is a reporting margin, not physicalnoise
confidence or a statistically significant effect threshold.

All10predeclared descriptors retained: logthermal/RGB ratios of factorcount,
median owncamera source depth,depthIQR,UVvariance,relative disparityvariance,
4x4coverage,residualenergy atRGBfit,normalizedflow,patchcontrast andpatchgradient.
Ratios use1e-12descriptorregularization only; no measurement covariance changed.
For eachrecord choose largesthelp,largesthurt andnearestneutral before spatial
intervention. Events therefore deliberately use knownreference outcomes and are
NOT independent or representative validation cases. Allselectedevents retained.

For9events remove each of16fixedthermal-targetimagecells independently, leaving
RGBpoints,originalcovariance,gate0.1and12stepsolverfixed. Fourseededrandomcontrols
remove exactly the same number ofthermalfactors.720poses captured/locked before
ablation scoring,144cellgroups,allreports retained. No newbudget/thresholdsearch.
Emptycellremovals retained asnoops. The numericaleffect belongs to the whole
conditional solver; cell contributions interact and must not be added together.

## Temporal outcome counts

| Recording | Helps >=1mm | Hurts >=1mm | Within +/-1mm |
|---|---:|---:|---:|
|day|38|12|49|
|night|18|8|73|
|rain|1|11|87|

One raininterval has no thermalpacket; fixedjoint falls back toRGB and remains
in the complete outcome inventory, but cannot enter paired-feature associations.
A sequence contains changing outcomes even without a change in its day/night/rain
label. This is not proof that environment alone caused those changes.

## Localized examples, with same-count controls

The displayed cell for eachfixed help/hurt event is selected AFTER seeing all16
cell-effects as the largestabsoluteeffect. This is an explanatory example, not a
preselected spatial predictor or an independentlyvalidated rejection rule.
Positivegain means fixedjoint has smaller translationerror thanRGB.

| Recording/event | Pair | Cell | Removed thermal factors | Original gain mm | Gain after removal mm | Same-count random removal: change in error mm |
|---|---:|---:|---:|---:|---:|---:|
|day/helps|6|13|10|+12.093|+8.870|+0.437 to +0.959|
|day/hurts|0|15|14|-5.745|+1.435|-2.489 to -0.438|
|night/helps|91|7|15|+5.355|+3.235|+0.144 to +0.919|
|night/hurts|62|0|15|-22.616|-13.861|-1.685 to -1.240|
|rain/helps|78|15|4|+1.478|+1.154|+0.000 to +0.104|
|rain/hurts|1|0|9|-7.508|-3.175|-1.270 to -0.256|

Concrete observations:

- Dayhelp(pair6): removing10thermalmatches from lower-middle cell13 increases
translationerror3.224mm; fourrandomcontrols increaseonly0.437–0.959mm. The thermal
points' inferredsource-depthmedian is6.03m. The displayedregion lies overroad.
- Dayhurt(pair0): removing14matches fromlower-rightcell15 changes a5.745mm
regression into1.435mmimprovement. Randomsame-countremovals do not reverse the
regression. The displayincludesnearbyvehicle/curbstructure; source-depthmedian
3.72m. It would be premature to label these movingobjects,reflectionfailures,
wrongdepth or badmatches without per-correspondence verification.
- Nighthelp(pair91): removing15matches fromright-middlecell7 increaseserror
2.120mm,versus0.144–0.919mmforrandomcontrols. The displayedregion contains facade/
roofline structure. It is dim inRGB,but no calibrated visibility comparison is
established by the side-by-sideviews.
- Nighthurt(pair62): removing15upper-leftcell0matches reduceserror8.755mm,versus
1.240–1.685mmforrandomcontrols. A physical explanation of the structure/error is
not established.
- Rainhelp(pair78): removing4lower-rightcell15matches increaseserror0.324mm,
more than fourrandomcontrols. The displayedregion includesroadsidebarrier
structure, source-depthmedian5.48m.
- Rainhurt(pair1): removing9upper-leftcell0matches reduceserror4.332mm,versus
0.256–1.270mmforrandomcontrols. The view is sky-dominated and may include pole/
thinstructure; do NOT assume allthese are false skymatches or invalidfinite depth.

These examples localize influence inside the estimator. They do NOT prove which
correspondence is true, physical temperaturepatterns, motion of a specificobject,
or a general semantic rule. Fourrandomcontrols are controls, not a significance
or confidence-interval calculation. Neutralcases are retained in thefullartifact;
their largestcellchanges are small(about0.100mmday,0.0043mmnight,0.0011mmrain).

## What is beyond the current RGB feature set?

Posthocnominalsource projection maps inferredthermal3Dpoints intoRGBcoordinates,
then measures distance to the nearestACCEPTED RGBsourcefeature. This uses the
providedrig and same-ID assumption. No imagewarp,visible-surfaceverification,
independentdepth or cross-spectralmatch is supplied. It measures sampled-support
separation, not information thatRGB fundamentallycannot see.

| Event | Selected region median nearest RGB accepted feature, pixels | Remaining thermal region median, pixels | Selected nominal points inside RGB view |
|---|---:|---:|---:|
|day/helps|68.38|27.01|9/10|
|day/hurts|188.68|29.02|12/14|
|night/helps|13.73|26.07|15/15|
|night/hurts|150.45|20.88|15/15|
|rain/helps|142.49|17.96|4/4|
|rain/hurts|8.21|21.38|9/9|

Usefulday/rain groups sit in relativelysparse acceptedRGBsupport,consistent with
complementarysampling. Usefulnightgroup is nearerRGBsupport and can contribute
without spatialnovelty. Harmfulday/nightgroups are also far fromRGBsupport; extra
coverage alone doesnotguarantee accuracy. Pixelvalues and inferreddepths differ
acrosssensors; this table is not an alignedembedding comparison or a certified
same-surface/covisibility test.

## Whole-recording and time-change associations

Allfeatures and allrecords are reported, without choosing only favorablecues.
Spearman levelrho describes relation to translationgain; adjacent-change rho uses
only consecutivevalidpairindices, without bridging missingrows. No iidpvalues,
causality or statisticallyindependentsamplecount claimed.

| Descriptor logratio | Day level/change | Night level/change | Rain level/change |
|---|---:|---:|---:|
|count_ratio|+0.298 / +0.591|+0.044 / -0.101|+0.189 / +0.130|
|depth_ratio|+0.062 / +0.004|+0.206 / +0.167|-0.213 / -0.154|
|depth_iqr_ratio|-0.021 / -0.202|-0.268 / -0.028|-0.207 / +0.032|
|uv_variance_ratio|-0.523 / -0.718|-0.094 / +0.001|+0.313 / +0.471|
|relative_disparity_variance_ratio|+0.143 / +0.209|+0.134 / -0.133|-0.166 / -0.070|
|coverage_ratio|+0.032 / -0.085|-0.048 / -0.088|+0.055 / +0.158|
|residual_energy_ratio|+0.319 / +0.453|+0.146 / -0.141|+0.305 / +0.129|
|flow_ratio|-0.624 / -0.690|-0.038 / -0.078|+0.077 / -0.055|
|patch_contrast_ratio|-0.096 / -0.056|+0.218 / +0.174|+0.393 / +0.117|
|patch_gradient_ratio|-0.074 / -0.053|+0.196 / +0.142|+0.384 / +0.092|

A posthocnuisancecheck residualizes rankedfeature/gain against rankedestimatedRGB
translationnorm,rotationangle andsourcepairindex. For example dayUVvariance
association shrinks−0.523→−0.127,dayresidualenergy+0.319→+0.081. Rawtemporal
association partly tracks changingmotion/time; this check does not isolate a
physicalcause or account for everyconfound. Allpartialcorrelations retained.

Secondary600group study covers two DIFFERENT day/nightrecordings andextendedrain,
allalreadyexposed. Sevengeometrydescriptors only; correspondingnewday/nightraw
images are not restored, so no inventedcontrast/appearance/semantic measurements.
This is exploratory corroboration, NOT locked independentpredictivevalidation.

| Descriptor | Secondary day level/change | Secondary night level/change | Secondary rain level/change |
|---|---:|---:|---:|
|count_ratio|+0.086 / +0.056|+0.128 / +0.080|+0.045 / -0.131|
|depth_ratio|-0.081 / -0.183|+0.004 / +0.018|-0.009 / +0.109|
|depth_iqr_ratio|-0.036 / -0.087|+0.036 / +0.027|-0.056 / +0.018|
|uv_variance_ratio|-0.203 / -0.230|+0.030 / -0.108|-0.134 / +0.329|
|relative_disparity_variance_ratio|-0.107 / -0.215|+0.083 / +0.030|-0.029 / +0.007|
|coverage_ratio|+0.079 / +0.085|+0.158 / +0.052|-0.015 / -0.048|
|residual_energy_ratio|+0.247 / +0.283|+0.249 / +0.181|+0.117 / -0.208|

DayUVvariance direction persists across bothdayrecords and timechanges, but not
as a universalnight/rain law. Residualenergy levelassociation is positive across
all6recording/cohorts, but weakens underprimarymotion/timecontrol and has mixed
adjacent-change signs. It is mechanically tied to thecurrentRGBfit, so cannot
be promoted as independentenvironmentalevidence. No universalcue is established.
This does not demonstrate the impossibility of learning contextual relationships.

## What the image inspection actually supports

![Selected target views and spatial interventions](../../results/dynamic-thermal-evidence/run-001/event-regions.png)

![All-primary-interval time traces](../../results/dynamic-thermal-evidence/run-001/time-traces.png)

Images are originaltargetviews; thermaldisplayusesp5–p95only forvisualization.
No cross-camera imagealignment is implied. Globalplots show changingobservations,
not a causaltemperature/noise process. The descriptor alone cannot distinguish a
legitimate thermalboundary from a confidentlywrongcorrespondence.

## Verification, stopping, and exact remaining question

Independentcell0replays for eachfixedhurtevent reproduce removals andposesexactly.
720scalartranslationerrorchecks,125emptycellnoops and593originalpackethashes pass.
Parentposeandcachelocks verified; originalinputs/predictions not changed. All
720ablationposes succeeded; no newlearning/modelselection. Label-basedeventselection
and posthoclargestcellillustrations explicitly recorded. No physicalGT or new
clock/extrinsicqualification; allposecomparisons retain providedreference limits.

We can now name useful/harmful measurementgroups and their localstructure/support,
instead of only reporting day/night averages. The unresolvednextquestion is why
specificgroups are geometricallyreliable or unreliable: actual matching/depth
error,unmodeledmotion,calibration/timing orRGBbias remain candidateexplanations.
A targeted correspondence/depth check on these FIXEDcases would address that gap;
another learnedweighthead would not. No new fusionmodel,semanticmask orthreshold
rule selected from this explanatorystudy. Broaderenvironment-conditioned learning
remains open. Standardpoint-removal attribution is not claimed as algorithmnovelty.

Artifacts: results/dynamic-thermal-evidence/run-001. Sources and priorreferences
unchanged: originalMAC/MACThermal measurement representation and previous local
jointsolver/feature-cache studies. No newpaper-method claim or author reproduction.
