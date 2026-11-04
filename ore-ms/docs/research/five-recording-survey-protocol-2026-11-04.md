# EXP128: fixed five-recording development survey

2026-11-04 / ore_ms,parentd8de5b4b2. Development, not blind or training-held-out.
Use exactly the five EXP127 additional BoxTD3archives; allhave prior project
image/LK exposure inEXP035/036. No protectedMS2 or new sourcepose/errorlabels.

Freeze before newselectedpixels/native outcomes: inherited chronological-nearest
without-reuse2msstereo/30mscross policy, exactDecimal source integerns, existing
filenames only, invalid/duplicate/missing rows counted. Preserve original source
indices; noclockoffset/scale fit. For eachrecord split its unique RGB-left elapsed
time into3equal thirds. In eachthird selectfirst100consecutive paired groups wholly
inside thethird andwithpositive<=150ms gaps in allfourstreams, plus nonoverlapping
cross-modality sourcegroups. No brightness/support/reference/error filtering,
threshold widening, smaller substitute windows or better-record replacements.
All15thirds reported, including missingmetadata/insufficient/failed windows.

Stage1 metadata extraction only: four associated processed image timestamplists,
two rawthermal timestamplists andfourcamera-info data.yaml files. Use prior verified
ZIPcentraldirectories, exact memberCRC+SHA and bounded reads<=2MiBmetadata/member,
<=8MiBprefetchchunk. No numericreference/pose stream. Freeze metadata+selectedrows
before pixels. Past pairing failures are not ignored or relabelled naturalfaults.

Only qualifiedwindows proceed to inputpreparation and nativecapture. Restore
existing MSAcalibration/upstreamproducer from old recovery packet, preserve its
recipe and sourcehashes; verify exact originalthermalproducer controls before
MSAregeneration. No newphotometric/calibration recipe or changing native estimator.
Input/source/runtime/capturefreeze before neuraloutcomes, one retained attempt
perqualifiedwindow, strict native terminalzero-match diagnostic wrapper only.
No native failures or inputerrors converted to successfultracking. Dense fullread
survey supplies constantRGB/thermal/union availability ceilings at original ordinary
deadlines, not adaptive superiority or poseaccuracy. Candidate remainsfrozen.
If no100groupwindow qualifies, stopbefore pixels/inference and report onlymetadata
eligibility under this protocol, not absentRGBdetail/complementarity/physicalsync.

Independent semantic tests, greedyassociation/selection reconstruction and complete
15case bookkeeping. No reservedpayload, labels, training, defaultpolicy change,
externalmessage or postoutcome criterion change. Final inputpreparation/capture
implementation may be frozen in a separately named stage2lock after metadata and
before images/nativeoutcomes; it cannot change the above sampling/preprocessing.
