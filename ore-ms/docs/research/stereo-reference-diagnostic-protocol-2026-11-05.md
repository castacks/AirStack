# EXP145 stereo/input/reference diagnostic

2026-11-05 America/New_York; parent EXP144/08c1583f5. Reuse its fixed first2,000 original night groups and both cameras; all predictions and provided references already development-exposed. No native SLAM run, model update, new payload, reserve access or change to production/scoring/calibration. Freeze diagnostic source/inputs before computing new values.

Check supplied calibration left/right K, stereo R/T, mm→m and fxB against prepared ORB and DROID plans. Verify EXP144 source/runtime/prediction/reference pins. Keep official loader camera-to-initial convention from saved sources; do not choose inversion/extrinsics by error. Timestamp diagnostics use original four integer clocks: signed right-minus-left, absolute deltas, cadence and counts. Recorded timing differences do not establish exposure offsets.

On all EXP144 primary scored approximately1s pairs, use each camera's unchanged common support and boundaries. Compute reference/predicted relative transforms and translation norms, ratio for nonzero reference motion, direction cosine when both nonzero, parallel and perpendicular translation-error RMS and squared shares. Four fixed source bins0–499/500–999/1000–1499/1500–1999 retain all pairs by start index. No fitted scale/time/axis/direction or metric change. These are descriptive components, not a learned correction. Cross-camera references keep their own source clocks and no physical common-time claim.

Fixed image indices0,100,...1900 for each camera. Check original/canonical SHA and exact inherited producer replay. Generic OpenCV ORB:2000 features,scale1.2,8levels,FAST20,edge31,Harris,patch31,WTA2; Hamming2NN ratio0.75 each direction plus mutual match. Do not apply an epipolar constraint before collecting matches. Save all accepted coordinates/distances. Report empty counts, positive left-minus-right disparity, abs(vertical)<=2px fraction, vertical residual and normalized vertical residual statistics. This proxy differs from native SLAM matching; correspondence outliers, scene depth/motion and preprocessing can affect it. It cannot alone establish rectification or physical reliability. No matching/recipe/threshold sweep or fitted F/H.

Use exact EXP136 runtime d3cf8a41dd8090c8939cd840b4735d526cec9a54e48ac36d506f91d13ee56e1c with CPU/no native inference. Independent audit reconstructs relative-motion translation with separate scalar arithmetic and match statistics from raw coordinates, checks all source populations/pins and rejects semantic mutations. Retain failures and immutable outputs. No inference retry, default change, EXP133 promotion, heldout/adaptive/physical/online claim. Persist findings/figure, git bundle and download-verified Box backup.

Amendment after initial diagnostic stop, before any completed output/score files:
original producer replay runs in original EXP134 image1542a9a2fe4f, OpenCV4.6.0.
The scoring image has OpenCV4.11.0 and differs by1uint8 level on the first thermal
pair; unchanged original producer/image reproduces saved bytes exactly. Preserve
initial freeze96151728c and failure log. New freeze pins the original-runtime
80-image replay report and verifier before repeating the deterministic diagnostic.
Descriptor extraction/scalar diagnostics remain in the originally declared scoring
runtime, on unchanged saved canonical images. No threshold/pose/input changes.
