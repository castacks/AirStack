# EXP118: fixed native gap replay

2026-11-03. Execution freeze `28db17670`; parent EXP117. [Full findings](../docs/research/native-gap-replay-findings-2026-11-03.md).

Fourteen fresh native captures use early-night 20, middle-night 100 and day 100 frames, both cameras, with dense and frozen sparse arms plus early dense repeats. Exact input/model/runtime recovery and source-row mapping retain gap endpoints. This is a dense-derived open-loop engineering test, not an adaptive sparse controller.

Actual pair calls fall 434 → 366 (15.7%); summed synchronized native processing falls 160.814 → 131.901 seconds (18.0%). Day RGB's matched translation proxy RMSE rises 0.043206 → 0.088447 m across 68 common-supported intervals; all 31 gap intervals remain supported. Middle RGB supports only 1 of 67 sparse intervals and none of its 32 gaps. Its sole matched result is terminal source 98 → 99, not tracking recovery. Early thermal results are mixed. Three all-read controls and both 533-array early repeats are bitwise exact.

Sixteen tests and 12,631 independent checks pass across all 366 sparse intervals; 1,437 execution pins remain unchanged. Full support/terminal counts, actual calls/times, matched finalized SLAM-proxy errors and repeat outcomes are retained. No training, reserve access or default adoption. Next: a controller using its own sparse observations and clock, with correct gap endpoint and deadline semantics. Existing exposed inputs suffice for that engineering test.

Backup: `backups/2026-11-03-native-gap-replay`. Original images/weights/compiler: EXP112; base runtime: EXP106.
