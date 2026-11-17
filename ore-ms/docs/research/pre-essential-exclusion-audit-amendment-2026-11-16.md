# Pre-essential exclusion: decomposition-audit correction

16 November2026 America/New_York. Amendment to the
[frozen experiment protocol](pre-essential-exclusion-protocol-2026-11-16.md),
made after refits/choices/scores locked but before accuracy summaries or paired
transitions were inspected. All original masks/source/protocol, predictions,
choices, reference values and thresholds remain unchanged.

The original independent audit fails its literal normalized rawE=[t]×R check
at1e-5. The first failure is original unmasked FlashLight SIFT RGB anchor0,
which already reproduced exactly in the required runtime. Raw E singular values
are approximately0.70713747,0.70707609,0.00005795, and its equality residual is
7.24e-5. The required exact unmasked-control parity remains PASS. The original
raw-E equality gate remains FAIL, with log/source and all1684failing variant rows
retained; it is not relabelled a successful raw-matrix-manifold check.

Official [OpenCV source, decomposeEssentialMat](https://raw.githubusercontent.com/opencv/opencv/4.11.0/modules/calib3d/src/five-point.cpp)
was inspected at the pinned4.11.0tag, alongside the
[OpenCV5API contract](https://docs.opencv.org/5.0/main_modules/d_projection.html#decomposeessentialmat).
The inspected source uses U/V from SVD to form the two rotations and t, rather
than requiring all raw singular values to be exactly the ideal essential values.
This older tagged source is a contract cross-check, not a claim to have obtained
the installed5.0binary's exact build source. Current runtime decomposition behavior
is verified numerically on the preserved experiment matrices.

Corrected audit verifies [t]×R against U diag(1,1,0) Vᵀ up to sign/scale, at the
same1e-5tolerance, and preserves every other original audit check. All1684raw-E
inequalities have SVD-consistency residual below8.2e-15 in the diagnostic receipt.
No stored E is projected, no pose is recomputed and no result array is edited.
The new audit certifies consistency with decomposition, not exact essential-
manifold membership of the original numerical E or independent physical truth.

`audit_pre_essential_exclusion_svd.py` is a necessary frozen audit variant: it
verifies the original auditor's SHA and substitutes only this numerical contract.
The old auditor remains byte-identical. An amendment lock pins both source paths,
this note, and all already-locked geometry/choices/scores/summary/transition files.
This explicit variant preserves old-source reproducibility rather than rewriting
the consumed freeze. It supersedes only the invalid raw equality requirement.

The original seven-test run cannot validate the full audit because its corrupted
cases also encounter that unrelated failure. Preserve that log. The corrected
suite reuses finite/corruption tests with the SVD auditor, adds an unmodified full
valid-result check and synthetic non-ideal-E/wrong-output controls. Stop any
remaining failure without a mask/solver/reference or numerical-tolerance rescue.
