# Sparse raw-thermal hook: implementation and execution gate

12 November 2026. Builds on the [paper-informed feasibility check](radiometric-tracking-preflight-2026-11-12.md), not a new method claim. No native accuracy experiment has run.

`tools/radiometric_native_hook.py` compiles a process-local experimental class from the exact SHA-256-pinned MACVO source. The only inserted statement invokes an endpoint callback immediately after native target-point construction. Original source bytes and the ordinary imported class remain unchanged. Only the selected execution runner's class binding is temporarily replaced, with cleanup on exceptions. The source hash or insertion-anchor mismatch aborts.

The callback uses exact thermal left-camera timestamps to retrieve hash-verified original uint16 images. No nearest-time search, second rectification or processed-to-raw coordinate rescaling is allowed. Selected source points and original endpoints enter the same frozen local aligner. Accepted fits replace only target endpoints; rejected fits retain their original endpoint. Source point ordering, dense flow, motion initializer, point selection, covariance and masks remain unchanged. Target-depth lookup correctly occurs after refinement.

This fixed retention policy avoids creating point-count gains by discarding difficult tracks. It does not guarantee motion availability or accuracy. Original covariance is an explicit uncalibrated control after refinement. The callback returns the original device/dtype and refuses integer endpoint arrays that would truncate subpixel corrections.

## Validation and remaining prerequisite

CPU tests verify a single insertion, structural equality after removing it, ordering before inbound/depth lookup, changed-source refusal, no-op object identity, known-shift correction, failed-track retention, array ordering, dtype preservation, input immutability and binding cleanup after failure. Native class loading and tensor execution require the original Torch/PyPose/compiled runtime and are not established by those checks. The Blackwell GPU is visible, but the original MAC runtime image and model weights are not loaded locally.

Do not run an accuracy arm until a fresh native original and no-op instrumented capture on the exact same input produce bitwise-identical poses, clocks, tracking flags and exported uncertainty arrays. Any parity discrepancy stops execution before refinement. A Python identity check alone cannot satisfy this gate. The original runtime/checkpoints must be recovered and verified from the existing Box packets, without rebuilding or updating dependencies.

## Fixed comparison design; execution freeze still pending

Population: first 100 groups of the same exposed day/night/rain recordings used by the longer comparison. Four arms per recording: fresh native thermal, no-op instrumented thermal, raw-refined original thermal, fresh native RGB. No TartanMatch replacement, NEC/PNEC solver or covariance fitting enters. Retain the existing 15×15 patch, ±2-pixel box, 30-evaluation budget and texture/rank checks. Do not change these after outcomes.

Use the existing nominal interval mapping and supplied reference conventions. Lock all predictions for all three recordings before reference-value scoring. Report every nominal interval, common-support translation and rotation mean/RMSE, tails, and own eligible coverage. Refinement must reduce thermal translation mean AND RMSE, avoid higher rotation mean/RMSE, and preserve thermal coverage separately in every recording. Any failure closes this exact configuration without parameter/window searches. Claims against RGB additionally require lower translation/rotation mean AND RMSE and no RGB coverage loss separately in every recording. Do not pool away a failure.

Before execution, freeze complete source, runtime, checkpoint, input, scorer and protocol hashes. That runtime-bound freeze is NOT yet created; this document is a comparison design. No original-model or default change, neural inference, pose solve or accuracy scoring occurred in this step. Direct real-flow truth remains unavailable, so a motion gain would not by itself prove improved endpoint accuracy.

Recovery prerequisite: `ore-ms/backups/2026-11-12-recovered-branches-dc96e91c5` contains complete Git history; original runtime/model recovery is described in the thermal matcher pilot findings. This hook is a mechanism prototype, not a reproduction of Self-TIO or TP-TIO.
