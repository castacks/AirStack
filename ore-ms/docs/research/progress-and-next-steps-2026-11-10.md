# RGB–Thermal Visual Odometry: Weekly Progress

**10 November 2026 · error and accuracy**

The objective is better RGB–thermal visual odometry. MACThermal already propagates uncertainty; improved geometric accuracy is the target.

Earlier covariance calibration, sensor selection and translation-fusion variants did not consistently beat RGB. A repetition review deferred more confidence variants. A reference-assisted depth ablation was parked because recovered LiDAR maps lacked joint support at native track endpoints.

We then completed a mean-changing experiment: 40 fixed updates on 20 day and 20 night frames, followed by a separate 20-frame rain evaluation. Only 2,305 horizontal mean-head scalars changed; uncertainty parameters and temporal-flow outputs stayed original. Rain stereo AbsRel worsened 0.1792 → 0.1827. VO changes were tiny and mixed, and training loss itself worsened 1.7177 → 1.7235. No accuracy gain or model promotion was established.

**Latest progress: a training-only correction now passes its descent checks.** The shared mean head is reused throughout a recurrent decoder whose coordinates are deliberately detached between iterations. On training day frame 0, autograd predicts descent while recomputed forwards move uphill at two tested scales, even with TF32 disabled. Night agrees in sign but differs substantially in derivative magnitude. This does not establish an upstream bug or completely explain the failed fit.

Tested a narrow alternative: freeze the complete decoder and add a zero-initialized horizontal residual only to its final output. Features and native upsampling weights remain frozen; the correction has 2,305 scalars and no recurrence feedback. Zero residual exactly preserves all 40 training-frame baseline outputs.

The 40-frame objective gradient matches FP64 finite differences at both declared scales. Three objective-checked, full-batch training updates produce monotonic decrease:

| Update | Training loss before → after |
|---|---:|
| 1 | 1.717747 → 1.708617 |
| 2 | 1.708617 → 1.699557 |
| 3 | 1.699557 → 1.690568 |

That is a **1.58% training-objective reduction**. It establishes descent for this parameterization; rain and VO accuracy have not been re-evaluated. The old checkpoint/results remain unchanged. Both the adapter and optimizer differ from the old run, so this does not isolate the cause of its failure.

Five tests and independent audit pass: 225 source/input hashes, 40 training caches, an analytic convolution/upsampling gradient, both finite-difference scales and all three acceptance conditions. [Detailed findings](mean-descent-contract-findings-2026-11-10.md).

**Next:** a fixed-budget fit of the verified final-output adapter, followed by the existing record-separated stereo and native VO controls. Lock the final adapter before new evaluation; keep uncertainty and temporal flow fixed. No training-budget or checkpoint search against rain scores.

All data remain exposed development recordings with supplied references and nominal alignment. Training descent is not a generalization or deployment result. No new data acquisition, rain evaluation, runtime benchmarking or default change occurred in this continuation.
