# RGB–Thermal Visual Odometry: Weekly Progress

**10 November 2026 · error and accuracy**

The objective is better RGB–thermal visual odometry. MACThermal already propagates uncertainty into pose estimation; improved geometric accuracy is the target.

Earlier work reproduced the native pipelines and tested covariance calibration, sensor selection and translation fusion. None of the recent variants consistently beats RGB. A repetition review deferred more confidence variants. Exact native replay was verified on 120 pairs. The proposed reference-assisted depth ablation was parked after recovering 40 rain maps: sparse labels provided insufficient support at both endpoints of native-selected tracks.

**New work completed:** a mean-changing thermal stereo experiment, using sparse depth pixels directly rather than requiring labeled native track endpoints.

- Trained on 20 day plus 20 night frames; evaluated a separate 20-frame rain prefix. All records were already development-exposed, with rain excluded from fitting/checkpoint selection.
- Adapted only 2,305 horizontal mean-head scalars for 40 fixed updates. Covariance parameters, all other state and vertical head rows remain unchanged. The final checkpoint was locked before evaluation; no parameter/checkpoint search.
- Evaluated stereo accuracy and ran original thermal, unchanged-copy, adapted-stereo and RGB native VO controls. Temporal flow and disparity variance retain original-model outputs. The unchanged copy reproduces all 20 pose/time arrays exactly.

**Outcome:** this configuration provides no usable accuracy gain. Excluded-record depth AbsRel worsens 0.1792→0.1827 (+1.96%); depth MAE worsens 2.613→2.646 m.

Native VO on 18 common intervals:

| Arm | Translation mean / RMSE (m) | Rotation mean (deg) |
|---|---:|---:|
| Original thermal |0.960001 /1.048964|0.075270|
| Adapted stereo mean |0.959302 /1.049632|0.074118|
| Original RGB |0.670000 /0.867444|0.031285|

The tiny 0.073% mean translation reduction accompanies worse translation RMSE/tail error. Thermal coverage stays 18 of 19 intervals; RGB has 19. No default is changed.

**Important limitation:** fitting itself did not demonstrate descent. A post-fit check finds training SmoothL1 worsening 1.7177→1.7235 (+0.335%). This does not establish successful fitting followed by poor transfer. A training-only direction probe highlights the distinction between the native recurrent stop-gradient graph and the recomputed forward response, without conclusively identifying the cause.

**Next prerequisite:** verify objective/descent behavior for the shared recurrent mean head before another adaptation run. Keep this checkpoint and result fixed. More data, a larger model or another learning rate is not automatically the next step. This is a concrete optimization issue exposed by a real mean-changing experiment.

Six tests and independent audit pass: 305 source/input hashes, 40 training steps, 200 stereo metric checks and 73 native motion reconstructions. [Detailed findings](stereo-mean-adaptation-findings-2026-11-10.md).

These are short, correlated, exposed development prefixes with supplied references and nominal alignment. Physical ground truth, fresh-record generalization and full-record trajectory improvement remain unestablished. No runtime benchmarking or model promotion.
