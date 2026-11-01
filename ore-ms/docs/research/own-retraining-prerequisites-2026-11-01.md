# Own retraining: a useful provenance path with separate contracts

2026-11-01 America/New_York. The user raised own retraining while authorizing
MS² adapter preservation. No training run or checkpoint replacement is started
here. Preserve the exact released weights as immutable numerical controls.

Own training can provide a recorded source commit, data/recording split,
preprocessing, objective, seeds, optimizer state and checkpoint identity. A
new finetune on our own split does not retroactively establish the training
membership of an unknown base checkpoint. A network-held-out claim needs a
source-backed initialization lineage and disjoint data at every training stage,
or an explicitly qualified claim about the new finetuning stage only.

Before a run, separately declare:

1. **What is trained.** Distinguish the flow/stereo frontend, covariance head
   and modality selector. Better covariance on stereo labels is not proof of
   better temporal flow, graph state or VO. Keep the current state boundary as
   a fixed control, not a moving target optimized on known failure labels.
2. **One variance interface.** The existing EXP068 audit and current source
   show inference emits exp(2*raw), while cov_loss consumes raw+epsilon and its
   flow_mask/max_cov arguments are unused. This does not identify the historical
   checkpoint recipe or establish the cause of a prior failure. A new trainer
   must explicitly choose parameterization and make objective/inference agree,
   test masking/invalid targets and finite gradients, and record the change as
   a new method. Do not silently reinterpret old checkpoint outputs.
3. **Recording-level data and labels.** Record exact train/validation IDs and
   aliases, initialization overlap and full source hashes. Existing exposed
   TD3/MS² material is development data. Keep the two conditional reserves out
   of training; no random adjacent-pair split may masquerade as recording-level
   independence. Stereo projected-depth labels, temporal-flow supervision and
   pose-reference labels have different assumptions and must stay explicit.
4. **Input and optimization recipe.** Freeze raw thermal uint16 handling,
   normalization/rectification, augmentation, calibration, batch/learning-rate/
   schedule, loss terms/masks, staged parameter freezing and software/runtime.
   Save full config/optimizer/step state and a manifest tied to each output SHA.
   Test loss semantics before a small development-only overfit/control run.
5. **Evaluation boundaries.** Freeze the candidate and baselines before test
   exposure. Report full coverage, matched conditional translation/rotation
   and calibration/ranking separately. A new network cannot supply missing
   physical frame, pose-direction, timestamp or reference-independence evidence.

The immediate engineering step remains native MS² temporal/shared preservation
on the already-exposed pilot. That input/backend path and a documented loss
contract are useful prerequisites for an eventual own-training experiment.
No hyperparameters, fitting run, new split assignment or promised quality gain
are selected by this note.

Sources: [existing training-semantics audit](uncertainty-training-semantics-2026-10-28.md),
`MAC-VO-main/Train/MatchingNet/loss.py`,
`MAC-VO-main/Module/Network/FlowFormerCov/flownet.py`, and the
[prospective readiness contract](prospective-state-shadow-readiness-2026-11-01.md).
