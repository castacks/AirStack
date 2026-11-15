# Method decision: learn query utility separately from measurement covariance

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus working sources.
Targeted prior-art/code/inventory review; no new training or accuracy experiment.

## Selected next hypothesis

**Can a pose-task-trained query policy improve thermal pose accuracy and
availability beyond both correspondence-quality selection and task-trained
residual weighting, while keeping the dense prediction model fixed?**

Select this as a finite baseline/adaptation hypothesis for original-paper V3 and
related T2/I2 questions. It is not an established novel algorithm. Its potential
contribution is a controlled separation of measurement uncertainty, query utility
and downstream accuracy in a dense learned thermal estimator. Evidence of a
specific methodological increment remains to be established; thermal application
alone does not establish novelty. Other original-paper gaps remain open.

The completed studies now support this question more specifically than a generic
“make the selector stable” objective. Query-policy changes cause pose response
with dense fields held fixed. On exact synthetic geometry, one case improves
both pose errors despite greater variance; five others worsen. Thus the next
objective must include true pose error and availability, rather than treating
reduced variance, high repeatability or small predicted covariance as success.
The 25.08% translation/8.44% rotation improvement is a real positive result of
that comparison; it is not evidence that a trained policy already works.

## What the literature already covers

| Source and reading depth | Established overlap | Consequence here |
|---|---|---|
| [NG-RANSAC, primary arXiv paper](https://arxiv.org/pdf/1905.04132), selected method; released training entry point inspected | Learns sampling guidance using task loss; the code supports pose, inlier and other objectives and a sampled expected-loss gradient. | A task-loss-trained stochastic selection policy is a prior-art baseline, not our new optimization principle. Essential/fundamental hypotheses and angular translation are different from this metric stereo registration harness. |
| [DISK, published NeurIPS paper](https://proceedings.neurips.cc/paper_files/paper/2020/file/a42a596fc71e17828440030074d15e74-Paper.pdf), selected method/reward and released training/loss files | Probabilistic feature/match selection with geometric per-match rewards. The inspected trainer uses depth/epipolar rewards; pose quality is a validation metric there. | Correspondence-reward learning is a necessary comparison. Do not claim that the whole DISK literature omits pose objectives or that its full detector/descriptor system equals our restricted policy. |
| [DNLS v2](https://arxiv.org/html/2305.09527v2),3.3/discussion rechecked | Learns correspondence covariance through pose loss; sparse gradient support and geometry diversity are explicitly discussed. | Pose-trained weighting and differentiating a pose objective are already established. A fixed-query task-weight control is required. |
| [Deeper-DICE](https://arxiv.org/pdf/2007.14943), abstract/III only | Predicts a pose correction and covariance with a joint error-model likelihood. | A generic learned mean/bias correction plus covariance head is also prior work. No Deeper-DICE implementation was inspected or reproduced here. |

Earlier R2D2 and Good Feature reading already covers repeatability/reliability
and geometry-aware selection. This is a bounded overlap check, not a systematic
survey or proof that the application-specific hypothesis is field-wide open.
The primary CVF NG-RANSAC PDF returned403; the primary arXiv paper was used.

Released source pins:

- NG-RANSAC `vislearn/ngransac@866530a287c0a788822ce147f609e9f93841a1bb`:
  README, `ngransac_train_e2e.py`, `ngransac_train_init.py`, license.
- DISK `cvlab-epfl/disk@8dc6d4d65b47210f629e21ad1fbfb56944b11696`:
  README, `train.py`, `loss/reinforce.py`, `loss/rewards.py`, `loss/pose_metric.py`,
  licenses. The attempted root `LICENSE` path returned404; `LICENSE.txt` exists.

These files were read as text, never imported, installed or executed. Relative
DISK paths above omit its top-level package directory for readability. Exact
paths, URLs, file hashes and the unsuccessful request remain in the source ledger.

## Why separate utility from covariance

Freeze the native dense flow/depth/covariance outputs. A candidate policy changes
which queries enter the same residual problem; it does not relabel arbitrary
pose-utility scores as variances or silently alter the trained covariance model.
No new bias-correction or covariance head is selected.

Freezing the covariance outputs does **not** guarantee that calibration on the
selected population remains unchanged. Selection changes the population and
conditioning. Any calibration claim must distinguish full eligible-pool and
selected-subset behavior on a qualified target. Lower task loss alone does not
prove the weights or readout represent a physical noise distribution.

For pose estimator A(M,S), dense fields M, query set S and reference T*, a generic
training target is expected pose loss over the policy's query sets, with explicit
failure handling. This is the existing policy-learning idea applied to the
present controlled interface, not new mathematics. Translation/rotation units,
query budget, policy parameterization, gradient estimator, training budget and
failure cost must be bound before training; none is chosen from the six exposed
scene outcomes in this intake.

## Required finite comparison

Use the same eligible candidate pool, dense fields, query budget, admission,
initialization and solver for the matched comparisons. Keep the variable-count
native policy as additional context rather than pretending its cost is matched.

| Arm | What changes | Question it answers |
|---|---|---|
| Native unchanged | Nothing; original variable-count policy | Application context and availability |
| Uniform, matched budget | Query identities only | Sampling/control floor |
| Standard information, matched budget | Established geometric subset rule | Whether geometry alone explains benefit |
| Fixed-query task-weight control | Pose-trained residual utility weights, fixed query rule | Whether learning weighting is sufficient; related local work already exists |
| Match-reward query policy | Same policy capacity, correspondence-oriented reward | Whether correspondence quality is a sufficient training target; DISK-inspired, not a DISK reproduction |
| Pose-reward query policy | Same capacity/budget, pose-task reward | The selected baseline/adaptation hypothesis; NG-RANSAC-inspired, not its full RANSAC system |

Only the policy or task-weight adapter is trained. Do not jointly change the
frontend means, covariance head, matcher, optimizer, robust kernel or calibration.
Task weights in the control are utility weights, not newly certified Gaussian
measurement covariance. Actual training/inference calls and preprocessing costs
must be reported, not only the number of selected points.

The pose-reward policy must be compared with both the match-reward policy and
the identically budgeted task-weight control, not just native or random selection.
Report per-scene/fold translation and rotation error, availability and conditioning,
with every attempted failure and matched-support denominator. A pooled-only
accuracy gain or lower jitter alone is insufficient to advance a method claim.
Exact thresholds and stopping rules belong in the later numerical protocol.

## Existing work and data prevent false novelty or leakage

The local copy's legacy `Train/MatchingNet/train_flowformer.py` calls dense
`sequence_loss`; its `loss.py` has flow/covariance objectives. Local optional
covariance/trace-risk contracts and training runners are separate existing work.
This source reading does not establish the historical MACT checkpoint's complete
training provenance or identify an author-code error. Do not “fix” legacy losses
as part of the selected policy comparison.

Existing final-layer NLL/trace-risk training and held-record development evaluation
already test dense score objectives. The completed six-parameter shared-weight
study already tests pose-trained RGB/joint weights on fixed graphs. Those are
not untested ideas. They do not evaluate a policy that changes the admitted query
set under the new dense replay contract, but must remain comparison context.

The six exact-pose scenes are already exposed, with one fixed motion and just two
geometries. They are excellent renderer/solver and effect checks; they are not an
independent training/evaluation corpus. Splitting their128noise draws, or fitting
a policy to the observed winning subset, would not demonstrate transfer.

A practical next data contract can use the qualified renderer and the60already
available thermal source images, without a new acquisition campaign. It must
include multiple motions and geometries, group adjacent textures by original
recording, and separate evaluation motion/geometry combinations from training.
All such data remain development-exposed artificial scenes; do not call them a
fresh real-world test or claim physical thermal calibration. Genuine natural
transfer requires a separately qualified real evaluation, not a relabeling of the
existing supplied-geometry proxy.

## Concrete next deliverable and disposition

Status: **HYPOTHESIS_SELECTED; DATA_AND_POLICY_DESIGN_REQUIRED**.

Next prepare one finite corpus/split and adapter specification: exact texture IDs,
training versus evaluation motion/geometry combinations, candidate-pool/budget
rules, allowed features/parameter count, translation/rotation loss units, fixed
training budget and failure handling. Qualify the policy-gradient/subset interface
with known small cases before a separate training freeze. The prepared design
must make pose-reward, match-reward and weight-only controls comparable.

No model training, new scene rendering or accuracy sweep was performed in this
review. Data/policy design and gradient qualification are the next work; the
completed numerical assessments remain completed. No head/geometry/seed tuning
follows from their outcomes.
If the finite comparison does not distinguish the policy hypothesis from the
strong controls, close that exact recipe and retain the result. Do not declare
thermal learning, covariance modeling or the other original-paper gaps closed.
