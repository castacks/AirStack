# What the available uncertainty code and checkpoints establish

Date: 2026-10-28 America/New_York. Source audit EXP068, base b727cb67e. The important distinction is between inference representation, available training code, published method and checkpoint-specific training history.

## Finding

There is a verified representation inconsistency in the available covariance trainer. The decoder returns an unconstrained two-channel raw field. `FlowFormerCov.inference` maps it to positive values using `exp(2*raw)`, consistent with treating raw as log standard deviation. The current `cov_loss` instead uses raw+epsilon directly as a variance, without the exponential. CPU probes reproduce this difference and unused mask/maximum arguments. This finding is in the local snapshot and public upstream source; it is not established as the training procedure that produced either supplied checkpoint or as the cause of our transfer failures.

No training, neural image forward, checkpoint mutation, scientific policy/default change or held-out acquisition was performed. Previous saved inference outputs remain intact.

## Source chain

| Stage | Source evidence | Established behavior |
|---|---|---|
| Head | `Module/Network/FlowFormerCov/covhead.py:19–22` | Final convolution has2 unrestricted outputs; no final positivity activation |
| Recurrent decoder | same file:117–140 | Accumulates covariance updates and appends raw upsampled field; inherited flow upsampler includes factor8 |
| Model forward | `Module/Network/FlowFormerCov/flownet.py:17–32` | Returns raw decoder outputs |
| Inference | same file:36–44 | Unpads final field and returns exp(2*raw) |
| Our recorded stereo runner | `Scripts/Setup/ms2_manifest_stereo.py:114–136` | Calls model.inference and saves returned flow/covariance; no added scale transform |
| Training call | `Train/MatchingNet/train_flowformer.py:58,92–97` | Sets model.train, passes temporal current-left/next-left pair to forward and raw outputs to sequence_loss |
| Covariance objective | `Train/MatchingNet/loss.py:18–34` | Residual squared and detached; raw+epsilon denominator/log; mean over tensor; no mask or max_cov use |
| Loss selection | same file:59–94 | cov/finalcov pass optional mask to functions that ignore it; finalcov selects only final prediction |
| Evaluation helper | `Train/MatchingNet/evaluate.py:37–49` | Applies exp(2*raw) before its Gaussian-style score |
| Pair/GT | `DataLoader/Dataset/Train.py:19–23`, TartanAir.py:128–146, TartanAir2.py:118–136 | Adjacent frames with dataset flow/mask labels; current training entry filters TartanAir variants |

The shared optical-flow network can be used for stereo matching, as our runner does, but this current trainer is a temporal-flow training path. It does not demonstrate MS² stereo-specific covariance training. A covariance variable name, positive inference values or successful strict loading cannot verify calibrated disparity moments. The head has two diagonal channels, not a full learned2×2 matrix.

## Source-contract probes

Exact loss functions were extracted by AST with only compiler decorators removed. The exact inference method ran with identity padding and a stub forward; the trainer freeze match ran on a tiny structural module. All tensors were CPU float64; no learned network or image dataset was run. Independent formulas check the numerical outputs.

| Contract probe | Observed result | Implication |
|---|---|---|
| Residual2, raw0.5, epsilon1e-7 | Current covariance loss7.30685141944 | Uses squared error/(raw+epsilon)+log(raw+epsilon) |
| Same inputs with inference variance exp(1) | Consistent unhalved Gaussian objective2.47151776469 | Different objective from current trainer |
| Raw-variance point optimum for squared error4 | raw=3.9999999 | Inference would exponentiate this quantity |
| Log-standard-deviation point optimum for same error | raw=log2=0.69314718056 | Inference variance would equal4 |
| All-false mask and tiny max_cov | Loss unchanged | Arguments unused by cov_loss |
| Negative raw=-0.2 | Training loss NaN; inference variance0.6703200460 | Inference permits raw values that loss cannot accept |
| Detached squared residual | No direct gradient to supplied prediction leaf; covariance gradient=-3.4999985 per element | Covariance objective directly supervises scale only |
| Current cov training freeze block | Only cov_update trainable in structural stub | Separate covariance-stage intent; no historical-run proof |

The no-direct-flow-gradient statement applies to the isolated residual argument. It does not rule out gradient through shared features used to predict covariance when those features are trainable. `finalcov` has no dedicated freeze branch in the current trainer, so its structural stub leaves all parameters enabled; this is not proof of a particular historical optimization run. Flow loss and covariance loss have different mask behavior; the ignored-mask finding is specific to the latter.

For a fixed predictor and a well-defined variance parameter s, minimizing E[e²]/s+log(s) gives s=E[e²|conditioning], which includes squared conditional bias around that predictor. It need not equal the variance of true disparity about the predicted disparity unless the prediction is the appropriate conditional mean. This is our population-objective derivation, not an empirical calibration finding. Under a log-standard-deviation parameter r, the corresponding unhalved expression is e²*exp(-2r)+2r. A global factor1/2 changes neither optimum. The local raw-variance objective is not that expression. Do not reinterpret or re-exponentiate the saved checkpoint outputs to try to repair training history.

## Public method versus available implementation

The [MAC-VO paper, SectionIII-A Eq1 and AppendixB-E](https://arxiv.org/html/2409.09479v2) specifies a diagonal flow covariance trained with Gaussian-style negative log likelihood and a recurrent uncertainty decoder. Its stated training is covariance learning on TartanAir after FlowFormer initialization. These method descriptions do not establish the source revision used for the release. Paper iteration weighting0.8 also differs from the local example gamma0.85, reinforcing the need for a run-specific recipe rather than equating the example config to a published training run.

The [thermal thesis, Section3.3, PDF39–40 / printed26–27](https://publications.ri.cmu.edu/storage/publications/2025/12/YifeiLiu_CMU_MSR_Thesis_ThermalOffroadAutonomy.pdf) describes FlowFormer pipeline fine-tuning on forest TartanAirV2 and TartanDrive2.5T, followed by covariance-head training on that combined data, with a masked likelihood-style objective. It does not bind that description to the exact SHA of our `MACT_Offroad.pth`. Selected PDF39–41 were read; the85-page17,420,576-byte PDF SHA is91d680477b21d2292e81877df8ed05dfef0dbe829784857baa1581add645d6a4. Full PDF/text is excluded from backups. The equation's notation alone cannot resolve the implementation parameterization or checkpoint recipe.

Read-only upstream comparison is pinned to MAC-VO/MAC-VO commit `8683b532d6f6e8981f6b1a646fb5f52f02c1ea61`. [loss.py](https://github.com/MAC-VO/MAC-VO/blob/8683b532d6f6e8981f6b1a646fb5f52f02c1ea61/Train/MatchingNet/loss.py) and the trainer are byte-identical to our local files. The [upstream forward and inference methods](https://github.com/MAC-VO/MAC-VO/blob/8683b532d6f6e8981f6b1a646fb5f52f02c1ea61/Module/Network/FlowFormerCov/flownet.py) are AST-identical; our checkpoint loader is the deliberate local difference. Thus this inconsistency was not introduced by EXP068 or a local training-loss edit. Current upstream identity does not locate the historical source that generated released weights.

## Checkpoint provenance

| Supplied file | Verified SHA256 | Container | What follows |
|---|---|---|---|
| MACVO_FrontendCov.pth | bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a | OrderedDict434 tensor entries,484 module metadata entries containing only version fields |77,369,368 bytes, byte-identical to official RGB release |
| MACT_Offroad.pth | eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a | OrderedDict428 tensor entries,477 module metadata entries containing only version fields |76,827,690 bytes; architecture-compatible supplied thermal state, no embedded run recipe |

Both were loaded CPU-only with weights_only=True. There are no non-tensor top-level run fields such as optimizer,train config,split or training commit. Tensor shapes and standard module version metadata are not training provenance. RGB official release identity was independently checked by streaming the [official asset](https://github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_FrontendCov.pth) through SHA256 without retaining another weights file. The release API lists this asset under the2024-09-25 model release but supplies no digest; direct bytes provide the identity check. Release target `main` is not a verified training commit.

Thermal's strict active-state match and prior preprocessing/runtime audits establish execution compatibility, not exact training data, input recipe, loss or split. The [recording-owner reply record](source-replies-2026-10-22.md) also leaves thermal checkpoint training membership unknown. No author was contacted. If checkpoint-specific provenance is later available, the minimal useful record is training source commit, actual loss parameterization/masking, config and preprocessing, training/validation recording lists, and how the saved file maps to that run. Architecture names or a thesis-level description cannot replace it.

## Research decision

Keep the current checkpoint interpretation at its documented inference interface, preserve the numerical baselines, and do not describe the source inconsistency as the cause of existing failure. Do not train or patch this path until its output/loss contract is made consistent and tested in an explicitly new experiment. This audit is a finding, not a training fix.

The next bounded useful check is empirical uncertainty-scale/interval coverage on the four already examined saved disparity populations, against each supplied native endpoint. This can test what current outputs do without asserting the historical recipe or a true positive-disparity distribution. Keep signed bias, ranking, coverage, scale and camera selection distinct; fix interval conventions and recording/range controls before scores, do not recalibrate on reused labels and call it held out. Retain the last two recordings. Stronger physical or generalization claims still require independent references and checkpoint-specific membership.
