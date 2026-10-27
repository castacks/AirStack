# Targeted uncertainty-method reading — 2026-10-27

Context: EXP059–061 show that within-camera ranking, metric camera selection, calibration and sensor visibility are different questions. User offers access to papers. Public primary-source versions suffice for this bounded reading; no PDF upload is needed now. This is targeted methodological reading, not a systematic literature review or evidence that any cited method works on our RGB–thermal task.

## Regression calibration and useful discrimination

Dan Levi, Liran Gispan, Niv Giladi and Ethan Fetaya, *Evaluating and Calibrating Uncertainty Prediction in Regression Tasks*. Read the [arXiv v3 text](https://arxiv.org/html/1905.11659), especially Sections2–3.1; the [2022 Sensors journal version](https://doi.org/10.3390/s22155540) is a separate version, not asserted text-identical. The authors show that marginal interval calibration can coexist with uninformative uncertainty. They compare binned predicted uncertainty with empirical error and supplement calibration with uncertainty dispersion. Their scaling calibration minimizes NLL on a separate recalibration set.

Our implication: report calibration and selection/discrimination separately. EXP060 minimizes selection risk, so its fitted multiplier is not an implementation or test of their NLL calibration method. Our rank-one 3D score also cannot silently be treated as a full Gaussian covariance in an NLL. Their experiments do not establish RGB–thermal transfer.

## Stereo error distributions

Liyan Chen, Weihan Wang and Philippos Mordohai, *Learning the Distribution of Errors in Stereo Matching for Joint Disparity and Uncertainty Estimation*, CVPR2023. [Official paper page](https://openaccess.thecvf.com/content/CVPR2023/html/Chen_Learning_the_Distribution_of_Errors_in_Stereo_Matching_for_Joint_CVPR_2023_paper.html); [author preprint](https://arxiv.org/abs/2304.00152). Read the downloaded v1 preprint’s introduction, Sections3.1–3.4 and4.3–4.4; not a full reproduction or exhaustive reading of the supplement. Their SEDNet combines disparity estimation with an uncertainty head and differentiable histogram matching of uncertainty/error distributions using a KL term. The formulation uses an L1/Laplace likelihood; reported evaluations include cross-domain experiments. Their training also examines inlier filtering.

Our implication: score scale and distribution/tails deserve separate checks; predicted scale does not automatically equal useful cross-camera risk. Matching marginal distributions alone would not prove correct per-point camera choices. Their work requires training and is not a justification for trimming our evaluation labels or an immediate replacement for MAC-VO. Temporary downloaded23-page PDF SHA256 `d74bb26b57a88e39b2ff9ec457db575d2a7ec10b5d7cb7fe3e13ad9de270f703`; full paper/text is not included in our recovery artifact.

## Transfer under distribution shift

Yaniv Ovadia et al., *Can You Trust Your Model’s Uncertainty? Evaluating Predictive Uncertainty Under Dataset Shift*, NeurIPS2019. Read the [primary proceedings PDF](https://proceedings.nips.cc/paper/2019/file/8558cb408c1d76621371888657d2eb1d-Paper.pdf), introduction, temperature-scaling results and takeaways. The classification benchmark finds that calibration on an i.i.d. validation set need not survive shifted inputs.

Our implication: keep recording-level exclusion and fixed-camera baselines, and retain genuine unexamined data for a future method. Their classification findings motivate a transfer test; they neither explain our night failure nor establish that Gaussian regression calibration or thermal sensors behave identically.

## Next method question

EXP061’s conditional range and tail patterns motivate testing whether deployment-available predicted geometry/context adds relative-error information beyond uncertainty and camera-wide scale. Any future fit should first declare its target loss, model, recording split and controls. Reference range and error-tail masks used in EXP061 are diagnostic labels and must not become unacknowledged inference-time inputs. These previously inspected windows are development evidence. Independent recordings/references would strengthen evaluation more than another favorable fit on the same windows. No model training or paper-based algorithm change is made here.
