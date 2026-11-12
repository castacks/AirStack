# ThermalSfMLearner-MS: reproduction intake

12 November 2026. **NOT_REPRODUCED_ASSETS_REQUIRED.** User approved checking a
published thermal-specific baseline before another local intervention. Working
reference: `thermal-sfm` in [the living library](references/README.md).

## Pinned sources

- [Paper, arXiv 2103.00760v3](https://arxiv.org/html/2103.00760v3): inspected
  thermal normalization, cross-spectral reconstruction and experimental setup.
  It learns depth and pose using thermal reconstruction plus RGB supervision,
  with thermal-only inference. ViViD day training and night evaluation are
  relevant to transfer; they are not our MS² recordings or a new holdout here.
- [Author source](https://github.com/UkcheolShin/ThermalSfMLearner-MS/tree/d2ab0fc5e92972e248b641572f4b595dd67af83d):
  shallow clone pinned at `d2ab0fc5e92972e248b641572f4b595dd67af83d`. Inspected
  README, environment, preparation/test scripts, thermal transforms and metrics.
  Source hashes are in the intake JSON. The clone stays outside AirStack.

## Availability and compatibility

The README routes processed ViViD data and pretrained networks through a
survey followed by an emailed download link. The checked source tree contains
no checkpoint binaries; the official GitHub releases API returns zero releases.
This scoped observation does not prove assets are unavailable elsewhere. A
bounded Box root/depth-two filename search found no ViViD/ThermalSfM match.
The author link is requested from the user; no survey or message was submitted.

Source scripts name separate depth and pose checkpoints for indoor/outdoor
ResNet18 models. The outdoor evaluator selects `outdoor_robust_night1` and
`outdoor_robust_night2`. Its documented environment is Python3.7/Torch1.5.1/CUDA10.2.
Compatibility with our Blackwell runtime is unverified; no installation or
replacement of the recovered MAC environment was attempted.

Source inspection, not reproduction: `test_pose.py` fits translation scale from
reference snippets. `eval_vivid/eval_depth.py` has a monocular median-scaling
path. These require explicit labels in any score table; neither is automatically
comparable to the unscaled metric stereo-VO errors reported in our experiments.
Thermal loaders use scene-specific temperature clipping via the supplied
`Celsius2Raw` camera constants. Our uint16 MS² storage type alone does not certify
that conversion. Reusing those constants on MS² without evidence would alter
the experiment's input meaning.

## Finite next execution

On receipt of author assets, verify file hashes, model variants, training/split
metadata and preprocessing against the pinned source. First target one
published outdoor ViViD evaluation with the original provided protocol. Record
any runtime-port changes separately; a modern-runtime port is not an exact
historical-runtime reproduction. Freeze inputs, checkpoints, evaluation settings
and numerical acceptance tolerance before inference. Confirm scale alignment
and sequence membership before comparing results.

Only after source-protocol reproduction can an MS² transfer experiment be
specified. It needs an evidenced raw-value conversion, explicit monocular scale
policy, source-training overlap review and recording-wise accuracy/coverage
controls. Do not feed our existing percentile/CLAHE inputs silently into the
author thermal loader, start training as an asset substitute, or compare a
reference-scaled pose result against an unscaled stereo metric result.

No model inference/training/data conversion/accuracy scoring occurred in this
intake. Previous raw-refinement and rotation configurations remain closed.
