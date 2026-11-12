# Fixed MS² depth-structure transfer

12 November 2026. Hypothesis: the reproduced original outdoor monocular thermal
pipeline improves depth structure against the original MACT stereo pipeline on
the same exposed day/night/rain references. This is not a metric-scale or VO test.

## Source input gate

Pinned official MS² source `0bb0fc1a544d8ad58c7fda5bc025cb0cbb400284`:
`utils/utils.py` explicitly names FLIR A65C default coefficients. Its demo applies
`Raw2Celsius` to the loader's native float codes; the loader decodes reference
depth as PNG/256. Both conversion function ASTs exactly match the ViViD author
source and the SupDepth4Thermal utility at `f6725678f9b5dba669ea1f0739506a613f2fb2fb`.
Eighteen forward/inverse checks pass. This supports a nominal source benchmark
convention, not independent calibration of the actual recorded temperatures.
SupDepth's own utility warns camera coefficients may differ. Its newer input
normalization is not substituted for the supplied ViViD checkpoint's recipe.

**Known before inference:** with the unchanged author outdoor 0–30°C nominal
raw-code clip, the median native-code clipping fractions are 99.7571% day,
93.5474% night and 99.6664% rain. These are source-code-domain statistics, not
certified temperature measurements or actual post-resize tensor statistics.
No frame is removed, and the range is not tuned in response. This comparison
therefore tests the exact provided default pipeline; failure does not reject
thermal depth learning or a separately justified input-adapted model.

## Frozen population and models

First 20 original frames, IDs000000–000019, of the same exposed recordings:
`_2021-08-06-11-23-45` (day), `_2021-08-13-21-18-04` (night),
`_2021-08-06-16-19-00` (rain). All180 own-camera left/right/depth inputs match
the verified prior stereo-adaptation packet. That packet's479 regular members
and archive hash were checked; no training artifact or fitted weights are used.

Candidate: author outdoor depth checkpoint epoch139, SHA256
`7bd81d3d3f844b648d90db197dfbeac2bcbce5b8fa63b26bb5da123c37e98b86`.
Unchanged author loader resizes raw thermal to256×320 and applies its fixed
normalization. Resample predicted inverse depth to the native640×256 grid with
fixed1e-6 epsilons, matching the prior author depth evaluation convention.
The aspect change is explicit; there is no alternate size/crop trial.

Control: original `MACT_Offroad.pth` with the existing named percentile/CLAHE/
bilateral input replay and native FP32 FlowFormerCovFrontend,12 recurrent
steps and `enforce_positive_disparity=False`. Preserve the native depth formula.
No optimizer, finetuning, covariance fit, checkpoint selection or architecture
change. This compares complete pretrained pipelines, not architecture alone.
Use the existing exact isolated runtime `fd7be8a3`, no package update.

## Shared support, scale and endpoints

Reference support is finite provided thermal-left depth in(0.001,80)m. Score
both models on its identical intersection with finite positive predictions
from both. Record every own/common pixel count and all attempted frames.
Candidate must preserve every reference-supported baseline-valid pixel; missing
candidate predictions cannot improve scores by removing difficult points.
No stereo-only correspondence/FOV/texture or predicted confidence mask is added.

Independently median-align each model on exactly that common support per image.
Primary endpoints: equal-frame mean AbsRel and equal-frame mean per-frame RMSE,
separately per recording. Both must be strictly lower than scaled MACT, all20
frames must score, and candidate support cannot regress. Every recording must
pass; pooled wins or one favorable condition cannot rescue a failure. Original
unscaled MACT errors on the same support are retained separately for context,
not compared to unscaled monocular outputs or silently substituted as primary.

Lock all60 two-model prediction packets before scoring. On any gate failure:
`CLOSE_THIS_CHECKPOINT_TRANSFER_CONFIGURATION_NO_RETUNING`. On all three passes:
only advance to a separate metric-scale/VO contract. Neither outcome establishes
full motion accuracy, physical temperature calibration, independent reference
truth, unknown upstream training exclusions or fresh-record generalization.
All source/input/model/plan pins are checked before and after execution.

## Verification

Six synthetic contract tests cover scale invariance, common masks, invalid
predictions and coverage costs, missing support, strict improvement, complete
frame populations and requiring both error endpoints. An independent saved-output
audit will reconstruct inverse-width interpolation, scaling, masks, counts,
metrics and all gates with fixed rtol/atol1e-9. Original input-range findings and
all failures remain visible; no post-outcome preprocessing rescue follows.
