# SmokeSeer synthetic relative-pose protocol

15 November2026. Prospective development diagnostic, not a reproduction of
SmokeSeer, MAC-VO or MACThermal. No new method or training. Parent camera audit:
[smoke/darkness assessment](smoke-darkness-readiness-2026-11-15.md).

## Question and population

Hypothesis: the same fixed monocular estimator loses pose support/accuracy on
smoky RGB relative to its supplied clean RGB counterpart; thermal may retain
support. This establishes an exposed synthetic failure only. It does not establish
real-smoke transfer, physical radiometry, stereo-VO accuracy or novelty.

Use all ten released train splits in camera JSON order: Caterpillar,Ficus,Mars,
Room,Musclecar,Hotdog,Forest,Chair,Lego,House. No learning occurs; “train” names
source folders only. Every adjacent pair is declared in each modality; no wrap,
subsampling or test-view substitution. Expected1165frames/1155pairs per arm:
smoky RGB, supplied `_gt` clean RGB and thermal. All3495images must qualify.
Thermal uses its own poses. Nine scenes permit a paired thermal/RGB comparison;
Caterpillar remains reported separately because its camera poses differ.

Clean RGB uses the smoky RGB pose metadata, consistent with the author's `_gt`
loader convention. The release supplies no separate clean camera file; exact
render provenance is an assumption, not independently verified sensor evidence.
Do not interpret differing modalities alone as the causal effect of smoke.

## Input and geometry gates

Bind the complete ZIP central directory to the prior range index; verify selected
members' ZIP CRC32/SHA256, every PNG chunk CRC and full OpenCV decode. Check all
camera JSON hashes, proper homogeneous poses and file bindings. Require strictly
increasing animation time and consecutive unique filename ordinals in train
order. Animation time is not seconds. Keep all zero-motion flags in the manifest.
Any missing, corrupt, inconsistent-size or nonmonotone input stops predictions;
retain failed records. Whole-archive SHA256 identifies downloaded bytes, without
claiming author-published authenticity. Retain full bit depth before normalization.

Use the pinned author's C2W convention: C=C_blender diag(1,-1,-1,1).
The source-to-next optical transform is inverse(C_next) C_source. Derive
f=width/(2 tan(fovx/2)), fx=fy=f, cx=width/2,cy=height/2; no distortion or resize.
These centered-pinhole assumptions follow the loader; no fitted calibration.
Check inverse composition and SO3 for every pair. A translation norm<=1e-9 in
render units is direction-undefined; rotation still has a reference. No metres,
absolute-scale trajectory errors or real-time/frame-rate claims.

Full-depth pixels normalize by the unsigned dtype maximum. Composite alpha onto
black, convert RGB to grayscale with coefficients0.299,0.587,0.114, round/clip to
uint8 once for SIFT. Grayscale thermal uses the same fixed normalization. No
CLAHE, percentile stretch, per-scene tuning or ground-truth-based image handling.
Record original dtype, channel count, alpha range and grayscale hash per image.

## Fixed estimator and controls, after preflight freeze

Classical reference: OpenCV SIFT(nfeatures=4096,nOctaveLayers=3,
contrastThreshold=0.04,edgeThreshold=10,sigma=1.6), explicit precise-upscale=false;
CPU one thread, RNGseed0reset per pair. BF L2 two-neighbor matching, ratio<0.75
in each direction, retain mutual matches. Same pipeline independently in all arms.
At least8matches required; estimate E with findEssentialMat(RANSAC,prob=0.999,
threshold=1pixel,maxIters=1000), then recoverPose with its inlier mask. Require a
single3×3E and>=8cheirality inliers; otherwise record unsupported with reason.
No retry, alternative estimator, parameter sweep, fused rescue or extra models.
Pin actual OpenCV/runtime and runner bytes before execution; API qualification may
stop this planned configuration but cannot silently change these choices.

Primary endpoint: per-scene fraction of all declared pairs with recovered pose
and rotation error<=5degrees. Unsupported pairs count as failures. Report raw
support separately, geodesic rotation error, and directed translation error
acos(clipped dot(t_est,t_ref)) on reference-defined translations. No absolute dot
sign forgiveness. Also report joint rotation<=5degrees/translation<=15degrees
success over all reference-defined pairs. Keep unsupported pairs in denominators;
conditional medians are secondary and must show support counts.

Identity-rotation baseline scores all declared pairs. It reveals whether tiny
camera motion makes the primary threshold trivial; do not claim improvement over
this baseline without its comparison. Report reference rotation/translation
magnitude distributions before predictions. No significance claims from treating
adjacent pairs as independent recordings; report scene macro and pooled counts.

## Stop and interpretation

First complete the whole-population preflight and freeze its manifest, decoded
input hashes, this protocol, helper/runner and runtime. Prediction execution is a
separate recorded stage. Any gate failure stops all arms; no successful-prefix
substitution. After one execution, retain every scene and unsupported output.
If clean RGB is also unsupported, diagnose this estimator/dataset combination
rather than assigning its failure to smoke. If identity rotation dominates,
rotation-threshold success does not establish useful odometry. No subsequent
tuning or new model follows automatically from this diagnostic.

Primary sources: [SmokeSeer paper](https://arxiv.org/html/2509.17329v1),
[pinned loader](https://github.com/cmu-ci-lab/SmokeSeer/blob/3fb5c0b45c9446343d62e560e6cab6b536171927/scene/dataset_readers.py),
[OpenCV geometry API](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)
and [SIFT API](https://docs.opencv.org/4.13.0/d7/d60/classcv_1_1SIFT.html).
This protocol chooses a fixed classical diagnostic; it is not the paper's
reconstruction evaluation or a state-of-the-art pose-baseline comparison.
