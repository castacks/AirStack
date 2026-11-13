# Relevance and prior-art boundaries for RGB–thermal covariance research

12 November2026, America/New_York. Targeted source/inventory check answering the
user's question, not a systematic novelty review or new experiment declaration.

The original question is cross-modal comparability and predictive geometric
reliability of RGB/thermal uncertainty, with downstream VO usefulness. Feature
weighting, pose covariance and fusion ablations are directly relevant components,
but relevance does not establish novelty.

## Existing overlap

- MACThermal thesis3.4 already uses learned thermal uncertainty for selection and
  covariance-weighted two-frame pose factors. Applying thermal covariance in a
  pose graph is already within this application.
- MAC-I² already derives pose covariance from feature uncertainty and weights
  vision/IMU constraints; its covariance ablation is not RGB–thermal evidence.
- AirLab's own off-road project description identifies uncertainty-driven RGB/
  thermal multispectral-inertial odometry. Shibo Zhao's institutional defense
  abstract explicitly describes per-feature covariance from photometric Hessians
  across RGB and thermal, used to weight factor-graph constraints. This is direct
  application overlap. Primary project/defense summaries read, not a full MSO
  paper/code/ablation inspection. No publication-status/novelty-gap claim follows.
- Unsupervised Balanced Covariance Learning for Visual-Inertial Sensor Fusion,
  RA-L2021, DOI10.1109/LRA.2021.3051571, explicitly addresses covariance balance
  across sensors in its publisher abstract. Abstract-level inspection only;
  full IEEE article access failed. This prevents treating generic cross-sensor
  covariance balancing as automatically new, but does not settle the exact
  RGB–thermal calibration question.

Primary sources:
https://publications.ri.cmu.edu/storage/publications/2025/12/YifeiLiu_CMU_MSR_Thesis_ThermalOffroadAutonomy.pdf
https://arxiv.org/html/2609.07116
https://theairlab.org/offroad/
https://www.ri.cmu.edu/event/unlock-robust-spatial-perception-towards-resilient-state-estimation-and-mapping-for-long-term-autonomy/
https://ieeexplore.ieee.org/abstract/document/9324985

## Correction to earlier conversational advice

A basic fixed-input covariance isolation was already in this repository:
`experiments/EXP-009-weight-ablation.md` freezes matches/depths/initialization and
changes covariance settings, finding translation/rotation tradeoffs. It is not
an untouched new research direction or a fresh end-to-end thermal-VO trial.
The recovered nominal benchmark also already contains `thermal_fixed` and
`thermal_cov` arms. On its99 common night intervals, covariance lowers translation
mean0.0594937→0.0573940m and rotation mean0.0796509→0.0776421deg, while translation
RMSE0.088036→0.088070m and rotation RMSE0.125399→0.125404deg slightly worsen.
These are conditional saved-source-landmark solver results under nominal supplied
pairing, not consistent full-system nighttime benefit or a new general result.
Earlier advice that these isolations had simply not been tested was too broad.

## Potential contribution boundary

A study of whether RGB/thermal covariances predict the same well-defined pose or
measurement-error target under changing conditions is relevant to the original
question. Isolating the stages makes the interpretation defensible. However,
a missing ablation in one thesis, use of another dataset or a basic covariance
weighting implementation does not establish research novelty. To claim a new
contribution, identify the exact quantity/assumption/failure already addressed
by MSO and related calibration work, then demonstrate an additional result or
method against their appropriate baseline. That gap has not been verified here.
Negative local fusion recipes do not settle the general question and should not
be substituted for this prior-art comparison. No further trial/training is selected.
