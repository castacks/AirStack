# Predicting dependence: recording-excluded development contract

15 November2026UTC, `ore-ms-orb-development`, baseline `a79658f63` plus frozen
working sources. This design precedes parameter fitting and new evaluation scores.

The [cross-match attribution](cross-match-covariance-findings-2026-11-15.md)
shows consequential dependence under one declared half-LSB input ensemble. It
does not show that a learned covariance predictor transfers. Select one finite
baseline: four common image-error modes propagated through the fixed geometric
estimator, compared with the same modes treated independently across matches.
Generic correlated-noise propagation and learned full covariance are established
by Censi/Bonnabel and [Deeper-DICE](https://arxiv.org/html/2007.14943v1), respectively.
This is an adaptation hypothesis, not a new statistical principle or author-system
reproduction. Fresh targeted Deeper-DICE II-C/III confirms the Gaussian likelihood,
mean/error distinction and full covariance parameterizations; no author code run.

For baseline residual r_i=R X1_i+t-X0_i, use four additive pixel/disparity modes:
constant temporal flowu,v and source/target positive disparity magnitudes.
Their local residual derivatives are

    B_i = [R e_x*z1/fx, R e_y*z1/fy, X0/d0, -R X1/d1].

Depths/lookup positions are otherwise frozen. This does not model the full
network derivative, changing feature identities or physical sensor noise.
Let D_ij=G_i B_ij with G_i the previously qualified baseline pose influence.
For modej, the common covariance basis is(sum_iD_ij)(sum_iD_ij)^T;
the independent-block basis is sum_iD_ijD_ij^T. Each family predicts

    C = a P_native + sum_j q_j M_j, a>0,q_j>0.

Both have five positive coefficients. Use one L-BFGS-B fit,200iterations maximum,
analytic likelihood gradient, log-coefficient bounds[−30,10], initiala equal to
the train-only scalar MLE and allq=1e−4pixel². No restarts, mode/rank/noise/optimizer
search or covariance-floor rescue. Bounds are model constraints, not measured
physical variance floors. Record all optimizer statuses/bound hits.

Leave out one whole texture recording(day/night/rain) and both its geometries;
train on the four scenes from the other recordings. Three folds use128saved
trials per scene. All scenes are already exposed development inputs, including
the preceding attribution; this is not a blind test, physical sensor transfer
or network-training-held-out experiment. No new neural/pose fits occur.

Controls are unchangednativeP, train-only scalarP, five-parameter independent
modes, five-parameter common modes and a train-pooled full native-whitened shape
(21independent entries, analytic Gaussian second moment). A test-fitted scalar
is a separately labeled **oracle diagnostic**, never an available predictor.
It helps distinguish shape effects from test-specific aggregate variance.

The target is leftLog(T_draw inverse(T_baseline)), with fixed zero mean for all
models. Do not center evaluation labels to fit a mean or call this true-pose
error. Primary score is scene-macro Gaussian NLL in the fixed native-whitened
tangent (shared6log2pi omitted). Also retain95%baseline-relative containment,
mean shift, centeredT/Rspread, full moments, prediction availability and every
training status. Covariance-only prediction changes no pose or accuracy output.

Baseline-only feature construction never uses perturbed residuals or empirical
covariances. All three fold fits and30predictive covariance matrices must lock
before evaluation scoring. Training uses only its four included scene labels;
held-out labels enter only the scorer. The oracle scalar is created there.
No pooled-only success criterion or post-result improvement threshold is allowed.

Before fitting,117checks qualify analytic mode derivatives against finite
pixel/disparity changes, Gaussian likelihood gradients against finite differences,
common-mode covariance identities and recording-disjoint splits. Next bind those
receipts/features/sources, perform the single fit-and-score experiment, and audit
from original pose receipts and baseline packets. No result is asserted here.
