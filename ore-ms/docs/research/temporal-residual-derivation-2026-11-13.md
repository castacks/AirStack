# Cross-time covariance for the saved conditional-ray solver

13 November2026, America/New_York. Derivation and source review; not a new
nonlinear estimator or a physical covariance qualification.

## The residual being modeled

Recovered `windowed_stereo_geometry.prepare` fixes the source ray using the
measured seed pixel z0 and initializes log inverse depth alpha from source
stereo disparity. In variable-depth arms alpha is optimized, with the original
source disparity retained as an observation. At frame t the unwhitened factor is

    r_t = [project_uv(T_t, ray(z0), alpha) - z_t,
           project_disparity(T_t, ray(z0), alpha) - d_t].

The original implementation whitens each3-vector separately with block-diagonal
UV/disparity covariance and omits correlations across time. At root frame0,
projected UV equals z0 exactly for every alpha: both UV residual and its state
Jacobian vanish. Those rows are not independent noisy constraints. Root disparity
still constrains alpha. Do not retain root UV with a singular covariance or
count it as extra evidence.

## Accumulated position model

For a first-order track-error model

    e_t = F_t e_(t-1) + w_t,    F_t = I + J_flow,t,
    Cov(e_0)=C0, Cov(w_t)=Q_t,

assume for this bounded model that increments are mutually independent and
independent of e0. Define Phi_(t,s)=F_t ... F_(s+1), Phi_(s,s)=I. Then

    C_ts = Phi_(t,0) C0 Phi_(s,0)^T
           + sum_(j=1..min(t,s)) Phi_(t,j) Q_j Phi_(s,j)^T.

This gives C_tt=F_t C_(t-1,t-1) F_t^T+Q_t, the recorder's current marginal
recursion. Independent increments do NOT imply independent accumulated pixels.
Additional correlations among w or with e0 require terms not present in this
model; the archive does not identify them.

## Two distinct source treatments

**Exact/conditioned source:** declare e0=0. The innovation covariance is

    S_ts = sum_(j=1..min(t,s)) Phi_(t,j) Q_j Phi_(s,j)^T.

For target UV residuals r_t=-e_t, use S_ts for t,s>0 and omit root UV. These
marginals differ from saved C_tt by the propagated source covariance. Thus
adding off-diagonal blocks to the existing marginals is not an exact source-
conditioned model. It also changes marginal weighting, which needs a control.

**Uncertain measured source ray:** at a fixed linearization, let B_t be the
Jacobian of predicted3-vector with respect to z0, holding alpha fixed. Write
L_t=[I2;0] (3x2) and k=[0,0,1]^T. For independent stereo errors eta_t,

    delta r_t = B_t e0 - L_t e_t - k eta_t,
    A_t = B_t - L_t Phi_(t,0),
    R_ts = A_t C0 A_s^T + L_t S_ts L_s^T
           + 1_(t=s) k Var(eta_t) k^T.

This includes predicted-disparity sensitivity to source-ray error; it is not
simply a UV-only cross-block extension. At root, B0=L0, Phi=I and S=0, so the
source UV cancels. Remove root UV and retain root disparity. Off-root UV and
disparity can correlate through the shared source even if primitive stereo
noise is declared independent.

An image pixel/depth Gaussian model is not automatically the same as this
conditional-ray model. Here alpha is an estimated parameter, not another
independent random measurement injected into R. Source disparity appears once
as its factor; do not add its variance again as a depth prior while retaining
that factor. A fixed-depth arm using disparity-derived alpha requires a separate
noise treatment, including its dependence on the same source measurement.
This derivation therefore targets the VARIABLE-depth arm only.

B_t follows from the existing camera/body transforms and pinhole derivative:

    B_t = D_projection(q_t) R_camera R_body,t R_source^T
          depth * [[1/fx0,0],[0,1/fy0],[0,0]].

The3-row projection derivative includes stereo disparity derivative. At identity
motion and unchanged intrinsics, the UV part of B_t is I and its disparity row
is zero. Away from that state the full expression is needed.

## Sufficiency and unresolved assumptions

Window files retain z_t, marginal C_tt, disparities/variances and valid track IDs.
Witnesses retain F_t-I, covariance-interpolation corners, source UV, active IDs
and accepted masks, permitting reconstruction of Q_t and track alignment. K and
baseline are in native tensor files; source calibration supplies rig transforms.
No ground-truth poses are needed to form this model. Nominal camera grouping and
rig assumptions remain those of the preceding benchmark.

These fields suffice for a declared independent-increment approximation and
its source-conditioned or source-uncertain residual covariance. They do NOT
establish statistical calibration of C0/Q, real temporal noise correlations,
UV/stereo error independence, cross-track independence or cross-camera independence.
The flow covariance is a conditional prediction-error head, not certified
additive process noise. The mathematical interface can be built without claiming
the process model is physically correct.

## Integration contract for any later comparison

1. Keep observations/IDs/support and geometry fixed. Start with variable depth.
2. Compare old marginal-independent weights, conditioned marginal-independent
   weights, conditioned joint weights, and a specified scale control. This
   separates source conditioning from cross-time dependence. An uncertain-root
   model is a separate comparison, not a silent replacement.
3. Whiten each complete track residual vector AND its pose/depth Jacobians by
   the same joint Cholesky. Assemble normal equations after whitening. Track
   depth remains one scalar, so within-track correlation preserves separability
   of depth variables across tracks under the declared independence assumption.
4. The source-uncertain R depends on state through B_t. If frozen at a declared
   initial state, label it fixed linearization and use that same R throughout.
   If recomputed, the objective and derivatives, including covariance dependence
   and any likelihood log determinant, require separate qualification. Reusing
   the old fixed-weight Jacobian/line search is not sufficient.
5. No new floor, clipping, rejected track or factor fallback without predeclaring
   its rule and retaining all failures. Root UV removal is deterministic,
   mathematically redundant-row removal, not outcome-based support selection.

The fixed synthetic assay established known-model linear mechanics only. The
present derivation does not estimate covariance from empirical errors, execute
a real solver, justify training or show that dependence caused a pose regression.
