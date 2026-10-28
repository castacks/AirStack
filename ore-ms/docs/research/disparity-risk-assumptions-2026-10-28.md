# Positive disparity does not identify depth risk

Date: 2026-10-28 America/New_York. Context: EXP065–066, source commit52b53e3a0. This is our mathematical derivation and a declared assumption audit, not a fitted probability model.

## Target and unverified assumptions

Let mu>0 be the existing predicted disparity, v>0 its horizontal variance output, F=fx*B>0 and q the native pixel ray with last coordinate1. The existing point estimate is q*F/mu, transformed rigidly for thermal. Suppose hypothetically that true disparity D>0 has E[D]=mu and Var[D]=v. Neither the interpretation of the checkpoint outputs as these calibrated conditional moments nor a family for D is established.

Set c=v/mu² and G=||q||²*(F/mu)². The original first-order trace is U=G*c. We examine expected squared point error about the **unchanged** estimate, G*E[(mu/D-1)²]. This is not just transformed covariance: if E[F/D] differs from F/mu, a squared mean-shift term must be included. It also does not model ray/pose/visibility/reference uncertainty or off-ray errors against the common midpoint target.

## Two distributions with identical first moments

Distribution conventions follow the primary [SciPy1.15.3 lognormal documentation](https://docs.scipy.org/doc/scipy-1.15.3/reference/generated/scipy.stats.lognorm.html) and [Gamma documentation](https://docs.scipy.org/doc/scipy-1.15.3/reference/generated/scipy.stats.gamma.html), read2026-10-28. The reciprocal-moment formulas and implications below are our derivations, checked by independent integration; the documentation does not validate these assumptions for MAC-VO.

For lognormal D, set s²=log(1+c), log-location=log(mu)-s²/2, with location shift0. Then E[D^-1]=(1+c)/mu and E[D^-2]=(1+c)^3/mu². The assumed risk is

`R_LN = G*(c+3c²+c³) = U*(1+3c+c²)`.

The transformed variance trace is G*(c+2c²+c³), and squared mean shift is G*c². Dropping the shift would answer a different question. All reciprocal moments are finite under this assumed family.

For Gamma D, choose shape a=1/c and scale theta=mu*c, with location shift0. For a>k, E[D^-k]=theta^-k*Gamma(a-k)/Gamma(a). If c<1/2, the assumed risk is

`R_Gamma = G*c*(1+2c)/((1-c)*(1-2c))`.

For c>=1/2 it is infinite: near zero, density times D^-2 is proportional to D^(a-3), whose integral diverges when a<=2. No algebraic cancellation of separate infinite terms is permitted. This is a property of the assumed distribution, not an observed infinite reconstruction error. A positive-truncated Gaussian with nonzero density at zero also has divergent inverse-square moment. Conditioning on D>0 by itself does not solve the problem.

## Even small variance gives no upper risk bound

Normalize E[D]=1 and Var[D]=c>0. For any epsilon in(0,1), choose support epsilon and b=1+c/(1-epsilon), with probability p=c/(c+(1-epsilon)²) at epsilon. Direct substitution gives the same mean1 and variance c for every epsilon. As epsilon tends to0, p tends to c/(1+c)>0 and E[(1/D-1)²] grows without bound. Thus mean, variance and positivity alone do not determine or upper-bound this expected error, even for small c. Twelve fixed numerical fixtures confirm the construction at c=.01,.1,1 and epsilon=.1,.01,.001,1e-6; the limiting argument is analytical, not inferred from the fixture count.

## Concrete bounded comparison

EXP067 fixes both formulas, no floors or fitted parameters, before evaluating the four already examined populations from EXP066. Existing point estimates and reference-selected support remain unchanged. Compare in log-score space to avoid overflow. Gamma infinity is represented explicitly; choose the finite camera when only one is infinite, and an equal mixture when both are infinite. The latter is a declared fallback, not a uniquely justified optimum. Exact finite ties are equal mixtures. Record infinite counts, all decisions and actual supplied-reference risks.

Primary observed target is common-midpoint squared3D error, so the own-ray hypothetical risk model is being checked against a broader target. Both endpoint anchors and mean-distance loss are sensitivities. Include first-order, image-wide camera-scale, angular/pixel geometry, unpropagated angular uncertainty, both fixed cameras and equal random. No score fitting, default change, new data or prospective claim follows from this audit. Arbitrary near-zero floors would change the assumed model and must never be chosen after observing evaluation tails.

## What a next method needs

The present outputs do not identify the required distribution. Before another calibrated-risk claim, trace the original uncertainty training objective and which conditional quantity the checkpoint was trained to estimate; distinguish code-level intended semantics from checkpoint recipe/provenance, particularly thermal. A future method needs supported tail/likelihood assumptions or a declared bounded task loss, plus empirical recording-level calibration/selection checks. A physical minimum disparity or maximum range could alter the problem only if justified independently of these evaluation outcomes. Preserve the final two captures until an actual new method and evaluation are declared.
