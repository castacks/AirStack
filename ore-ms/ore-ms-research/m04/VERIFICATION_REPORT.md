# M04 verification report

[Protocol and tolerances](EXPERIMENT_PLAN.md) and the scene were frozen before implementation. The separate oracle in `analytical.py` projects scene corners with rational arithmetic and checks screen-space rectangles. The forward solver instead intersects normalized rays with world planes. The independent radiometric reference uses triangle area and exact-per-interval Simpson for sampled RGB spectra, and a 3072-interval spectral Simpson reference for LWIR. Verification is procedurally separate within the same development effort.

| Criterion | Independent reference and measured result | Frozen tolerance | Status |
|---|---|---|---|
| V01 coordinate geometry | Known transform, projection and range. Finite-difference camera-coordinate Jacobian error 7.57e−11 px/m. Transform error 0 m. | 1e−10 absolute for projected and geometric values, 1e−7 px/m gradients | PASS |
| V02 depth order | Rational depth ordering. Zero mismatches in near/far and overlap arms. Reversing order changed zero labels and radiometric values. Tie probes agreed in both orders. | Exact labels and tie rule | PASS |
| V03 visibility and range | Rational projected support. Zero label or mask mismatches across 5,610 rays. Maximum target-only/first-hit range and depth error 3.55e−15 m. Empty support gave a blank fraction. | Exact support and labels, 1e−10 m maximum | PASS |
| V04 time | Rational trajectory and exact configured timestamps. Zero sample labels or frame-count mismatches. Foreground enters, crosses and leaves. Moving target denominator falls to zero. | Exact sampled counts and timestamps | PASS |
| V05 first-hit radiometry | Independent RGB difference ≤5.56e−17 W·m⁻²·sr⁻¹. Clear LWIR component error ≤1.01e−4 W·m⁻²·sr⁻¹. Foreground and target attenuated probes at their respective ranges had maximum LWIR relative error 2.08e−6. Hidden material perturbation changed zero observations. | RGB 2e−12 absolute, clear LWIR 5e−4 absolute, attenuated 2e−5 relative, hidden observations exact | PASS |
| V06 geometry and contrast | Zero geometry mismatch in matched, attenuated and detector-clamp controls. Matched foreground and target signals and codes equal on selected probes. Attenuation changed direct and path terms. Clipped probe reached code 4095 in R and LWIR. | Exact matching masks and distinct measured effects | PASS |
| V07 boundary and tolerance | Exact edge hit and just-outside miss, backward and near-origin rejection, both sides of near-parallel cutoff, and both sides of depth-tie tolerance passed. Zero failed edge probes. | Frozen closed-edge, 1e−12 direction, 1e−8 m range and 1e−9 m tie rules | PASS |
| V08 regression/replay | All three M04 CSVs and prior archived CSVs reproduced byte for byte. M01–M03 manifests, acceptance regression harnesses and catalog invariants passed. | Exact token and byte equality | PASS |
| External camera/scene validation | Synthetic fixtures and no external verification. | Separate qualified physical or simulator evidence | NOT TESTED |

Four automated M04 tests passed. Prior regressions passed for all three milestones. See [M04 verification](evidence/verification.log), [replay](evidence/replay.log), [regression log](evidence/prior_regressions.log) and [provenance](evidence/provenance.log).
