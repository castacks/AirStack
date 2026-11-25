# Joint degradation analysis

The primary analysis samples exactly one 6 s center ray for each of 128 independent stochastic seeds on fixed target geometry at 10 m. It does not pool multiple timestamps as independent draws. Since both coefficients at 550 nm and 11 µm are affine functions of the same longitudinal burden, their log transmission satisfies

`log(T_LWIR)=a log(T_RGB)+(a k_RGB,floor−k_LWIR,floor) s`

where a is the ratio of LWIR to RGB burden slopes and s is slant range. Maximum absolute residual was 1.04e−16. The relation concerns reference monochromatic transmission, rather than band radiances or detector codes.

Raw transmission Pearson r was **0.99222**, with descriptive Fisher 95% interval [0.98897, 0.99451]. Sample covariance was 0.00526657. A cyclic re-pairing of LWIR with a different seed reduced r to −0.06276. This comparator illustrates dependence on the realization pairing. The Fisher approximation is descriptive because the two transformed fields are not bivariate normal and the physical coefficients are constrained. Positive correlation is expected by construction and neither establishes causal identification nor implies recovery of hidden state.

| Control | RGB transmission at 550 nm | LWIR transmission at 11 µm |
|---|---:|---:|
| Mean below burden 0.5, 52 realizations | 0.62037 [0.59302, 0.64773] | 0.90366 [0.89689, 0.91043] |
| Mean at or above burden 0.5, 76 realizations | 0.40002 [0.38795, 0.41209] | 0.83619 [0.83159, 0.84079] |
| Matched environment, 5 m foreground minus 10 m target | +0.20281 [0.19389, 0.21174] | +0.06504 [0.06156, 0.06852] |
| Fixed geometry, within-realization 12 s minus 0 s | −0.01781 [−0.05088, 0.01525] | −0.00537 [−0.01551, 0.00477] |

Intervals are pointwise approximate normal intervals over realization-level contributions. A shorter path increased both transmissions for every seed **while hiding the target**. Geometry-induced parallel changes can therefore coexist with weaker target visibility. Neither temporal-change interval excludes zero, as compatible with the stationary ensemble design. The forced advected example separately demonstrates a moving coefficient field. The [complete estimates](evidence/statistics.csv) also retain a one-point OU marginal check, with mean 0.17047 [−0.00068, 0.34161] against zero and second moment 0.99740 [0.74543, 1.24937] against one.
