# Posthoc recorded-interval diagnostic, EXP114

Declared after observing the frozen persistence/error results: several adjacent covariance correlations are negative and lag10 correlations positive. This suggests checking sampling before choosing a temporal model. This is exploratory development analysis, not an independent validation or replacement for the fixed comparisons.

For all nine windows and both sensors, report native recorded interval quantiles and the association between grid angularized flow-covariance trace and its interval. Compare its predeclared lag1/3/10 persistence to trace/dt², the elementary covariance rescaling for a flow-derived angular rate. Also report association with unchanged future pose error and with future pose-error magnitude divided by the target common RGB interval, with exact valid counts. No exponent/lag fitting, window selection, policy, model training, threshold change or candidate promotion. Pair indices and missing targets remain unchanged.

The recorded clocks have not been validated as physical exposure times. A rate normalization that fails here does not prove sensor dynamics cause alternation, and an improvement does not establish a useful online predictor. Conditioning on motion, exposure, scene or model behavior would remain separate hypotheses.
