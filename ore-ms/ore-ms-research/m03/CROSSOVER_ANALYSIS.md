# Thermal crossover analysis

In the visible unsaturated sweep, target emissivity is 0.5, reflected effective temperature is 280 K, and the background is a 300 K blackbody. The sampled target contrast changes from −0.37481 W·m⁻²·sr⁻¹ at 316 K to +1.25960 W·m⁻²·sr⁻¹ at 320 K. Linear interpolation estimates zero at **316.917 K** inside that bracket. The detector threshold is 0.33941 W·m⁻²·sr⁻¹. Neither bracketing sample meets it, so the coarse scan brackets the crossover without directly sampling the low-contrast region. The estimate depends on the response, emissivity, reflection and path in this fixture.

A separate visible 300 K blackbody target and background match exactly in band. An isothermal common path at τ=0 also removes target/background contrast while retaining the geometric visibility flags. A supplied hidden-target control has an emitting foreground matching the background. That row retains foreground-to-background difference and leaves target contrast empty. Equal detector outputs alone cannot distinguish these mechanisms.

Across the 320 K target emissivity sweep, contrast against the fixed background changes from −4.8060 at ε=0.2 to +2.7441 at ε=0.5. With fixed ε=0.5, the two explicit reflected spectra give −9.1421 and +3.4954. Therefore even the sign of apparent contrast depends on more than target temperature. Two saturation arms lose digital contrast even though their incoming band difference remains +10.2943. The raw reports preserve those cases and the zero-response control.

All comparisons are synthetic forward-model results. The [metrics](evidence/metrics.csv) and [verification log](evidence/verification.log) retain the individual observations, radiance values and calculated bracket.
