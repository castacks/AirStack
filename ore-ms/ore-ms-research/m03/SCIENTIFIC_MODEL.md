# M03 thermal radiative transfer

At vacuum wavelength λ, the blackbody spectral radiance is
Bλ(T) = 2hc²/[λ⁵(expm1(hc/(λkT)))]. The kernel evaluates λ in metres and multiplies by 10⁻⁶ to return W·m⁻²·sr⁻¹·µm⁻¹. The constants h, c and k use exact SI values. The occupation factor is evaluated as exp(−x)/(−expm1(−x)) for stability in both tails. The supported domain is 0.05–10000 µm and 100–2000 K. Numerical underflow to zero is permitted far into the short-wave tail. The detector uses only 8–14 µm. For a Lambertian blackbody, π∫Bλdλ=σT⁴. The Stefan–Boltzmann comparison spans much more than the detector band. [NIST radiance thermometry reference](https://nvlpubs.nist.gov/nistpubs/Legacy/SP/nistspecialpublication250-43.pdf) · [CODATA 2022](https://physics.nist.gov/cuu/pdf/JPCRD2022CODATA.pdf)

## Opaque surface and reflection

The surface boundary is ελBλ(T)+(1−ελ)Lrefl,λ. Under thermal equilibrium Kirchhoff's law relates emissivity and absorptivity for the matching direction, wavelength and polarization, with reciprocity assumed here. Energy conservation gives absorptivity plus reflectivity equal to one for an opaque surface with no transmission. Here the surface is diffuse and represented by scalar spectral values, and Lrefl,λ is an isotropic equivalent incident radiance. This reduces reflected output to (1−ελ)Lrefl,λ. An anisotropic or specular environment needs angular radiance and a reflectance distribution. A semitransparent material also requires a transmitted term. The experiment uses either a blackbody-equivalent environment temperature or explicitly specified spectral radiance. Emissivity can vary with wavelength and is held independent of temperature within each controlled case.

## Atmospheric transfer

For each path layer, local thermodynamic equilibrium and absorption-only transport give dLλ/ds=κλ[Bλ(Tair)−Lλ], with path s increasing from surface to camera, κλ in m⁻¹ and length in m. A uniform layer transmits τλ=exp(−κλd), and its emitted contribution at the exit is (1−τλ)Bλ(Tair). Ordered composition attenuates all previously acquired path emission. Total transmission is the exponential of the negative sum of depths. The output is

`L_obs = transmission × surface_emission + transmission × surface_reflection + path_emission`.

The model and CSV distinguish boundary emission, boundary reflection, transmission, each attenuated direct contribution and emitted path radiance. Each absorbing layer is isothermal with a prescribed spectral coefficient, sampled by piecewise-linear interpolation at 8/11/14 µm. The fixed gray τ sweep represents an idealized one-layer model. τ=0 is the infinite-depth limit under an isothermal common path, yielding only that path's blackbody radiance. Scattering, gas line structure, and coupled surface–atmosphere illumination are outside this approximation. The coefficients are synthetic LWIR parameters and are independent of the RGB model. The approximation applies where thermal absorption and emission dominate and prescribed profiles are adequate. No physical claim is made for strongly scattering fog, dust or smoke. [NASA ASTER thermal atmospheric correction formulation](https://eospso.gsfc.nasa.gov/sites/default/files/atbd/atbd-ast-09t.pdf)

## Detector and uncertainty

The detector integrates RλLobs,λ in W·m⁻²·sr⁻¹ using composite trapezoids. Rλ is a dimensionless relative response on a fixed 8–14 µm band. Acceptable weights are between zero and one, with an illustrative 0.65/1/0.72 profile, a top-hat and zero-throughput controls. The configured response and material profiles are linear between their knots. The integral is unnormalized. The quantitative error bounds apply to the frozen smooth and piecewise-smooth profiles, with additional discretization needed for sharp lines or narrower bands.

Additive Gaussian input-equivalent noise in band radiance is independent across draws. The configured range maps to an integer code with nearest rounding and ties to even. Noise is added before either rail clamps. Codes at the rails can suppress apparent variance even when radiance-domain error is large. Detector observations, noise-free response, codes, clipping and input-equivalent decoded metrics are stored separately. There is no empirical gain, fixed pattern, PSF, nonuniformity correction, noise-equivalent temperature calibration or hardware response claim.

## Contrast and inference limits

Band contrast is the signed target-minus-background radiance and, where their sum is nonzero, the difference divided by that sum. The within-threshold flag compares absolute contrast to the larger of one code LSB and twice the standard deviation of the difference of two independent equal-σ noises. This is a descriptive threshold for the configured detector. Crossover is investigated by sign reversal in a visible, unsaturated sweep. A supplied static visibility flag selects the foreground before transport in one control. Hidden targets have no target contrast metric. Zero throughput is a detector limitation, and an optically thick common path can collapse radiometric contrast despite the geometry being open.

One band measurement generally cannot identify temperature, emissivity, reflected radiance and path state jointly. The forward model provides controlled truth and observation predictions without a temperature retrieval claim.
