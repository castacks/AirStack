# Environmental degradation taxonomy

| Mechanism | RGB effect | LWIR effect and coupling |
|---|---|---|
| Fog | Extinction and airlight | Droplet extinction and atmospheric emission. Shared range and liquid-water content. |
| Mist | Usually weaker or patchier contrast loss | Same mechanisms as fog with different loading, droplet distribution and geometry. |
| Dust | Attenuation, backscatter, deposits | Size- and mineral-dependent attenuation/emission and window contamination. |
| Smoke | Soot extinction, veiling, illumination dependence | Spectral absorption/emission depends on composition and temperature, including warm plume radiance. |
| Rain | Streaks, scatter and window droplets | Attenuation, obscured window and wet-surface emissivity/temperature changes. |
| Snow | Flakes occlude, raise background brightness | Flakes occlude and alter both apparent temperature and surface emissivity. |
| Illumination variation | Shadows, headlight flare, low photon flux | Indirect heating can change thermal contrast on a different timescale. |
| Temperature variation | Usually indirect effects for visible reflective scenes | Changes emitted radiance of scene, atmosphere and sensor. |
| Thermal crossover | RGB appearance may stay informative | Target and background band radiances converge. Reflections can shift crossover. |
| Geometric occlusion | Background replaced by foreground | Same under registration and opacity in both bands. |
| Contamination | Loss of throughput, haze, droplet distortion | Band-dependent window transmission plus self-emission. |
| Blur | Motion or defocus reduces spatial contrast | Shared motion with potentially different exposure, focal response and temporal lag. |
| Saturation | Bright-source clipping | Hot-source clipping or gain/range transition. |
| Noise | Read, shot and pattern noise | Read, thermal drift and fixed pattern. Shared electronics can induce correlations. |
| Sensor faults | Dropouts, stale frames, bad pixels | Similar failures plus LWIR-specific nonuniformity correction interruptions. Shared power and timestamps can cause common failures. |

M01 explicitly models occlusion, prescribed transport, surface thermal differences and additive Gaussian noise. The attenuated case is an illustrative slab with synthetic coefficients and has no weather label.
