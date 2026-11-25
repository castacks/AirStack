# Development record

The original research specification, current roadmap and previous acceptance records established the M03 scope. The plan and configuration were frozen before model implementation. M01 provided the equilibrium slab concept and gray LWIR baseline. The M03 kernel adds broad-band stable Planck evaluation and a separate LTE layer and detector interface. M02 RGB coefficients and detector assumptions were kept outside the thermal module.

The forward solver, experiment runner, independent references and verification harness live in separate files. The recorded verification uses synthetic inputs and no external numerical packages, sensor hardware, datasets or models. A single supplied static foreground case exercises reporting only. No later milestone was implemented.
