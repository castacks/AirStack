# RGB baseline

Each R/G/B band invokes E1 with its configured boundary radiance, extinction and path source. The foreground is a visible RGB surface at 4 m axial depth. With the clear configuration the center pixel reports foreground RGB radiance (7, 3, 1). Under finite attenuation, the model independently reproduces S + (L₀−S)exp(−κd). Read `physics.py`, `simulate.py` and the raw capture.
