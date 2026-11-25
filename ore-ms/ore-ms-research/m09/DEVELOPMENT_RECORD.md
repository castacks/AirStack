# Development record

M08 conditional acceptance and AC18 closure were recorded before freezing the M09 plan. The teacher uses unchanged M08 forecasts on distinct generated training and validation realizations. The data model, physics model and ablation were trained with the same prescribed sample populations and independent initialization seeds. The core uses NumPy only. The optional visualization uses Matplotlib.

The initial scorer's archived oracle coverage parity check identified a single-ULP scalar-versus-vector exponential difference. Restoring the archived scalar path resolved the zero-width interval issue. This corrected scoring arithmetic without changing issued predictions or thresholds. Negative mismatch, OOD and benchmark findings were retained. Prior scientific code and evidence remain unchanged.
