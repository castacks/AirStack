# Development record

M04 started on `ore-ms-orb-development` at `e7e0d86d25b26f394f155d32f054c29d99d3dc8e`. The experiment matrix and tolerances were hashed before implementation. The forward model uses `geometry.py` for ray-level selection and `radiometry.py` as a small adapter to the unchanged M02 and M03 models. `analytical.py` and `verify.py` separately compute reference observations and verify the captured output. Prior milestone files and checksums were preserved. No model weights or experimental datasets were used.
