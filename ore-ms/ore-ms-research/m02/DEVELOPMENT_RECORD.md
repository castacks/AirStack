# Development record

The starting branch was `ore-ms-orb-development` at `1a4e4e9735f28ad64efc84f0e48f28f91c82ad50`. The M02 plan and scene were hashed before implementation and experiment execution. The design preserves the M01 implementation and uses a separate visible-only layer solver and detector.

`rgb_model.py` handles forward physics and sensing, `experiment.py` writes raw captures and metrics, and `metrics.py` holds measurement functions. `reference.py` and `verify.py` contain the separately constructed verification algorithms. The detector-headroom diagnostic was frozen after the primary run revealed saturation and before the higher-full-well arm was evaluated. Both runs and the original saturation are retained.

No external data, model weights, external runtime packages or additional hardware were required. No later milestone was implemented.
