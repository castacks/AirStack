# Compute measurements

Linux x86-64, AMD EPYC-Genoa, Python 3.12.3, NumPy 2.5.5, one OpenBLAS thread and no accelerator. Runs used the same 13,824 input opportunities and emitted all three arm outputs in one serial pass. Sharing B01 and B02 maps with B03 is part of that measurement.

| Measurement | Observed |
|---|---:|
| Complete pipeline from corpus generation through calibrated holdout scoring | 16.56 s wall |
| Largest child peak RSS across the pipeline | 201.74 MiB |
| Candidate stage with input load and all output writes | 1.75 s total |
| Per-frame candidate stage plus row write, all arms, p50 | 0.116 ms |
| Per-frame candidate stage plus row write, all arms, p95 | 0.168 ms |
| Per-frame candidate stage plus row write, all arms, p99 | 0.180 ms |

The pipeline ran in a shared execution environment. The per-frame timings start after loading frame metadata and include the candidate computations, score generation and output-row write call. They omit sensor capture, hardware registration and offline calibration. The pass timer includes array opening and file closure. This benchmark does not assign an isolated incremental cost to each arm. [Runtime and environment measurements](evidence/benchmark.json)
