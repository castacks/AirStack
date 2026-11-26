# GPU feasibility

One NVIDIA RTX PRO 5000 Blackwell was visible on the pod and in Isaac. Driver 580.126.20 reports CUDA 13.0 capability. Isaac 5.1.0 rendered and supplied RGB and depth; this verifies compatibility for the exercised path, without measuring CUDA library ABI coverage.

Eighteen snapshots showed 3,185 of 48,935 MiB in use and 4–7% GPU utilization. This is the sampled steady-state maximum, not the startup or fixture peak. Isaac used about 5.93 GiB RAM. Median container CPU usage ranged from 564% to 578% across two observation windows. The ROS observer measured about 19.5 RGB and 19.4 depth messages/s in domain 1. Clock advanced about 7.11 simulated seconds in the 12 s observation window; this is an approximate 0.59 simulated/wall ratio including subscriber acquisition. Renderer FPS, actual physics-step throughput and separate scene-load time were not instrumented. Boot markers are in [boot verification](BOOT_VERIFICATION.md).

One GPU was adequate for this workload. No fixture peak, matched workload bottleneck test, or evidence supporting a multi-GPU request was obtained.
