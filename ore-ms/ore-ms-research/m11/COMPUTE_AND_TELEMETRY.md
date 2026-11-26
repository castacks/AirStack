# Compute and failure telemetry

M11 ran on Ubuntu 24.04.4 LTS on an AMD EPYC-Genoa x86-64 host with 48 visible logical CPUs and 216 GiB total RAM. CPython 3.12.3, NumPy 2.2.6 and SciPy 1.15.3 used one OpenBLAS thread. The logged phase total was 3.33 s with 271,080 KiB peak RSS (about 265 MiB). GPU compute was unnecessary for the verification package.

| Measured scope | Calls | p50 / p95 / p99 wall ms | Measured throughput |
|---|---:|---|---:|
| Frozen contrast frontend | 5,184 | 0.106 / 0.149 / 0.157 | 9,081 calls/s across frontend calls |
| Five-arm proposal and qualification | 5,184 | 0.015 / 0.025 / 0.029 | 62,667 calls/s across policy calls |
| Fusion phase including generation, file contract, summary and decision CSV | 5,184 | Phase wall time 2.245 s | 2,309 opportunities/s |

These call latencies exclude renderer capture, device transfers and ROS 2 transport. Fusion telemetry retains reason counts by arm, conflict warnings, candidate qualification misses, abstentions and false supported outputs. A01 recorded 2,103 unqualified score-bin decisions, 1,020 unresolved conflicts and 880 no-candidate decisions. No missing rows were observed. Known failure reason counts are distinguished from real-time transport/dropout telemetry, which remains untested. [Timings](evidence/timing.json) · [Decision telemetry replay](REPRODUCTION.md)

Host preflight found an NVIDIA RTX PRO 5000 Blackwell with driver 580.126.20 and 48,935 MiB VRAM, and about 4.0 TB free at inspection. The local Docker image inventory had AirStack robot/GCS images and no Isaac Sim image. Runtime compatibility, image pull/build, headless Vulkan/RTX, writer rates, GPU consumption and concurrent fleet feasibility remain unmeasured. NVIDIA lists a 16 GB VRAM and 32 GB RAM baseline for this version, with higher floors for complex scenes. The proposed pilot would need Linux container access, NVIDIA Container Toolkit, resolved assets, pinned Isaac/Pegasus versions and a functioning Jazzy bridge. [NVIDIA 5.1 requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)
