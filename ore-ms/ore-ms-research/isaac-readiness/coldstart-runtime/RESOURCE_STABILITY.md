# Resource and stability observation

The workspace saw one NVIDIA RTX PRO 5000 Blackwell with 48,935 MiB VRAM. Spot checks during the failed startup state showed 1 MiB used and 0% GPU utilization. Isaac and GCS each had a zero restart count and a zero start timestamp. No robot container was present. The Docker data root was `/osmo/run/docker` with the overlay2 driver.

The requested 10–15 minute steady-load observation was blocked by the failed startup. No steady-workload CPU, RAM, GPU, clock, sensor, MAVROS or Foxglove continuity figures were collected. No lifecycle operation or recovery experiment was performed.
