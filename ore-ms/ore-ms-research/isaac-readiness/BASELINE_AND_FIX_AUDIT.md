# Baseline and fix audit

On 2026-11-26 UTC, local HEAD and the live remote branch both resolved to `a91d61e6fa7b7489b6ada263151da4ede6400428`. The M11 directory has no diff from its research checkpoint `1f04d8bd0`. The pre-existing untracked robot core dump was preserved. No research control or runtime configuration was edited during this qualification.

- `abda623df` puts PX4 host dependencies in a Python 3.12 venv and uses it to compile SITL. The tested image contains both. This fixes pip attempting to remove apt-managed NumPy.
- `a91d61e6f` passes the empty namespace to the MAVROS include so ROS topics use the canonical single `mavros` prefix. The live installed launch file and source both hash to Git blob `429e10d2184e3327265ef8ef0363806dfe50df47`. Live canonical odometry is confirmed.
- The Isaac Python 3.11 runtime downgrade from NumPy 2.4.6 to 1.26.4 is a writable container change. It has no source commit and the pristine image still has 2.4.6.
- These two commits change one Dockerfile and one launch file. They do not change Compose, the OSMO entrypoint, DDS routing, port forwarding, or the Foxglove layout. Those components were audited as deployed.

See [image provenance](IMAGE_PROVENANCE.md) and [reproduction](REPRODUCIBILITY.md).
