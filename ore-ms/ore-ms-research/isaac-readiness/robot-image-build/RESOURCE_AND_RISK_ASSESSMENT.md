# Resource and risk assessment

Stage 1 is a CPU build. Request an isolated Linux/amd64 Docker/Buildx worker with **8 vCPUs, at least 24 GiB free RAM, at least 60 GiB free Docker storage and a 120-minute initial window**. These are planning estimates for cold downloads, two build stages and native dependency compilation, with no measured build peak for this candidate. No GPU is needed for the build or static package probe. Stage 3 requires a separate one-GPU integration allocation.

The current workspace exposed an amd64 Docker 29.5.0 daemon and BuildKit 0.30.0. The backing filesystem showed 4.1 TiB available and both existing images totalled 30.64 GB in Docker's accounting. The workspace cgroup showed 10 CPU quota, 48 GiB memory limit and about 37.8 GiB memory usage at sampling. Host-level memory and disk statistics do not establish usable workload headroom or task storage quota. The observed workspace headroom falls below the build planning estimate. Prefer a dedicated worker and verify its effective memory and storage budget before approval.

| Risk | Required control |
|---|---|
| Network inputs move | Keep the pinned CUDA platform manifest. Capture Ubuntu APT and ROS repository/package resolution, build logs and runtime inventories. |
| Dependency drift | The ROS signing key comes from a moving branch. MAVROS and other APT packages resolve at build time. Pip includes pinned and incompletely pinned dependencies. TPM follows a shallow default-branch clone. DDS Router uses a versioned repos file with dependencies to resolve. Foxglove uses a latest download. Record the versions and available identities before qualification. A pinned base alone cannot reproduce the original image byte-for-byte. |
| Registry or package access | Confirm the builder can reach Docker Hub, Ubuntu and ROS package repositories, PyPI, GitHub and Foxglove endpoints. This gate verified the base manifest and did not download packages or layers. |
| Shared daemon or cache effects | Use the dedicated daemon and temporary tag, retain image inventories and build evidence, and avoid Compose build tags. |
| Published candidate differs behaviorally | Require static inspection and grounded runtime integration, then separately approve publication. Preserve the qualified Isaac digest, GCS tag and startup behavior. |
