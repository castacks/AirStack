# Implementation diff

The start and remote reference were both `e5436fc3c821ac26caeae1258beea512905322da` on `ore-ms-orb-development`. That commit contains the [cross-branch investigation](../ORE_PROJ_NUMPY_COMPARISON.md). The reference was `ore_proj` at `169c83a58bbb387e9a9086ef49ca2dd5887a7eb7`. The starting checkout had only the pre-existing untracked robot core dump.

- `simulation/isaac-sim/docker/Dockerfile.isaac-ros` pins Isaac's NumPy to 1.26.4, constrains the subsequent Pegasus and lark-parser installs and asserts the version through `/isaac-sim/python.sh` after those installs.
- `simulation/isaac-sim/docker/docker-compose.yaml` gives both Isaac profiles an optional `ISAAC_SIM_IMAGE` override through inheritance. The default release reference and build tag/cache behavior stay intact. An override by digest is for deployment with `--no-build`.
- `tests/system/check_isaac_numpy.sh` independently checks a disposable container from a supplied local image reference. It fails if the Isaac interpreter cannot import NumPy or has a different version.

PX4 retains its separate virtual environment. The MAVROS namespace fix and the robot/GCS image selection were preserved. The default `.env`, M11 and earlier readiness reports and evidence were left intact. See [source hashes](evidence/source-provenance.txt).
