# ore_proj NumPy compatibility investigation

**Finding.** `ore_proj` already has an image-level fix for the Isaac Sim 5.1.0 NumPy 2.x camera and RTX LiDAR writer failure. The reusable changes are in the **inner Isaac image** and its Compose image selection. The outer workspace tag serves a separate startup role.

## Repository and workflow evidence

- Inspected on 2026-11-26 UTC without changing branches. Starting branch `ore-ms-orb-development` and HEAD `63b5619a6b8a35331144182fbc23e6778109f403` matched the live remote branch. The only pre-existing change was untracked `robot/ros_ws/core` (171,737,088 bytes, SHA-256 `3425b3f3aa5874b73c9866aa95ecc78ff053e27b30ea063299eae6fdf2f2e111`).
- `origin` fetch and push use `https://github.com/castacks/AirStack.git`. Fetch and live ref lookup agreed on `ore_proj` SHA `169c83a58bbb387e9a9086ef49ca2dd5887a7eb7`, unchanged from the pre-fetch tracking ref. There is no local `ore_proj` branch to compare. Inspection used Git objects directly.
- `osmo/workflows/airstack-live-replan-ore-proj-one-gpu.yaml` was absent from the fetched branch, local checkout, supplied attachments and known Git path history. Its presence on the user's Mac or another checkout could not be verified. The tracked `osmo/workflows/airstack-live-replan-ore-proj.yaml` and `osmo/workflows/airstack-live-replan-dev.yaml` both use the requested outer image and clone `ore_proj`. Each declares separate workspace and Cosmos GPU tasks. Neither establishes what the missing one-GPU file contains.

## What the fixes do

| Layer | Verified change |
|---|---|
| Outer OSMO workspace | `osmo/workspace/Dockerfile` is identical across the compared branch tips. It packages `osmo/workspace/entrypoint.sh`; its header and `osmo/README.md` document the amd64 Buildx build from `osmo/workspace`. The `ore_proj` entrypoint adds a post-start epoch check that restarts a robot container which predates Isaac. It does not install Isaac's NumPy. Registry inspection resolved `airstack-osmo-workspace:ore-proj-epoch-numpy126-1` to index `sha256:a301f275a69cf74d837f41c17aa7ee7b515ab564a464467880422b3598f56965` and amd64 manifest `sha256:e51b399822dcbd1732f92b1b3bdc1a464d69309c5cbeb4b7296ca533453650ae`. Published image history records the entrypoint copy. The exact entrypoint bytes were not extracted. |
| Inner Isaac build | `a2e61d77e3693557c0188e75cd04ec1634c405c4` changes the `/isaac-sim/python.sh -m pip install` list in `simulation/isaac-sim/docker/Dockerfile.isaac-ros` from unpinned `numpy` to `numpy==1.26.4`. After the later pip installs it runs `/isaac-sim/python.sh -c 'import numpy; assert numpy.__version__ == "1.26.4", numpy.__version__'` so build-time drift fails the build. This targets native render writers and directly addresses the 2.4.6 versus 1.26.4 runtime issue from R1. |
| Inner Isaac deployment | `0e226f50f7b33cad45e8453efe36b2ad0a4eb3bc` adds `ISAAC_SIM_IMAGE` selection in `simulation/isaac-sim/docker/docker-compose.yaml`, shared by standard and livestream services, and pins it in `.env` to `airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:3e08f31c208fb5fd4435d782761f105289903d30debc8bdfaca92349ddb4a49d`. Robot and GCS keep the release version. Registry inspection resolved the Linux/amd64 manifest. Its image history contains both the NumPy install pin and final import assertion, with no subsequent Python package install recorded. |
| Manual recovery | `RRM-transfer/rrm-9d26a58/README.md` documents the older container-local `--force-reinstall numpy==1.26.4` workaround, and separately reports that the published `v0.20.8-rrm-numpyfix-20261003t1930edt_isaac-sim` candidate retained 1.26.4 after recreation and restored sensor output. This is historical runtime evidence. Neither registry image was pulled or executed in this investigation. |

The outer tag name alone establishes no installed NumPy version and does not make a workspace that clones ORE-MS select a different inner image. The persistent fix follows from using the pinned inner build or its verified digest. The separately observed apt-managed NumPy uninstallation failure occurred in PX4's system Python path. `ore_proj` does not contain ORE-MS commit `abda623df`'s PX4 venv isolation in that Dockerfile.

## Minimal remediation for ORE-MS

Port the inner NumPy pin and late import assertion while retaining the PX4 venv and MAVROS namespace fixes. Verify later Isaac pip installs against the pin and constrain conflicting dependencies if resolution requires it. Build and publish an ORE-MS candidate, add the Isaac-only Compose override, and pin the qualified digest for both simulator profiles. Test a clean recreation with the pristine interpreter and a live RGB/depth/LiDAR smoke check. The existing `ore_proj` digest is a useful comparison candidate. It does not establish provenance for the combined ORE-MS build. Keep the outer epoch repair independent of NumPy, and avoid importing the Cosmos workflow for this fix.

No workflow launch, image rebuild, image pull, implementation port or M11 change was performed. The original branch and pre-commit HEAD stayed in place throughout inspection. The report commit is the intended HEAD advance.
