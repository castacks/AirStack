# Repository backup and reproduction

Keep repositories separate; do not upload the workspace wholesale. The scope
below contains the WS2 source, documentation and the supplied small patch asset.

| Repository / branch | Changes to preserve |
|---|---|
| `castacks/AirStack`, `eungchang/adv-ws2` | `tools/ws2_bench/` source/YAML/tests/docs/layout catalogue/patch manifest **and `assets/learned_patch.png`**, `mononav_bridge/` source/config/launch/package/README changes, `mkdocs.yml`, runtime `.gitignore` |
| `engcang/MonoNav`, `main` | `mononav_airstack.py` headless mode, shared goal-radius option and actual inference preview export, README |
| `engcang/Collision-avoidance`, `main` | `collision_avoidance_airstack.py` headless mode and actual inference preview export, README |
| `CyAirDSTA-internal-CyLab/Humanflow` | No modifications in this task. Record the source revision; no duplicate checkout or checkpoint in AirStack |

Ravi's metrics/report sources were reused from FloareDor/AirStack revision
`a8adef094872bddc40053e534a62d5f2cb74a4a1`; see [REUSE.md](REUSE.md). Do not back up
`ws2_offline/` as another runnable AirStack. It is an inspection snapshot; the
working implementation is now under `AirStack/tools/ws2_bench/`.

Baseline revisions before this implementation: AirStack `b9b31b7cd`,
MonoNav `99a86fb`, Collision-avoidance `45a4b63`, Humanflow `e393826`.
Record the checked-out exact SHAs for reproduction; episode provenance also
records repository and runtime source identities.

Companion worker revisions for this bench backup:

- `engcang/MonoNav`: `cc5ec88` — headless execution, goal-radius override and
  actual inference preview export.
- `engcang/Collision-avoidance`: `395485b` — headless execution and actual
  FCRN/D3QN inference preview export.

The AirStack revision is the commit containing this document; record it with
`git rev-parse HEAD`. The future dynamic-placement LLM plan is in
[AGENT_ROADMAP.md](AGENT_ROADMAP.md), and is not current functionality.

## Exclude from Git

- `robot/ros_ws/ws2_runtime/` contents except its `.gitignore`: bags, images,
  NPY arrays, reports, logs, transient commands, traces and screenshots.
- Office USD/MDL/textures, `Isaac_Office_4.5_complete.zip`, CMU/Novare data,
  `ws2_offline/local_assets/`, simulator caches and Docker image archives.
- FCRN/ZoeDepth downloaded weights, converted PyTorch checkpoints,
  `ws2_assets/`, `.venvs/`, ROS `build/install/log`, `__pycache__`.
- Videos, PPT/PDF exports, authentication files, private keys and machine-specific
  connection configuration.

**The necessary 41 KB patch is included in the AirStack backup** at
`tools/ws2_bench/assets/learned_patch.png`, per the user's instruction. Its
`.gitignore` exception permits normal Git addition. Clone restores it together
with `assets/patch_manifest.json` (SHA-256, dimensions and provenance).
It has no public download URL and must not be removed as a downloadable cache.
The original workspace copy is redundant once this copy's hash is verified.

`save_model/D3QN_V_3_single.h5` is **already tracked** in Collision-avoidance
(about 9.2 MB). It is restored by cloning that repository, not copied into AirStack
or uploaded again as a new generated file. Removing it from existing Git history
is outside this task; no history rewrite was done.

Keep selected experimental bags/videos separately if needed for research evidence;
excluding them from Git does not mean deleting them. `VALIDATION.md` is the small
text record of checks, not a substitute for raw experimental evidence.

## Restore code and model dependencies

Clone these as siblings in any workspace directory:

```bash
git clone --recursive -b eungchang/adv-ws2 git@github.com:castacks/AirStack.git
git clone --recursive https://github.com/engcang/MonoNav.git
git clone --recursive https://github.com/engcang/Collision-avoidance.git
```

Use revisions containing this bench and both headless planner adapters.
AirStack setup/image access and optional GUI requirements remain those of the
repository README. Build planner images from
the checked-in Dockerfiles:

```bash
(cd MonoNav && bash docker/build_image.sh)
(cd Collision-avoidance && bash docker/build_image.sh && bash docker/download_models.sh)
```

- FCRN download: `Collision-avoidance/docker/download_models.sh` downloads the
  [public NYU checkpoint archive](https://storage.openvinotoolkit.org/repositories/open_model_zoo/public/2022.1/fcrn-dp-nyu-depth-v2-tf/NYU_FCRN-checkpoint.zip)
  and verifies SHA-256 `9d97ed165c4a5b3f085eb83b8814de1e883c6348da60da4b2568ddd64bb2d5c4`.
- ZoeDepth: first model load downloads
  [ZoeD_M12_N.pt](https://github.com/isl-org/ZoeDepth/releases/download/v1.0/ZoeD_M12_N.pt)
  via the pinned ZoeDepth configuration. The runtime uses Docker volume
  `mononav-torch-cache`; this is a cache, not a Git asset. Warm the model using the
  documented MonoNav demo before a timed campaign so a network download does not
  become a startup failure. Episode provenance hashes the installed weights.
- The PyTorch FCRN conversion is only needed for training/parity diagnosis, not
  for normal Kim inference or rendering the PNG. Use Humanflow's
  `attk/scripts/convert_fcrn_checkpoint.py` with the downloaded TF checkpoint;
  preserve the script revision and recreate the converted file instead of committing it.

Optional conversion/parity workflow from the sibling workspace root (Humanflow
access required; obtain its `e393826` source revision for the comparison above):

```bash
# CPU torch, tensorflow-cpu and numpy are needed for conversion/parity tooling.
python Humanflow/attk/scripts/convert_fcrn_checkpoint.py \
  --checkpoint Collision-avoidance/airstack_models/NYU_FCRN-checkpoint/NYU_FCRN.ckpt \
  --output /path/to/private-model-cache/NYU_FCRN_pytorch.pt
python AirStack/tools/ws2_bench/check_fcrn_parity.py /path/to/saved/captures \
  --humanflow Humanflow --weights /path/to/private-model-cache/NYU_FCRN_pytorch.pt
```

The recorded CPU tooling used torch2.6.0, tensorflow-cpu2.18.0 and numpy1.26.4.
`inspect_patch_depth.py` must first create the reference input/output NPY arrays
inside the Kim Docker image, with Collision-avoidance mounted at
`/workspace/planner`, bench scripts mounted read-only, and the capture directory
writable. `--diagnose-pooling` temporarily changes only the diagnostic model;
it does not alter the training repository, deployed policy or learned PNG.

## Restore Office assets and local containers

This implementation was validated with the Office **4.5 asset set** in the
existing local AirStack v0.18 containers. The simulator itself reports Isaac Sim
5.1; record the actual image ID from each episode's `provenance.json`.
Fresh-host provisioning has not been independently rerun in this task.

NVIDIA documents Office in the [4.5 environment catalogue](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/assets/usd_assets_environments.html)
and provides [asset-pack download/setup instructions](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_faq.html#local-assets-packs).
The asset root is `Assets/Isaac/4.5`; Office is
`Isaac/Environments/Office/office.usd`. Obtain the asset packs from NVIDIA,
preserving both `Isaac` and `NVIDIA` directories and all relative references.
Alternatively use Isaac's asset collection/export workflow to collect Office
and its dependencies. Copying only `office.usd` will lose materials/references.

After repository setup and authenticated image pull, create the simulator and
robot containers with automatic application startup disabled, build the bridge,
and stop them before running this bench's cold-start adapter:

```bash
cd AirStack
AUTOLAUNCH=false ./airstack.sh up isaac-sim robot-desktop
docker exec airstack-robot-desktop-1 bash -lc 'bws --packages-select mononav_bridge'
# For an extracted asset tree whose top folder is Assets:
docker cp /path/to/Assets isaac-sim:/tmp/ws2_assets
docker stop --timeout 3 isaac-sim airstack-robot-desktop-1
```

The default scene path inside the simulator is
`/tmp/ws2_assets/Isaac/4.5/Isaac/Environments/Office/office.usd`.
Set `WS2_OFFICE_USD` to another **container-visible** Office path if needed.
Container deletion loses `/tmp/ws2_assets`; repeat the copy or use a persistent
bind mount. The bench now defaults to headless Isaac rendering in the web UI.
X11 is needed only for the optional `WS2_HEADLESS=0` desktop mode; do not copy
the old user's Xauthority or absolute home path into shared config.

The small checked-in `layouts.json` records generated offsets and source hashes.
To regenerate against an equivalent local Office asset tree, install `usd-core`
and `numpy` in a CPU tooling environment, then run:

```bash
python3 tools/ws2_bench/prepare_layouts.py /path/to/office.usd
python3 tools/ws2_bench/prepare_difficulty.py /path/to/office.usd
```

Recheck geometry/materials and protected spawn if the Office version changes.
The second command adds24nested Easy/Medium/Hard placements (1/3/5plants and
columns each), using the first command's legacy catalogue as a base. The checked-in
catalogue restores these placements without regenerating them. Density selection
is stored in campaign configuration, and resolved layouts are saved per episode.
No claim is made that arbitrary future asset versions reproduce identical scenes.

## Review before a future commit

On 2026-09-22, `git add --dry-run .` was checked separately in AirStack, MonoNav
and Collision-avoidance. Each repository's root currently selects the intended
source/docs/config changes; bags/checkpoints/runtime outputs are ignored and the
learned PNG is included. Thus `git add .` from each of those repository roots is
appropriate for the current tree. Do not run it from the parent workspace.
`git status --short`, `git diff --stat` and `git ls-files --others --exclude-standard`
should show source/docs/config changes plus the explicitly included learned PNG.
Other `assets/*.png` and runtime outputs remain ignored. This guide does not
stage, commit or push anything automatically. Commit stores changes locally;
remote backup also requires push. MonoNav's backup remote is `fork` (engcang),
while AirStack and Collision-avoidance use `origin`.
