# WS2 automated flight bench

Runs MonoNav, Kim/FCRN and PULP-DroNet v3 through the same AirStack/PX4
execution and evaluation path. Office supports feedback/random/search/Claude;
the corridor suite uses fixed common maps and recorded clean/attack tests.

## Installation

Requirements: Linux, NVIDIA GPU/driver, Docker Compose/NVIDIA Container Toolkit,
AirLab registry access, and the AirStack v0.18 Isaac/robot images used by this
branch. Keep the selected model repositories beside `AirStack/`.

```bash
git clone --recursive --branch eungchang/adv-ws2 git@github.com:castacks/AirStack.git
cd AirStack
./airstack.sh setup
docker login airlab-docker.andrew.cmu.edu
./airstack.sh image-pull
```

Install/build the selected workers using their own READMEs:
[MonoNav](https://github.com/engcang/MonoNav),
[Kim](https://github.com/engcang/Collision-avoidance),
[PULP](https://github.com/engcang/pulp-dronet/blob/master/README_AIRSTACK.md).
The expected images are `mononav-demo:1.0`,
`collision-avoidance-airstack:1.0`, and `pulp-dronet-airstack:1.0`.

Prepare the existing containers once, with application autostart disabled:

```bash
sudo apt-get install python3-venv fonts-dejavu-core
AUTOLAUNCH=false ./airstack.sh up isaac-sim robot-desktop
docker exec airstack-robot-desktop-1 bash -lc 'bws --packages-select mononav_bridge local_bringup'
docker stop --timeout 3 isaac-sim airstack-robot-desktop-1
python3 -m venv .venv-ws2
.venv-ws2/bin/pip install PyYAML Pillow imageio-ffmpeg
```

The runner checks that container source/runtime mounts belong to this checkout.
Corridors are generated locally; no Office asset download is needed for them.
For Office, restore the USD/textures using [BACKUP.md](BACKUP.md).
Keep weights, runtime videos/bags, build outputs and caches out of Git.
Rui's small original patch is already in `assets/learned_patch.png`.

## Run

Use the venv Python below, or an existing Python with those dependencies.
Start the viewer in one terminal:

```bash
.venv-ws2/bin/python tools/ws2_bench/dashboard.py --port 8892
```

Open **http://127.0.0.1:8892**. Simulator and actual model inference appear side
by side; Pause, Resume and Stop control the active campaign. Only one flight
worker may execute at a time. The viewer's **Start tests** form launches Office
campaigns. Choose a target and feedback/random/search, or optional Claude
([authentication and agent options](CLAUDE_AGENT.md)).

Launch recorded corridor suites from another terminal:

```bash
.venv-ws2/bin/python tools/ws2_bench/corridor_suite.py \
  --targets mononav kim pulp_dronet --tiers easy \
  --output robot/ros_ws/ws2_runtime/campaigns/corridor_recorded
```

Shapes are `elbow`, `curve`, `zigzag`, `u_obstacles`; use `--shapes` to select a
subset. All targets receive the same geometry for each shape/tier. Other tiers
are `medium` and `hard`. `--phase clean` runs only two clean repeats per map;
`--phase attacks` resumes these checks and runs attacks after the clean gate.
Default `all` performs both. For medium/hard, first inspect the observed clean
rates and adjust a new map version if needed. These are small preliminary
checks, not statistical reliability estimates.
For a bounded follow-up, `--clean-repeats 4 --phase clean` explicitly extends
selected maps to four repeats. All previous failures remain in qualification
and attack comparisons; resuming with the default never discards extra repeats.

Clean gates require at least one success per map and aggregate success of
100%/75%/50% for easy/medium/hard. On qualified tiers, attacks run on every map
using a successful clean reference. A map with variable clean outcomes is
labeled `baseline_variable_no_attack_attribution`. RGB noise and delay are isolated; their combination is tried
if both individually complete. Kim additionally uses the original FCRN patch
where an obstacle exists. PULP/MonoNav patch requests are rejected.
Clean and attacked flights share geometry, lighting and target settings.
A reused successful clean reference is explicitly recorded in `comparison.json`.

Each flight starts fresh and records **takeoff command through landing or
collision/termination**. `flight.mp4` contains two 960×540 panels, without
cropping, plus labels; timing/source-frame metadata are in `video.json` and
`video_frames.jsonl`. Model loading is labeled until the current worker has a
frame. `videos.html` opens all videos, `flights.csv` lists measured outcomes,
and `qualification.json` contains the clean counts. Failures and surviving
attacks are retained. A recorder/runtime error stops the suite for review.

Check source/configuration consistency and decode the saved recordings:

```bash
.venv-ws2/bin/python tools/ws2_bench/audit_corridor_suite.py \
  robot/ros_ws/ws2_runtime/campaigns/corridor_recorded --decode-videos
```

Add `--require-complete` for a final audit of all three targets, four shapes and
three tiers. `recording_catalog.csv` includes video paths, conditions and metrics.

Re-running the same command resumes completed results. Use a **new output
directory** after changing source, maps, target settings or weights.
For one custom flight:

```bash
.venv-ws2/bin/python tools/ws2_bench/episode.py \
  tools/ws2_bench/scenarios/corridor_kim.yaml --no-bag --camera follow --record-video
```

Omit `--no-bag` to also record ROS data, beginning on the ground. Omit
`--record-video` when a video is unnecessary. Automatic recording does not
capture or interrupt the desktop.

## Configuration

| File | Edit here |
|---|---|
| `scenarios/corridor_suite.yaml` | Common width/scale, wall style, lighting, noise sigma, delay, patch size and observer camera |
| `scenarios/corridor_mononav.yaml` | MonoNav speed, depth scale, clearance, map radius, height/timeout |
| `scenarios/corridor_kim.yaml` | Kim policy, speed limits, FOV, camera pitch, input rate, trajectory horizon |
| `scenarios/corridor_pulp.yaml` | PULP variant, speed, FOV, yaw gain, smoothing, height/timeout |
| Individual episode YAML `condition` | Custom scene/seed/light/noise/delay/patch settings for `episode.py` |

The suite replaces target YAML geometry with the shared suite map. Layout v2
keeps Easy unchanged (bare L/curve/zigzag, two-block U). Medium places two blocks
(four on U) inside the route, away from takeoff and finish. Hard preserves those
Medium blocks and adds three, giving five blocks (seven on U). Walls and route
stay fixed. These are map designations; measured difficulty must be checked.
Saved specs without `layout_version` retain v1 coordinates for historical replay.
Scene generation is in `corridors.py`, target registration in `model_adapters.py`.

Noise sigma uses RGB pixel units (0–255); delay and timeouts use simulation
seconds. Patch is an unchanged opaque image with size/on-off/timing controls.
MonoNav receives explicit corner task goals, while Kim/PULP receive no route
or goal. All-scene PhysX contact and ordered gates determine corridor results.
Camera/control integration safeguards are separate from the pretrained networks.
Detailed historical evidence is in [CORRIDOR_VALIDATION.md](CORRIDOR_VALIDATION.md);
source/asset recovery is in [BACKUP.md](BACKUP.md).
