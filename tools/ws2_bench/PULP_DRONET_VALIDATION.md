# PULP-DroNet v3: initial Office integration evidence

This page preserves the **earlier Office/ResBlock development results**. It is
not the latest corridor qualification result. See
[CORRIDOR_VALIDATION](CORRIDOR_VALIDATION.md) for the subsequent Tiny model:
four clean traversals of the obstacle U-course, one matched noise collision,
one completed delay traversal, and a separate completed zigzag trial. This
does not retrospectively qualify the Office scene or the ResBlock variant.

Date: 2026-10-14. This document records **clean-flight qualification and control
tuning**, not an attack experiment or a comparison of agent/random/search.
Runtime evidence: `robot/ros_ws/ws2_runtime/campaigns/pulp_dronet_20261014/`.
The original results of every attempted flight are retained, including failures.

**Office/ResBlock status: integrated, but clean-flight qualification failed.** Ten development
flights produced 8 mission collisions and 2 insufficient-progress outcomes, with
no passing 120-second reactive mission. These varied tuning attempts do not
estimate a final-policy success rate. Replacing MonoNav/Kim with PULP on the
strength of these results is not recommended. No training, attack campaign,
LLM comparison, commit or push was performed in that initial Office experiment.
The later corridor task includes local PULP commits; the user will push them.

## What was integrated

- Official pretrained float v3 ResBlock1.0;320,928 parameters; no retraining.
  Strict checkpoint loading passed. Eight deterministic random input tensors
  produced exactly the same outputs as the upstream float architecture
  (maximum absolute difference 0; `network_parity.json`).
- RGB-only grayscale preprocessing, steering/collision outputs, speed and yaw
  commands through the existing bridge/controller/PX4. No GT depth, GT obstacle
  lookup, learned patch or manually supplied flight path controls the model.
- Third target `pulp_dronet` is accepted in episode, feedback, random/grid,
  adaptive campaign and dashboard entrypoints. Noise/delay are allowed; Rui's
  FCRN patch and GT-depth substitution are rejected before flight.
- Actual RGB/crop, outputs, commands, simulation time and recent inference
  images are recorded. The web viewer displays the third target.
- Common avoidance mission:120 simulation seconds, collision-free, path>=3m,
  displacement>=1m, stationary<=50%. Clean failures remain failures. Passing
  this horizon does **not** by itself prove completion of the whole slalom.

## Tuning evidence

These are development attempts with different settings, **not ten repeated
trials of one final policy**. Unless noted, the height is1.2m, speed cap0.35m/s,
filter old-value weight 0.3 and tracker stop threshold0.2 m/s. `integrated v1`
adds the accumulated heading offset to a future yaw profile; `integrated v2`
uses the accumulated reference as the heading of every trajectory point.

| Flight | Configuration | Measured result |
|---|---|---|
| `initial_protected` | Protected seed42;60-degree crop;120deg/s yaw gain;0.35m/s cap;initial trajectory yaw | Window collision after51.87sim s;9.26m path;0% stationary |
| `slalom_wide` | Slalom seed42;75-degree crop;90deg/s yaw gain;0.35m/s cap;initial trajectory yaw | Reception-table collision after76.41sim s;3.71m path;59.9% stationary |
| `slalom_integrated_yaw` | Same slalom and crop; integrated yaw reference | Insufficient progress:120.03sim s;4.26m path;67.6% stationary. No mission collision; table contact occurred during subsequent landing. |
| `slalom_native_fov` | Same slalom;60-degree crop;90deg/s integrated yaw | Column contact after20.73sim s;2.53m path;5.4% stationary |
| `slalom_native_yaw120` | Slalom42;60-degree crop;120deg/s;integrated v1 | Column collision after17.07sim s;1.83m;6.3% stationary |
| `slalom_fov65` | Same as above,65-degree crop | Column collision after18.12sim s;1.97m;11.8% stationary |
| `slalom_slow_low` | Slalom42;60-degree crop;120deg/s;integrated v1;0.2 m/s cap;0.8m height | Insufficient progress at120.09sim s;2.80m;85.9% stationary |
| `slalom_controller_fixed` | Same image/speed/height settings;tracker threshold0.01;integrated v2 | Column collision after26.79sim s;2.39m;0% stationary |
| `slalom_clearance_tuning` | Same as above,70-degree crop;filter0.6 | Column collision after35.01sim s;3.20m;3.1% stationary |
| `protected_reference_seed7` | Same placement/lighting as Kim's passing reference;current defaults:60-degree crop,120deg/s,0.35m/s,1.2m height,filter0.3,tracker0.01,integrated v2 | Window collision after 55.05 sim s;8.90m;0% stationary |

The first flight's last previews show exterior scenery through the window.
The network kept predicting near-zero collision probability. This is evidence
of a perception problem in this camera/scene configuration; it is not a claim
that RGB navigation is generally incapable of avoiding glass. Lowering speed
alone does not supply a missing observation.

The slalom flight slowed substantially at the first column, but drifted toward
the reception table. Three integration issues were investigated:

1. A wider crop changes obstacle appearance and predicted collision risk.
   On one saved stalled frame (`stalled_input.jpg`), the same network predicted
   approximately 0.00021 at 60 degrees versus 0.927 at 75 degrees. This is a crop
   sensitivity observation, not a calibrated risk estimate or a general rule.
2. Replacing the trajectory every frame can weaken a yaw-rate command if its
   heading is repeatedly reanchored to measured yaw. `integrated` yaw references
   accumulate the requested rotation in simulation time and bound tracking
   error to 45 degrees. This changes the **control wrapper**, not network weights
   or steering predictions.
3. The running AirStack tracker had `min_virtual_tracking_velocity=0.2 m/s`.
   DroNet's normal deceleration commands often fall below that threshold, where
   frequent replacement trajectories make actual progress much slower than
   requested. For PULP only, the threshold is now0.01 m/s. Integrated v2 also
   avoids integrating yaw a second time through distance-based waypoint timing.
   With these two wrapper changes, the slow/low slalom's stationary fraction
   fell from 85.9% to 0%, but a column collision remained. This is a short paired
   development observation; the effects of the two changes were not isolated.

The repeated column contacts show insufficient physical clearance for this
Iris/controller/camera setup. The policy was trained for a much smaller drone;
airframe/camera mismatch is a hypothesis, not an established sole cause. PhysX
contact remains the collision criterion. The reported 0.25 m clearance sphere
does not exactly enclose all drone parts, so positive reported clearance at
physical contact is not evidence that the contact was false.

## Comparing against MonoNav and Kim

Earlier same-day clean runs in
`campaigns/agent_followups_20261014/{mononav,kim}/round_01/` used the same
slalom seed42 geometry. MonoNav stopped after4.54m; Kim collided after6.68m.
They are contextual evidence, not a statistically controlled ranking: MonoNav
has a goal mission, while Kim and DroNet have a reactive survival horizon.
Camera update rates, wrappers and achieved speeds also differ.

Earlier protected-route evidence is stronger for the existing targets:
MonoNav reached its8m goal twice on seed42; Kim completed 120 sim s twice on
seed7 (26.70m and25.45m paths). Their slalom/gap clean failures still prevent a
claim of broad obstacle-avoidance reliability. These flights were not rerun as
part of PULP tuning.

Do not select a new main target from a single attractive flight. Separate
development trials from repeated qualification and a different seed/layout,
and report collisions, stopped time, traversal distance and whether obstacles
were actually passed. A passing clean baseline is required before adding attacks.

## Decision and remaining work

- The adapter/HTTP/GT-evaluation path works; the official float network's output
  parity passed. This does **not** certify that preprocessing and airframe/control
  behavior reproduce the original Crazyflie system.
- Current defaults are the settings actually flown in `protected_reference_seed7`.
  They are an **experimental starting point**, not a qualified operating point.
- PULP encountered both a near-zero predicted risk at glass and inadequate
  clearance around columns. Speed/crop/yaw tuning has not resolved these cases.
  The next useful step would be camera-and-airframe-matched validation or
  adaptation using recorded failures, rather than more unstructured gain trials.
  Any retrained or additional navigation policy must be named separately from
  the original pretrained PULP-DroNet v3.
- For immediate bench development, the existing protected MonoNav/Kim baselines
  have actual passing evidence. Kim is the supported target for Rui's FCRN patch;
  MonoNav provides a goal-directed mission. Neither has passed the harder clean
  slalom/gap qualification. A fair three-model ranking remains future work.

All 148 bench CPU tests passed. The checkpoint SHA and strict loading were checked;
the actual third worker was exercised in 10 simulator episodes. Runtime summaries,
all trajectories and the final artifact audit are saved alongside the trials.
Downloaded weights and runtime data are excluded from Git. Owned flight
containers are stopped after validation; the viewer remains available.

## Reproduce and inspect

See [adapter README](planners/pulp_dronet/README.md) for sources, checksum and
installation. Each runtime trial contains `scenario.json`, `worker_command.json`,
`provenance.json`, `result.json`, `samples.json`, `diagnostic_trace.json` and recent
camera/inference previews. Replay a retained configuration with:

```bash
python3 tools/ws2_bench/episode.py /path/to/scenario.json --no-bag
```

Use the recorded controller/camera settings. The two initial trials predate the
explicit `yaw_reference_mode` field: specify `trajectory` to repeat their
original measured-yaw behavior. Exact replay of trials1–7 additionally requires
their earlier wrapper and0.2 m/s tracker threshold; a saved YAML alone does not
restore old source code. Worker-source hashes, adapter values and launch hashes
are recorded in provenance (launch hashes were added during this task).
Later defaults must not silently stand in for the original settings.
Large runtime data and downloaded weights remain outside Git; source, small
documentation and the weight downloader are eligible for backup. No commit or
push was requested for this integration.
