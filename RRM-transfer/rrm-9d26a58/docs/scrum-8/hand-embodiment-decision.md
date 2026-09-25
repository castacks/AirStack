# Phase-1 manipulation embodiment decision — 2026-09-24

Status: **Kuka arm + Allegro hand selected; isolated prerequisites permit preparation of one bounded contact trial, but execution remains disconnected.** This record follows the user's 2026-09-24 direction to resume the hand-workspace track. It supersedes the aerial-first ordering in [architecture.md](architecture.md) for this increment. The AirStack Iris/Office setup remains a separate transport regression fixture.

## Available assets and choice

The live Isaac Sim 5.1 container has no local hand USD under its standard asset tree. A read-only `omni.client` query from that container succeeded against the configured AirLab Nucleus root. The following assets were available at `omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1/Isaac/` on 2026-09-24:

| Candidate | Nucleus USD | SHA-256 of USD | Decision |
| --- | --- | --- | --- |
| Kuka + Allegro | `IsaacLab/Robots/KukaAllegro/kuka.usd` | `935957108f80625b58afb3ace9fff326edbb820a0b3629793c5f4931adb58fb2` | **Selected.** An arm plus dexterous hand can support a contextual grasp followed by placement in a tabletop workspace. The asset loads as one 23-DOF articulation; contact behavior and controller remain untested. |
| Allegro hand alone | `Robots/WonikRobotics/AllegroHand/allegro_hand.usd` | `6fdf6499dca915d23a75691ce14a6a0e35d4319b315fcffcb84d497305a81048` | Useful for fixed-base grasp/repose; no arm transport for the proposed place task. NVIDIA lists 16 DOFs. |
| Shadow Hand alone | `Robots/ShadowRobot/ShadowHand/shadow_hand.usd` | `0305ff5e8728f86b8e2d2a7c812c5a6634be1743dc20d67f56789349820e15bc` | Richer hand, but 24 DOFs and no arm in this asset make the first two-step tabletop task more complex. |
| Franka Panda | `Robots/FrankaRobotics/FrankaPanda/franka.usd` | Not read | Available, but its simple gripper does not satisfy the selected dexterous-hand profile. The legacy prototype's Panda prescription is not adopted. |

The USD digest identifies the top-level file, not its referenced dependencies; a frozen scene export must record all asset dependencies before a reproducible SIL campaign. NVIDIA's [Isaac Sim 5.1 robot catalog](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html) confirms the standalone Allegro and Shadow articulations. Isaac Lab also documents a [Kuka-Allegro lift environment](https://isaac-sim.github.io/IsaacLab-Arena/main/pages/example_workflows/dexsuite_lift/index.html), which supports this choice as a practical starting point; that environment is not installed or validated in the current AirStack image.

An isolated headless run of [hand_scene_probe.py](../../simulation/hand_scene_probe.py) loaded the selected USD without touching the active AirStack simulator. The articulation root was `/World/KukaAllegro/root_joint`, with seven `iiwa7_joint_*` arm joints and 16 finger joints. After setting the initial joint state as the reset default and stepping ten physics frames, three resets produced the same position/velocity state SHA-256 at 1e-4 rounding: `dbb2377dd5299a28975d88110d0601beec6f223f49c62aa893e89e638261218c`. The settled state differed from the initial default by up to 0.27925 rad and had up to 0.01418 rad/s joint velocity. This is repeatability for a bare articulation over three short resets, not deterministic reset proof for the eventual tabletop scene.

## Proposed controlled workspace

Create a separate tabletop Isaac stage with one fixed-base Kuka-Allegro articulation, a support surface, two distinguishable graspable blocks, a target tray, fixed lighting/cameras, and a versioned scene/entity manifest. S01's first task is “place the context-selected block in the tray,” decomposed into `GRASP(block)` then `PLACE(block, tray)`. Entity IDs and semantic effects stay in RRM; joint targets, contact handling, trajectories, and numeric limits belong to the embodiment adapter. The first action-enabled trial will be one bounded simulator action only after the gates below pass.

| Concern | Selected contract and required proof |
| --- | --- |
| Controller | Use Isaac Sim's `ArticulationController`/`ArticulationAction` at the adapter boundary for bounded joint commands. [Isaac Sim 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_simulation/articulation_controller.html) documents position, velocity, and effort control. A Kuka-Allegro joint-name map, limits, gains, contact stability, and motion-chunk checks must be measured in the selected stage; a generic API is not a validated grasp controller. |
| Reset determinism | Set a recorded seed, default arm/hand joint positions and velocities, object poses and velocities, camera pose, and physics settings; call `World.reset`, settle a fixed number of steps, and compare state hashes across at least three resets. A new episode ID invalidates previous C02 facts. Isaac's [core API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.api/docs/index.html) supports joint default states and reset, but repeatability for this scene is unmeasured. |
| Observations | Teacher: simulator object IDs/poses/contact and articulation joint state with frame, clock, and episode provenance. Candidate visual state: fixed RGB/depth frame with image hash, entity catalog, explicit fact claims, and independent teacher scoring. Controller state, joint position/velocity/effort, fingertip contact, and object pose/velocity must be observable before any effect or safe-state claim. [Isaac Sim physics sensors](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physics.html) provide articulation/contact channels; their installation in the new scene remains to be verified. |
| Safe-state proof | Admission closes before cancellation. `received`, `cancel_accepted`, `motion_stopped`, and `safe_confirmed` are separate C08 records. Proposed safe condition: arm/hand and object speeds below measured limits for a configured consecutive window, controller in a declared hold/disabled mode, fresh contact/object support evidence, and no active motion command. If the hand is holding an object, a controlled hold may be safer than release. Missing evidence yields `SAFE_UNCONFIRMED`. Thresholds and stop latency require measured scene trials. |
| Supported verbs | The first shadow C03 declares `GRASP` and `PLACE` as candidate semantics using `arm` and `hand` resources. `RELEASE` is a possible later profile extension. `OPEN`, `CLOSE`, `NAVIGATE_TO`, aerial verbs, bimanual actions, and free-form joint/pose commands are unsupported. Semantic support does not establish numeric feasibility or permission. |

## Qualification gates

1. Load the chosen asset in an isolated hand stage; record all referenced asset hashes, joint names/limits, contact setup, controller behavior, and a stage digest.
2. Demonstrate deterministic reset with three repeated start-state measurements and explicit episode changes.
3. Demonstrate fresh, frame/clock-bound teacher and visual observations; score the visual claims against separate simulator truth.
4. Demonstrate independent stop delivery and measured safe-state confirmation while the reasoner is stalled; unknown acknowledgement must remain unconfirmed.
5. Complete C06 single-use admission and C09 append-before-dispatch evidence at the final adapter boundary, then test exactly one bounded simulator action before composing the two-step task.

The isolated probe launched a bare hand asset and issued no joint command. The profile in the shadow fixture is a declared candidate, not an executable capability certificate.

## Measured prerequisite result

The accepted isolated run is
`.rrm-artifacts/hand-controller-probe-20260924-s/probe.json`, evaluated by the
fail-closed `rrm/hand_qualification.py`. Its controller target converged to 0.00330 rad
maximum error; all 23 observed joint peaks stayed within unchanged USD limits; three
post-command reset hashes matched; the independent arm interruption reached the
whole-hand safe window in 0.917 s; and raw contact data named the deliberately injected
`red_block` pair after a zero-force baseline. ROS was disconnected and dispatch false.

The injected overlap produced a 489 N transient. It qualifies contact observation only,
not force safety, contact stability, grasp, or placement. The evaluator therefore sets
`ready_for_single_bounded_contact_trial=true` while retaining
`contact_stability_qualified=false`, `grasp_execution_qualified=false`,
`c06_c08_c09_complete=false`, and `execution_dispatch=false`. Gate 5 remains open.

The subsequent CPU-only boundary increment replaces the boolean authorization shortcut
with signed, purpose-scoped, single-use grants and durable consumption. It also restores
consumed grants and dispatch IDs across restart. This advances the C06/C09 contract but
does not close Gate 5: the HMAC verifier is local prototype identity infrastructure,
the gateway is still motion-disabled, and live liveness/stop plus one supervised
non-contact calibration acceptance remain outstanding.

The gateway now also has CPU-only heartbeat, stop-deadline, and external-watchdog
instrumentation with deterministic fake-clock stress coverage. A subsequent live
no-action smoke bound that disabled gateway to the 23-joint articulation and completed
1,000 direct `IDLE` ticks with healthy evidence, an 18.12 us maximum externally
measured tick duration against a 0.1 s limit, and zero action calls. This narrows the
live binding and idle-overhead gap but does not close Gate 4: the probe did not use a
physics callback or step physics after reset, and Isaac scheduler behavior under load,
independent watchdog deployment, and an action-applying stop/hold remain unmeasured.

A follow-up callback smoke verified the live profile, then deliberately removed and
deactivated the asset before stepping so authored drives could not move it. The
disabled gateway received 240/240 Isaac physics callbacks, all `IDLE`, with healthy
liveness, a 3.116 ms maximum callback gap, and zero action calls. A rejected
zero-gravity-only diagnostic had changed joint state by 0.279244 rad, proving that
gravity removal alone is not a no-motion fixture. The accepted inactive-asset result
validates callback wiring only; it still does not close Gate 4 or establish timing with
a dynamic articulation, independent watchdog behavior, or an applied hold.
