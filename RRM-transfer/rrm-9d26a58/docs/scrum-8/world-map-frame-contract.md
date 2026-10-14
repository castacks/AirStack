# Proposed Office frame contract — 2026-10-14 EDT

Status: **frame design for review; target provenance gate implemented**. This proposal
follows the [grounded world/map measurements](world-map-binding.md).

## What the measurement says

The ROS `world -> map` broadcaster publishes a valid identity TF message. It does
not measure a relationship to the Isaac stage. Across fresh grounded Office boots,
moving the physical spawn 0.75 m along X or Y left MAVROS local position near its
boot origin; rendered-versus-ROS camera error became 73–76 cm. Thus the error is
**using the ROS identity as if `world` meant Isaac physical world**, not malformed
TF bytes. The separate stationary boots do not establish a transform valid during
motion. No world-to-map correction has been fitted or applied.

## Frame meanings and authority

| Name | Meaning in this proposal | Allowed use |
| --- | --- | --- |
| `map` | Current PX4/MAVROS estimator-local coordinates, exposed by odometry and `map -> base_link`. Its origin and orientation must be treated as episode dependent. | Existing robot control and map-frame plans, subject to their own safety gates. It is not automatically aligned with Isaac stage coordinates. |
| ROS `world` | Current shared-launch compatibility parent of `map`, published with identity. It is an alias in the live ROS graph, **not** an independently measured global or Isaac frame. | Existing ROS consumers pending a separate stack-wide migration. RRM may not use this edge to admit simulator-world targets. |
| `isaac_world` | Explicit name for privileged stage/rigid-body truth and authored marker positions in evaluation records. It is a data provenance label, not a new live TF edge. | Offline assessment and diagnostics only. It cannot supply live localization facts or command waypoints. |
| `base_link`, `camera_left` | Vehicle and sensor frames under the published odometry/robot model. | Pose composition only with timestamps and a declared parent frame. The camera mount check is separate from world/map binding. |

The Office marker fixture is authored in Isaac stage coordinates. Proposed
evidence would label those coordinates `isaac_world`. The current
`DroneNavigationTarget` has waypoint values but no source-frame/provenance field;
its bridge emits them as `map`. Therefore explicit rejection of teacher/world
coordinates at this boundary is **new work**, not a protection already supplied
by that model. The existing `missing_map_binding` hold applies when no target is
configured; it cannot detect a misframed configured target.

## Proposed boundary

1. Keep marker identity, image regions, and privileged stage coordinates as
   separate facts. Proposed teacher-position records must carry
   `frame_id=isaac_world`, scene revision, episode ID, source stamp, producer
   hash and `authority=EVALUATION_ONLY`.
2. Treat any candidate `T_isaac_world_map` as an explicit, episode-scoped evidence
   record with source/target frames, clock epoch, estimator reset generation,
   method, source hashes, validity interval, uncertainty and held-out residuals.
   Default binding state is `UNBOUND`; `CANDIDATE` is diagnostic only.
3. A **diagnostic** bridge may use Isaac physical truth to test the geometry but
   stays outside the action/console path. An **operational** bridge needs
   independent vehicle-available evidence for both the vehicle frame and the
   target location; simulator truth alone cannot establish C02 `localized=true`
   or supply C07 waypoints.
4. Invalidate either bridge on simulator or estimator epoch change, scene/fixture
   revision change, stale source stamp, loss of the independent source, or failed
   residual check. Never reuse a constant translation from a prior boot.
5. Keep C02 localization, scene binding, numerical feasibility, C06 safety
   admission and effect verification as separate gates. A geometrically plausible
   transform does not make a learned identity or inspection effect true.

For pose matrices using column vectors, each independent pose pair would imply
`T_isaac_world_map = T_isaac_world_base × inverse(T_map_base)`. Agreement among
pose pairs and held-out poses must be measured; computing this expression once is
not validation. The present grounded captures contain one stationary pose per
boot, so this operation is not authorized as a calibrated transform.

## Verification required before implementation or admission

- Obtain multiple non-collinear physical poses and differing headings **within
  one clock/estimator episode**, with independent world and local pose readings,
  native camera/ROS stamp and source hashes. Prior Office takeoff drift means a
  flight-based collection needs its own bounded control/recovery qualification;
  this design does not authorize a flight.
- Reserve poses and camera frames that are never used to derive a candidate
  transform. On these held-out samples, apply the existing frozen 3 cm / 2°
  world-camera gate and report distributions, age, failures and estimator resets.
  Do not relax the gate or declare success from one coincident pose.
- Check whether source covariance and observed residuals support a validity
  window under actual motion; reject an unstable bridge. Repeat in a fresh boot
  to verify invalidation and reacquisition rather than reuse.
- For an operational bridge, separately prove the localization source does not
  receive privileged teacher/rigid-body truth and that missing or stale binding
  leaves `DroneDecisionStatus.HOLD` / console scene binding `UNKNOWN`.

The seven Office evaluation cases stay `PREPARED_NOT_RUN` until their own live
observation, localization, execution and outcome gates pass.

## Source impact and migration order

`robot.launch.xml` publishes identity `world -> map` in the shared preamble for
every stack. The active odometry converter's `OVERWRITE` mode changes labels, not
pose coordinates. The current trajectory controller configuration targets `map`;
RRM's live observation checks require `map`/`base_link`, and its drone bridge
emits map-frame waypoints even though `DroneNavigationTarget` lacks source-frame
provenance. Changing or removing the shared identity TF is therefore a separate
stack-wide compatibility decision, not a local Office patch.

Two legacy Office entry points need explicit treatment: `rrm_import_office.py`
and `rrm_authorized_live_mission.py` build `DroneNavigationTarget` from manifest
`map_waypoint`/`map_route` values. The reviewed scene manifest contains a blue
marker `map_waypoint` but no measured world/map binding or waypoint provenance.
The implementation should restrict these values to explicitly labeled historical
fixture/replay use until a reviewed current-episode binding exists. Their presence
does not prove current-episode map localization.

The first implementation slice should add typed frame/provenance checks at the RRM
teacher-to-target boundary, cover both legacy manifest import paths, and add
diagnostic bridge records **without** publishing a new live TF edge or changing
robot control. A later migration can audit every
consumer of ROS `world`, decide whether to retire its alias, and validate the full
stack before altering the shared launcher. None of these steps should promote
simulator truth to operational authority by default.

The target-provenance slice now requires a live navigation binding to match the
current episode, `map` frame, a fresh SENSOR localization fact, and a fresh SENSOR
fact committing to the exact waypoint route hash. The legacy manifest loader
labels routes `FIXTURE_REPLAY`; the live compiler holds them, and the standalone
dispatcher refuses a replay-labeled proposal. This is an admission contract,
not proof that a map localization producer exists yet. The offline importer
continues to emit a labeled replay proposal for historical evaluation.
