# Office world/map binding checkpoint — 2026-10-13 EDT

The Office camera mount and same-render RGB/ROS stamp pairing passed their separate
checks on 2026-10-09. World-camera consistency remains blocked: the retained
physical-base versus ROS `map`-base difference was 8.44–8.47 cm and 1.05–1.06°
in that episode, above the frozen 3 cm translation limit. This is a measurement
of one episode, not a reusable world-to-map calibration.

## Source and live-state audit

`robot_interface/src/odometry_conversion.cpp` treats output type `2` as `OVERWRITE`:
it replaces the odometry header and child frame IDs, then republishes the pose and
broadcasts TF. It does not apply a world-to-map transform in that branch. The
current `interface.launch.py` sets output type `2`, `new_frame_id=map`, and
`new_child_frame_id=base_link`; read-only ROS parameter queries on the running
`/robot_1/odometry_conversion/odometry_conversion` node returned those same values.
An eight-second read-only dual-topic sample paired 193 raw and converted odometry
messages by exact source stamp; all paired positions matched exactly (maximum
difference 0 m). Both topics carried `map`/`base_link` frame labels already. Thus
the converter adds no measured world alignment, and the published `map` name does
not independently prove that simulator world coordinates and MAVROS local odometry
are aligned. This audit does not identify the cause of the retained residual.

The running command console reported `scene_context_status=UNKNOWN`, no active
scene binding, and mission `IDLE`/inactive. Its read-only camera refresh returned
one 480×300 `camera_left` image at ROS source stamp 305449993172 ns and near-ground
map odometry (z≈0.003 m). Visual inspection showed floor and wall only; this view
cannot establish either Office marker's visibility. The console remains unbound.

## Next bounded diagnostic

Acquire a fresh same-episode, read-only pose set while the vehicle is grounded:

1. Record the actual Isaac world rigid-body and rendered camera poses, the camera
   mount transform, raw MAVROS local odometry, converted `map` odometry, and the
   exact-time `map -> base_link` and base-to-camera TF. Retain source stamps,
   receipt times, simulator epoch, frame IDs, source/image hashes and collector
   identity.
2. Check whether raw and converted odometry positions match at identical source
   stamps, and separate any odometry-to-TF disagreement from simulator-world-to-raw
   odometry disagreement. Reject unmatched epochs or stale samples.
3. Report world-base versus map-base residuals separately. Compose the measured
   camera poses/transforms and compare the resulting world-camera residual with
   the already frozen 3 cm/2° world-camera consistency bounds. Report repeated
   grounded samples and uncertainty. Do not infer a transform from a single pose
   or silently raise the bound.
4. If a stable frame transform is supported by independent measurements across
   episodes and poses, propose its ownership and validation path for review before
   applying it to target coordinates or console admission.

No map transform, visual localization fact, motion plan or flight authority follows
from this checkpoint. The seven Office cases remain `PREPARED_NOT_RUN`.

## Grounded capture result — 2026-10-13 EDT

After explicit approval, only the inner Isaac service was recreated with the
Office marker launcher, scale 1.0 and the opt-in observation directory. The robot
service was recreated once to recover the new simulator clock epoch. The seven
readiness gates passed; ROS state reported connected, disarmed and landed, and the
console mission remained idle. GCS was not restarted and no task action was sent.

The requestable Office writer acquired three frames in episode
`b1214ecd5917422f906fb2d4876c6e66`, detached at its three-frame limit, and
reported no error. Each render simulation stamp had exact-stamp raw MAVROS odometry,
converted odometry and `map -> base_link` TF in the concurrent ROS trace. All three
raw/converted positions and converted/TF poses matched exactly. The static ROS
`world -> map` TF was identity. This is a ROS frame declaration, not independent
proof that the simulator rigid body and PX4 estimate share an origin.

Rendered camera versus composed ROS camera translation residuals were 8.47, 8.51
and 8.54 cm; rotations were about 1.48°. The first and third physical body reads
matched render simulation time; the second was one 30 ms step later. Rendered
camera versus physical-body-plus-mount residuals were about 2 µm and 0.00054°.
Thus the camera mount still passes, while the frozen 3 cm world-camera translation
gate still fails. The ROS image subscriber began after these three native frames,
so this capture does not independently re-establish native-to-ROS image equality.

A separate bounded physical recorder in the same uninterrupted simulator run
retained 162 grounded samples. Of these, 77 had exact numeric
simulation/odometry/TF stamps over 14.07 s.
Across those matched samples, physical-body versus ROS `map`-base translation was
7.06–9.14 cm (median 7.79 cm) and rotation was 1.67–1.80° (median 1.76°).
The physical recorder does not embed the camera writer's episode ID in each line;
run identity follows the unchanged container and advancing clock, so this is not
an independently encoded episode join. Physical speed was at most 0.000020 m/s
in the matched records; all 20 sampled MAVROS states were connected/disarmed and
all 97 sampled extended states reported landed. Exact numeric stamps do not prove
equal sensor-acquisition times; these
readings are a grounded residual distribution, not an estimator root-cause or a
calibration. The recorder stopped with `request_disabled`, 162 records, no error.

Two later read-only GUI camera refreshes advanced in source stamp and image hash;
the second visibly contained both Office markers. Those later GUI images are not
the three native teacher frames. Saved console state/mission snapshots still report
scene context `UNKNOWN` and mission `IDLE`/inactive.
The seven Office cases remain `PREPARED_NOT_RUN`; no localization or execution
authority was granted. Raw records, collector source, analysis scripts, status and
checksums are in the local gitignored
`.rrm-artifacts/world-map-capture-20261013/` bundle. This workspace bundle still
needs separate durable export before the OSMO task is replaced.
