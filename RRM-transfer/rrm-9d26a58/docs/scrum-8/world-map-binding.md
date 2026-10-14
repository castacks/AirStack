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

## Prior bounded diagnostic plan

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
equal sensor-acquisition times; these readings are a grounded residual
distribution, not an estimator root-cause or a calibration. The recorder stopped
with `request_disabled`, 162 records, no error.

Two later read-only GUI camera refreshes advanced in source stamp and image hash;
the second visibly contained both Office markers. Those later GUI images are not
the three native teacher frames. Saved console state/mission snapshots still report
scene context `UNKNOWN` and mission `IDLE`/inactive.
The seven Office cases remain `PREPARED_NOT_RUN`; no localization or execution
authority was granted. Raw records, collector source, analysis scripts, status and
checksums are in the local gitignored
`.rrm-artifacts/world-map-capture-20261013/` bundle. This workspace bundle still
needs separate durable export before the OSMO task is replaced.

## Fresh-boot estimator-origin and repeatability check — 2026-10-13 EDT

With approval for the next grounded diagnostic, a second Office/scale-1.0 simulator
boot used the same pinned measurement source and image. Only inner Isaac was
recreated, then robot once to join its clock epoch. All seven readiness gates
passed. GCS stayed up; ROS state samples were connected/disarmed/landed, and
no control or mission action was dispatched. The native writer acquired three
frames and detached cleanly. Two later GUI camera refreshes advanced in stamp
and image hash and visibly showed both markers. Console binding remained
`UNKNOWN`, mission `IDLE`.

At native render stamps 65.780, 65.810 and 65.840 s, rendered-versus-composed-ROS
camera translation residuals were 6.257, 6.227 and 6.197 cm, with rotation about
1.27°. All exceed the frozen 3 cm translation gate. Rendered camera versus
physical-body-plus-mount translation stayed about 2 µm. Raw MAVROS and converted
odom matched, as did converted odom and map-base TF, at the paired stamps. All
three ROS image *headers* matched the native simulation stamps; this run did not
independently check RGB pixel equality. The second physical read was one 30 ms
step after its render; the other two matched render simulation time.

| Grounded comparison | Exact numeric pairs | Physical versus ROS map-base translation | Rotation |
| --- | ---: | --- | --- |
| First boot, later trace | 77 over 14.07 s | 7.06–9.14 cm; median 7.79 cm | 1.67–1.80°; median 1.76° |
| Second boot, early trace | 64 over 15.72 s | 5.50–8.66 cm; median 7.48 cm | 1.09–1.28°; median 1.14° |
| Second boot, late trace | 63 over 11.43 s | 5.37–7.44 cm; median 6.23 cm | 0.37–0.46°; median 0.43° |

The late trace was at simulator time about 5127–5139 s in the same second boot,
not at a matched elapsed time in the first. Across these grounded traces, measured
physical speed was at most 0.000020 m/s. The physical body location at the two
native captures was nearly identical, while converted odometry position differed
between boots. That supports an episode-dependent estimated/local-pose offset;
it does not identify a fixed world/map transform or prove the estimator's cause.
The first boot's native camera residual was 8.47–8.54 cm; the second boot's was
6.20–6.26 cm. A single stationary pose per boot cannot establish a transform's
rotation, behavior under motion, or repeatability across poses.

Source inspection shows the active odometry converter's `OVERWRITE` mode only
relabels incoming MAVROS odometry as `map`/`base_link`. The launch file declares
identity ROS `world -> map`. PX4 EKF2 labels its published local position as the
body origin in local NED; the active `EKF2_HGT_REF` readback was `1` (GPS) and
`EKF2_GPS_CTRL` was `7`. Pegasus GPS simulation derives altitude from physical
height plus configured origin altitude, with noise and bias. These facts make a
local estimator origin a plausible contributor, not a measured causal diagnosis.
The read-only home-position sample is near local zero; no origin-setting parameter
or frame transform was changed.

The second boot's concurrent ROS recorder missed the static `world -> map` TF at
startup. A later read-only `tf2_echo` in the same run observed identity; it is
explicitly labeled as later same-run evidence. Physical recorder rows do not embed
the camera writer episode ID; linkage again uses the uninterrupted container and
clock. Exact numeric stamp matches are not independent proof of equal acquisition
time. No multi-pose calibration, flight, inference or console admission occurred.
The seven Office cases remain `PREPARED_NOT_RUN`.

Raw records, GUI images, source and analysis files, and a SHA256 manifest are in
the local gitignored `.rrm-artifacts/world-map-episode-b-20261013/` bundle. Both
workspace capture bundles still need separate durable export before workflow
replacement. A decision about world/map ownership or target coordinates should
wait for independent multi-pose evidence and review of the proposed contract.
