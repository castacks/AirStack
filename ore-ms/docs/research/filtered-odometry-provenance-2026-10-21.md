# Filtered-odometry provenance: historical lead, recording verification pending
**Owner follow-up received 2026-10-22:** [attributed replies and updated status](source-replies-2026-10-22.md) identify the stream as direct GNSS output in approximately described UTM axes. MSA applicability has owner support; non-use of GNSS odometry by SO is tentatively reported. Exact axes, quality/RTK, timing and reference origin remain open. The original audit below is historical and is not the latest provenance assessment.

Checked2026-10-21 (America/New_York), branch `ore_ms`, base `391a70fcb`. This is a source audit following [EXP-028](../../experiments/EXP-028-reference-timing.md), not a new accuracy experiment. **Public primary sources support a NovAtel GPS/IMU origin as the historical lead. They do not identify the producer, sensor dependencies, clock behavior, quality or physical frame of the October2025 recording. Filtered odometry remains an unvalidated auxiliary stream.**

## What the primary sources establish

The [TartanDrive2.0 paper, III-C3](https://arxiv.org/html/2402.01913v1#S3.SS3.SSS3) describes a NovAtel PROPAK-V3-RT2i supplying100Hz IMU data and a50Hz pose estimate from GPS/IMU fusion. III-D3 separately describes the Super Odometry product. This distinguishes the historical platform's GPS/IMU pose source from its SLAM product; it does not establish how either October2025 topic was generated or whether they share sensor inputs.

A [CMU lab report by Kelvin Shen, September8,2022, Table1](https://mrsdprojects.ri.cmu.edu/2022teami/wp-content/uploads/sites/62/2022/10/TeamI_shihaosh_ILR06.pdf) attributes `/odom`, `/odometry/filtered_odom` and `/novatel/imu/data` to the legacy package `physics_atv_ws/src/novatel_span` on `desktop-legacy`. The same table lists `/novatel/imu/data` as a Super Odometry input. This supplies a concrete repository/package lead and historical shared-IMU evidence; it is not proof of the2025 graph or independent reference errors.

The lab's pinned [metadata configuration](https://github.com/castacks/metadata_utils/blob/f64458797b40bd97f9eecf4cb38e77603367efea/config.yaml) groups filtered odometry and NovAtel IMU under the NovAtel sensor label. Its [topic monitor](https://github.com/castacks/metadata_utils/blob/f64458797b40bd97f9eecf4cb38e77603367efea/topics_monitor.py) computes speed from supplied twist, checks timestamp proximity and labels coordinate values using simple regional bounds. Its positive coordinate indicator is not a fix-quality, covariance or accuracy certificate. The supplied recording contains pose/twist arrays but no saved monitor outcome certifying this interval.

## Candidate code: useful questions, no identified producer

The public ROS [SPAN publisher at4dfd8830b](https://github.com/ros-drivers/novatel_span_driver/blob/4dfd8830b73cf8b4d9ac8d6148b0de3f6990e9b9/novatel_span_driver/src/novatel_span_driver/publisher.py) repackages receiver messages. Its defaults publish `navsat/odom` with `odom_combined -> base_link`, unlike the recorded `map -> gps_frame`. Parameters/remaps or a custom fork could change those values, but none has been recovered for this recording. This repository must not be substituted for the lab's `novatel_span` package solely because their names resemble each other.

The candidate assigns odometry and IMU stamps using ROS time, and its INSCOV callback leaves that covariance log unused. Thus identifying a SPAN receiver alone would not prove receiver-time timestamps or meaningful exported covariance. These are candidate-code behaviors, not diagnoses of our data.

Its [example launch](https://github.com/ros-drivers/novatel_span_driver/blob/4dfd8830b73cf8b4d9ac8d6148b0de3f6990e9b9/novatel_span_driver/launch/example.launch) enables external velocity aiding by default, while [the aiding handler](https://github.com/ros-drivers/novatel_span_driver/blob/4dfd8830b73cf8b4d9ac8d6148b0de3f6990e9b9/novatel_span_driver/src/novatel_span_driver/wheel_velocity.py) subscribes to an odometry input. Actual aiding configuration and its source must therefore be checked rather than assuming every receiver-based output depends only on GNSS and IMU. This does not establish aiding or SLAM feedback in the October2025 recording.

A different public Yamaha-related [dual-EKF launch](https://github.com/weihsinc/robot_localization/blob/50e105001eafbc61f820d36e1e0ea1a2e4493776/launch/dual_ekf.launch) also uses `odometry/filtered_odom`; its [local filter parameters](https://github.com/weihsinc/robot_localization/blob/50e105001eafbc61f820d36e1e0ea1a2e4493776/params/dual_ekf.yaml) consume velocity and IMU, while GPS is configured for the separately named global filter. Its frames differ from our metadata. It is a counterexample to inferring a producer from a topic string, not evidence that this EKF ran in our data.

## Recording-specific evidence and missing fields

| Question | What the supplied October2025 exports support | Still required |
|---|---|---|
| Topic binding | Converter config and recording statistics associate `novatel_gps_odom` with `/odometry/filtered_odom`;10,168 source samples | Actual publishing node/package and commit, launch/remaps/runtime parameters |
| Sensor dependencies | Historical sources suggest NovAtel GPS/IMU; exact2025 inputs unverified | Receiver model/config; external aiding; whether any SLAM-derived velocity/pose is consumed; shared IMU identity |
| Timing | Exported timestamps join consistently in EXP-028 | Stamp assignment in actual driver; receiver vs ROS/bag time, clock synchronization and latency |
| Quality |13 columns contain position, quaternion, linear/angular twist | Original odometry covariance and applicable receiver fix/solution status/uncertainty for the interval |
| Physical origin |Metadata says `map -> gps_frame`; supplied calibration names `novatel/imu_frame` separately | Meaning of gps_frame; applicable gps_frame-to-vehicle transform and receiver lever-arm configuration |
| Motion state |Late auxiliary displacement0.0161m; thermal VO0.0085m; SLAM vehicle0.2608m; zero mux/teleop requests | Acquisition/operator context or independently justified motion reference |

No `gps_frame` transform was found in the restored night TF/calibration files inspected. Absence from this subset does not prove it was absent on the vehicle or in the original bags. Do not equate `gps_frame` with the named IMU frame, infer axes from large coordinate values, or adopt a different platform's static transform.

The [recording-derived EXP-028 evidence](../../experiments/EXP-028-reference-timing.md) remains intact. All402 frozen scientific input hashes are unchanged. No inference, reference replacement, alignment fitting, calibration change or new numerical test was performed for this documentation audit.

## User-supplied ATV workspace lead

The user supplied a Slack-search summary identifying a February2024 discussion
by Matthew Sivaprakasam about ATV RViz mesh/launch files, with a follow-up path
`/home/physics_atv/physics_atv_ws/`. The summary suggests Matthew or Wenshan in
`#learningphysics` as possible contacts. This is a reported historical workspace
lead: the original Slack messages/links were not independently retrieved, the
driver package was not identified, and applicability to October2025 remains
unconfirmed. The supplied search summary itself found no definitive driver or
recording-specific messages.

The reported path is absent from this compute workspace. A bounded local search
under `/home`, `/mnt`, `/opt` and the AirStack checkout also found no ATV workspace
or NovAtel package. This does not establish that the path is absent on the ATV
computer. No remote host, login or mount is inferred from a local filesystem path.

On the actual ATV computer, these read-only searches can locate candidate files:

```bash
find /home/physics_atv/physics_atv_ws/src -name package.xml -print | rg -i 'novatel|gps|gnss'
rg -l --glob '*.launch' --glob '*.yaml' --glob '*.py' --glob '*.cpp' \
  'filtered_odom|novatel_span|gps_frame' /home/physics_atv/physics_atv_ws/src
```

Then identify the owning repository/commit and launch/remap chain. A present-day
checkout alone still does not prove which revision/configuration ran on
October27,2025. The first focused request is to confirm whether this workspace
held the recording's driver and identify that specific revision/configuration.

## Smallest useful source request

For `20251027/night_thermal_so_salon2`, focus on **2025-10-27 22:13:40.207–22:13:42.107 EDT**, equivalently **2025-10-28 02:13:40.207–02:13:42.107 UTC**:

1. Repository/commit and launch/config that published `/odometry/filtered_odom`, starting with the legacy `novatel_span` package lead if applicable. Include remaps, timestamp code and any velocity/odometry aiding input.
2. Applicable `gps_frame` definition/vehicle transform and receiver lever-arm/orientation setup.
3. Original odometry messages with covariance plus receiver solution/fix status or equivalent quality logs, if recorded, spanning this short interval with a few seconds of context. A source-only explanation is useful if those messages were not saved.
4. Whether the vehicle was stopped/creeping, and whether the two odometry products consume the same IMU or any shared SLAM estimate.

A repository URL/local path is the first missing item. A GitHub integration was offered but is not installed/connected in this session; account access to the actual ATV source is unconfirmed. Connected mail searches returned no matches, which does not establish that correspondence does not exist. No request or other message has been sent to anyone.

## Preservation

Pinned source URLs, commits and SHA256 hashes for nine public files are recorded under `results/filtered-odometry-provenance/source-registry.json`; primary publications are linked, with no full copyrighted publication copied into recovery. `verification.json` records remote source/hash equality and all402 unchanged scientific hashes. The source audit, local request and incremental Git bundle are preserved in Box `ore-ms/backups/2026-10-21-filtered-odom-provenance`, requiring the earlier project/EXP-028 recovery context.

This completes the available-source audit. Recording-specific producer/accuracy verification is pending the requested source; no claim of independent ground truth is made.
