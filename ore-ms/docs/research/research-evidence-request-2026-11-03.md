# Research evidence request — draft only, not sent

2026-11-03 / ore_ms. Supports EXP127 evidence inventory. No new recording or
reserved data access is requested automatically by this document.

## 1. Existing TD3 intervals: reference metadata and quality, not another image ZIP

For the already exposed recordings below, can the recording-specific rig/TF
configuration, reference publisher configuration/code and original reference
quality fields be provided, if retained?

- 20251027/0_salon_so_day
- 20251027/1_salon_mac_cov_day
- 20251027/night_mac_salon_warehousefig8
- 20251027/night_thermal_so_salon2

The bounded first/last recorded stereo source groups for every audited window
are listed in results/exposed-data-adequacy/run-001/reference-request-intervals.json.
Those recorded timestamps are identifiers, not attested physical exposure times.
Please identify applicable calibration/mounting revisions and the actual body,
rectified optical, GNSS antenna/solution and IMU frames, including any lever-arm
compensation already applied. Which exact signed axes/orientation/UTM mapping and
acquisition/publisher timestamp relation does filtered_odom use? Which selected
intervals have saved solution type/fix/RTK status and covariance? Are receiver
or reference aiding inputs shared with the evaluated estimator?

The existing13-field export omits covariance/receiver quality. A bounded original
message/config export for these intervals, or a source explanation when not saved,
is useful. Raw bag timestamps and quality streams are preferable to an inferred
coordinate/time fit against evaluated VO errors. Missing evidence stays unknown;
this is not a demand to manufacture exact physical truth from unavailable logs.

## 2. MS2: authoritative provided-reference interpretation

Can the relevant reference-generation/export description identify camera-to-world
versus world-to-camera convention, rectification/camera anchor, timestamp-to-pair
association, units and available validity/quality flags for the already exposed
sequences? Keep the existing day002018 reference anomaly and its neighboring
interval exclusions explicit. Do not choose convention or scale by lower VO error.

This can support a clearly scoped provided-reference benchmark; independent
physical truth is a separate claim. Exact MACT_Offroad/MACVO checkpoint recording
lists, base/finetuning membership and preprocessing recipe would clarify overlap,
but unknown membership alone need not prevent a narrower project benchmark.

## 3. Existing unused-by-this-native-survey recordings before another upload

Five Box TD3archives are already project-exposed but outside the paired-native
inventory. Their ZIP directories now verify GNSS/IMU/TF export names; native
availability remains unassessed:

- night_macthermal_salon_redcourse_waypointskip1
- night_so_waypoint_skip2
- night_teleop_so_salon_fig8
- night_thermal_so_salon
- night_thermal_so_salon3

No additional image upload is needed to begin a separately predeclared bounded
development survey of these existing sources. Directory searches found no explicit
RTK/fix/covariance-named exports; they did not inspect generic numeric members or
prove original quality messages absent. Recording-specific quality and reference
interpretation remain useful source inputs. All five stay development-exposed.

## 4. If offering a different temporal recording, metadata first

The audited dense cohort has no complementary availability headroom beyond one
fixed camera within any recording. Metadata for a separate recording with changing
lighting/visibility or other intended sensor challenges, both stereo streams,
sustained fail/return opportunities and appropriate motion-reference provenance
would be more useful than more clips from these same development recordings.
Provide recording identity, duration, rig/calibration and available reference/
quality streams first. Design whole-recording development/validation membership
and candidate/input/deadline criteria before inspecting confirmation payloads.
A recording inspected to find a useful condition becomes discovery-exposed.

The two protected MS2recordings remain closed. No email, Slack message or request
has been sent, and no test split or new download is authorized by this draft.
