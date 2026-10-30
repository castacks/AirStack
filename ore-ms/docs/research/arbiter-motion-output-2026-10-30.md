# Arbiter motion output admission

2026-10-30 America/New_York; branch `ore_ms`, implementation base `ffc4c25b2`.

The user requested continuing the agent/layer path while the MACT_Offroad
training-split reply from Yifei remains pending. Training a new checkpoint is a
possible separate project: it can document training membership/preprocessing,
but it does not establish the recording rig, GNSS origin or reference timing.
No new checkpoint training is undertaken here.

## Implemented boundary

`Scripts/Setup/arbiter_motion_output.py` is a separate opt-in adapter for an
existing RGB/THERMAL/ABSTAIN recommendation. It emits a 4x4 relative motion only
if the selected expert has complete supported camera-local evidence, its native
sensor interval matches both requested endpoints exactly, its health and first
motion have been received by the declared controller deadline on the same
controller clock, and a documented verified frame contract names the exact
source/target frames. It validates finite SE(3) geometry and computes
`D_target = T_target_from_camera D_camera inverse(T_target_from_camera)`.
Quaternion inputs use xyz translation followed by xyzw rotation. Only bounded
unit-quaternion serialization roundoff is normalized; no extrinsic is repaired.

Every rejection returns ABSTAIN, a reason and null motion. It preserves the
advisory choice in a separate field and does not choose a different expert on
rejection. Advice-level abstention remains abstention. Frame verification and
its evidence reference are caller attestations, not a proof that the supplied
calibration physically applies. No pose covariance or shared trajectory is
created. A caller must not compose increments across gaps or differing intervals
without a separately justified trajectory policy.

The incremental replay adapter uses the shared controller journal and unchanged
EXP073 availability advice. Each requested interval is explicitly the native RGB
interval; a thermal recommendation with different endpoints is rejected. No
approximate pairing, time shifting, reference endpoint bridge or interpolation
is available. The receipt time is the later controller receipt of the applicable
pair-health and first-motion events. Worker monotonic clocks and sensor epoch
timestamps are retained in source evidence but never subtracted from controller
receipt times. This checks replay-prefix causality, not a live latency budget or
sensor/controller clock synchronization. No new physical verification is inferred.

## Validation

The CPU run used the existing image
`airlab-docker.andrew.cmu.edu/airstack/airstack:v0.20.8_gcs`, Python3.12.3,
pytest7.4.4. The repository's default jaxtyping pytest argument is unavailable in
this image; `-o addopts=` disables that unrelated plugin option for these stdlib
contract tests. The initial default invocation failed at argument parsing; no
tests ran until that correction. Final result: **84 tests pass**, comprising the
new admission/prefix tests and existing pairing, shared-advice and history tests.
Analytic checks cover a rotated translation and the translation induced by a
rotating camera with a two-metre lever arm. Rejection cases include bad support,
nonfinite/reflected/nonrigid geometry, mismatched endpoints/frames/clocks, late
receipt, missing evidence and non-boolean verification. Prefix tests exclude
future results, retain existing choices and test unrelated worker clocks.

Restored the small existing EXP073 recovery archive and verified all107 archive
member SHA256 values before restoring only four top-level replay evidence files.
The timeline matches its saved independent PASS proof. Replaying its19 decisions
retains18RGB recommendations and1ABSTAIN; all19 motion outputs abstain, with
18`unverified_frame_contract` and1`advisory_abstain`. This is the intended boundary
for the currently missing provenance, not a tracking-coverage or pose-accuracy
result. No inference, fitting, new scientific data, reserved capture access,
production default change, external message or quality-gain claim occurred.

Reproduce the stdlib tests from MAC-VO-main in that container:

```bash
python3 -m pytest -o addopts= -q Scripts/UnitTest/test_arbiter_motion_output.py Scripts/UnitTest/test_pair_arbiter_events.py Scripts/UnitTest/test_shared_arbiter_replay.py Scripts/UnitTest/test_arbiter_history.py
python3 -m Scripts.Setup.arbiter_motion_output --run ../results/shared-arbiter-events/run-001 --output /tmp/arbiter-motion-output.json
```

The CLI intentionally supplies no frame contracts and writes a fresh output file.
The Python `admit` API supports externally verified contracts for exact-interval
inputs. Asynchronous sensor-causal transport, a measured live deadline and
trajectory continuity remain separate work; they are not silently approximated
here. A learned quality gate still needs a declared target and recording split;
this admission adapter supplies neither labels nor a trained selector.

Recovery target: `ore_ms:ore-ms/backups/2026-10-30-arbiter-motion-output`.
Contains source, tests, documentation, replay inputs/proofs, test results and
incremental Git history based on `ffc4c25b2`. Prior source history is in
`2026-10-30-onboarding-ffc4c25b2`. No checkpoints are needed for this CPU replay.
