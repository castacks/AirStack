# Static Isaac geometry receipt R1

27 November 2026 UTC. AirStack base `d6f08c62735c7931d08a233c73ba72eedd56605b`. **The procedure and offline verifier are ready for review. Actual capture remains unqualified and requires separate approval.** Synthetic receipts passed the checks below. No USD stage, live graph, simulator or model ran or changed. [Offline adapter](OFFLINE_GEOMETRY_M08_ADAPTER_R1.md) · [Isaac audit](ISAAC_SIM_CAPABILITY_AUDIT_R1.md) · [Isaac entry gates](isaac-readiness/ISAAC_ENTRY_CRITERIA.md)

## Fixture specification

The NVIDIA Default Environment has a retained ambient load receipt. The proposed central ray has no surveyed 10 m first-hit surface there, and the retained records cannot exclude nearer geometry. Use an isolated stage with one camera and one opaque planar target. This specifies the first hit without editing unrelated scene geometry. The [fixture specification](static-isaac-receipt/fixture.json) is descriptive JSON.

| Item | Frozen R1 choice |
| --- | --- |
| Stage | Local self-contained USD with metres per unit 1, Z up and a `/World` default prim. Use Isaac Sim 5.1.0. Pin the exact image, AirStack/Pegasus revisions, renderer, root USD and dependency closure, and exporter in the capture lock. |
| Camera | `/World/ReceiptCamera` at `[0,0,1]` m in `ore_stage`. In ROS optical frame `ore_camera_optical`, X right = stage +X, Y down = stage −Z, and Z forward = stage +Y. Author +90° X rotation for the USD camera, whose local +Y is up and −Z is forward. |
| Intrinsics | 33 × 25 pinhole, `fx=fy=20` px, `cx=16, cy=12`, zero-based integer pixel centers, zero distortion and clips at 0.1 m and 16 m. Candidate USD focal length and horizontal/vertical apertures are 20, 33 and 25 in consistent camera units. Confirm the effective projection from camera metadata. |
| Target | `/World/ReceiptTarget`, a quad with vertices `[-2,10,-1], [2,10,-1], [2,10,3], [-2,10,3]` m in that order. Center `[0,10,1]`, normal `[0,-1,0]`, half-extents 2 m in X/Z. This is the only rendered surface. Exclude ground planes and other mesh prims. |
| Rays | Center pixel `(16,12)` at camera origin with normalized stage direction `[0,1,0]` and 10 m range. Off-axis pixel `(18,12)` has optical direction proportional to `[0.1,0,1]`, 10 m axial depth and `sqrt(101)` m ray range. |
| Clock | One epoch, 100,000,000 ns sampling period, issue tick 10, target tick 30. Retain the complete 21-sample simulation-clock sequence. Camera and target remain stationary. |

[NVIDIA pinhole and USD axes](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/action_and_event_data_generation/ext_replicator-object/camera.html) · [M11 fixture conventions](isaac-readiness/GEOMETRY_FIXTURE_PROTOCOL.md)

## Capture procedure for a separately authorized run

1. Materialize the two-prim USD recipe in scratch and pin the resolved geometry and asset closure. Use a fixed pinhole, one render product, and a local capture graph without a robot backend. Survey the resolved target and camera independently of the renderer. Record raw survey, stage and exporter. Use fixed render/readout settings throughout. Freeze the lock inputs and recipe before collecting the final 10 m observations.
2. In isolated preflight variants at 5 m and 15 m, retain axial center-depth samples on the same plane orientation and camera settings. Survey both control ranges and check the rendered target instance IDs. Preserve the raw preflight evidence and the two measurements. Use the pinned 10 m stage for the final capture. Record the final lock digest through the approval record independently of the receipt.
3. Step the approved stage through a single epoch at 100 ms observation cadence. Record raw `/clock` from Isaac simulation time and the stamped camera metadata, depth and instance-ID render product at ticks 10 and 30. All samples for a tick must come from the same rendered frame. Resolve both sampled IDs to the target prim. Query `ore_stage → ore_camera_optical` TF at each exact stamp and retain the resolved transform and authored camera pose. A dropped clock sample, ambiguous frame association or unproved clock source blocks the receipt. Wall time serves no role. [Isaac ROS clock](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_clock.html) · [Isaac camera ROS integration](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera_publishing.html)
4. Normalize the observations under the [receipt and lock schema](static-isaac-receipt/receipt.schema.json). The lock contains hashed `stage`, `exporter`, `preflight` and any stage dependency files. Preserve the full survey and render settings in preflight. The receipt identifies hashed `clock`, `issue` and `target` artifacts. Paths are relative to each JSON's directory. Raw artifacts must let an independent reviewer reconstruct the normalized observations, lookup results, and data source.
5. Run the offline verifier from the checkout root with the trusted lock digest. It verifies hashes and normalized values. It does not decode USD, ROS bags or render buffers. An independent reviewer checks the capture extraction, resolved asset closure, stage survey and clock source against the pinned files before accepting a real receipt.

```bash
python3 ore-ms/ore-ms-research/static-isaac-receipt/verify.py \
  /path/to/receipt.json /path/to/lock.json \
  --expected-lock-sha256 "$APPROVED_LOCK_SHA256"
```

## Geometry, timing and adapter admission

The [verifier](static-isaac-receipt/verify.py) rebuilds optical rays from intrinsics and the sampled pose. It independently intersects the surveyed quad using a binary64 plane solve and checks positive interior hits. Pose, TF and ray comparisons permit roundoff of 64 binary64 ULPs at the 10 m scale. The single-surface premise depends on the witnessed stage survey. Unknown transforms, extra renderable surfaces and unresolved first-hit IDs invalidate the receipt.

For the second check, `distance_to_image_plane` contributes binary32 axial depth and an associated instance ID. Convert axial depth to range using the norm of the unnormalized optical ray. The off-axis sample detects an axial/ray-range swap even when the center agrees. Use the larger control residual plus one binary32 ULP at that control, then add one ULP at the observed sample, convert that allowance to ray range, and add the binary64 roundoff allowance. Reject a derived range allowance above the previously specified M11 1 mm ceiling and any measured disagreement outside the frozen allowance. [NVIDIA camera depth](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.sensors.camera/docs/index.html) · [M11 range tolerance](isaac-readiness/GEOMETRY_FIXTURE_PROTOCOL.md)

The admitted adapter input is the verified exact surveyed 10 m path, rebased to `receiver_origin_x_forward` with `receiver_m=[0,0,0]` and `source_m=[10,0,0]`, plus the integer issue tick and period. The verifier calls the existing adapter's fixture loader and path-length check. Render depth corroborates the surveyed path. It is never rounded into the M08 input. The issue binding excludes target measurements, while the full receipt must pass at both ticks. It emits no detector codes and does not invoke the forecast. Target metadata stays outside the existing M08 issue CSV.

Canonical output is sorted-key compact UTF-8 JSON with a trailing newline and finite values. The result gives SHA-256 of the canonical receipt record and of the issue-only binding. The trusted lock SHA-256 instead pins exact lock file bytes. Normal mode accepts only `isaac_capture`; synthetic receipts require the explicit test flag.

## Verification and approval boundary

Seven deterministic synthetic tests passed. They cover nominal receipt and M08 geometry, future-only changes with invariant issue binding, wrong stamps and TF, incorrect transforms and rays, unsupported depth convention, wrong first-hit ID and control range, and changed stage/provenance. The receipt and lock matched the JSON Schema. Frozen M05–M08 manifests and documentation checks passed.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  ore-ms/ore-ms-research/static-isaac-receipt/test_verify.py -v
```

**Approval is required** to materialize the isolated USD and exporter, add its capture graph, launch and step the simulator, acquire the 5 m and 15 m controls, and collect the two 10 m frames. The remaining dependencies are those pinned assets, the approved image/renderer, demonstrated 100 ms clock cadence, measured depth and instance-ID receipts, and independent extraction review. This procedure does not require attaching to the running OSMO vehicle stack.
