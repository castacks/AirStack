# Bounded new-recording and independent-reference intake

17 November 2026 America/New_York. Follows the
[baseline recommendation](baseline-recommendation-2026-11-17.md).
No model, match, geometry or pose-error computation is authorized by this intake.
Recording novelty does not establish sensor-independent physical reference.

## Candidate screen frozen before payload inspection

Existing SmokeRoom/FlashLight/OutdoorNight stay exposed development data. Current
primary-release and extension folder metadata and targeted SubT-MRS source reading
were inspected before this payload plan. The current v5 paper section3.2states
reference trajectories fuse map, visual odometry, LiDAR and IMU constraints.
Consequently independent camera/IMU truth is not established merely by a reference
file. Preserve prior v2 reading separately; v5 is a fresh targeted section review,
not a full-paper or released reference-generator reproduction.

Screen the remaining three unexposed extension candidates in alphabetical order:
BlockVisual, LongCorridor, MultiFloor. Choose none by eventual matcher/error scores.
Freeze observed Drive IDs/sizes from folder metadata before ZIP reads. For each,
use existing strict HTTPS range reader and standard ZIP parser: complete central
directory, member/folder/type counts and metadata. Maximum16MiBtransport per ZIP,
8MiBper request. Fetch only native timestamp/IMU text members≤4MiBuncompressed,
and at most first/last image per visible/thermal camera if both exist. Check each
retrieved member's ZIP CRC and SHA; keep failed requests and no full sensor ZIP.

Use the already-inspected extension reference inventory: LongCorridor and
MultiFloor have small ZIP sidecars; BlockVisual has none in that scoped inventory.
Fetch these two via Drive, verify ZIP CRC/all-member hashes, examine exact column
headers and finite monotonic timestamp/reference conventions. No re-scan of the
closed OutdoorNight listings, fitted axes/clocks or hidden reference assumption.
Metadata-only exclusion for missing modality may stop that candidate; no need to
consume image payloads or calibration for a stream that the complete ZIP lacks.
If both camera streams/reference remain viable, inspect own published calibration
and endpoint/reference-time association before considering a full qualification.
Bounded samples do not certify whole-record decoding, physical exposure metrology,
stop/start/condition coverage or full-span interpolation.

## Decision gates and stop

Report separately: (1)new to this project's model outcomes, (2)actual paired RGB+
thermal availability, (3)supplied reference presence/time/frame association,
(4)sensor-independent physical truth, (5)full-span readiness. A passed transport
or numerical schema check cannot promote gates3–5. Record exact missing evidence.
No already-exposed sequence is relabelled held out; unknown checkpoint-training
membership stays qualified even for a new local recording.

A candidate with paired cameras but no qualified reference is input-only, not an
independent pose-validation recording. A reference-bearing candidate without
thermal cannot answer the paired thermal comparison. Stop this finite screen
with all positive/negative results and retained access failures; do not run a
model or replace a failed candidate with an unbounded search. Next intake may use
a distinct source or separately qualified reference-generation contract.
Save source/member/range receipts, source-to-verdict checks, notebook, findings,
registry/index/handoff and a member-readback-verified incremental local packet.
No upload, outreach, private-data request, training, Git push or automatic commit.
