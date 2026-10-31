# Recording exposure before the next evaluation

2026-10-31 America/New_York; branch `ore_ms`, source baseline `0106e156d`.

The [machine-readable ledger](../../experiments/recording-exposure-ledger-2026-10-31.json)
records 20 named entities from this project's existing notes. It distinguishes
project exposure, payload type, reported reserves and checkpoint membership.
This is a scoped historical audit, not a census of every public recording,
CATS scene or alternate release. No scientific payload was opened for this audit;
only existing notes and a current Box root filename/size listing were read.

## What remains available as new evidence

| Scoped pool | Named entities | Development-exposed for future methods | Reported reserves |
|---|---:|---:|---:|
| October 27 TartanDrive archives listed in Box |9|9|0|
| MS² recordings named in prior experiments |9|7|2|
| CATS v2 outdoor scene explicitly inspected |1|1|0|
| STheReO Valley Evening |1|1|0|
| Total scoped entities |20|18|2|

All nine current Box recording ZIP names match the nine ledger identities. The
four archives screened in EXP036 are exposed even though that screen performed
no network inference. Image previews, tracking support and outcome-guided scene
selection are scientific exposure. Changing the clip window or spelling an
archive name differently does not produce an unseen recording. Early/middle
windows from one recording belong to the same partition.

The two MS² records reported unexamined in EXP064 are:

- day `_2021-08-13-16-31-10`;
- night `_2021-08-13-22-16-02`.

These remain **reported candidate reserves**, not newly assigned test records.
The ledger does not inspect their pixels or labels to certify non-access.
Prior small-odometry archive staging is recorded, while per-record reference
inspection is left `not_established_by_recording` for these two IDs. Staging an
archive neither proves nor disproves inspection of a particular member. No
unknown access field is silently converted to “never inspected.” MACT_Offroad
training membership remains unknown for the exact checkpoint; project-unexamined
is distinct from network-training-held-out.

CATS `OUTDOOR/CARS/scene1/` was inspected together with disparity references and
is development material. Its static geometry cannot validate a temporal VO
arbiter. Other scene names and alternate versions are not automatically unused;
they remain unclassified here. STheReO Valley Evening had reference poses
inspected in EXP040 despite image retrieval failing in that audit. For this
whole-recording temporal protocol, reference-only scientific inspection still
counts as exposure. Current STheReO image availability was not rechecked.

Previously prospective MS² results retain their historical meaning for the
methods frozen before those tests. Their captures are now exposed for future
methods. This audit does not retrospectively withdraw those prior declarations,
forbid development reuse or set account access permissions.

## Evidence and reproducibility

Each ledger status links exact source quotes to a repository path, the source
commit and a SHA256-pinned note snapshot. Fifteen existing note snapshots are
preserved under `results/recording-exposure-ledger/run-001/source-snapshots/`.
This avoids circular provenance when the living handoff is subsequently updated.
The audit verifies every source hash and quote, recording/alias uniqueness,
reserved-status evidence, unknown checkpoint membership and the nine-name Box
inventory. Pinned notes are historical project evidence, not an independent
access log of every past filesystem operation. The Box listing proves metadata
presence at audit time, not archive content or global dataset completeness.

`Scripts/Setup/audit_recording_exposure.py` also offers a read-only proposed-
partition diagnostic. Canonical IDs and explicit aliases resolve to the same
record. It reports cross-partition reuse, exposed development material in a
prospective validation/test role, static scenes used for temporal validation,
and unlisted recording/clip aliases. A reported reserve may pass this identity
and exposure-structure check while the complete prospective method/test freeze
and checkpoint-training-held-out support remain false. The tool neither opens
records nor assigns or executes an evaluation.

Fifteen hermetic CPU tests pass. They use bounded synthetic source snapshots
and fixed filename metadata fixtures; no ignored recovery files are required
for unit tests. A separate CLI audit verifies the actual historical snapshots
and captured Box listing. Five read-only software examples demonstrate alias
leakage, image-only exposure, pose-only exposure, unlisted IDs and the distinction
between a reserved candidate and a complete test freeze. These examples are
explicitly not scientific test assignments. Runtime uses Python3.12.3 and
pytest7.4.4 in the existing GCS image; no new package/model is needed.

From MAC-VO-main in that image, with ore-ms mounted at `/workspace`:

```bash
python3 -m pytest -o addopts= -q Scripts/UnitTest/test_recording_exposure.py
python3 -m Scripts.Setup.audit_recording_exposure --root /workspace --ledger /workspace/experiments/recording-exposure-ledger-2026-10-31.json --box-metadata /workspace/results/recording-exposure-ledger/run-001/box-root-metadata.json --output /tmp/fresh-exposure-audit.json
```

To inspect a proposed partition, supply `--assignments` with a JSON list of
`{"recording_id":"<canonical ID or explicit alias>","partition":"development|validation|test"}`
objects. Use a fresh output path. Unlisted IDs remain unknown; update the dated
ledger with evidence rather than interpreting absence as non-exposure.

## Research implication

The existing Box TartanDrive pool supplies no untouched recording-level temporal
test for a new candidate. It can still support implementation, training and
explicitly retrospective checks. A future temporal transfer evaluation would
materially benefit from a distinct whole RGB/thermal recording with both experts
supported and applicable frame/time/reference metadata, frozen before content
inspection. The two named MS² reserves are limited existing candidates; no
additional upload is needed to plan around their recorded metadata.

Before consuming any reserve, a distinct candidate method, development/test
assignment, target, acquisition-clock and output/deadline policy, fixed baselines
and immutable code/model artifacts still need a complete declaration. The
current history rule's known-window fallback result does not establish a new
quality selector. No record is selected or opened by this audit, and no model,
threshold, inference run, reserved capture or production default is changed.

Recovery: `ore_ms:ore-ms/backups/2026-10-31-recording-exposure-ledger`. Contains
source, ledger, pinned note snapshots, Box metadata, audit, notebook tests and
examples, plus incremental Git history requiring `0106e156d`. No original
recording archives, scientific pixels, model weights or credentials are included.
