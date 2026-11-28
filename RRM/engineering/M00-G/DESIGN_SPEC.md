# M00-G implementation design

Date started / updated: 2026-11-28T02:05:16.028144+00:00; branch ore_proj; parent dc17fa3918c5e8af12ecabade3cee3efdc223f85.
Overall DONE. All implementation and test evidence remain under RRM.

## Context
M00-F reduced desktop topology approved for offline implementation only. Need concrete self-contained Compose intent, real upstream interface references, explicit unresolved identities, original guard compatibility, observable append-only recorder, reproducible offline tests and fail-closed preflight. No Docker/runtime/device/network operations or protected edits.

## (a) Configuration and provenance — DONE
JSON Compose (YAML subset) without inheritance/includes, fixed private network and no privileges/devices/ports. Images and missing guard commands remain required unresolved fields. Real MAVROS XML interface and upstream Isaac script references, explicit dependencies, strict closed config validation plus provenance hashes. Synthetic positive/negative mutations, identity/checksum drift tests.

## (b) Event recorder and startup semantics — DONE
Stable JSON event schema with native producer clock provenance and recorder receipt clocks, sequence IDs, hash chain and health/terminal checks. Append writer takes file-like sink only, no control APIs. Guard host runtime adapter explicitly blocked rather than substitute Python model for C++. Offline reference model tests subset of deadline/latch ordering, clearly no runtime equivalence. Recovery never resets latch.

## (c) Preflight and continuity — DONE
All14 original IG rows runtime NOT VERIFIED; aggregate BLOCKED for unresolved images/assets/GPU/recorder/host/network. Synthetic positive tests can establish static compliance only. Source/protected/input/pinned history integrity,20 original CHKs/29 mappings unchanged, results reproducible around local commit. Runtime safety never inferred.
