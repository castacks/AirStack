# MS² native temporal/shared preservation

2026-11-01 America/New_York. Code989bd63c0; before-inference declaration0ca24bac5.
[Protocol](ms2-temporal-adapter-protocol-2026-11-01.json).
[Retraining prerequisites](own-retraining-prerequisites-2026-11-01.md).

The opt-in MS² adapter passes the complete temporal/shared preservation check
on the already-exposed EXP043/044 20-group night pilot
`_2021-08-13-21-18-04`, first stems000000–000019. All four frozen GPU attempts
succeeded once; no threshold, window, recipe or checkpoint retry was needed.
This is engineering validation, not a new quality or independent-test result.

## What changed

Three new entrypoints reuse the unchanged native `MS2ManifestSequence`, v3
runner and shared controller. The adapter copies exact source associations and
all four timestamps into scheduling rows, retains native frame fields, arrays,
K, baseline and thermal recipe, and tags each run with `adapter-contract.json`.
It requires explicit camera-local geometry acknowledgement and uses process-local
scoped runner binding and one Torch/OpenCV thread. Prior bindings/thread counts
are restored even on failure; existing output directories are refused. The old
runner/report schema and defaults stay intact; the added contract identifies MS²
and its unverified recorded-time/physical interpretation explicitly.

The 33,399,261-byte prior EXP043 recovery archive and all277members were
verified. Only80source images and the original source input manifest were
restored, plus the exact archived thermal producer code. No pose/depth reference
payload was restored. Preflight independently decoded/hashed all80images and
AST-executed the archived thermal functions: all40camera frames/80image tensors
agree exactly with both native loader and archived producer, including clipping,
original uint16 preservation, stereo sides, calibration and nanosecond times.
Preflight passed800checks and a CUDA availability probe before any neural calls.

## Results

| Verification | Result |
|---|---|
| Adapter/loader/recorder/shared/state CPU tests |109pass:19new+90regressions|
| Scientific attempts |standalone RGB,standalone thermal,shared root-off,shared root-on;allpass|
| Standalone vs shared root-off |533arrays/23NPZ per camera bitwiseequal;97legacy events per camera equal except observer clock|
| Shared root-off vs root-on |same533arrays/23NPZ and97legacy events per camera;one explicit root added per camera|
| Independent timeline audit |40scheduled camera frames/19decisions/798decision-field checks per shared arm;prefixesexact|
| Root binding |release0<receivedroot<completed0;identity camera-map pose and exact native left time;physicaladmissionfalse|
| Raw advice |18RGB/1ABSTAIN each shared arm;availability/reasons unchanged|

Image `8f7bf1c66f251755fe796a0ebd281b2131d870bc1f936c5585dcc28e08569a33`;
Torch2.7.1+cu128/OpenCV4.10.0/RTX PRO5000Blackwell. Exact released RGB
`bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a`
and thermal
`eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a`
weights remain unchanged. The capture freezes588source files,83input files,
2checkpoints,4commands,driver/preflight/auditors. All source/input/checkpoint
hashes were checked before and after every attempt and again in the audit.

A separately locked **postcapture metadata-only coverage replay** uses the
unchanged existing raw/history/shadow methods on all38decisions/76slots.
Each arm has34ordinary supported-parent evidence passes and2raw-unavailable
slots. Control additionally has2missing-root slots; witness has2documented-root
passes. Raw/history/shadow available decisions are18/17/17 in control and18/17/18
in witness. This demonstrates root consumption on native MS² event streams;
it does not establish pose accuracy, uncertainty calibration or quality gains.

Artifacts are `results/ms2-temporal-adapter/run-001/`: source/paired manifests,
archived producer/input-parity, declaration/capture-plan, exact execution logs,
standalone controls, both shared arms, preservation proof, state-coverage replay
and updated readiness. Input files and local audit scripts are in the Box packet.

## Readiness and next work

The versioned [prospective plan amendment](../../experiments/prospective-state-shadow-evaluation-2026-11-01-adapter-v2.json)
records this development-only adapter verification. The original plan is preserved.
Actual readiness remains INCOMPLETE with7requirements: two pixel and two reference
exposure attestations, exact test assignment, source-backed common-frame temporal
reference target and exact future test input metadata. No conditional reserve was
opened and no test split was assigned. The20-group result does not qualify an
unseen first100-group recording. Unknown original training overlap still restricts
network-training-held-out claims; it does not alone veto a narrower benchmark.

Own retraining is a useful route to recorded provenance. Before fitting, establish
one covariance output/loss/masking contract and recording-level data lineage.
The available trainer's raw+epsilon versus inference exp(2raw) mismatch is real,
but is not proven to be the recipe that produced the supplied weights or the
cause of their failures. Finetuning an unknown base does not remove unknown base
training overlap, and retraining cannot resolve temporal reference/frame semantics.
No fitting or existing scientific/default modification was performed here.

Recovery: `backups/2026-11-01-ms2-temporal-adapter` on the ore-ms Box root.
