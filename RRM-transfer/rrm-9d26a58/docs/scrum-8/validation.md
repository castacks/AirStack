# Implementation and validation — 2026-09-13 UTC

Source baseline: RRM archive `9d26a58eb8516b6754c5d12b041cdc789950e047`, AirStack `ore_proj` at `a6dad8caf54722e5eba3481367e914ce213e6135`. Changes are staged source files, not a new Git revision or published baseline.

## Implemented increment

[`rrm/contracts.py`](../../rrm/contracts.py) adds independent primitives for C02 evidence truth, C03 declared semantic support, C06 immutable context/single-use admission and C08 stop-generation/reset checks. A fresh authority epoch prevents permits from a previous guard instance being reused after reset. The initial guard starts inhibited. These primitives do not change `rrm/loop.py`, `rrm/verbs.py`, the oracle, mock safety or the Isaac stubs.

Trusted supervision must supply validated safety decisions and fresh safe-state evidence. The guard does not authenticate callers or move/stop a robot. Its lock protects only local admission bookkeeping; transport/adapter checks, current-state synchronization, durable deduplication and independent stop delivery remain required. Capability support does not validate physical limits, required operation/resource combinations or permission. The old loop still has the handoff's uncertainty, capability, permission and active-stop gaps.

## Results

Verification ran inside the existing AirStack robot image `v0.20.8_robot-x86-64_dev`, with explicit CPU runtime `runc`, 2 CPU / 2 GiB limits, and a source bind mount. Container-local Pydantic 2.13.5 satisfied the original dependency. No GPU/model package or weight was downloaded.

| Check | Result | Interpretation |
| --- | --- | --- |
| Original oracle suite | 5/5 passed: T1, T2, T6, T8, T9 | Regression baseline only; T6 begins with a human present, not entry during motion |
| Original relation inference | PASS | Geometry helper self-test, not an Isaac integration test |
| New contract unit tests | 14/14 passed | Unknown negation, capability/profile handling, immutable inputs, stale context, expiry/nonallow, duplicate admission, stop/reset, restart epoch |
| Architecture allocation | 11/11 rows linked to owner, contract, scenario and planned evidence | Documentation coverage, not requirement satisfaction |
| GPU execution | NOT REQUESTED | This transfer workspace was deliberately submitted with `gpu: 0`; its NVML failure is not an OSMO infrastructure finding |
| Integrated S01–S10 SIL | NOT RUN | No running Isaac scene or conforming adapter/supervisor |

Reproduce CPU checks inside a container with source mounted at `/workspace/rrm` and Pydantic v2 installed:

```sh
cd /workspace/rrm
python3 scripts/oracle_loop.py --suite --trace-dir /evidence/traces
python3 simulation/isaac_backend.py
python3 -m unittest discover -s tests -v
```

Source/evidence checksums, container identity/digest, dependency versions, timestamps and raw test output accompany the export bundle. Runtime files from the input archive were compared byte-for-byte; only the new contract module was added. The original input ZIP and checksum files remain intact.

## Remaining work

Review the architecture against Phase 1; implement task/context interaction and evidence-backed state; connect capability-aware planning, safety/permission authority, monitoring and replayable telemetry; select and validate the hand scene; restore remote GPU access; run the planned deterministic SIL campaign before comparing model candidates. Model selection and adapter numeric/safe-state limits need measured feasibility. No SCRUM-8 Done transition or external publication was made.
