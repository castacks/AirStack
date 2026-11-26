# Frozen corpus manifest

The corpus has 512 environmental realizations, each with the 27 cases in [the frozen configuration](config.json). It contains 13,824 opportunities with 33×25 pixel centers and four bands, plus 41,472 baseline-arm candidate records. Within each realization, case controls share scene placement and base process/noise inputs. Realizations use separate deterministic streams and never cross partitions.

| Partition | Realization IDs, inclusive | Opportunities | Feasible | Negative |
|---|---|---:|---:|---:|
| Development | 0–127 | 3,456 | 2,560 | 896 |
| Calibration | 128–255 | 3,456 | 2,560 | 896 |
| Locked holdout | 256–511 | 6,912 | 5,120 | 1,792 |

The raw evidence package contains split label and prediction tables, a common observation store, a scoring-only geometry and scene store, the calibration receipt and all holdout results. There are 16 hashed scientific outputs. The fixed output order in each store is opaque opportunity ID. The split label-table SHA-256 values are:

| Development | Calibration | Locked holdout |
|---|---|---|
| `e503d5b442f350d346b6330ff88e789395e4260432419e843313e4ccc83a3363` | `b27f0fa9d85f96430bd57496993892587927f99418a448cb6ba433fca8fb0338` | `7c2ad29c8cbafc95c0b4ce53facafd64eebf3dadba08cfc7db40976a28effd02` |

Two clean end-to-end runs produced byte-identical arrays, tables, calibration receipt and holdout statistics. Timing and resource measurements vary by run. [Exact hashes](evidence/corpus.sha256) · [Replay results](evidence/replay.log)
