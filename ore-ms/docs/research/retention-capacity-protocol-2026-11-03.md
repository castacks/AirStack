# EXP123: retained encoded payload capacity

Frozen before replay; parent EXP122 / 521bfa164. Branch ore_ms, 2026-11-03.

Replay all six EXP122 captures in their recorded arrival order using actual encoded
source bytes. Each camera advances one source at a time. A recorded query occurs
immediately after its camera arrival; actions and subsequent queries remain fixed.
No estimator is run, and no eligible-delivery/pose counterfactual is estimated.

Use unlimited, 0, 2, 4 and 8 MiB shared encoded payload capacities (MiB=1048576).
Preserve current/prior source TTL per camera and the frozen synthetic outages.
Oversize stereo pairs are rejected atomically without evicting existing entries.
Otherwise evict oldest source globally until the entire pair fits; RGB precedes
thermal on ties. Expiration precedes capacity admission. Never partial stereo,
future/older query, fabricated unavailable input, or outcome-selected budget.

The bound covers bytes in store-owned entries after every mutation. Input staging,
returned payload leases, Python metadata, transient copies, IPC, decoded tensors,
GPU, RSS and OS cache are excluded. All arrivals still incur encoded image reads;
this is a storage admission experiment, not sensor bandwidth or total memory savings.

Primary contract: every resident state obeys capacity; unlimited state and query
payload match original traces; an independent metadata oracle reproduces all
bounded states, expiry, deterministic evictions, rejected pairs, actual image
hashes and counters. Report misses even if a budget loses the known handover frame.
Capacity success is separate from serving every historical catch-up query. Do not
infer live recovery, deadlines, native call savings or production readiness.

Freeze source and test hashes, old plan and all source-image hashes, six arrival
reports/timelines and protocol. Run each of 30 cases once into exclusive output
paths, retain failures, lock artifacts before analysis. Existing exposed images
suffice; sustained natural returns and real bounded IPC/acquisition remain future
evidence. No reserve access, fitting, default adoption or extra data acquisition.
