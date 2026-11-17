# Controller-local command admission leases

2026-11-17; `ore_proj`, starting at `62e9e3057` plus pinned source changes. Actual
controller and a delayed RPC proxy on isolated DDS192; no simulator endpoints.
Normal0.25s, paused-ROS-clock0.25s and configured-default1.0s controls each pass26
checks. All three actually submit and hold a leased trajectory RPC until expiry;
forwarding then rejects with lease_expired. Exact command CDRs and ordering are saved.

Other checks cover missing/mismatched tokens, changed mode/path payloads, preparation
without effect or sequence consumption, duplicate preparation without deadline
renewal, expired sequence refusing renewal, explicit fresh attempts, generation
invalidation, superseded preparation, invalid preparation preserving a prior valid
lease, protective STOP and idempotent acknowledgement of an already accepted command
past expiry. These are local protocol controls, not independent accuracy trials.

Full packages pass: controller11/11CTest, message2/2; aggregate29tests, zero errors/
failures, two skips. Five invalid config combinations/budgets reject at startup.
Caller labels and tokens are not authenticated permission. RESET does not prove
safe state; full C06/C08 admission, execution expiry, durable deduplication and
physical stopping remain outside this result. Default profiles/live installation
are unchanged. See controller README for exact configuration and request protocol.

[Compact report](../command-leases-20261117.json). Offline audit:

```bash
python docs/uav-results/command-leases-20261117/audit.py
```
