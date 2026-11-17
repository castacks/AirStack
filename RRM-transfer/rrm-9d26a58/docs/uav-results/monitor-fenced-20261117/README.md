# Actual monitor and fenced controller

2026-11-17; `ore_proj`, source based on `7e83f8a16` plus the pinned changes.
Both actual C++ nodes run on isolated DDS193 with synthetic odometry. A delayed
RPC proxy holds an actually submitted reset while a newer stale-source fault causes
the monitor's independent legacy PAUSE to advance controller generation. Only then
does it forward the reset. The controller rejects the old generation and stays
latched. The exact request CDR, hash and delivery ordering are saved.

All 25 final contract checks pass, including subsequent explicit fresh motion
commands, a protective stop during reference advance, controller outage escalation,
restart identity rejection and acknowledgement of the new controller's PAUSE.
Legacy monitor resume cannot release the strict latch. Acknowledgement leaves the
restarted controller latched; it does not authorize automatic motion.

The first run retained 34 NaN completion samples from empty-buffer division0/0.
The arithmetic guard now reports0 for an empty buffer; final samples are finite.
The initial motion fixture also used the0.3default low-speed clamp and sampled only
0.4s, shorter than the controller's post-PAUSE ramp needed for the0.02m criterion.
Final fixture uses the earlier unit test's explicit0.1clamp and waits for the same
0.02m excursion plus nonzero feedforward within2s. The controller ramp and motion
threshold were not relaxed. Failed precursor outputs and harness remain saved.

Controller package9/9CTest passes; aggregate26tests, zero failures/errors, two skips
(includes unchanged message-package results). The integration test declares its
monitor test dependency and uses explicit executable paths. Build/install remains
separate; no live binary, profile, scene or aircraft endpoint changed. This is a
protocol and reference-stream result, not physical stop confirmation, authenticated
authority, full C06/C08 admission or vehicle-wide exclusive control.

[Compact report](../monitor-fenced-20261117.json). Offline standard-library audit:

```bash
python docs/uav-results/monitor-fenced-20261117/audit.py
```
