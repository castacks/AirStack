# Passive current-view sample — November 17, 2026

Twenty exact source-header RGB/depth/calibration triples from the existing default
Isaac scene, with no actuation or scene change. All detector results were ABSENT;
this is not scene-wide target truth. Only three PNG/NPZ detector-input samples were
saved. The other seventeen detector outputs cannot be independently recomputed.
No physical execution or renderer acquisition synchronization is established.

The source-matched audit checked all twenty retained headers and recomputed all
three saved inputs. Input hashes and scope are in the adjacent
`passive-camera-audit-20261117.json`. `provenance.json` records capture, detector
and pairing-helper hashes. Hashes detect changed bytes; they do not authenticate
the capture issuer. The fixed-case audit source is retained here as provenance.

From the RRM root with the source revision matching those hashes and optional
NumPy/OpenCV dependencies installed:

```bash
PYTHONPATH=. python3 docs/uav-results/passive-default-20261117/audit_snapshot.py \
  docs/uav-results/passive-default-20261117
```

The audit prints JSON and does not write or dispatch anything. Its assertions are
for this fixed 20-record/3-input sample; it is not a general capture admission tool.
The earlier capture without individual header fields remains a separate local
development record and was not upgraded into this audited sample.
