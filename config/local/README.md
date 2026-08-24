# `config/local/` — this machine only (gitignored)

Everything in this directory except this README is **gitignored**: it belongs
to one machine / one physical vehicle and must never be committed or shared
through trunk (RFC #380 §1, §3).

```
config/local/
├── calibration/<serial>/    # per-UNIT intrinsics/extrinsics (SN-0042/, ...)
└── overrides.yaml           # (future) 4th precedence layer, below CLI flags
```

## Calibration overlays (vehicle units)

A vehicle **type** (`config/vehicles/<name>/`) is shared and committed; a
vehicle **unit** is one serial number whose calibration drifts and gets
re-measured in the field. A fleet entry's `unit: SN-0042` binds
`config/local/calibration/SN-0042/`, and `tools/fleet/resolve_fleet.py`
exports it to the robot container as `CALIBRATION_DIR`
(`/root/AirStack/config/local/calibration/SN-0042`). Recalibrating writes
here — never into the shared vehicle package.

No enforced file layout yet: put the camera intrinsics / extrinsics files
your drivers consume in the unit directory and point the driver config at
`$CALIBRATION_DIR`. `CALIBRATION_DIR` is empty when the fleet entry declares
no `unit:`.
