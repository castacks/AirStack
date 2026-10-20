import argparse
import json
from pathlib import Path

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--log", required=True, type=Path)
parser.add_argument("--output", required=True, type=Path)
args = parser.parse_args()
events = [json.loads(line) for line in args.log.read_text().splitlines() if line.strip()]
segments = []
current = []
pending = None
for event in events:
    if event["event"] in ("start", "trajectory_start", "engage_requested"):
        if current:
            segments.append(current)
        current, pending = [], None
    elif event["event"] == "selected":
        if event.get("mode") == 1:
            current.append(dict(event, elapsed_s=event["ros_s"]))
        elif current:
            segments.append(current)
            current = []
    elif event["event"] == "prepared_command":
        pending = event
    elif event["event"] == "published" and pending is not None:
        if event["elapsed_s"] != pending["elapsed_s"]:
            raise ValueError("Published/prepared command mismatch")
        current.append(pending)
        pending = None
if current:
    segments.append(current)
report = {"scope": "Prepared selected-command samples (legacy runner logs use published pairs). Use bag acknowledgements and PX4 outputs to establish forwarding; these logs do not prove flight.",
          "segments": [], "faults": [e for e in events if e["event"] == "fault"]}
for rows in segments:
    t = np.array([r["elapsed_s"] for r in rows])
    if np.any(np.diff(t) <= 0):
        raise ValueError("Nonmonotonic timestamps within session")
    tracking = np.array([r["phase"] == "tracking" for r in rows])
    errors = np.array([np.array(r["position_enu"])-r["reference_enu"] for r in rows])
    dt = np.diff(t)
    entry = {"command_samples": len(rows), "tracking_samples": int(tracking.sum()),
             "duration_s": float(t[-1]-t[0]),
             "inference_ms_p50_p95_max": (np.percentile([r["inference_s"] for r in rows], [50,95,100])*1000).tolist(),
             "source_age_ms_p50_p95_max": (np.percentile([r["ros_s"]-r["source_s"] for r in rows], [50,95,100])*1000).tolist(),
             "command_interval_ms_p50_p95_max": (np.percentile(dt, [50,95,100])*1000).tolist() if len(dt) else None}
    if tracking.any():
        magnitudes = np.linalg.norm(errors[tracking], axis=1)
        entry.update(position_rmse_m=float(np.sqrt(np.mean(magnitudes**2))),
                     position_max_m=float(magnitudes.max()),
                     position_axis_rmse_m=np.sqrt(np.mean(errors[tracking]**2, axis=0)).tolist())
    report["segments"].append(entry)
args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
print(json.dumps(report, indent=2))
