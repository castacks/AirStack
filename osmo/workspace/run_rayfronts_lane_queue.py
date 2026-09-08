#!/usr/bin/env python3
"""Run the remaining one-cell shared-RAVEN missions on one held OSMO pod.

The nested Docker daemon exposes host GPU indices, while OSMO's outer task
reports only its two assigned devices renumbered from zero. Consequently the
correct Isaac/offboard indices vary by worker. This wrapper materializes each
committed mission with the worker-audited indices immediately before launch.
Every child still goes through run_held_mission.sh and its <12 h hard bound.
"""

import argparse
from pathlib import Path
import subprocess
import sys

import yaml


LANES = {
    "a": [
        "firesuburbanl1v1", "firesuburbanl3v1",
        "hurricanesuburbanl2v1", "tornadosuburbanl1v1",
        "tornadosuburbanl3v1", "earthquakesuburbanl2v1",
        "fireurbanl1v1", "fireurbanl3v1", "hurricaneurbanl2v1",
        "tornadourbanl1v1", "tornadourbanl3v1", "earthquakeurbanl2v1",
    ],
    "b": [
        "firesuburbanl2v1", "hurricanesuburbanl1v1",
        "hurricanesuburbanl3v1", "tornadosuburbanl2v1",
        "earthquakesuburbanl1v1", "earthquakesuburbanl3v1",
        "fireurbanl2v1", "hurricaneurbanl1v1", "hurricaneurbanl3v1",
        "tornadourbanl2v1", "earthquakeurbanl1v1", "earthquakeurbanl3v1",
    ],
}


def launcher_is_held(pid: int) -> bool:
    stat = Path(f"/proc/{pid}/stat").read_text().split()[2]
    cmd = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ")
    return stat == "T" and b"mission_launcher.sh" in cmd


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--launcher-pid", type=int, required=True)
    parser.add_argument("--lane", choices=sorted(LANES), required=True)
    parser.add_argument("--isaac-gpu", required=True)
    parser.add_argument("--offboard-gpu", required=True)
    parser.add_argument("--root", type=Path, default=Path("/root/AirStack"))
    args = parser.parse_args()

    if not launcher_is_held(args.launcher_pid):
        raise SystemExit(f"launcher {args.launcher_pid} is not held")

    mission_dir = args.root / "osmo" / "missions"
    for scene in LANES[args.lane]:
        if not launcher_is_held(args.launcher_pid):
            raise SystemExit(f"launcher {args.launcher_pid} lost its hold")
        source = mission_dir / f"raven_{scene}_raven_remaining_2gpu1.yaml"
        mission = yaml.safe_load(source.read_text())
        assert mission["iterations"] == 1
        assert len(mission["environments"]) == 1
        mission["env"]["ISAAC_SIM_ACTIVE_GPU"] = str(args.isaac_gpu)
        mission["env"]["OFFBOARD_COMPUTE_GPU"] = str(args.offboard_gpu)
        assert mission["env"]["ZED_TIME_SLICE_GROUPS"] == "8"
        assert mission["env"]["ZED_TIME_SLICE_BURST"] == "8"
        assert mission["env"]["ZED_HYDRA_TIME_SLICE"] == "true"
        search = next(s["action"] for s in mission["steps"]
                      if s.get("action", {}).get("task") == "semantic_search")
        assert search["timeout_s"] == 21600
        assert search["goal"]["max_sim_seconds"] == 600.0

        runtime = Path("/tmp") / f"{source.stem}.lane_{args.lane}.yaml"
        runtime.write_text(yaml.safe_dump(mission, sort_keys=False, width=100))
        print(f"[{scene}] GPU Isaac={args.isaac_gpu} "
              f"offboard={args.offboard_gpu}", flush=True)
        rc = subprocess.call([
            "bash", str(args.root / "osmo/workspace/run_held_mission.sh"),
            str(args.launcher_pid), str(runtime),
        ], cwd=args.root)
        if rc != 0:
            print(f"[{scene}] stopped queue after rc={rc}", file=sys.stderr)
            return rc
        print(f"[{scene}] passed, uploaded and verified", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
