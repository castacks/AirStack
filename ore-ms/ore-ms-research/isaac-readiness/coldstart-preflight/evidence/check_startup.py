"""Exercise the entrypoint launch block and airstack up without a daemon."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[5]
DIGEST = (
    "airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:"
    "ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697"
)
REGISTRY = "airlab-docker.andrew.cmu.edu/airstack/airstack:"
DEFAULT = REGISTRY + "v0.20.8_isaac-sim"

MOCK = """#!/usr/bin/env python3
import json, os, subprocess, sys
from pathlib import Path
args = sys.argv[1:]
if args[0] == 'info':
    sys.exit(0)
if args[0] == 'version':
    print('29.5.0')
    sys.exit(0)
if args[:2] == ['image', 'inspect']:
    sys.exit(1)
if args[0] == 'compose' and 'config' in args:
    subprocess.run([os.environ['REAL_DOCKER'], *args, '--no-env-resolution'], check=True)
    sys.exit(0)
if args[0] == 'compose' and 'up' in args:
    with Path('up.json').open('w') as output:
        json.dump({'args': args[args.index('up') + 1:],
                   'profile': os.environ['COMPOSE_PROFILES'],
                   'image': os.environ.get('ISAAC_SIM_IMAGE', '')}, output)
    sys.exit(int(os.environ['MOCK_UP_STATUS']))
raise SystemExit(f'unexpected Docker command: {args!r}')
"""

def main():
    docker = shutil.which("docker")
    if not docker:
        raise SystemExit("Docker CLI with Compose is required")
    entrypoint = (ROOT / "osmo/workspace/entrypoint.sh").read_text()
    launch = entrypoint.split("6. airstack up", 1)[1].split("# ─── 7.", 1)[0]
    launch = launch[launch.index("\n") + 1:]

    with tempfile.TemporaryDirectory(prefix="coldstart-preflight-") as directory:
        sandbox = Path(directory)
        compose_files = subprocess.check_output([
            "git", "ls-files", "-z", "*compose*.yaml", "*compose*.yml"
        ], cwd=ROOT, text=True).strip("\0").split("\0")
        for name in [".env", "airstack.sh", ".airstack/modules/_lib.sh", *compose_files]:
            target = sandbox / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / name, target)
        (sandbox / "docker-config").mkdir()
        (sandbox / "docker").write_text(MOCK)
        (sandbox / "docker").chmod(0o755)
        (sandbox / "xhost").write_text("#!/bin/sh\nexit 0\n")
        (sandbox / "xhost").chmod(0o755)
        (sandbox / "launch.sh").write_text('set -euo pipefail\nlog() { printf "%s\\n" "$*"; }\n' + launch)
        base_env = {
            "PATH": f"{sandbox}{os.pathsep}{os.environ['PATH']}",
            "HOME": str(sandbox),
            "DOCKER_CONFIG": str(sandbox / "docker-config"),
            "DOCKER_HOST": "unix:///nonexistent-coldstart-test.sock",
            "REAL_DOCKER": docker,
            "AIRSTACK_ROOT": str(sandbox),
            "AIRSTACK_BRANCH": "ore-ms-orb-development",
            "ISAAC_SIM_LIVESTREAM": "true",
            "AUTOLAUNCH": "true",
            "NUM_ROBOTS": "1",
        }
        for name, image, flag, status, no_build in [
            ("default", "", "", 0, False),
            ("pinned", DIGEST, "", 0, True),
            ("pinned_pull_failure", DIGEST, "", 42, True),
            ("explicit_no_build", "", "1", 0, True),
        ]:
            (sandbox / "up.json").unlink(missing_ok=True)
            env = {
                **base_env, "MOCK_UP_STATUS": str(status),
                "AIRSTACK_NO_IMAGE_BUILD": flag,
            }
            if image:
                env["ISAAC_SIM_IMAGE"] = image
            result = subprocess.run(["bash", "launch.sh"], cwd=sandbox, env=env,
                                    capture_output=True, text=True, check=True)
            call = json.loads((sandbox / "up.json").read_text())
            assert call == {
                "image": image, "profile": "desktop,isaac-sim-livestream",
                "args": ["--no-build", "-d"] if no_build else ["-d"],
            }, (call, result.stdout, result.stderr)
            assert ("airstack up exited non-zero" in result.stdout) == bool(status)
            assert ("Containers started." in result.stdout) == (not status)

            config = subprocess.check_output([
                docker, "compose", "config", "--no-env-resolution", "--format", "json"
            ], cwd=sandbox, env={**env, "COMPOSE_PROFILES": call["profile"]},
               stderr=subprocess.DEVNULL, text=True)
            services = json.loads(config)["services"]
            images = {key: val["image"] for key, val in services.items()}
            assert images["isaac-sim-livestream"] == (image or DEFAULT), images
            assert images["robot-desktop"] == REGISTRY + "v0.20.8_robot-x86-64_dev"
            assert images["gcs"] == REGISTRY + "v0.20.8_gcs"
            assert services["robot-desktop"]["deploy"]["replicas"] == 1
            gpu = services["isaac-sim-livestream"]["deploy"]["resources"]["reservations"]["devices"]
            assert len(gpu) == 1 and gpu[0]["count"] == 1
            print(json.dumps({"case": name, "compose_up": call["args"], "images": images,
                              "simulator_gpus": 1, "robot_replicas": 1,
                              "simulated_docker_exit": status}))


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as exc:
        print(exc.stdout or "", exc.stderr or "", file=sys.stderr)
        raise
