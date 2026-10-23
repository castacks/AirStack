"""Read-only installation checks. Never starts containers, models or flights."""
import argparse
import importlib.util
import json
import os
import shutil
from pathlib import Path
import subprocess
import sys

from model_adapters import REGISTRY, get_adapter

ROOT = Path(__file__).resolve().parents[2]
RUNTIME = ROOT / 'robot/ros_ws/ws2_runtime'


def docker(*args):
    result = subprocess.run(['docker', *args], capture_output=True, text=True, timeout=15)
    if result.returncode:
        raise RuntimeError(result.stderr.strip()[-1200:] or result.stdout.strip()[-1200:])
    return json.loads(result.stdout)


def check_install(planner):
    adapter = get_adapter(planner)
    errors = []
    warnings = []
    for module, package in [('yaml', 'PyYAML'), ('PIL', 'Pillow')]:
        if importlib.util.find_spec(module) is None:
            errors.append(f'Missing {package}. Run {sys.executable} -m pip install {package}')
    if not (shutil.which(os.environ.get('WS2_FFMPEG', 'ffmpeg')) or importlib.util.find_spec('imageio_ffmpeg')):
        warnings.append(f'Optional video recording needs ffmpeg or imageio-ffmpeg. Run {sys.executable} -m pip install imageio-ffmpeg')
    if not (adapter.repository / adapter.entrypoint).is_file():
        errors.append(f'Missing worker: {adapter.repository / adapter.entrypoint}. Install the selected model fork beside AirStack.')
    for pattern in adapter.weight_patterns:
        if not any(p.is_file() for p in adapter.repository.glob(pattern)):
            errors.append(f'Missing model weights: {adapter.repository / pattern}. Follow the model README download instructions.')
    install = ROOT / 'robot/ros_ws/install'
    for package in ('desktop_bringup', 'local_bringup', 'mononav_bridge', 'trajectory_controller', 'pid_controller'):
        markers = [install / package / 'share/ament_index/resource_index/packages' / package,
                   install / 'share/ament_index/resource_index/packages' / package]
        # colcon --symlink-install creates absolute /root/... symlinks that are
        # valid inside the robot container, not on the host. Check the marker
        # entry itself here; this is not a container-time import test.
        if not any(os.path.lexists(p) for p in markers):
            errors.append(f'ROS package {package} is not built in this checkout. Run the full robot bws build in the installation guide.')
    if not RUNTIME.is_dir() or not os.access(RUNTIME, os.W_OK | os.X_OK):
        errors.append(f'Runtime directory is missing or not writable: {RUNTIME}. Create it as the user running the viewer.')
    try:
        containers = docker('inspect', 'isaac-sim', 'airstack-robot-desktop-1')
        for info in containers:
            name = info['Name'].lstrip('/')
            targets = [('/isaac-sim/AirStack/tools/ws2_bench', ROOT / 'tools/ws2_bench'),
                       ('/isaac-sim/AirStack/robot/ros_ws/ws2_runtime', RUNTIME)] if name == 'isaac-sim' else [('/root/AirStack/robot/ros_ws', ROOT / 'robot/ros_ws')]
            for target, expected in targets:
                mounts = [m for m in info['Mounts'] if target == m['Destination'] or target.startswith(m['Destination'].rstrip('/') + '/')]
                mount = max(mounts, key=lambda m: len(m['Destination'])) if mounts else None
                actual = (Path(mount['Source']) / Path(target).relative_to(mount['Destination'])).resolve() if mount else None
                if actual != expected.resolve():
                    errors.append(f'{name} does not mount this checkout at {target}. Recreate the bench containers from this AirStack checkout.')
        docker('network', 'inspect', 'airstack_airstack_network')
        docker('image', 'inspect', adapter.image)
        if adapter.torch_cache:
            docker('volume', 'inspect', 'mononav-torch-cache')
    except (OSError, RuntimeError, subprocess.TimeoutExpired, ValueError, KeyError) as exc:
        errors.append(f'Docker/container/image check failed: {exc}. Run docker info as this user, then prepare the containers and {adapter.image} using the README.')
    return {'ok': not errors, 'planner': planner, 'python': sys.executable, 'errors': errors, 'warnings': warnings,
            'scope': 'Static prerequisites only; GPU execution, downloaded weight contents, Office assets and LLM login require runtime verification.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planner', required=True, choices=sorted(REGISTRY))
    args = parser.parse_args()
    result = check_install(args.planner)
    print(json.dumps(result, indent=2))
    sys.exit(0 if result['ok'] else 1)
