#!/usr/bin/env python3
"""Prepare four isolated 50-s RAVEN diagnostics; never launch or upload."""
import argparse
import copy
from pathlib import Path
import shlex
import yaml


def prepare(source, output, isaac_gpu, offboard_gpu, skips_list=(4, 8, 16, 24)):
    baseline = yaml.safe_load(source.read_text())
    assert baseline['iterations'] == 1 and len(baseline['environments']) == 1
    assert int(baseline['env']['NUM_ROBOTS']) == 8
    output.mkdir(parents=True, exist_ok=True)
    paths = []
    probe = Path(__file__).resolve().parents[2] / 'scripts/measure_camera_sim_window.py'
    for skips in skips_list:
        spec = copy.deepcopy(baseline)
        spec['name'] = f'diagnostic_raven_empty{skips}_50sim'
        spec.pop('nas_dest', None)
        spec['iteration_attempts'] = 1
        spec['env'].update(ZED_TIME_SLICE_GROUPS=str(8 + skips),
                           ZED_TIME_SLICE_BURST='8', ZED_HYDRA_TIME_SLICE='true',
                           RAYFRONTS_WAIT_TIMEOUT_S='1800',
                           ISAAC_SIM_ACTIVE_GPU=str(isaac_gpu),
                           OFFBOARD_COMPUTE_GPU=str(offboard_gpu))
        search = next(s['action'] for s in spec['steps']
                      if s.get('action', {}).get('task') == 'semantic_search')
        search['goal']['max_sim_seconds'] = 50.0
        search_index = next(i for i, step in enumerate(spec['steps'])
                            if step.get('action', {}).get('task') == 'semantic_search')
        start_probe = {
            'run': {'container': 'airstack-robot-desktop-{n}', 'timeout_s': 60,
                    'cmd': 'source /opt/ros/jazzy/setup.bash\nexport ROS_DOMAIN_ID={n}\n'
                    'test ! -e /tmp/raven_robot_{n}.log || { echo STALE_RAVEN_LOG; exit 1; }\n'
                    'nohup python3 -u -c ' + shlex.quote(probe.read_text()) +
                    ' --robot {robot} --sim-seconds 50 --wall-timeout 18000'
                    ' --wait-for-file /tmp/raven_robot_{n}.log'
                    ' > /tmp/raven_camera_benchmark.json 2> /tmp/raven_camera_benchmark.err < /dev/null &\n'
                    'echo $! > /tmp/raven_camera_benchmark.pid\n'}}
        collect_probe = {
            'run': {'container': 'airstack-robot-desktop-{n}', 'timeout_s': 900,
                    'cmd': 'for i in $(seq 1 420); do\n'
                    '  if [ -s /tmp/raven_camera_benchmark.json ]; then\n'
                    '    python3 -c ' + shlex.quote(
                        'import json; d=json.load(open("/tmp/raven_camera_benchmark.json")); '
                        'assert d["sim_seconds"] >= 50; '
                        'assert all(s["unique_frames"] > 0 for s in d["streams"].values()); '
                        'print(json.dumps(d))') + '; exit $?\n'
                    '  fi\n  sleep 2\ndone\n'
                    'cat /tmp/raven_camera_benchmark.err; exit 1\n'}}
        spec['steps'].insert(search_index, start_probe)
        spec['steps'].insert(search_index + 2, collect_probe)
        path = output / (spec['name'] + '.yaml')
        path.write_text(yaml.safe_dump(spec, sort_keys=False))
        paths.append(path)
    return paths


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--isaac-gpu', required=True)
    parser.add_argument('--offboard-gpu', required=True)
    parser.add_argument('--skips', nargs='+', type=int, default=[4, 8, 16, 24])
    args = parser.parse_args()
    for path in prepare(args.source, args.output, args.isaac_gpu, args.offboard_gpu, args.skips):
        print(path)
