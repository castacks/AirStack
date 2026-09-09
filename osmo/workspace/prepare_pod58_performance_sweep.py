"""Prepare matched 50-s diagnostics; never upload or count as production."""
import argparse
import copy
from pathlib import Path
import shlex
import yaml


def prepare(source, output):
    base = yaml.safe_load(source.read_text())
    assert base['iterations'] == 1 and len(base['environments']) == 1
    assert base['environments'][0]['method'] == 'lawnmower'
    assert str(base['env']['NUM_ROBOTS']) == '8'
    probe = (Path(__file__).resolve().parents[2] /
             'scripts/measure_camera_sim_window.py').read_text()
    output.mkdir(parents=True, exist_ok=True)
    paths = []
    for groups, gpu_physics in ((8, False), (32, False), (8, True)):
        spec = copy.deepcopy(base)
        spec['name'] = f'diagnostic_eqsub_l3_lawn_groups{groups}_gpu{int(gpu_physics)}_50s'
        spec.pop('nas_dest', None)
        spec['iterations'] = 1
        spec['iteration_attempts'] = 1
        spec['env'].update(SEARCH_MAX_SIM_SECONDS='50',
                           ZED_TIME_SLICE_GROUPS=str(groups),
                           ZED_TIME_SLICE_BURST='8', ZED_HYDRA_TIME_SLICE='true',
                           ISAAC_SIM_GPU_PHYSICS=str(gpu_physics).lower())
        for step in spec['steps']:
            action = step.get('action', {})
            if action.get('task') == 'takeoff':
                action['timeout_s'] = 900
                action['feedback_timeout_s'] = 900
        # Begin each robot's probe when its fresh planner starts. All flight,
        # perception and completion gates from the production spec remain.
        planner_index = next(i for i, s in enumerate(spec['steps'])
                             if 'nohup ros2 launch search_baselines' in
                             s.get('run', {}).get('cmd', ''))
        spec['steps'].insert(planner_index, {'run': {
            'container': 'airstack-robot-desktop-{n}', 'timeout_s': 60,
            'cmd': 'test ! -e /tmp/search/planner.log || exit 1\n'
                   'source /opt/ros/jazzy/setup.bash\nexport ROS_DOMAIN_ID={n}\n'
                   'nohup python3 -u -c ' + shlex.quote(probe) +
                   ' --robot {robot} --sim-seconds 50 --wall-timeout 7200'
                   ' --wait-for-file /tmp/search/planner.log'
                   ' --output /tmp/pod58_perf.json'
                   ' > /tmp/pod58_perf.log 2>&1 < /dev/null &\n'}})
        spec['steps'].append({'run': {
            'container': 'airstack-robot-desktop-{n}', 'timeout_s': 180,
            'cmd': 'for i in $(seq 1 80); do\n'
                   ' if [ -s /tmp/pod58_perf.json ]; then cat /tmp/pod58_perf.json; exit 0; fi\n'
                   ' sleep 2\ndone\ncat /tmp/pod58_perf.log; exit 1\n'}})
        path = output / (spec['name'] + '.yaml')
        path.write_text(yaml.safe_dump(spec, sort_keys=False))
        paths.append(path)
    return paths


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    for path in prepare(args.source, args.output):
        print(path)
