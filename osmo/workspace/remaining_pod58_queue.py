#!/usr/bin/env python3
"""Durable one-cell production queue; each launch has its own <=12h cap.

Preserves source scene/spawn/method settings and all flight acceptance gates.
Failed cells are recorded for diagnosis, never uploaded or silently accepted.
Restarting the queue skips cells already passed and uploaded by this queue.
"""
import copy
import datetime
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml


def selections():
    yield "urban_fire_remaining_8robot_optimized_pod58.yaml", {"fireurbanl2v1_lawnmower"}
    # Dev 191 exhausted both Hurricane Urban L2 Frontier attempts without an
    # accepted run, so pod 58 owns all four L2 methods now.
    yield "hurricane_urban_l2_8robot_optimized_dev191.yaml", {"hurricaneurbanl2v1_" + m for m in ("frontier", "lawnmower", "vlfm", "conavgpt2_team")}
    yield "hurricane_urban_l3_8robot_optimized_dev191.yaml", None
    for level in (1, 2, 3):
        yield f"tornado_urban_l{level}_8robot_optimized_pod57.yaml", None
    yield "earthquake_suburban_l1_8robot_optimized_pod57.yaml", {"earthquakesuburbanl1v1_" + m for m in ("lawnmower", "vlfm", "conavgpt2_team")}
    yield "earthquake_suburban_l2_8robot_optimized_pod56.yaml", {"earthquakesuburbanl2v1_" + m for m in ("vlfm", "conavgpt2_team")}
    yield "earthquake_suburban_l3_8robot_optimized_pod57.yaml", None


def completion_command(team):
    condition = '!=' if team else '=='
    domain = '0' if team else '{n}'
    robot = 'robot_1' if team else '{robot}'
    return f'''if [ "{{{{env.method}}}}" {condition} "conavgpt2_team" ]; then echo SKIP; exit 0; fi
source /root/.bashrc >/dev/null 2>&1 || true
export ROS_DOMAIN_ID={domain}
python3 - <<'PY'
import os, time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
rclpy.init()
node = rclpy.create_node('benchmark_completion_guard')
done = [False]
def received(msg):
    done[0] = done[0] or msg.data
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
sub = node.create_subscription(Bool, '/{robot}/search/run_complete', received, qos)
deadline = time.monotonic() + 13680
last_report = 0
while not done[0] and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=1)
    if done[0]:
        break
    try:
        os.kill(int(Path('/tmp/search/planner.pid').read_text().strip()), 0)
    except (OSError, ValueError):
        raise SystemExit('PLANNER_EXITED before completion')
    if time.monotonic() - last_report > 120:
        print('Waiting for latched run_complete=true: {robot}', flush=True)
        last_report = time.monotonic()
node.destroy_node()
rclpy.shutdown()
if not done[0]:
    raise SystemExit('RUN_DID_NOT_COMPLETE')
print('RUN_COMPLETE {robot}', flush=True)
PY
'''


def prepare(root, output):
    output.mkdir(parents=True, exist_ok=True)
    missions = []
    for source, wanted in selections():
        base = yaml.safe_load((root / 'osmo/missions' / source).read_text())
        selected = [e for e in base['environments'] if wanted is None or e['name'] in wanted]
        if wanted is not None:
            assert {e['name'] for e in selected} == wanted, source
        for env in selected:
            spec = copy.deepcopy(base)
            spec['name'] = 'remaining58_' + env['name']
            spec['environments'] = [env]
            spec['iterations'] = 1
            spec['environment_order'] = 'round_robin'
            spec['nas_dest'] = '/volume2/coa-sei'
            # Sep 7 user-approved sensor cohort: eight occupied groups plus
            # FOUR empty groups. Preserve the eight-update wake burst.
            spec['env'].update(ISAAC_SIM_ACTIVE_GPU='2', OFFBOARD_COMPUTE_GPU='2',
                               START_RAYFRONTS_SERVER='false', ISAAC_SIM_GPU_PHYSICS='false',
                               ZED_TIME_SLICE_GROUPS='12', ZED_TIME_SLICE_BURST='8',
                               ZED_HYDRA_TIME_SLICE='true')
            replaced = 0
            for step in spec['steps']:
                run = step.get('run', {})
                if 'ros2 topic echo' in run.get('cmd', '') and '/search/run_complete' in run['cmd']:
                    run['cmd'] = completion_command(run['container'] == 'offboard-compute')
                    run['timeout_s'] = 13800
                    replaced += 1
            assert replaced == 2, (source, replaced)
            path = output / (spec['name'] + '.yaml')
            path.write_text(yaml.safe_dump(spec, sort_keys=False))
            missions.append(path)
    assert len(missions) == 30, len(missions)
    return missions


def main():
    root = Path(__file__).resolve().parents[2]
    output = root / 'osmo/results/remaining58_queue'
    missions = prepare(root, output / 'missions')
    if '--prepare-only' in sys.argv:
        print(f'Validated {len(missions)} one-cell missions in {output}')
        return
    lock = (output / 'queue.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    launcher = int(sys.argv[1])
    # Literal environment values: never shell-source passwords containing '$'.
    for pair in Path(f'/proc/{launcher}/environ').read_bytes().split(b'\0'):
        if pair.startswith(b'AIRLAB_STORAGE_'):
            key, value = pair.decode().split('=', 1)
            os.environ[key] = value
    assert os.environ.get('AIRLAB_STORAGE_USER') and os.environ.get('AIRLAB_STORAGE_PASS')
    state_path = output / 'state.json'
    state = json.loads(state_path.read_text()) if state_path.exists() else {}
    def save():
        tmp = state_path.with_suffix('.tmp')
        tmp.write_text(json.dumps(state, indent=2))
        tmp.replace(state_path)
    import mission_runner
    # Leave at least one complete batch's allowance before the pod expires.
    hard_deadline = datetime.datetime(2026, 9, 11, 10, 1, tzinfo=datetime.timezone.utc).timestamp()
    for path in missions:
        name = path.stem
        if state.get(name, {}).get('status') in ('passed_uploaded', 'needs_investigation'):
            continue
        if hard_deadline - time.time() < 43200:
            print('Insufficient pod lifetime for another capped cell', flush=True)
            break
        state[name] = {'status': 'running', 'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat()}
        save()
        with (output / (name + '.log')).open('a') as log:
            rc = subprocess.call(['bash', str(root / 'osmo/workspace/run_held_mission.sh'), str(launcher), str(path)], stdout=log, stderr=subprocess.STDOUT)
        passed = [p for p in (root / 'osmo/results' / name).glob('*/iter_*/iteration.json') if json.loads(p.read_text()).get('status') == 'passed']
        status = 'needs_investigation'
        if passed:
            os.environ['OSMO_MISSION_UPLOAD_DEST'] = '/volume2/coa-sei'
            uploaded = False
            for attempt in range(3):
                try:
                    uploaded = all(mission_runner.upload_iteration(p.parent) for p in passed)
                except Exception as exc:
                    print(f'Upload retry: {type(exc).__name__}', flush=True)
                if uploaded:
                    break
            status = 'passed_uploaded' if uploaded else 'upload_blocked'
        state[name].update(status=status, returncode=rc, results=[str(p.parent) for p in passed])
        save()
        print(name, status, flush=True)
        if status == 'upload_blocked':
            raise SystemExit('Upload needs repair; retaining local results and stopping queue')


if __name__ == '__main__':
    main()
