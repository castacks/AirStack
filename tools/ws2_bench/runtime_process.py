"""Process isolation inside a single disposable OSMO task (no Docker socket)."""
import json
import os
from pathlib import Path
import signal
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PROCESSES = {}


def enabled():
    return os.environ.get('WS2_EXECUTION_BACKEND', 'local') == 'process'


def ros_argv(shell, *args):
    return ['bash', str(HERE / 'osmo/ros_env.sh'), shell, *map(str, args)]


def start(name, argv, logfile, *, env=None, cwd=None):
    if name in PROCESSES and PROCESSES[name].poll() is None:
        raise RuntimeError('Process already running: ' + name)
    with Path(logfile).open('w') as stream:
        process = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT,
                                   env=env, cwd=cwd, start_new_session=True)
    PROCESSES[name] = process
    return process


def signal_group(process, sig):
    if process is not None:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass


def stop_process(process):
    if process is None or getattr(process, '_ws2_stopped', False):
        return
    signal_group(process, signal.SIGCONT)
    signal_group(process, signal.SIGTERM)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        signal_group(process, signal.SIGKILL)
        process.wait(timeout=5)
    # A ROS launch process can exit before all descendants. Clean its owned group.
    signal_group(process, signal.SIGKILL)
    process._ws2_stopped = True


def stop_all():
    for process in reversed(list(PROCESSES.values())):
        stop_process(process)
    PROCESSES.clear()


def verify():
    if os.environ.get('WS2_REMOTE_TASK') != '1':
        raise RuntimeError('The process backend is restricted to the disposable WS2 task image')
    for path in [ROOT / 'image_manifest.json', ROOT / 'robot/ros_ws/install/setup.bash',
                 Path('/isaac-sim/python.sh'), Path('/opt/ws2/model/bin/python')]:
        if not path.exists():
            raise RuntimeError('Incomplete WS2 task image: ' + str(path))


def start_stack(config, runtime, env):
    """Use the same launch files, frame rate and bench control mode as local."""
    env = dict(env, ROBOT_NAME='robot_1', ROS_DOMAIN_ID='1', WS2_GT_HOST='127.0.0.1',
               SIM_IP='127.0.0.1', WS2_PX4_DIR='/isaac-sim/PX4-Autopilot',
               WS2_EPISODE_CONFIG=str(runtime / 'episode.json'),
               WS2_PATCH_TEXTURE=str(HERE / 'assets/learned_patch.png'))
    if config.get('camera_pitch_deg', 0):
        subprocess.run(ros_argv('python3 "$1" --pitch "$2" --output /tmp/ws2_camera.urdf',
                                HERE / 'camera_urdf.py', config['camera_pitch_deg']),
                       check=True, env=env, timeout=30, capture_output=True)
        env['URDF_FILE'] = '/tmp/ws2_camera.urdf'
    start('robot', ros_argv('exec ros2 launch desktop_bringup robot.launch.xml role:=full'),
          runtime / 'robot_gt.log', env=env)
    start('truth', ros_argv('exec python3 "$1"', HERE / 'ground_truth.py'),
          runtime / 'ground_truth.log', env=env)
    start('bridge', ros_argv('exec ros2 launch mononav_bridge vision_planner_bridge.launch.xml '
          'vision_planner_name:=ws2_bench vision_planner_execute_commands:=true '
          'vision_planner_max_frame_rate:="$1"', env['WS2_FRAME_RATE']),
          runtime / 'bridge_gt.log', env=env)
    sim_env = env.copy()
    # ROS Python 3.12 must not contaminate Isaac's bundled Python interpreter.
    sim_env.pop('PYTHONPATH', None)
    start('sim', ['/isaac-sim/python.sh', '-u', str(HERE / 'launch_office.py'),
          '--ext-folder', '/isaac-sim/.local/share/ov/data/documents/Kit/shared/exts'],
          runtime / 'simulation_gt.log', env=sim_env, cwd='/isaac-sim')


def worker_env(runtime, run_id):
    env = dict(os.environ, WS2_INFERENCE_DIR=str(runtime / 'inference'), WS2_RUN_ID=run_id)
    for key in ('PYTHONPATH', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
        env.pop(key, None)
    env['PYTHONNOUSERSITE'] = '1'
    # pip CUDA libraries belong to the worker only, never Isaac/ROS.
    libraries = sorted(Path('/opt/ws2/model/lib/python3.10/site-packages/nvidia').glob('*/lib'))
    if libraries:
        env['LD_LIBRARY_PATH'] = ':'.join([*(str(p) for p in libraries),
                                         env.get('LD_LIBRARY_PATH', '')]).rstrip(':')
    return env


def image_manifest():
    return json.loads((ROOT / 'image_manifest.json').read_text())
