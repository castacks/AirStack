#!/usr/bin/env python3
"""Read process/file/package identity without importing simulator or changing state."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


result = {'captured_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
          'processes': [], 'files': [], 'commands': []}
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():
        continue
    try:
        args = (proc / 'cmdline').read_bytes().decode(errors='replace').split('\0')
        exe = (proc / 'exe').resolve()
        if exe.name not in ('px4', 'robot_interface_node') and not any(
                'launch_scripts/' in a and a.endswith('.py') for a in args):
            continue
        stat_fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
        item = {'pid': int(proc.name), 'exe': str(exe), 'exe_sha256': sha(proc / 'exe'),
                'proc_start_ticks': stat_fields[19],
                'args': args if exe.name in ('px4', 'robot_interface_node') else
                        [a for a in args if 'launch_scripts/' in a]}
        libraries = set()
        for line in (proc / 'maps').read_text().splitlines():
            if 'libmavros_interface' in line:
                libraries.add(line.split()[-1])
        item['actuation_libraries'] = [{'path': p, 'sha256': sha(Path(p))}
                                       for p in sorted(libraries)]
        result['processes'].append(item)
    except (OSError, ValueError) as error:
        if proc.exists() and 'exe' in locals() and exe.name in ('px4', 'robot_interface_node'):
            result.setdefault('errors', []).append(str(error))
paths = ['/etc/os-release', '/isaac-sim/VERSION',
         '/root/AirStack/robot/ros_ws/install/interface_bringup/share/interface_bringup/config/px4_config.yaml',
         '/root/AirStack/robot/ros_ws/install/mavros_interface/lib/libmavros_interface.so',
         '/root/AirStack/robot/ros_ws/src/interface/mavros_interface/include/mavros_interface/px4_actuation_guard.hpp',
         '/root/PX4-Autopilot/build/px4_sitl_default/bin/px4',
         '/isaac-sim/AirStack/simulation/isaac-sim/extensions/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd']
for name in paths:
    path = Path(name)
    entry = {'path': name, 'exists': path.is_file()}
    if path.is_file():
        entry.update(sha256=sha(path), bytes=path.stat().st_size, resolved=str(path.resolve()))
        if name.endswith('VERSION') or name.endswith('os-release'):
            entry['text'] = path.read_text()
        if name.endswith('px4_config.yaml'):
            entry['profile_lines'] = [x for x in path.read_text().splitlines()
                                      if 'thrust_scaling' in x or 'setpoint_raw' in x]
        if name.endswith('libmavros_interface.so'):
            output = subprocess.run(['strings', name], capture_output=True, text=True)
            entry['diagnostic_strings'] = [x for x in output.stdout.splitlines()
                if any(y in x for y in ('Actuation NOT_READY', 'startup: %s', 'actuation_status',
                                       'STARTUP_DEADLINE', 'no safe startup', 'actuation_ready'))]
    result['files'].append(entry)
commands = [['dpkg-query', '-W', '-f=${Package} ${Version}\n', 'ros-jazzy-mavros',
             'ros-jazzy-rmw-fastrtps-cpp'],
            ['git', '-C', '/root/PX4-Autopilot', 'rev-parse', 'HEAD'],
            ['git', '-C', '/isaac-sim/AirStack/simulation/isaac-sim/extensions/PegasusSimulator',
             'rev-parse', 'HEAD']]
for command in commands:
    completed = subprocess.run(command, text=True, capture_output=True, timeout=10)
    result['commands'].append({'argv': command, 'exit_code': completed.returncode,
                               'stdout': completed.stdout, 'stderr': completed.stderr})
print(json.dumps(result, indent=2))
