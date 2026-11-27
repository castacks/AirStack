#!/usr/bin/env python3
"""Read existing logs/process metadata only; output new evidence under RRM."""
import datetime
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'RRM/evidence/M00-C'
PATTERN = re.compile(r'Actuation|HEARTBEAT|MAVLink|mavlink|PX4|px4|ready|Ready|Simulation App|app ready|process started|process has|Starting|startup|start time|ROS2 Bridge|ROS 2|DDS|PhysX|Error|ERROR|TCP|UDP|Connected', re.I)


def run(args, timeout=25):
    p = subprocess.run(args, text=True, capture_output=True, timeout=timeout)
    return {'argv': args, 'exit_code': p.returncode, 'stdout': p.stdout, 'stderr': p.stderr}


def main():
    doc = {'captured_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
           'commands': [], 'filtered_logs': [], 'original_ros_logs': []}
    fmt = '{"name":{{json .Name}},"image":{{json .Image}},"started":{{json .State.StartedAt}},"restart_count":{{.RestartCount}}}'
    doc['commands'].append(run(['docker', 'inspect', '--format', fmt,
                                'isaac-sim-livestream', 'airstack-robot-desktop-1', 'airstack-gcs-1']))
    for container in ['isaac-sim-livestream', 'airstack-robot-desktop-1']:
        p = run(['docker', 'logs', '--timestamps', '--since', '2026-11-27T18:40:00Z',
                 '--until', '2026-11-27T18:50:00Z', container])
        records = []
        for stream in ['stdout', 'stderr']:
            lines = p[stream].splitlines()
            records += [{'stream': stream, 'line': i, 'text': s}
                        for i, s in enumerate(lines, 1) if PATTERN.search(s)]
        doc['filtered_logs'].append({'container': container, 'argv': p['argv'],
                                     'exit_code': p['exit_code'], 'records': records,
                                     'limitation': 'Selected startup lines, Docker receipt clocks plus embedded producer clocks. Absence is not proof an event did not occur.'})
    code = r'''
import json,hashlib,os
from pathlib import Path
paths=['/root/.ros/log/robot_interface_node_20295_1795804990835.log','/root/.ros/log/mavros_node_20294_1795804990840.log']
d={'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'clk_tck':os.sysconf('SC_CLK_TCK'),'logs':[],'processes':[],'clock_map':'No historical realtime-to-monotonic calibration; do not map process ticks onto UTC by current uptime.'}
for name in paths:
 p=Path(name);v={'path':name,'exists':p.is_file()}
 if p.is_file():
  b=p.read_bytes();v.update(bytes=len(b),sha256=hashlib.sha256(b).hexdigest(),text=b.decode(errors='replace'))
 d['logs'].append(v)
for pid in [20295,20294]:
 p=Path('/proc')/str(pid)
 if p.exists():d['processes'].append({'pid':pid,'start_ticks':int((p/'stat').read_text().rsplit(')',1)[1].split()[19]),'exe':str((p/'exe').resolve())})
print(json.dumps(d))
'''
    result = run(['docker', 'exec', '-i', '-e', 'PYTHONDONTWRITEBYTECODE=1',
                  'airstack-robot-desktop-1', 'python3', '-B', '-c', code])
    doc['original_ros_logs'] = json.loads(result['stdout']) if result['exit_code'] == 0 else result
    code = "from pathlib import Path; import os,json; print(json.dumps({'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'clk_tck':os.sysconf('SC_CLK_TCK'),'processes':[{'pid':i,'start_ticks':int((Path('/proc')/str(i)/'stat').read_text().rsplit(')',1)[1].split()[19]),'exe':str((Path('/proc')/str(i)/'exe').resolve())} for i in [210,975] if (Path('/proc')/str(i)).exists()]}))"
    result = run(['docker', 'exec', '-e', 'PYTHONDONTWRITEBYTECODE=1', 'isaac-sim-livestream', 'python3', '-B', '-c', code])
    doc['isaac_processes'] = json.loads(result['stdout']) if result['exit_code'] == 0 else result
    target = OUT / 'startup-source-records.json'
    with target.open('x') as stream:
        json.dump(doc, stream, indent=2)
        stream.write('\n')
    print(json.dumps({'evidence': str(target), 'bytes': target.stat().st_size,
                      'log_counts': [len(x['records']) for x in doc['filtered_logs']]}))


if __name__ == '__main__':
    main()
