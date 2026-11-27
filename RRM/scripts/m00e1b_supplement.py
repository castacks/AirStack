#!/usr/bin/env python3
"""Read-only follow-up: installed executables/configuration and device FD metadata."""
from datetime import datetime,timezone
import importlib.util
import json
import os
from pathlib import Path
import re
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'RRM/evidence/M00-E1B'
s=importlib.util.spec_from_file_location('discovery',ROOT/'RRM/scripts/m00e1b_discover.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
def main():
    before=json.loads((E/'passive-discovery.json').read_text());out=dict(captured_utc=datetime.now(timezone.utc).isoformat(),scope='Existing candidate only; no target designation',process_artifacts=[],static_workflow_sources=[])
    for c in before['containers']:
        root=Path('/proc')/str(c['init_host_pid'])/'root'
        for p in c['processes']:
            q=Path('/proc')/str(p['host_pid']);rec=dict(container=c['name'],host_pid=p['host_pid'],comm=p['comm'],files=[])
            try:
                if p['comm'] in {'px4','robot_interface','mavros_node'}:rec['files'].append(m.file_record(q/'exe',limit=128*1024*1024))
                args=(q/'cmdline').read_bytes().decode(errors='replace').split('\0')
                rec['selected_settings']=[a for a in args if any(k in a for k in ['fcu_url:=','gcs_url:=','__ns:=','use_sim_time:='])]
                for i,a in enumerate(args[:-1]):
                    if a=='--params-file':
                        f=m.file_record(root/args[i+1].lstrip('/'),content=True,limit=1024*1024)
                        if f.get('text'):
                            text=f.pop('text');f['selected_lines']=[s for s in text.splitlines() if re.search(r'fcu_url|gcs_url|target_system|target_component|thrust_scaling|actuation_|use_sim_time|type:|profile',s)]
                        rec['files'].append(f)
                if p['comm']=='robot_interface':
                    rec['guard_maps']=[s for s in (q/'maps').read_text().splitlines() if 'libmavros_interface.so' in s]
                    for lib in sorted({s.split()[-1] for s in rec['guard_maps']}):rec['files'].append(m.file_record(root/lib.lstrip('/'),limit=128*1024*1024))
                # Socket inode ownership and device-node paths only; no FD/device contents read.
                fds=[]
                for fd in (q/'fd').iterdir():
                    try:target=os.readlink(fd)
                    except OSError:continue
                    if target.startswith(('socket:[','/dev/tty','/dev/serial','/dev/bus/usb','/dev/can')):fds.append(dict(fd=fd.name,target=target))
                rec['selected_fd_targets']=fds
            except OSError as ex:rec['access_error']=str(ex)
            out['process_artifacts'].append(rec)
    for p in sorted((ROOT/'osmo/workflows').glob('*.yaml')):
        f=m.file_record(p,content=True,limit=131072);text=f.pop('text','')
        f['selected_lines']=[s for s in text.splitlines() if not s.lstrip().startswith('#') and re.match(r'^\s*(name:|image:|privileged:|AUTOLAUNCH:|AIRSTACK_BRANCH:|ISAAC_SIM_|cpu:|gpu:|memory:|storage:)',s)]
        f['state']='SOURCE_DEFINITION_ONLY_NOT_DEPLOYMENT_EVIDENCE';out['static_workflow_sources'].append(f)
    out['limitations']=['Executable file hashes identify backing files via proc, not in-memory/reproducible build attestation.','Selected params are startup-file contents, not service readback of current effective parameters.','Open socket/device FDs and namespace tables are an instantaneous bounded observation, not proof of physical path exclusion.','No credential/config login file read; no active socket, ROS, OSMO or vehicle query.']
    with (E/'installed-artifacts.json').open('x') as f:json.dump(out,f,indent=2);f.write('\n')
    print(json.dumps({'artifact':'RRM/evidence/M00-E1B/installed-artifacts.json','active_operations':0}))
if __name__=='__main__':main()
