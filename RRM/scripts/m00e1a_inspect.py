#!/usr/bin/env python3
"""Passive reference metadata only; no exec, service control, ROS or network probes."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'RRM/evidence/M00-E1A'
NAMES=['isaac-sim-livestream','airstack-robot-desktop-1','airstack-gcs-1']
SAFE_ENV={'ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE',
          'FASTDDS_DEFAULT_PROFILES_FILE','ROS_DISCOVERY_SERVER','ROBOT_NAME','NUM_ROBOTS',
          'ISAAC_SIM_SCENE','ISAAC_SIM_STAGE_SCALE','ISAAC_SIM_SCRIPT_NAME','ISAAC_SIM_HEADLESS',
          'ISAAC_SIM_AUTOPLAY','ISAAC_SIM_IMAGE','PX4_SYS_AUTOSTART','PX4_SIM_MODEL',
          'OSMO_WORKFLOW_ID','OSMO_WORKFLOW_NAME','OSMO_TASK_NAME','OSMO_POOL'}
SAFE_LABELS={'com.docker.compose.project','com.docker.compose.service',
             'com.docker.compose.project.working_dir','com.docker.compose.project.config_files',
             'com.docker.compose.config-hash','com.docker.compose.version'}

def query(argv):
    # Only call sites below supply fixed Docker inspect/identity arguments. Raw full inspect
    # data stays in memory: credentials/environment not selected below are never persisted.
    p=subprocess.run(argv,capture_output=True,text=True,timeout=20)
    return p.returncode,p.stdout,p.stderr

def selected_env(entries):
    return {k:v for k,_,v in (entry.partition('=') for entry in entries or []) if k in SAFE_ENV}

def passive_snapshot():
    rc,raw,err=query(['docker','inspect',*NAMES])
    result={'captured_utc':datetime.now(timezone.utc).isoformat(),'scope':'Existing reference only; target unassigned; no safety certification',
            'commands':[{'argv':['docker','inspect',*NAMES],'exit_code':rc,'stderr':err}], 'containers':[],'images':[],'networks':[]}
    if rc:return result
    for c in json.loads(raw):
        h=c['HostConfig'];config=c['Config'];state=c['State']
        result['containers'].append(dict(id=c['Id'],name=c['Name'],image_config_id=c['Image'],
            requested_image=config.get('Image'),started=state.get('StartedAt'),restart_count=c['RestartCount'],running=state.get('Running'),
            privileged=h.get('Privileged'),network_mode=h.get('NetworkMode'),pid_mode=h.get('PidMode'),ipc_mode=h.get('IpcMode'),
            cap_add=h.get('CapAdd'),cap_drop=h.get('CapDrop'),devices=h.get('Devices'),device_requests=h.get('DeviceRequests'),
            binds=h.get('Binds'),mounts=c.get('Mounts'),security_opt=h.get('SecurityOpt'),restart_policy=h.get('RestartPolicy'),
            memory_limit_bytes=h.get('Memory'),memory_swap=h.get('MemorySwap'),nano_cpus=h.get('NanoCpus'),cpu_quota=h.get('CpuQuota'),
            cpu_period=h.get('CpuPeriod'),cpuset=h.get('CpusetCpus'),pids_limit=h.get('PidsLimit'),
            environment=selected_env(config.get('Env')),labels={k:v for k,v in (config.get('Labels') or {}).items() if k in SAFE_LABELS},
            entrypoint=config.get('Entrypoint'),command_sha256=hashlib.sha256(json.dumps(config.get('Cmd'),sort_keys=True).encode()).hexdigest(),
            command_note='Opaque Cmd identity only; reviewed exact effective launcher remains unresolved',
            working_dir=config.get('WorkingDir'),published_ports=c['NetworkSettings'].get('Ports'),
            networks={k:{f:v.get(f) for f in ['NetworkID','IPAddress','Gateway','Aliases']} for k,v in c['NetworkSettings'].get('Networks',{}).items()}))
    for ident in sorted({c['image_config_id'] for c in result['containers']}):
        argv=['docker','image','inspect',ident];rc,raw,err=query(argv)
        result['commands'].append(dict(argv=argv,exit_code=rc,stderr=err))
        if rc:continue
        c=json.loads(raw)[0]
        result['images'].append({k:c.get(k) for k in ['Id','RepoDigests','RepoTags','Created','Architecture','Os']})
    for network in sorted({c['network_mode'] for c in result['containers']}):
        argv=['docker','network','inspect',network];rc,raw,err=query(argv)
        result['commands'].append(dict(argv=argv,exit_code=rc,stderr=err))
        if rc:continue
        c=json.loads(raw)[0]
        result['networks'].append({k:c.get(k) for k in ['Id','Name','Driver','Scope','Internal','Attachable','EnableIPv6','IPAM','Options','Containers']})
    result['host']={'platform':platform.platform(),'os_release':{},'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                    'visible_cpu_count':os.cpu_count(),'memtotal':[x for x in Path('/proc/meminfo').read_text().splitlines() if x.startswith('MemTotal:')],
                    'cgroup_membership':Path('/proc/self/cgroup').read_text(),'cgroup_visible_root':{},'workflow_environment':selected_env([k+'='+v for k,v in os.environ.items()])}
    for path in ['/etc/os-release','/sys/fs/cgroup/cpu.max','/sys/fs/cgroup/memory.max','/sys/fs/cgroup/memory.swap.max','/sys/fs/cgroup/cpuset.cpus.effective','/sys/fs/cgroup/pids.max']:
        p=Path(path)
        if p.exists():
            if path=='/etc/os-release':
                result['host']['os_release']={k:v.strip('"') for k,_,v in (line.partition('=') for line in p.read_text().splitlines()) if k in {'ID','VERSION_ID','PRETTY_NAME'}}
            else:result['host']['cgroup_visible_root'][path]=p.read_text().strip()
    argv=['nvidia-smi','--query-gpu=name,uuid,driver_version,memory.total','--format=csv,noheader']
    rc,raw,err=query(argv);result['commands'].append(dict(argv=argv,exit_code=rc,stderr=err));result['gpu_identity']=raw.strip() if rc==0 else None
    result['limits']='Visible enclosing cgroup and Docker configuration only; no OSMO scheduler allotment/host exclusivity proof; shared resources and physical/network isolation remain unverified.'
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--after',action='store_true');args=p.parse_args()
    target=OUT/('passive-after.json' if args.after else 'passive-before.json')
    with target.open('x') as f:json.dump(passive_snapshot(),f,indent=2);f.write('\n')
    print(json.dumps({'artifact':str(target.relative_to(ROOT)),'active_operations':0,'target_verified':False}))
if __name__=='__main__':main()
