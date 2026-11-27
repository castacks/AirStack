#!/usr/bin/env python3
"""Passive candidate discovery: Docker metadata, proc/sys files and existing logs only.
No docker exec, probes, ROS clients, vehicle access, service or infrastructure mutations.
"""
from datetime import datetime,timezone
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'RRM/evidence/M00-E1B'
NAMES=['isaac-sim-livestream','airstack-robot-desktop-1','airstack-gcs-1']
ENV_KEYS={'ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','ROS_DISCOVERY_SERVER','ROBOT_NAME','ISAAC_SIM_SCENE','ISAAC_SIM_STAGE_SCALE','ISAAC_SIM_SCRIPT_NAME','PLAY_SIM_ON_START','PX4_SIM_MODEL','PX4_SYS_AUTOSTART','PX4_PARAM_SET','PX4_PHYSICS_HZ','PX4_RENDERING_HZ','OSMO_WORKFLOW_ID','OSMO_WORKFLOW_NAME','OSMO_TASK_NAME','OSMO_POOL','OSMO_CONFIG_FILE_DIR'}
NEEDLES=['Simulation App Startup Complete','] app ready','Simulation App Starting','PX4 node at','px4 starting.','Simulator connected on TCP port','Received first hearbeat','on udp port 14581 remote port 14541','Startup script returned successfully','Ready for takeoff!','Actuation NOT_READY: STARTUP_DEADLINE','CON: Got HEARTBEAT','FCU URL:']
def stamp():return datetime.now(timezone.utc).isoformat()
def digest(data):return hashlib.sha256(data).hexdigest()
def selected_env(data):
    entries=data.split(b'\0') if isinstance(data,bytes) else [s.encode() for s in data or []]
    return {k.decode():v.decode(errors='replace') for k,_,v in (s.partition(b'=') for s in entries) if k.decode(errors='replace') in ENV_KEYS}
def query(argv):
    p=subprocess.run(argv,capture_output=True,text=True,timeout=30)
    return p

def file_record(path,content=False,limit=32*1024*1024):
    p=Path(path);r={'path':str(p),'acquired_utc':stamp()}
    try:
        s=p.stat();r.update(bytes=s.st_size,mtime_ns=s.st_mtime_ns)
        if not stat.S_ISREG(s.st_mode):r['result']='NOT_REGULAR_FILE_NOT_OPENED';return r
        if s.st_size>limit:r['result']='SKIPPED_SIZE_BOUND';return r
        raw=p.read_bytes();r.update(result='READ',sha256=digest(raw),read_bytes=len(raw),stable_during_read=p.stat().st_mtime_ns==s.st_mtime_ns)
        if content:r['text']=raw.decode(errors='replace')
    except OSError as ex:r.update(result='INACCESSIBLE',error=type(ex).__name__+': '+str(ex))
    return r

def decode_endpoint(text,ipv6=False):
    addr,port=text.split(':')
    if ipv6:raw=b''.join(bytes.fromhex(addr[i:i+8])[::-1] for i in range(0,32,8))
    else:raw=bytes.fromhex(addr)[::-1]
    return str(ipaddress.ip_address(raw)),int(port,16)

def sockets(root):
    out=[]
    for proto in ['tcp','tcp6','udp','udp6']:
        p=root/'net'/proto
        try:
            lines=p.read_text().splitlines();entries=[]
            for line in lines[1:]:
                f=line.split();a,ap=decode_endpoint(f[1],proto.endswith('6'));b,bp=decode_endpoint(f[2],proto.endswith('6'))
                entries.append(dict(local_address=a,local_port=ap,remote_address=b,remote_port=bp,state_hex=f[3],uid=f[7],inode=f[9]))
            out.append(dict(path=str(p),protocol=proto,raw_lines=lines,entries=entries))
        except OSError as ex:out.append(dict(path=str(p),error=str(ex)))
    for name in ['route','ipv6_route','dev']:
        p=root/'net'/name
        try:out.append(dict(path=str(p),raw_lines=p.read_text().splitlines()))
        except OSError as ex:out.append(dict(path=str(p),error=str(ex)))
    return out

def devices(path):
    out=[]
    try:
        for p in sorted(Path(path).iterdir()):
            s=p.lstat()
            if stat.S_ISCHR(s.st_mode) or stat.S_ISBLK(s.st_mode) or any(x in p.name for x in ['ttyUSB','ttyACM','serial','usb','gpio','i2c','spi','can']):
                out.append(dict(name=p.name,type='symlink' if p.is_symlink() else ('character' if stat.S_ISCHR(s.st_mode) else 'block' if stat.S_ISBLK(s.st_mode) else 'directory'),major=os.major(s.st_rdev),minor=os.minor(s.st_rdev),target=os.readlink(p) if p.is_symlink() else None))
    except OSError as ex:return dict(path=str(path),error=str(ex))
    return dict(path=str(path),entries=out,note='Device metadata only; no device nodes opened. Absence at this instant is not denial of future hardware access.')

def main():
    commands=[]
    def command(argv):
        p=query(argv);commands.append(dict(argv=argv,exit_code=p.returncode,stderr=p.stderr));return p
    p=command(['docker','ps','-a','--no-trunc','--format','{{json .}}'])
    containers_inventory=[json.loads(s) for s in p.stdout.splitlines()] if p.returncode==0 else []
    # Only identity/status fields retained, not arbitrary command strings.
    containers_inventory=[{k:c.get(k) for k in ['ID','Names','Image','Status','State','Networks']} for c in containers_inventory]
    p=command(['docker','inspect',*NAMES]);all_meta=json.loads(p.stdout) if p.returncode==0 else []
    result=dict(captured_utc=stamp(),scope='Existing candidate only; intended target not designated',target_designation='UNVERIFIED',docker_inventory=containers_inventory,containers=[],host={})
    for c in all_meta:
        pid=c['State']['Pid'];proc=Path('/proc')/str(pid);root=proc/'root';h=c['HostConfig']
        item=dict(name=c['Name'],id=c['Id'],init_host_pid=pid,started=c['State']['StartedAt'],restart_count=c['RestartCount'],image_config_id=c['Image'],requested_image=c['Config']['Image'],privileged=h['Privileged'],network_mode=h['NetworkMode'],devices=h.get('Devices'),device_requests=h.get('DeviceRequests'),binds=h.get('Binds'),mounts=c['Mounts'],published_ports=c['NetworkSettings']['Ports'],config_environment=selected_env(c['Config'].get('Env')),processes=[],files=[],log_directories=[],devices_metadata=devices(root/'dev'),socket_tables=sockets(proc))
        p=command(['docker','top',c['Id'],'-eo','pid,comm']);item['process_listing_raw']=p.stdout
        for line in p.stdout.splitlines()[1:]:
            f=line.split(None,1)
            if len(f)!=2 or not f[0].isdigit():continue
            q=Path('/proc')/f[0];r=dict(host_pid=int(f[0]),comm=f[1])
            try:
                r['exe']=os.readlink(q/'exe');r['environment']=selected_env((q/'environ').read_bytes())
                r['cgroup']=(q/'cgroup').read_text();r['stat_raw']=(q/'stat').read_text()
                args=(q/'cmdline').read_bytes().decode(errors='replace').split('\0')
                # Paths only; no general argv or arbitrary environment persisted.
                r['configuration_arguments']=[a for a in args if a.startswith('/') and a.endswith(('.py','.xml','.yaml','.params','/rcS'))]
                if f[1] in {'px4','robot_interface','mavros_node'}:r['executing_binary']=file_record(q/'exe')
                if f[1]=='robot_interface':
                    lines=[s for s in (q/'maps').read_text().splitlines() if 'libmavros_interface.so' in s]
                    r['guard_library_maps']=lines
                    for path in sorted({s.split()[-1] for s in lines}):item['files'].append(file_record(root/path.lstrip('/')))
                if f[1] in {'mavros_node','ddsrouter','robot_interface','python3'}:
                    # Snapshot relevant already-created parameter/DDS files, not device or live API streams.
                    for arg in r['configuration_arguments']:
                        if arg.endswith(('.yaml','.params','.xml')):
                            rec=file_record(root/arg.lstrip('/'),content=True,limit=1024*1024)
                            if rec.get('text'):
                                text=rec.pop('text');rec['selected_lines']=[s for s in text.splitlines() if re.search(r'fcu_url|gcs_url|target_system|target_component|thrust_scaling|actuation_|use_sim_time|domain|address|port|kind|type:|name:',s)]
                            item['files'].append(rec)
            except OSError as ex:r['access_error']=str(ex)
            item['processes'].append(r)
        fixed=['etc/os-release','opt/ros/jazzy/share/mavros/package.xml','opt/ros/jazzy/share/rmw_fastrtps_cpp/package.xml','isaac-sim/VERSION','isaac-sim/PX4-Autopilot/.git/HEAD','isaac-sim/.ros/fastdds.xml']
        for rel in fixed:
            rec=file_record(root/rel,content=True,limit=65536)
            if rec.get('text') and rel.endswith('package.xml'):
                try:rec['package_version']=ET.fromstring(rec.pop('text')).findtext('version')
                except ET.ParseError:rec['parse_error']='Malformed installed XML'
            item['files'].append(rec)
            if rel.endswith('.git/HEAD') and rec.get('text','').startswith('ref: '):
                item['files'].append(file_record(root/'isaac-sim/PX4-Autopilot/.git'/rec['text'][5:].strip(),content=True,limit=4096))
        for rel in ['root/.ros/log','isaac-sim/.ros/log','root/AirStack/.rrm-artifacts','isaac-sim/.nvidia-omniverse/logs']:
            directory=root/rel
            try:item['log_directories'].append(dict(path=str(directory),resolved=str(directory.resolve()),entries=[p.name for p in sorted(directory.iterdir())][:80],truncated=len(list(directory.iterdir()))>80))
            except OSError as ex:item['log_directories'].append(dict(path=str(directory),error=type(ex).__name__))
        p=command(['docker','logs','--timestamps','--tail','2000',c['Id']])
        lines=(p.stdout+'\n'+p.stderr).splitlines()
        # Retain only named existing startup/timing events, never arbitrary log content.
        item['existing_log_excerpt']=dict(command=['docker','logs','--timestamps','--tail','2000',c['Id']],exit_code=p.returncode,selected_lines=[s for s in lines if any(n in s for n in NEEDLES)],returned_line_count=len(lines),complete_startup_coverage=False,note='Bounded existing-log tail; missing event here does not prove absence in full logs. No follow or process attachment.')
        result['containers'].append(item)
    result['host']['os_release']=file_record('/etc/os-release',content=True,limit=65536)
    result['host']['architecture']=os.uname().machine;result['host']['kernel']=os.uname().release
    result['host']['boot_id']=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    result['host']['device_metadata']=devices('/dev')
    result['host']['network_tables']=sockets(Path('/proc/self'))
    result['host']['resource_snapshot']={str(p):p.read_text() for p in [Path('/proc/pressure/cpu'),Path('/proc/pressure/memory'),Path('/proc/pressure/io')] if p.exists()}
    result['host']['pid1_selected_environment']=selected_env(Path('/proc/1/environ').read_bytes())
    result['host']['workflow_access_note']='OSMO_CONFIG_FILE_DIR points to /osmo/login/config with login.yaml/config.yaml. Credential contents intentionally not read; no authenticated control-plane request.'
    result['commands']=commands
    # Docker logs uses stderr for ordinary logs; command stderr is NOT persisted unfiltered.
    for r in result['commands']:
        if r['argv'][1:2]==['logs']:r['stderr']='EXCLUDED_RAW_LOG_STREAM; selected event lines retained in container record'
    with (OUT/'passive-discovery.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps({'containers_inspected':len(result['containers']),'artifact':'RRM/evidence/M00-E1B/passive-discovery.json','target_designation':'UNVERIFIED','active_operations':0}))
if __name__=='__main__':main()
