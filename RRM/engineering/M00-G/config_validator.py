"""Closed offline Compose policy and provenance validator (JSON/YAML subset only).

Does not invoke Docker/Compose or perform environment interpolation. Supplied
unresolved identities BLOCK; unsafe settings FAIL. Static PASS is not readiness.
"""
import re
from pathlib import Path
from common import canonical, file_sha, load

SERVICES = {'sim', 'mavros', 'guard', 'recorder'}
KEYS = {'profiles','privileged','read_only','cap_drop','cap_add','security_opt','ipc',
        'restart','ports','devices','device_cgroup_rules','user','entrypoint','tmpfs',
        'environment','volumes','image','pull_policy','depends_on','command','networks','network_mode'}
ENV = {'AUTOLAUNCH':'false','ROS_DOMAIN_ID':'1','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp',
       'FASTRTPS_DEFAULT_PROFILES_FILE':'/rrm/config/dds.xml'}
IMAGES = re.compile(r'^[a-z0-9][a-z0-9./:_-]*@sha256:[0-9a-f]{64}$')
PEERS = {'sim':'172.30.50.2','mavros':'172.30.50.3','guard':'172.30.50.4'}


def inspect(compose, manifest, package, repository):
    failures, blocked = [], []
    def require(ok, text):
        if not ok:
            failures.append(text)
    if not isinstance(compose, dict) or not isinstance(manifest, dict):
        return {'status':'FAIL','failures':['Configuration/manifest must be objects'],'blocked':[]}
    require(set(compose)=={'name','services','networks','volumes'}, 'Unknown/missing root configuration; no includes/extends')
    require(compose.get('name')=='rrm-m00g-offline-prototype', 'Unexpected project identity')
    expected_network={'rrm_internal':{'internal':True,'driver':'bridge','ipam':{'config':[{'subnet':'172.30.50.0/24'}]}}}
    require(canonical(compose.get('networks'))==canonical(expected_network), 'Unsafe network: private internal network required')
    require(compose.get('volumes')=={'event_spool':{},'evidence':{},'sim_cache':{}}, 'Unapproved volume driver/options')
    services=compose.get('services',{})
    if not isinstance(services,dict):
        return {'status':'FAIL','failures':failures+['Services must be object'],'blocked':blocked}
    require(set(services)==SERVICES, 'Missing/extra service configuration')
    for name,s in services.items():
        if name not in SERVICES or not isinstance(s,dict):
            failures.append('Unknown service or malformed definition');continue
        expected_keys=KEYS-({'network_mode'} if name!='recorder' else {'networks'})
        require(set(s)==expected_keys, name+': unknown/missing service settings')
        for key,value in {'privileged':False,'read_only':True,'cap_drop':['ALL'],'cap_add':[],
                          'security_opt':['no-new-privileges:true'],'ipc':'private',
                          'restart':'no','pull_policy':'never','ports':[],'devices':[],'device_cgroup_rules':[],
                          'user':'10001:10001','profiles':['rrm-disabled-pending-review'],
                          'tmpfs':['/tmp:rw,noexec,nosuid,nodev,size=67108864']}.items():
            require(canonical(s.get(key))==canonical(value), name+': prohibited '+key)
        if name=='recorder':
            require(s.get('network_mode')=='none', 'Recorder must have no network')
        else:
            require(s.get('networks')=={'rrm_internal':{'ipv4_address':PEERS[name]}},name+': unsafe network attachment')
        env={} if name=='recorder' else dict(ENV)
        if name=='sim':env.update({'NUM_ROBOTS':'1','ISAAC_SIM_HEADLESS':'true','PLAY_SIM_ON_START':'false',
                                  'ISAAC_SIM_SCENE':'/rrm/assets/UNRESOLVED_PINNED_SCENE.usd','ISAAC_SIM_LIVESTREAM':'false'})
        if name=='mavros':env.update({'FCU_URL':'udp://:14541@172.30.50.2:14581','GCS_URL':'','TGT_SYSTEM':'2'})
        require(s.get('environment')==env,name+': unsafe endpoint/DDS/environment override')
        require(s.get('entrypoint')==['/UNRESOLVED_M00G_ENTRYPOINT_NO_EXECUTION_AUTHORIZED'],name+': unreviewed executable entrypoint')
        blocked.append(name+': entrypoint intentionally disabled pending image/startup review')
        cmds={'sim':['/isaac-sim/python.sh','/isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py'],
              'mavros':['ros2','launch','/rrm/config/mavros_only.launch.xml'],
              'guard':['/rrm/install/rrm_startup_guard/lib/rrm_startup_guard/rrm_startup_guard','--ros-args','-r','__ns:=/robot_1/interface','-p','actuation_startup_timeout_s:=10.0','-p','mavros_actuation_config:=/rrm-upstream/px4_config.yaml'],
              'recorder':['python3','/rrm/recorder.py']}
        require(s.get('command')==cmds[name],name+': command override')
        deps={} if name=='recorder' else {'recorder':{'condition':'service_started'}}
        if name=='mavros':deps['sim']={'condition':'service_started'}
        if name=='guard':deps['mavros']={'condition':'service_started'}
        require(s.get('depends_on')==deps,name+': unreviewed startup dependencies')
        mounts=[{'type':'bind','source':'../','target':'/rrm','read_only':True}]
        if name=='recorder':mounts += [{'type':'volume','source':'event_spool','target':'/rrm/spool','read_only':True},{'type':'volume','source':'evidence','target':'/rrm/evidence','read_only':False}]
        else:mounts += [{'type':'volume','source':'event_spool','target':'/rrm/spool','read_only':False}]
        if name=='sim':mounts += [{'type':'bind','source':'../../../../simulation/isaac-sim/launch_scripts','target':'/isaac-sim/AirStack/simulation/isaac-sim/launch_scripts','read_only':True},{'type':'volume','source':'sim_cache','target':'/isaac-sim/.cache','read_only':False}]
        if name=='guard':mounts += [{'type':'bind','source':'../../../../robot/ros_ws/src/interface/interface_bringup/config/px4_config.yaml','target':'/rrm-upstream/px4_config.yaml','read_only':True}]
        require(canonical(s.get('volumes'))==canonical(mounts),name+': unapproved physical-device/host/writable mount')
        image=s.get('image');record=manifest.get('images',{}).get(name,{})
        if not isinstance(image,str) or not IMAGES.fullmatch(image) or not isinstance(record,dict) or record.get('reference')!=image or record.get('identity_status')!='RESOLVED':
            blocked.append(name+': immutable image identity unresolved')
    assets=manifest.get('required_assets')
    if not isinstance(assets,list) or not assets:
        blocked.append('Required asset provenance missing')
    else:
        for asset in assets:
            if not isinstance(asset,dict) or asset.get('identity_status')!='RESOLVED' or not asset.get('path') or not asset.get('sha256'):
                blocked.append('Required asset identity unresolved');continue
            p=(Path(package)/asset['path']).resolve()
            require(p.is_relative_to(Path(package).resolve()),'Asset path escapes declared package')
            if not p.is_relative_to(Path(package).resolve()):continue
            if not p.is_file():blocked.append('Required asset missing: '+asset['path'])
            else:require(file_sha(p)==asset['sha256'],'Asset checksum mismatch: '+asset['path'])
    # Middleware/launch content is checked independently of its recorded checksum.
    import xml.etree.ElementTree as ET
    try:
        xml=(Path(package)/'config/dds.xml').read_text()
        require('<!DOCTYPE' not in xml.upper() and '<!ENTITY' not in xml.upper(), 'Unsafe DDS XML entity input')
        tree=ET.fromstring(xml)
        leaf=lambda element:element.tag.split('}')[-1]
        addresses=[node.text for node in tree.iter() if leaf(node)=='address']
        require(sorted(addresses)==sorted(list(PEERS.values())*2+['0.0.0.0']), 'Unsafe DDS external peers/interfaces')
        allowed={'profiles','transport_descriptors','transport_descriptor','transport_id','type','interfaceWhiteList',
                 'address','participant','rtps','userTransports','useBuiltinTransports','builtin',
                 'avoid_builtin_multicast','metatrafficMulticastLocatorList','metatrafficUnicastLocatorList','port','initialPeersList','locator','udpv4'}
        require(all(leaf(node) in allowed for node in tree.iter()), 'Unapproved DDS routing/discovery element')
        require([node.text for node in tree.iter() if leaf(node)=='useBuiltinTransports']==['false'] and
                [node.text for node in tree.iter() if leaf(node)=='avoid_builtin_multicast']==['true'], 'DDS discovery not confined')
        require([node.text for node in tree.iter() if leaf(node)=='port']==['0'] and
                len([node for node in tree.iter() if leaf(node)=='metatrafficUnicastLocatorList'])==1,
                'DDS unicast listener policy missing')
        launch=ET.parse(Path(package)/'config/mavros_only.launch.xml').getroot()
        args={node.attrib.get('name'):node.attrib.get('value') for node in launch.iter('arg')}
        require(args=={'fcu_url':'udp://:14541@172.30.50.2:14581','gcs_url':'','tgt_system':'2',
                       'namespace':'','respawn_mavros':'false'}, 'Unsafe MAVROS launch endpoints')
        require([node.attrib.get('file') for node in launch.iter('include')]==
                ['$(find-pkg-share interface_bringup)/launch/mavros_px4.launch.xml'] and
                all(node.tag in {'launch','group','push_ros_namespace','include','arg'} for node in launch.iter()),
                'Unexpected command node/launch inclusion')
    except (OSError,ET.ParseError) as exc:
        failures.append('Missing/malformed DDS or MAVROS configuration: '+str(exc))
    required_paths={'upstream_sources':{
        'airstack.sh','robot/ros_ws/src/interface/interface_bringup/launch/mavros_px4.launch.xml',
        'robot/ros_ws/src/interface/mavros_interface/include/mavros_interface/px4_actuation_guard.hpp',
        'simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py'},
        'configuration_inputs':{'config/compose.json','config/environment.env.template','config/dds.xml',
                                'config/mavros_only.launch.xml','guard_host/src/guard_host.cpp',
                                'guard_host/CMakeLists.txt','guard_host/package.xml','COMPONENT_DEPENDENCIES.json',
                                'EVENT_SCHEMA.json','recorder.py','config_validator.py','preflight.py'}}
    for group,base in [('upstream_sources',repository),('configuration_inputs',package)]:
        entries=manifest.get(group)
        if not isinstance(entries,list) or not entries:
            blocked.append('Missing configuration provenance: '+group);continue
        if not required_paths[group] <= {x.get('path') for x in entries if isinstance(x,dict)}:
            blocked.append('Missing required configuration provenance: '+group)
        for entry in entries:
            if not isinstance(entry,dict) or not isinstance(entry.get('path'),str) or not isinstance(entry.get('sha256'),str):
                failures.append('Malformed provenance entry');continue
            p=(Path(base)/entry['path']).resolve()
            if not p.is_relative_to(Path(base).resolve()):
                failures.append('Provenance path escapes root');continue
            require(p.is_file() and file_sha(p)==entry['sha256'],'Unexpected '+group+' checksum/change: '+entry['path'])
    require(manifest.get('repository_revision')=='dc17fa3918c5e8af12ecabade3cee3efdc223f85','Unexpected repository baseline')
    status='FAIL' if failures else ('BLOCKED' if any('identity' in x or 'provenance' in x or 'asset missing' in x for x in blocked) else 'PASS')
    return {'status':status,'static_policy':'FAIL' if failures else 'PASS',
            'failures':failures,'blocked':blocked,'runtime_containment':'NOT VERIFIED',
            'note':'Static PASS can include deliberately disabled entrypoints; it never permits execution.'}


def check_package(package, repository):
    try:
        return inspect(load(Path(package)/'config/compose.json'),load(Path(package)/'PROVENANCE.json'),package,repository)
    except (ValueError,OSError,TypeError,KeyError,AttributeError) as exc:
        return {'status':'FAIL','static_policy':'FAIL','failures':[str(exc)],'blocked':[],
                'runtime_containment':'NOT VERIFIED'}


if __name__=='__main__':
    import json
    p=Path(__file__).resolve().parent
    result=check_package(p,p.parents[2])
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['status']=='PASS' else 2)
