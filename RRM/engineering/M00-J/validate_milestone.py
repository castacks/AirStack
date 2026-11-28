#!/usr/bin/env python3
"""Validate M00-J passive build-readiness evidence; no runtime operations; no runtime operations."""
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
from collections import Counter

ROOT = Path(__file__).resolve().parents[3]
E = ROOT / 'RRM/evidence/M00-J'
V = ROOT / 'RRM/engineering/M00-J'
PARENT = 'd678846801e859c68902bbb9c2e6f0c77760e8b0'
SUBJECT = 'RRM-F00 M00-J: verify native build environment readiness'


def must(ok, reason):
    if not ok:
        raise ValueError(reason)


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def load(path):
    return json.loads(path.read_text())


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def validate():
    checks = []

    def passed(name, detail):
        checks.append(dict(check=name, status='PASS', detail=detail))

    before = load(E / 'repository-before.json')
    must(before['head'] == PARENT and before['branch'] == 'ore_proj'
         and not before['status_porcelain'], 'Starting checkout mismatch')
    must(git('branch', '--show-current').decode().strip() == 'ore_proj', 'Branch mismatch')
    committed = git('rev-parse', 'HEAD').decode().strip() != PARENT
    if committed:
        must(git('rev-parse', 'HEAD^').decode().strip() == PARENT, 'Unexpected parent')
        must(git('log', '-1', '--format=%s').decode().strip() == SUBJECT, 'Unexpected subject')
    must(load(E / 'starting-integrity.json')['result'] == 'PASS', 'Prior M00-I validator not passing')
    for name, identity in before['tracked_entries'].items():
        if name == 'RRM/PROJECT_STATUS.md':
            continue
        path = ROOT / name
        if identity['type'] == 'file':
            must(path.is_file() and not path.is_symlink()
                 and sha(path.read_bytes()) == identity['sha256']
                 and path.stat().st_mode & 0o777 == identity['mode'], 'Protected change: ' + name)
        elif identity['type'] == 'symlink':
            must(path.is_symlink() and os.readlink(path) == identity['target'], 'Symlink change: ' + name)
        else:
            must(subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD']).decode().strip()
                 == identity['head'], 'Submodule change: ' + name)
            must(not subprocess.check_output(['git', '-C', str(path), 'status', '--porcelain']),
                 'Dirty submodule: ' + name)
    for filename in ['source-manifest.json', 'input-inventory.json']:
        for entry in load(E / filename)['files']:
            data = (ROOT / entry['path']).read_bytes()
            must(len(data) == entry['bytes'] and sha(data) == entry['sha256'],
                 'Source/input change: ' + entry['path'])
    inputs = load(E / 'input-inventory.json')['files']
    must(inputs == load(ROOT / 'RRM/evidence/GOV-001B/input-inventory.json')['files'],
         'Controlled input changed from prior milestone')
    must({x['path'] for x in inputs} == {str(p.relative_to(ROOT)) for p in
         (ROOT / 'RRM/RRM-C001-F00-EXT').rglob('*') if p.is_file()}, 'Input set mismatch')
    pins = {'M00-B': '93b8196e835effc3da34cac5677eab4d30c1378a',
            'M00-C': 'b4ac9467b073b6ae7927a47b939f46e9407cb6c0',
            'M00-D': '5b11d4d348db399e0e3ae638e318da9d7079ace6',
            'M00-E1': '31e33893ef2fa0f724862a980f6b5db0d62509d2',
            'M00-E1A': 'df8ebd18a6e16909cca461e18dfba882b53c8120',
            'M00-E1B': '4326dd079a2d80518983a5b793b9bab7f751a5db',
            'M00-E1C': '9d60ee46b3348fbaa1811a5b234f80446b62d5bd', 'M00-E1D': 'e6f0c81eff15538809b9745cef665de04ba4e796', 'M00-E1E': '0fa876fd8191be61a3e6724e1a3c93fe64510f01', 'GOV-001A': 'a72b8f412f2567e870903e2f4fec8082bc437e21', 'GOV-001B': '381db1c4dcdd1bb71d047176e681bd18c72e342d', 'GOV-002': '6c63054b2d1ac0dee0c5840ed3d4c31083874177', 'M00-F': 'dc17fa3918c5e8af12ecabade3cee3efdc223f85', 'M00-G': '45d026e2b8c46b0e1c44013d8dd9d7ed49e3d5ac', 'M00-H': 'f5b14e08cbf0ef54bfe3c986ea377fbe50adbf61', 'M00-I': PARENT}
    count = 0
    for milestone, pin in pins.items():
        for line in (ROOT / f'RRM/evidence/{milestone}/SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ', 1)
            must(sha(git('show', pin + ':' + name)) == digest, 'Sealed history mismatch: ' + name)
            count += 1
    passed('starting_and_protected_integrity', f'Clean pinned M00-I start; '
           f'{len(before["tracked_entries"])-1} protected entries, {count} pinned sealed checksums '
           f'and {len(inputs)} unchanged controlled input files')


    import re
    host=load(E/'M00J_PASSIVE_HOST_SNAPSHOT.json')
    old=load(E/'M00J_HISTORICAL_IMAGE_REFERENCE.json')
    native=load(E/'M00J_HOST_DEPENDENCY_SNAPSHOT.json')
    policy=load(V/'M00J_COMPILATION_ONLY_POLICY.json')
    activity=load(E/'M00J_ACTIVITY_RECORD.json')
    values={}
    for line in (ROOT/'.env').read_text().splitlines():
        match=re.fullmatch(r'(PROJECT_NAME|VERSION|DOCKER_IMAGE_BUILD_MODE|PROJECT_DOCKER_REGISTRY)="([^"\n]+)"',line)
        if match:values[match.group(1)]=match.group(2)
    expected=(values['PROJECT_DOCKER_REGISTRY']+'/'+values['PROJECT_NAME']+':v'+values['VERSION']
              +'_robot-x86-64_'+values['DOCKER_IMAGE_BUILD_MODE'])
    image=policy['image']
    must(expected==image['declared_tag']=='airlab-docker.andrew.cmu.edu/airstack/airstack:v0.20.8_robot-x86-64_dev',
         'Declared tag interpolation mismatch')
    historical=load(ROOT/old['image_source'])
    must(old['image_capture_utc']==historical['captured_utc'] and old['image']==historical['images'][0],
         'Historical image identity drift')
    must(expected in old['image']['RepoTags'] and old['image']['Id']==image['historical_config_id']
         and old['image']['RepoDigests']==[] and old['image']['Architecture']=='amd64',
         'Historical reference/image digest conflation')
    must(image['approved_current_local_image_id'] is None and image['registry_manifest_digest'] is None,
         'Invented current image identity')
    current=host['daemon_inspection']
    must(current['status']=='STOPPED_AT_PRIVILEGED_CONTROL_SOCKET' and current['current_images'] is None
         and current['current_containers'] is None and current['server_version'] is None,'Daemon inspection overclaim')
    socket=host['paths']['/var/run/docker.sock']
    must(socket['socket'] and socket['uid']==0 and socket['mode']=='0o660','Socket evidence mismatch')
    must(host['daemon_configuration']['default-runtime']=='nvidia','NVIDIA default evidence lost')
    must(host['uname']['machine']=='x86_64' and host['os_release']['VERSION_ID']=='24.04'
         and 'kubepods' in host['cgroup'] and host['pid1_comm']=='osmo_exec','Host provenance mismatch')
    must(policy['host']['approved_daemon_host_identity'] is None and policy['host']['permission_reference'] is None
         and policy['host']['observed_workspace_node']==host['uname']['node'],'Invented host/permission')
    passed('development_identity_and_passive_availability','Exact default development tag reconstructed from protected sources; '
           'matching historical config ID/platform preserved as historical only; current daemon/image state NOT INSPECTABLE; '
           'system socket stop honored; managed workspace identity distinct from unknown execution host')
    required={'cxx','cmake','ament_cmake','rclcpp','ament_index_cpp','mavros_msgs','rcl_interfaces','std_msgs','yaml-cpp'}
    must(set(native['native_missing'])==required and native['readiness']=='BLOCKED','Host dependency mismatch')
    coverage=rows(V/'M00J_DEPENDENCY_COVERAGE.csv')
    must(len(coverage)==9 and {x['prerequisite'] for x in coverage}==
         {'C++ compiler','CMake','ament_cmake','rclcpp','ament_index_cpp','mavros_msgs','rcl_interfaces','std_msgs','yaml-cpp'},
         'Nine-prerequisite horizon')
    statuses={'VERIFIED PRESENT','DECLARED PRESENT — NOT EXECUTION VERIFIED','MISSING','VERSION UNRESOLVED','NOT INSPECTABLE'}
    must(all(x['verification_status'] in statuses and x['verification_status']=='NOT INSPECTABLE'
             and x['observed_current_host'].startswith('MISSING') and x['declared_availability']
             and x['source_of_evidence'] and x['version_status']=='VERSION UNRESOLVED' for x in coverage),
         'Declaration/current-host/image/package conflation')
    passed('nine_dependency_coverage','Nine candidate-image requirements NOT INSPECTABLE; declarations/transitive '
           'expectations separate from nine missing host prerequisites; package/version/generated-typesupport proof absent')
    must(policy['schema']=='M00J_COMPILATION_ONLY_DESIGN_V1_NOT_A_DEPLOYMENT_MANIFEST'
         and not policy['execution_authorized'] and policy['status'].startswith('BLOCKED'), 'Runnable/unblocked policy')
    container=policy['container']
    must(container['runtime']=='runc' and not container['runtime_availability_verified']
         and not container['privileged'] and container['read_only_root'] and container['user']=='10001:10001'
         and not container['user_mapping_permissions_verified'],'Privilege/runtime/non-root drift')
    must(container['network']=='none' and container['publish_ports']==[] and container['ipc']=='private'
         and container['uts']=='private' and container['cgroup_namespace']=='private'
         and container['pid']=='private default; no host or container sharing','Namespace/network drift')
    must(container['cap_drop']==['ALL'] and container['cap_add']==[]
         and container['security_opt']==['no-new-privileges:true']
         and 'UNVERIFIED' in container['seccomp'] and 'UNVERIFIED' in container['apparmor_or_equivalent'],
         'Capability/security policy drift')
    must(container['devices']==container['device_requests']==container['device_cgroup_rules']==[]
         and not container['gpus'] and not container['daemon_socket_mount'],'Device/socket grant')
    must(container['restart']=='no' and not container['stdin_open'] and not container['tty']
         and container['image_command_overridden'] and container['image_entrypoint_overridden']
         and container['image_healthcheck_disabled'] and not container['host_config_inheritance'], 'Startup inheritance')
    mounts=policy['mounts']
    must(len(mounts)==3 and [x['target'] for x in mounts]==['/inputs/guard_host',
         '/inputs/upstream/include/mavros_interface/px4_actuation_guard.hpp','/output']
         and [x['read_only'] for x in mounts]==[True,True,False]
         and all(x['type']=='bind' and x['propagation']=='rprivate' and x['recursive_submounts']=='forbidden'
                 for x in mounts),'Mount policy drift')
    must(mounts[0]['source']==str(ROOT/'RRM/engineering/M00-I/guard_host')
         and mounts[1]['source']==str(ROOT/'robot/ros_ws/src/interface/mavros_interface/include/mavros_interface/px4_actuation_guard.hpp')
         and mounts[2]['source']==str(ROOT/'RRM/build-output/M00-native-guard-next'), 'Input/output path drift')
    must(policy['output']['host_path']==mounts[2]['source'] and policy['output']['only_writable_host_mount']
         and not policy['output']['created_in_m00j'] and policy['output']['temporary_dir']=='/output/tmp'
         and policy['output']['home']=='/output/home','Output/scratch confinement drift')
    entry=policy['entrypoint']
    must(entry[:2]==['/usr/bin/env','-i'] and entry[-4:]==['/bin/bash','--noprofile','--norc','-c']
         and 'HOME=/output/home' in entry and 'TMPDIR=/output/tmp' in entry
         and 'CMAKE_PREFIX_PATH=/opt/ros/jazzy' in entry,'Clean entrypoint missing')
    command=policy['command']
    expected_command=('mkdir -p /output/tmp /output/home && cmake -S /inputs/guard_host -B /output/native '
        '-DRRM_UPSTREAM_INCLUDE_DIR=/inputs/upstream/include -DCMAKE_PREFIX_PATH=/opt/ros/jazzy '
        '-DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && '
        'cmake --build /output/native --target rrm_startup_guard --parallel 2')
    must(command==expected_command and image['pull_policy']=='never','Non-actuating command/pull policy changed')
    must(command in (V/'M00J_NATIVE_BUILD_AUTHORIZATION_PACKAGE.md').read_text(),'Authorization command mismatch')
    bounds=policy['resource_proposal_not_measured']
    must(bounds==dict(cpus=2,memory_bytes=4294967296,memory_swap_bytes=4294967296,pids_limit=256,
         wall_time_seconds=600,output_quota_bytes=2147483648,limits_enforcement_verified=False,stop_grace_seconds=10),
         'Resource proposal/enforcement conflation')
    must(len(policy['approval_blockers'])==5 and not policy['host']['approved_daemon_host_identity'], 'Essential blockers erased')
    for key in ['privileged_daemon_api_requests','remote_daemon_connection','container_execution','service_or_container_start_restart',
                'image_build_pull','package_install_download','external_network_access','physical_device_access',
                'native_compile_link','protected_writes','push']:
        must(activity[key] is False,'Unauthorized action recorded '+key)
    passed('compilation_only_static_policy','Blocked documentary policy: three RO/RO/RW binds; clean env/entrypoint, '
           'named compile target only, network none/private namespaces, no GPU/devices/privilege/hooks; non-GPU runtime '
           'and UID/security/resource enforcement expressly unverified; no runtime configuration executed')
    prior=rows(ROOT/'RRM/evidence/M00-I/M00I_PREREQUISITE_CONTINUITY.csv')
    continuity=rows(E/'M00J_PREREQUISITE_CONTINUITY.csv')
    must(len(continuity)==20,'CHK count')
    for a,b in zip(prior,continuity):
        must(a['prerequisite_id']==b['prerequisite_id'] and a['original_requirement']==b['original_requirement']
             and a['current_disposition']==b['prior_disposition']==b['current_disposition']
             and a['remaining_closure_evidence']==b['remaining_closure_evidence']
             and a['target_designation']==b['target_designation'] and b['closed_by_m00j']=='NO'
             and b['execution_authorized']=='NO','Prerequisite drift')
    must(Counter(x['current_disposition'] for x in continuity)=={'SATISFIED':3,'UNSATISFIED':8,'UNVERIFIED':9},
         'Prerequisite status promotion')
    mapping=rows(ROOT/'RRM/verification/matrices/M00_REQUIREMENTS_RECONCILIATION.csv')
    must(len(mapping)==29 and Counter(x['reconciliation_status'] for x in mapping)==
         {'MAPPED_NOT_TESTED':28,'OUT_OF_SCOPE':1},'Requirement drift')
    for file in [ROOT/'RRM/PROJECT_STATUS.md',ROOT/'RRM/handoffs/M00J_HANDOFF.md',V/'M00J_PD_READINESS_DECISION.md']:
        text=file.read_text()
        for token in ['G0 BLOCKED','RA01 PENDING','WP01 NOT AUTHORIZED','Tier B target undesignated',
                      'execution NOT AUTHORIZED','runtime gate NOT PASSED','J4']:
            must(token in text,'Missing governance '+str(file)+' '+token)
    decision=(V/'M00J_PD_READINESS_DECISION.md').read_text()
    must('| PD decision | |' in decision and '| Approved scope | |' in decision,'Invented approval')
    passed('governance_prerequisite_and_mapping_continuity','All20 original prerequisites unchanged3 SAT/8 UNSAT/9 UNVER; '
           '29 mappings unchanged28 NOT_TESTED/1 OUT_OF_SCOPE; no qualification/closure/execution authority; '
           'G0 blocked/RA01 pending/WP01 unauthorized/Tier B undesignated and unauthorized; runtime gate NOT PASSED')
    required_files=['M00J_DEVELOPMENT_ENVIRONMENT_IDENTITY.md','M00J_LOCAL_AVAILABILITY_REPORT.md',
        'M00J_DEPENDENCY_COVERAGE.csv','M00J_COMPILATION_ISOLATION_DESIGN.md','M00J_HOST_SUITABILITY.md',
        'M00J_NATIVE_BUILD_AUTHORIZATION_PACKAGE.md','M00J_PD_READINESS_DECISION.md']
    must(all((V/name).is_file() for name in required_files),'Required report missing')
    must((ROOT/'RRM/handoffs/M00J_HANDOFF.md').is_file(),'Handoff missing')
    package=load(V/'PACKAGE_MANIFEST.json')
    actual_package={str(p.relative_to(V)) for p in V.rglob('*') if p.is_file()
                    and not set(p.relative_to(V).parts)&{'notebook','__pycache__'} and p.name!='PACKAGE_MANIFEST.json'}
    must(actual_package=={x['path'] for x in package['files']},'Package manifest horizon')
    for x in package['files']:must(sha((V/x['path']).read_bytes())==x['sha256'],'Package identity '+x['path'])
    compile((V/'validate_milestone.py').read_bytes(),'validate_milestone.py','exec')
    paths=(E/'M00J_CHANGED_FILES.txt').read_text().splitlines()
    actual=set(git('diff','--name-only',PARENT).decode().splitlines())
    actual.update(git('ls-files','--others','--exclude-standard','RRM/').decode().splitlines())
    must(paths==sorted(set(paths)) and all(x.startswith('RRM/') for x in paths) and set(paths)==actual,'Scope mismatch')
    must({x['path'] for x in rows(E/'M00J_EVIDENCE_INDEX.csv')}==set(paths),'Evidence index horizon')
    sums={}
    for line in (E/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  ',1);sums[name]=digest
        must(sha((ROOT/name).read_bytes())==digest,'New checksum mismatch '+name)
    must(set(sums)==set(paths)-{'RRM/evidence/M00-J/SHA256SUMS',
         'RRM/evidence/M00-J/M00J_VALIDATION_RESULTS.json'},'Checksum coverage')
    passed('deliverable_integrity_and_scope',f'{len(paths)} RRM-only changed files, PROJECT_STATUS sole prior-file mutation; '
           'all seven reports plus static policy/handoff/passive evidence and checksum manifests complete')
    return dict(result='PASS',checks=checks,outcome='J4',changed_file_count=len(paths),
        authorization_package='BLOCKED',native_build_readiness='BLOCKED',current_image_availability='NOT INSPECTABLE',
        image_dependency_coverage='NOT INSPECTABLE',host_native_missing_count=9,
        production_compilation=False,production_linking=False,runtime_containment='NOT VERIFIED',
        runtime_isolation_gate='NOT PASSED',target_designation='UNDESIGNATED',execution_authorized=False,
        warnings=['Root-owned system Docker socket is a privileged control boundary; section4 inspection stop applied; no daemon request or access-denied response claimed.',
                  'Matching exact image ID/tag and privileged GPU container are historical evidence only; no current availability or absence established.',
                  'Dockerfile/Jazzy declarations and historical build caches do not prove nine installed native requirements, versions or ABI compatibility.',
                  'Observed Kubernetes/OSMO workspace is not exact daemon-host/controller/exclusive-ownership/resource permission evidence.',
                  'Local daemon config declares default NVIDIA runtime; non-GPU runc availability/security/mount/UID/quota enforcement remains unverified.',
                  'Compilation-only resource limits are proposals, not measured duration/demand or granted quota; authorization package remains blocked.'],
        failures=[],skipped_checks=['Current Docker/containerd/image/container/rootfs/package inspection: privileged-daemon path stopped; no remote or bypass path.',
                                   'Native configure/compile/link or dependency install: outside M00-J preparation scope; host dependencies missing.',
                                   'Container/service/ROS/PX4/MAVROS/Isaac/startup/exec/pull/build/network/hardware operations: unauthorized and not performed.',
                                   'Broad unchanged synthetic suites and M00-I production source analysis: not repeated; sealed evidence referenced.',
                                   'Effective confinement, UID mappings, quota/stop survival and exact host/image permissions: not verifiable passively here.',
                                   'Original M00 lacks SHA256SUMS; protected baseline inventory covers unchanged history.'],
        limitations=['PASS attests documentary/static/integrity checks, not J1 execution readiness, runtime containment or qualification.',
                     'Local stat/PATH/cgroup evidence is namespace- and capture-time-specific; visible free space is not an allocation.',
                     'No active/network/hardware operations is an executed-action record; byte hashes alone cannot prove their absence.',
                     'No new synthetic regression test suite was needed for this documentation/static-policy-only milestone.'])


if __name__=='__main__':print(json.dumps(validate(),indent=2))
