#!/usr/bin/env python3
"""Validate M00-H artifact integrity and offline design gates; no runtime operations."""
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
from collections import Counter

ROOT = Path(__file__).resolve().parents[3]
E = ROOT / 'RRM/evidence/M00-H'
V = ROOT / 'RRM/engineering/M00-H'
PARENT = '45d026e2b8c46b0e1c44013d8dd9d7ed49e3d5ac'
SUBJECT = 'RRM-F00 M00-H: verify guard build and offline runtime adapters'


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
    must(load(E / 'starting-integrity.json')['result'] == 'PASS', 'Prior M00-G validator not passing')
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
            'M00-E1C': '9d60ee46b3348fbaa1811a5b234f80446b62d5bd', 'M00-E1D': 'e6f0c81eff15538809b9745cef665de04ba4e796', 'M00-E1E': '0fa876fd8191be61a3e6724e1a3c93fe64510f01', 'GOV-001A': 'a72b8f412f2567e870903e2f4fec8082bc437e21', 'GOV-001B': '381db1c4dcdd1bb71d047176e681bd18c72e342d', 'GOV-002': '6c63054b2d1ac0dee0c5840ed3d4c31083874177', 'M00-F': 'dc17fa3918c5e8af12ecabade3cee3efdc223f85', 'M00-G': PARENT}
    count = 0
    for milestone, pin in pins.items():
        for line in (ROOT / f'RRM/evidence/{milestone}/SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ', 1)
            must(sha(git('show', pin + ':' + name)) == digest, 'Sealed history mismatch: ' + name)
            count += 1
    passed('starting_and_protected_integrity', f'Clean pinned M00-G start; '
           f'{len(before["tracked_entries"])-1} protected entries, {count} pinned sealed checksums '
           f'and {len(inputs)} unchanged controlled input files')


    import io
    import unittest
    from run_offline import suite
    from h_support import decode,file_sha
    from h_preflight import evaluate
    log=io.StringIO();result=unittest.TextTestRunner(stream=log).run(suite())
    must(result.wasSuccessful(),log.getvalue())
    report=decode((E/'M00H_TEST_RESULTS.json').read_bytes())
    must(report['status']=='PASS' and report['test_count']==result.testsRun and
         report['failures']==report['errors']==report['skipped']==0,'Saved test mismatch')
    passed('offline_integration_and_reliability',f'{result.testsRun} tests PASS,0 failures/errors/skips; '
           'all20 required synthetic integration cases; original-source reference and temporary-file/process faults')
    build=decode((E/'build/BUILD_RESULT.json').read_bytes())
    must(build['build_status']=='BLOCKED' and build['dependency_free_python']=='PASS'
         and build['commands']==[] and not build['production_build_verified'],'Native build overclaim')
    must({'cxx','cmake','ament_cmake','rclcpp','ament_index_cpp','mavros_msgs','rcl_interfaces','std_msgs','interface_bringup','yaml-cpp'}
         == set(build['dependencies']['missing']),'Unexpected build dependency disposition')
    for x in build['python_sources']:
        must(file_sha(ROOT/x['path'])==x['sha256'],'Stale dependency-free compilation identity')
    passed('build_verification','Full guard compile/link BLOCKED by10 explicit missing tool/package dependencies; '
           'all H Python sources compiled/tested; no C++/CMake invocation, stubs or syntax-only production claim')
    current=evaluate(ROOT,build,report)
    must(current==decode((E/'M00H_PREFLIGHT_RESULTS.json').read_bytes()),'Saved preflight mismatch')
    must(current['BUILD STATUS']=='BLOCKED' and current['OFFLINE TEST STATUS']=='PASS'
         and current['STATIC ISOLATION STATUS']=='PASS' and current['RUNTIME CONTAINMENT STATUS']=='NOT VERIFIED'
         and current['OVERALL EXECUTION READINESS']=='BLOCKED' and not current['execution_ready'], 'Preflight conflation')
    must(len(current['original_isolation_gate'])==14 and
         all(x['result']=='NOT PASSED' for x in current['original_isolation_gate']),'Isolation gate promotion')
    must(current['guard_compatibility']['full_guard_compatibility']=='NOT VERIFIED','Full guard compatibility overclaim')
    passed('source_compatibility_adapters_and_preflight','Original guard header preserved; failed-transaction reference '
           'correction tested; observational adapters implemented, ROS binding/full guard NOT VERIFIED; runtime/overall remain blocked')
    prior=rows(ROOT/'RRM/evidence/M00-G/M00G_PREREQUISITE_CONTINUITY.csv')
    continuity=rows(E/'M00H_PREREQUISITE_CONTINUITY.csv')
    must(len(continuity)==20,'CHK count')
    for a,b in zip(prior,continuity):
        must(a['prerequisite_id']==b['prerequisite_id'] and a['original_requirement']==b['original_requirement']
             and a['current_disposition']==b['prior_disposition']==b['current_disposition']
             and a['remaining_closure_evidence']==b['remaining_closure_evidence']
             and b['closed_by_m00h']=='NO' and b['execution_authorized']=='NO','CHK drift')
    mapping=rows(ROOT/'RRM/verification/matrices/M00_REQUIREMENTS_RECONCILIATION.csv')
    must(len(mapping)==29 and Counter(x['reconciliation_status'] for x in mapping)==
         {'MAPPED_NOT_TESTED':28,'OUT_OF_SCOPE':1},'Mapping drift')
    passed('prerequisite_requirement_continuity','All20 original CHKs unchanged3 SATISFIED/8 UNSATISFIED/9 UNVERIFIED; '
           '29 mappings28 NOT_TESTED/1 OUT_OF_SCOPE; zero new qualification/whole closure')
    package=decode((V/'PACKAGE_MANIFEST.json').read_bytes())
    actual_package={str(p.relative_to(V)) for p in V.rglob('*') if p.is_file()
                    and not set(p.relative_to(V).parts)&{'notebook','__pycache__','.tmp','build'}
                    and p.name!='PACKAGE_MANIFEST.json'}
    must(actual_package=={x['path'] for x in package['files']},'Package manifest horizon')
    for x in package['files']:
        must(file_sha(V/x['path'])==x['sha256'],'H package identity changed')
    required=['README.md','BUILD_CONFIG.json','BUILD_VERIFICATION.md','check_build_dependencies.py','build_guard.py',
              'GUARD_COMPATIBILITY.md','GUARD_SEMANTIC_COMPATIBILITY.csv','guard_reference.py','observation_adapters.py',
              'reliable_recorder.py','RECORDER_RELIABILITY.md','ros_binding_contract.json','integration_harness.py',
              'test_h.py','run_offline.py','DEPENDENCY_IDENTITY_REGISTER.csv','INTEGRATION_TEST_REPORT.md',
              'EXECUTION_READINESS.md','h_preflight.py']
    must(all((V/x).is_file() for x in required),'Required deliverable missing')
    must(len(rows(V/'GUARD_SEMANTIC_COMPATIBILITY.csv'))==12 and
         len(rows(V/'DEPENDENCY_IDENTITY_REGISTER.csv'))==25,'Trace register count')
    status=(ROOT/'RRM/PROJECT_STATUS.md').read_text()
    for token in ['M00-H','G0 BLOCKED','RA01 PENDING','WP01 NOT AUTHORIZED','Tier B target undesignated',
                  'execution NOT AUTHORIZED','runtime gate NOT PASSED']:
        must(token in status,'Missing governance '+token)
    paths=(E/'M00H_CHANGED_FILES.txt').read_text().splitlines()
    actual=set(git('diff','--name-only',PARENT).decode().splitlines())
    actual.update(git('ls-files','--others','--exclude-standard','RRM/').decode().splitlines())
    must(paths==sorted(set(paths)) and all(x.startswith('RRM/') for x in paths) and set(paths)==actual,'Scope mismatch')
    sums={}
    for line in (E/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  ',1);sums[name]=digest
        must(sha((ROOT/name).read_bytes())==digest,'New checksum mismatch '+name)
    must(set(sums)==set(paths)-{'RRM/evidence/M00-H/SHA256SUMS','RRM/evidence/M00-H/M00H_VALIDATION_RESULTS.json'},'Checksum coverage')
    must((ROOT/'RRM/handoffs/M00H_HANDOFF.md').is_file(),'Missing handoff')
    passed('package_integrity_governance_and_scope',f'{len(paths)} RRM-only changed paths; PROJECT_STATUS sole prior-file mutation; '
           '25 dependency records,12 semantic rows and complete implementation/provenance evidence')
    return dict(result='PASS',checks=checks,test_count=result.testsRun,failures=0,errors=0,test_skipped=0,
                changed_file_count=len(paths),guard_build='BLOCKED',dependency_free_python='PASS',
                guard_source_correspondence='PASS_SYNTHETIC_ONLY',full_guard_compatibility='NOT VERIFIED',
                adapter_implementation='OFFLINE_IMPLEMENTED;PRODUCTION_ROS_BINDING_NOT_VERIFIED',
                recorder_reliability='LOCAL_SYNTHETIC_FAILURE_TESTS_PASS;RUNTIME_NOT_VERIFIED',
                static_isolation='PASS',runtime_containment='NOT VERIFIED',overall_execution_readiness='BLOCKED',
                target_designation='UNDESIGNATED',execution_authorized=False,
                warnings=['No compiler/CMake/ROS/YAML dependency set found in checked host PATH/prefixes; no full guard compile/link occurred.',
                          'G reference omitted failed-transaction attempt latch; corrected in H synthetic reference only; original/header/G history unchanged.',
                          'ROS/native event bindings and source authentication/clock mapping remain unverified; graph presence is not safety readiness.',
                          'Local fsync/partial-write/process-exit tests do not prove storage power-loss, independent recorder/host survival or REC-01 qualification.',
                          'Historical image/binary/asset facts do not establish assigned target identity or access; no digest/version invented.'],
                skipped_checks=['Full native C++ guard build: exact10 dependency blockers, not replaced by stubs or syntax-only tests.',
                                'ROS/PX4/MAVROS/Isaac/Docker/network/device/GPU/runtime operations: unauthorized, none performed.',
                                'Independent runtime recorder supervision/overhead/host/power-failure proof and effective containment: not authorized or supplied.',
                                'Formal frozen F00 equivalence/qualification/original approval chains: unavailable.',
                                'Original M00 lacks a SHA256SUMS; protected baseline inventory covers unchanged history.'],
                limitations=['Pure adapters and synthetic reference are engineering diagnostics, not original C++ runtime execution or qualification.',
                             'One offline Python child intentionally exits abruptly; no robot/simulator/service process is launched.',
                             'Checksums and terminal bytes attest structure, not source authenticity or successful durability acknowledgment; external health evidence required.',
                             'No active operations outside authorized offline tests/build inventory occurred, based on executed tool actions; file hashes alone cannot prove absence.'])


if __name__=='__main__':print(json.dumps(validate(),indent=2))
