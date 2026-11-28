#!/usr/bin/env python3
"""Validate M00-G artifact integrity and offline design gates; no runtime operations."""
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
from collections import Counter

ROOT = Path(__file__).resolve().parents[3]
E = ROOT / 'RRM/evidence/M00-G'
V = ROOT / 'RRM/engineering/M00-G'
PARENT = 'dc17fa3918c5e8af12ecabade3cee3efdc223f85'
SUBJECT = 'RRM-F00 M00-G: implement offline AirStack simulation package'


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
    must(load(E / 'starting-integrity.json')['result'] == 'PASS', 'Prior M00-F validator not passing')
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
            'M00-E1C': '9d60ee46b3348fbaa1811a5b234f80446b62d5bd', 'M00-E1D': 'e6f0c81eff15538809b9745cef665de04ba4e796', 'M00-E1E': '0fa876fd8191be61a3e6724e1a3c93fe64510f01', 'GOV-001A': 'a72b8f412f2567e870903e2f4fec8082bc437e21', 'GOV-001B': '381db1c4dcdd1bb71d047176e681bd18c72e342d', 'GOV-002': '6c63054b2d1ac0dee0c5840ed3d4c31083874177', 'M00-F': PARENT}
    count = 0
    for milestone, pin in pins.items():
        for line in (ROOT / f'RRM/evidence/{milestone}/SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ', 1)
            must(sha(git('show', pin + ':' + name)) == digest, 'Sealed history mismatch: ' + name)
            count += 1
    passed('starting_and_protected_integrity', f'Clean pinned M00-F start; '
           f'{len(before["tracked_entries"])-1} protected entries, {count} pinned sealed checksums '
           f'and {len(inputs)} unchanged controlled input files')


    import unittest
    import io
    from test_offline import ConfigurationTests, StartupTests, RecorderTests, PreflightTests
    from preflight import evaluate
    from common import load as strict_load
    suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromTestCase(t)
                             for t in [ConfigurationTests,StartupTests,RecorderTests,PreflightTests])
    output=io.StringIO()
    result=unittest.TextTestRunner(stream=output).run(suite)
    must(result.wasSuccessful(),output.getvalue())
    passed('full_offline_suite',f'{result.testsRun} tests PASS;0 failures;0 errors;{len(result.skipped)} skipped; '
           'all20 directive cases covered, synthetic events/identities only')
    preflight=evaluate(V,ROOT)
    must(preflight['overall']=='BLOCKED' and not preflight['execution_ready']
         and preflight['static_configuration']['static_policy']=='PASS'
         and preflight['runtime_containment']['status']=='NOT VERIFIED','Preflight scope conflation')
    must(len(preflight['acceptance_gate'])==14 and
         all(x['status']=='NOT VERIFIED' and x['result']=='NOT PASSED'
             for x in preflight['acceptance_gate']),'Runtime gate promoted')
    must(strict_load(E/'M00G_PREFLIGHT_RESULTS.json')==preflight,'Saved preflight differs')
    passed('static_policy_and_runtime_preflight','Static policy PASS; package/execution BLOCKED; '
           'all14 original isolation rows NOT VERIFIED/NOT PASSED; disabled entrypoints retained')
    prior=rows(ROOT/'RRM/evidence/M00-F/M00F_PREREQUISITE_CONTINUITY.csv')
    current=rows(E/'M00G_PREREQUISITE_CONTINUITY.csv')
    must(len(current)==20,'CHK count')
    for a,b in zip(prior,current):
        must(a['prerequisite_id']==b['prerequisite_id'] and a['original_requirement']==b['original_requirement']
             and a['current_disposition']==b['prior_disposition']==b['current_disposition']
             and a['remaining_closure_evidence']==b['remaining_closure_evidence']
             and b['closed_by_m00g']=='NO' and b['execution_authorized']=='NO','CHK drift')
    mapping=rows(ROOT/'RRM/verification/matrices/M00_REQUIREMENTS_RECONCILIATION.csv')
    must(len(mapping)==29 and Counter(x['reconciliation_status'] for x in mapping)==
         {'MAPPED_NOT_TESTED':28,'OUT_OF_SCOPE':1},'Mapping drift')
    passed('prerequisite_and_requirement_continuity','All20 original identifiers/dispositions3/8/9 '
           'and closure conditions preserved;29 mappings28 NOT_TESTED/1 OUT_OF_SCOPE;0 qualification')
    manifest=strict_load(V/'PROVENANCE.json')
    must(len(manifest['unresolved_dependencies'])==11 and manifest['execution_authorized'] is False,'Unresolved dependencies hidden')
    must(all(x['identity_status']=='UNRESOLVED' and x['reference'] is None
             for x in manifest['images'].values()),'Image identity invented')
    required=['config/compose.json','config/environment.env.template','config/mavros_only.launch.xml',
              'config/dds.xml','COMPONENT_DEPENDENCIES.json','STARTUP_SEQUENCING.md','GUARD_COMPATIBILITY.md',
              'guard_host/src/guard_host.cpp','guard_host/CMakeLists.txt','guard_host/package.xml',
              'recorder.py','RECORDER_SPEC.md','EVENT_SCHEMA.json','config_validator.py','preflight.py',
              'test_offline.py','PROVENANCE.json','README.md','UNRESOLVED_DEPENDENCIES.csv','COMPLETION_REPORT.md',
              'QUALIFICATION_LIMITS.md','DESIGN_SPEC.md']
    must(all((V/x).is_file() for x in required),'Missing implementation deliverable')
    # Closed manifest horizon prevents omitting newly introduced package code/config.
    package_paths={str(p.relative_to(V)) for p in V.rglob('*') if p.is_file()
                   and not set(p.relative_to(V).parts)&{'notebook','__pycache__'} and p.name!='PROVENANCE.json'}
    must(package_paths=={x['path'] for x in manifest['configuration_inputs']},'Package provenance horizon mismatch')
    passed('implementation_provenance_and_dependencies',f'{len(manifest["upstream_sources"])} hashed source references; '
           f'{len(package_paths)} package inputs;11 unresolved dependencies; all runtime images unassigned')
    status=(ROOT/'RRM/PROJECT_STATUS.md').read_text()
    for token in ['G0 BLOCKED','RA01 PENDING','WP01 NOT AUTHORIZED','Tier B target undesignated',
                  'execution NOT AUTHORIZED','runtime gate NOT PASSED','M00-G']:
        must(token in status,'Missing status boundary '+token)
    paths=(E/'M00G_CHANGED_FILES.txt').read_text().splitlines()
    actual=set(git('diff','--name-only',PARENT).decode().splitlines())
    actual.update(git('ls-files','--others','--exclude-standard','RRM/').decode().splitlines())
    must(paths==sorted(set(paths)) and all(x.startswith('RRM/') for x in paths) and actual==set(paths),'Unauthorized changed paths')
    sums={}
    for line in (E/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  ',1);sums[name]=digest
        must(sha((ROOT/name).read_bytes())==digest,'New checksum mismatch '+name)
    must(set(sums)==set(paths)-{'RRM/evidence/M00-G/SHA256SUMS','RRM/evidence/M00-G/M00G_VALIDATION_RESULTS.json'},'New checksum coverage')
    must((ROOT/'RRM/handoffs/M00G_HANDOFF.md').is_file(),'Missing handoff')
    passed('repository_scope_and_deliverables',f'{len(paths)} RRM-only changed files; PROJECT_STATUS sole prior edit; '
           'all new sums valid; historical evidence/protected files unchanged')
    return dict(result='PASS',checks=checks,test_count=result.testsRun,test_failures=len(result.failures),
                test_errors=len(result.errors),test_skipped=len(result.skipped),changed_file_count=len(paths),
                static_configuration_compliance='PASS',configuration_dependency_readiness='BLOCKED',
                execution_readiness='BLOCKED',runtime_containment='NOT VERIFIED',runtime_isolation_gate='NOT PASSED',
                tier_b_target='UNDESIGNATED',execution_authorized=False,
                warnings=['Static policy and synthetic test PASS are not runtime safety, guard equivalence or qualification.',
                          'C++ original-header host source is unbuilt; Python deadline reference covers a subset only.',
                          'Recorder is an observational writer/auditor; runtime adapters, fsync durability and independent supervisor are unresolved.',
                          'Images/assets, GPU/user/filesystem compatibility, effective DDS/device/network isolation and P1 readiness are unresolved.',
                          'Official documentation URLs are retrieved references, not authenticated pinned deployment identities.'],
                skipped_checks=['All container/simulator/ROS/PX4/MAVROS/service startup or restart and active network/device/hardware checks prohibited; none performed.',
                                'C++ compile/link and ROS executor integration: exact build dependencies/install/image identity unresolved; no build invoked.',
                                'Actual recorder failure survival, overhead/drop/resource/clock calibration and independent stop: require later explicit scope.',
                                'Formal F00 equivalence/qualification and original records: not supplied or tested.',
                                'Original M00 has no own SHA256SUMS; protected baseline inventory covers it.'],
                limitations=['Static validator handles only this closed JSON Compose subset, not general YAML merging/interpolation or running configuration.',
                             'Caller-supplied event provenance and approval labels are untrusted observations; hash chain proves stream consistency, not authentic clocks or authority.',
                             'No active operations occurred based on recorded tool actions; file checksums alone cannot attest absence of runtime actions.'])


if __name__=='__main__':
    print(json.dumps(validate(),indent=2))
