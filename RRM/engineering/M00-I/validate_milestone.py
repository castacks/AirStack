#!/usr/bin/env python3
"""Validate M00-I artifact integrity and native-path evidence; no runtime operations."""
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
from collections import Counter

ROOT = Path(__file__).resolve().parents[3]
E = ROOT / 'RRM/evidence/M00-I'
V = ROOT / 'RRM/engineering/M00-I'
PARENT = 'f5b14e08cbf0ef54bfe3c986ea377fbe50adbf61'
SUBJECT = 'RRM-F00 M00-I: resolve native guard build and production interfaces'


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
    must(load(E / 'starting-integrity.json')['result'] == 'PASS', 'Prior M00-H validator not passing')
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
            'M00-E1C': '9d60ee46b3348fbaa1811a5b234f80446b62d5bd', 'M00-E1D': 'e6f0c81eff15538809b9745cef665de04ba4e796', 'M00-E1E': '0fa876fd8191be61a3e6724e1a3c93fe64510f01', 'GOV-001A': 'a72b8f412f2567e870903e2f4fec8082bc437e21', 'GOV-001B': '381db1c4dcdd1bb71d047176e681bd18c72e342d', 'GOV-002': '6c63054b2d1ac0dee0c5840ed3d4c31083874177', 'M00-F': 'dc17fa3918c5e8af12ecabade3cee3efdc223f85', 'M00-G': '45d026e2b8c46b0e1c44013d8dd9d7ed49e3d5ac', 'M00-H': PARENT}
    count = 0
    for milestone, pin in pins.items():
        for line in (ROOT / f'RRM/evidence/{milestone}/SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ', 1)
            must(sha(git('show', pin + ':' + name)) == digest, 'Sealed history mismatch: ' + name)
            count += 1
    passed('starting_and_protected_integrity', f'Clean pinned M00-H start; '
           f'{len(before["tracked_entries"])-1} protected entries, {count} pinned sealed checksums '
           f'and {len(inputs)} unchanged controlled input files')


    from native_environment import inspect,GUARD,GUARD_SHA
    for path in V.glob('*.py'):compile(path.read_bytes(),str(path),'exec')
    build=load(V/'M00I_NATIVE_BUILD_RESULTS.json')
    must(build['outcome']=='I2' and not build['genuine_native_build'] and not build['execution_ready']
         and not build['guard_executed'] and not build['commands'],'Native build overclaim')
    must(build['environment']==inspect(ROOT),'Stale native inventory')
    must(set(build['environment']['native_missing'])=={'cxx','cmake','ament_cmake','rclcpp','ament_index_cpp',
         'mavros_msgs','rcl_interfaces','std_msgs','yaml-cpp'},'Native blocker mismatch')
    must(build['stages']=={'SOURCE CHECK':'PASS','CONFIGURATION':'BLOCKED','COMPILATION':'BLOCKED',
         'LINKING':'BLOCKED','OFFLINE UNIT TESTING':'NOT RUN BY BUILD TOOL',
         'PRODUCTION RUNTIME INTEGRATION':'NOT VERIFIED'},'Stage conflation')
    for item in build['python_source_identities']:
        must(sha((ROOT/item['path']).read_bytes())==item['sha256'],'Stale Python compile identity')
    resource=build['environment']['runtime_resource']
    must(resource['availability']=='AVAILABLE AND VERIFIED' and not resource['native_build_requirement'],
         'Runtime resource/build dependency conflation')
    deps=rows(V/'M00I_NATIVE_DEPENDENCY_MATRIX.csv');interfaces=rows(V/'M00I_PRODUCTION_INTERFACE_MATRIX.csv')
    must(len(deps)==14 and len(interfaces)==12,'Trace matrix horizon')
    allowed={'AVAILABLE AND VERIFIED','DECLARED BUT NOT LOCALLY AVAILABLE','VERSION UNRESOLVED',
             'GENERATED ARTIFACT NOT AVAILABLE','UNSUPPORTED OR INCOMPATIBLE'}
    must(all(x['upstream_requirement'] and x['availability'] in allowed for x in deps),'Dependency provenance/status')
    sources={x['path']:x['sha256'] for x in load(E/'source-manifest.json')['files']}
    must(all(x['upstream_source'] in sources and x['source_revision_or_checksum'].startswith(sources[x['upstream_source']])
             and x['required_adaptation'] and x['compatibility_status'] for x in interfaces),'Interface trace')
    references=load(E/'upstream-reference-register.json')
    must(len(references['references'])==10 and all(x['url'].startswith('https://') for x in references['references']),
         'External source trace missing')
    passed('native_path_dependency_and_interface_resolution','B supported robot development container/Jazzy path; '
           'nine actual native blockers; runtime-only interface_bringup resolved;14 dependency rows/12 interface rows; '
           'source PASS/configure compile link BLOCKED/runtime NOT VERIFIED; no native command or stubs')
    report=load(V/'M00I_REGRESSION_RESULTS.json')
    must(report['status']=='PASS' and report['tests']==14 and report['failures']==report['errors']==report['skipped']==0
         and not report['production_runtime_executed'],'Focused regression evidence failed')
    must(set(report['source_sha256'])=={p.name for p in V.glob('*.py') if p.name!='validate_milestone.py'},
         'Regression source horizon')
    for name,digest in report['source_sha256'].items():must(sha((V/name).read_bytes())==digest,'Stale test source '+name)
    must(sha((ROOT/GUARD).read_bytes())==GUARD_SHA,'Original guard changed')
    original=(ROOT/'RRM/engineering/M00-G/guard_host/src/guard_host.cpp').read_bytes()
    must((V/'guard_host/src/guard_host.cpp').read_bytes().endswith(original),'Guard host body changed')
    must((V/'guard_host/CMakeLists.txt').read_bytes()==(ROOT/'RRM/engineering/M00-G/guard_host/CMakeLists.txt').read_bytes()
         and (V/'guard_host/package.xml').read_bytes()==(ROOT/'RRM/engineering/M00-G/guard_host/package.xml').read_bytes(),
         'Unexplained native build declaration change')
    passed('targeted_regression_and_guard_preservation','Saved14 focused tests PASS (11 I plus3 affected H adapters), '
           'source hashes current; original guard and G host body/declarations unchanged; H baseline60 PASS captured '
           'before edits; unchanged recorder/failure suites retained without broad rerun')
    prior=rows(ROOT/'RRM/evidence/M00-H/M00H_PREREQUISITE_CONTINUITY.csv')
    continuity=rows(E/'M00I_PREREQUISITE_CONTINUITY.csv')
    must(len(continuity)==20,'CHK count')
    for a,b in zip(prior,continuity):
        must(a['prerequisite_id']==b['prerequisite_id'] and a['original_requirement']==b['original_requirement']
             and a['current_disposition']==b['prior_disposition']==b['current_disposition']
             and a['remaining_closure_evidence']==b['remaining_closure_evidence']
             and a['target_designation']==b['target_designation'] and b['closed_by_m00i']=='NO'
             and b['execution_authorized']=='NO','Prerequisite drift')
    must(Counter(x['current_disposition'] for x in continuity)==
         {'SATISFIED':3,'UNSATISFIED':8,'UNVERIFIED':9},'Disposition drift')
    mapping=rows(ROOT/'RRM/verification/matrices/M00_REQUIREMENTS_RECONCILIATION.csv')
    must(len(mapping)==29 and Counter(x['reconciliation_status'] for x in mapping)==
         {'MAPPED_NOT_TESTED':28,'OUT_OF_SCOPE':1},'Mapping drift')
    status=(ROOT/'RRM/PROJECT_STATUS.md').read_text()
    for token in ['M00-I','G0 BLOCKED','RA01 PENDING','WP01 NOT AUTHORIZED','Tier B target undesignated',
                  'execution NOT AUTHORIZED','runtime gate NOT PASSED']:
        must(token in status,'Missing governance '+token)
    must('| PD decision | |' in (V/'M00I_PD_TECHNICAL_DECISION.md').read_text(),'Invented PD disposition')
    passed('prerequisite_requirement_and_governance_continuity','20 original CHKs unchanged3 SAT/8 UNSAT/9 UNVER; '
           '29 mappings unchanged28 NOT_TESTED/1 OUT_OF_SCOPE; G0 blocked/RA01 pending/WP01 unauthorized; '
           'Tier B undesignated/unauthorized; runtime isolation gate NOT PASSED; PD fields blank')
    package=load(V/'PACKAGE_MANIFEST.json')
    actual_package={str(p.relative_to(V)) for p in V.rglob('*') if p.is_file()
                    and not set(p.relative_to(V).parts)&{'notebook','__pycache__','.tmp','native-output'}
                    and p.name!='PACKAGE_MANIFEST.json'}
    must(actual_package=={x['path'] for x in package['files']},'Package manifest horizon')
    for x in package['files']:must(sha((V/x['path']).read_bytes())==x['sha256'],'Package identity '+x['path'])
    required=['M00I_BUILD_ENVIRONMENT_ASSESSMENT.md','M00I_NATIVE_DEPENDENCY_MATRIX.csv',
              'M00I_PRODUCTION_INTERFACE_MATRIX.csv','M00I_SOURCE_COMPATIBILITY_REPORT.md','M00I_BUILD_PROCEDURE.md',
              'M00I_NATIVE_BUILD_RESULTS.json','M00I_REGRESSION_RESULTS.json','M00I_BUILD_HANDOFF.md',
              'M00I_PD_TECHNICAL_DECISION.md']
    must(all((V/name).is_file() for name in required),'Required engineering deliverable missing')
    must((ROOT/'RRM/handoffs/M00I_HANDOFF.md').is_file(),'Handoff missing')
    paths=(E/'M00I_CHANGED_FILES.txt').read_text().splitlines()
    actual=set(git('diff','--name-only',PARENT).decode().splitlines())
    actual.update(git('ls-files','--others','--exclude-standard','RRM/').decode().splitlines())
    must(paths==sorted(set(paths)) and all(x.startswith('RRM/') for x in paths) and set(paths)==actual,'Scope mismatch')
    must({x['path'] for x in rows(E/'M00I_EVIDENCE_INDEX.csv')}==set(paths),'Evidence index mismatch')
    sums={}
    for line in (E/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  ',1);sums[name]=digest
        must(sha((ROOT/name).read_bytes())==digest,'New checksum mismatch '+name)
    must(set(sums)==set(paths)-{'RRM/evidence/M00-I/SHA256SUMS',
         'RRM/evidence/M00-I/M00I_VALIDATION_RESULTS.json'},'Checksum coverage')
    passed('package_scope_and_deliverable_integrity',f'{len(paths)} RRM-only changed files; '
           'PROJECT_STATUS sole prior-file mutation; source/package/current evidence integrity verified')
    return dict(result='PASS',checks=checks,outcome='I2',test_count=14,failures=0,errors=0,test_skipped=0,
                changed_file_count=len(paths),native_build='BLOCKED',native_compilation=False,native_linking=False,
                dependency_free_python='PASS',production_interfaces='SOURCE ASSESSED; INSTALLED ABI/NATIVE BINDINGS UNVERIFIED',
                original_guard_sha256=GUARD_SHA,full_guard_equivalence='NOT VERIFIED',
                overall_execution_readiness='BLOCKED',runtime_containment='NOT VERIFIED',
                target_designation='UNDESIGNATED',execution_authorized=False,
                warnings=['No C++/CMake/ROS native package set available; no native configuration, compilation or linking.',
                          'Installed workspace configs and historical GNU13.3.0/CMake3.28.3 metadata are not current tool/ABI proof.',
                          'Exact apt versions/image digest unresolved; remote source tags/branches do not identify installed binaries.',
                          'Kit application-complete/OmniGraph events, native ROS bindings, effective QoS/provider identity and clock mapping remain unresolved.',
                          'Schema2 legacy vehicle_graph_ready event is explicitly scoped to ROS interface presence, not simulator/guard/safety readiness.',
                          'Initial focused test setup/source-comment comparison failures corrected before final14-test PASS; no outstanding failures.',
                          'docs.ros.org rendered pages blocked by Anubis; primary ROS raw source fallback used.'],
                skipped_checks=['Native configure/compile/link: nine explicit missing dependencies; no further build attempts/stubs.',
                                'Unchanged broad H synthetic suites: baseline60 PASS captured before edits; only affected3 adapter tests repeated.',
                                'Production runtime/ROS/simulator/container/device/probe/deployment/Tier B/C/WP01: unauthorized; none performed.',
                                'Original M00 has no SHA256SUMS; protected baseline inventory covers unchanged history.'],
                limitations=['Source contract checks and14 focused offline tests are not native compilation, integration or qualification.',
                             'No frozen F00 equivalence, G0 closure, prerequisite closure or execution authority inferred.',
                             'No active operations is an executed-action record, not something file hashes alone can prove.'])


if __name__=='__main__':print(json.dumps(validate(),indent=2))
