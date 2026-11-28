#!/usr/bin/env python3
"""Validate M00-F artifact integrity and offline design gates; no runtime operations."""
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
from collections import Counter

ROOT = Path(__file__).resolve().parents[3]
E = ROOT / 'RRM/evidence/M00-F'
V = ROOT / 'RRM/engineering/M00-F'
PARENT = '6c63054b2d1ac0dee0c5840ed3d4c31083874177'
SUBJECT = 'RRM-F00 M00-F: assess AirStack-native simulation isolation'


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
    must(load(E / 'starting-integrity.json')['result'] == 'PASS', 'Prior GOV-002 validator not passing')
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
            'M00-E1C': '9d60ee46b3348fbaa1811a5b234f80446b62d5bd', 'M00-E1D': 'e6f0c81eff15538809b9745cef665de04ba4e796', 'M00-E1E': '0fa876fd8191be61a3e6724e1a3c93fe64510f01', 'GOV-001A': 'a72b8f412f2567e870903e2f4fec8082bc437e21', 'GOV-001B': '381db1c4dcdd1bb71d047176e681bd18c72e342d', 'GOV-002': PARENT}
    count = 0
    for milestone, pin in pins.items():
        for line in (ROOT / f'RRM/evidence/{milestone}/SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ', 1)
            must(sha(git('show', pin + ':' + name)) == digest, 'Sealed history mismatch: ' + name)
            count += 1
    passed('starting_and_protected_integrity', f'Clean pinned GOV-002 start; '
           f'{len(before["tracked_entries"])-1} protected entries, {count} pinned sealed checksums '
           f'and {len(inputs)} unchanged controlled input files')


    import unittest
    from validate_design import read_design, validate as policy
    from test_validate_design import DesignPolicyTests
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(DesignPolicyTests)
    import io
    log = io.StringIO()
    outcome = unittest.TextTestRunner(stream=log).run(suite)
    must(outcome.wasSuccessful(), log.getvalue())
    must(not policy(read_design(V / 'M00F_NONEXECUTABLE_POLICY.json')), 'Design policy invalid')
    passed('offline_security_policy_tests', f'{outcome.testsRun} test methods;28 fixture subcases '
           '(one positive,27 negative),22 missing-field cases and4 malformed roots; '
           'privilege/network/devices/namespaces/caps/ports/entrypoints/env/duplicate keys rejected')
    gate = rows(V / 'M00F_ISOLATION_ACCEPTANCE_GATE.csv')
    must(len(gate) == 14 and len({x['gate_id'] for x in gate}) == 14, 'Gate coverage')
    must(all(all(x.values()) and x['runtime_status'] == 'NOT YET VERIFIED'
             and x['gate_result'] == 'NOT PASSED' for x in gate), 'Premature execution gate pass')
    passed('future_execution_gate', '14 explicit methods/evidence/pass conditions/reviewer roles; '
           'none assigned or runtime verified; static intent never grants execution')
    prior = rows(ROOT / 'RRM/evidence/GOV-002/GOV002_PREREQUISITE_CONTINUITY.csv')
    current = rows(E / 'M00F_PREREQUISITE_CONTINUITY.csv')
    must(len(current) == 20, 'CHK coverage')
    for a,b in zip(prior,current):
        must(a['prerequisite_id'] == b['prerequisite_id'] and a['original_requirement'] == b['original_requirement']
             and a['current_disposition'] == b['prior_disposition'] == b['current_disposition']
             and b['closed_by_m00f'] == 'NO' and b['execution_authorized'] == 'NO', 'CHK continuity')
    mapping = rows(ROOT / 'RRM/verification/matrices/M00_REQUIREMENTS_RECONCILIATION.csv')
    must(len(mapping) == 29 and Counter(x['reconciliation_status'] for x in mapping)
         == {'MAPPED_NOT_TESTED':28,'OUT_OF_SCOPE':1}, 'Requirements coverage altered')
    passed('prerequisite_and_requirement_continuity', 'All20 original CHKs traceable, dispositions3/8/9 '
           'unchanged;29 mapping rows28 NOT_TESTED/1 OUT_OF_SCOPE, zero verification/qualification')
    sources = rows(E / 'M00F_SOURCE_REGISTER.csv')
    must(len({x['source_id'] for x in sources}) == len(sources), 'Duplicate source IDs')
    must(len([x for x in sources if x['classification'] == 'DOCUMENTED']) == 4, 'Official documentation coverage')
    profiles = rows(V / 'M00F_SIMULATION_PROFILE_COMPARISON.csv')
    must(len(profiles) >= 10 and all(x['classification'] in {'OBSERVED','INFERRED','UNKNOWN','DOCUMENTED'} for x in profiles), 'Profile comparison')
    for row in profiles:
        must(set(row['sources'].split(';')) <= {x['source_id'] for x in sources}, 'Unregistered profile source')
    passed('source_and_profile_trace', f'{len(sources)} registered source records with4 retrieved official '
           '/0.20/ pages; modern site commit unknown, checked-out configs hashed separately;11 comparison dimensions')
    status = (ROOT / 'RRM/PROJECT_STATUS.md').read_text()
    for token in ['G0 BLOCKED BY EXTERNAL INPUT','RA01 PENDING PD REVIEW','WP01 NOT AUTHORIZED',
                  'D2 DEFERRED','UNVERIFIED','execution NOT AUTHORIZED','M00-F1']:
        must(token in status, 'Missing governance status: '+token)
    required = ['M00F_AIRSTACK_ARCHITECTURE_MAP.md','M00F_SIMULATION_PROFILE_COMPARISON.csv',
                'M00F_PHYSICAL_ISOLATION_ANALYSIS.md','M00F_MINIMUM_SIMULATION_DESIGN.md',
                'M00F_ISOLATION_ACCEPTANCE_GATE.csv','M00F_F00_EQUIVALENCE_LIMITS.md','M00F_PD_TECHNICAL_DECISION.md']
    must(all((V/n).is_file() for n in required), 'Missing engineering deliverable')
    must((ROOT/'RRM/handoffs/M00F_HANDOFF.md').is_file(), 'Missing handoff')
    paths = (E/'M00F_CHANGED_FILES.txt').read_text().splitlines()
    must(paths == sorted(set(paths)) and all(p.startswith('RRM/') for p in paths), 'Invalid changed paths')
    actual = set(git('diff','--name-only',PARENT).decode().splitlines())
    actual.update(git('ls-files','--others','--exclude-standard','RRM/').decode().splitlines())
    must(actual == set(paths), 'Changed path mismatch')
    for line in (E/'SHA256SUMS').read_text().splitlines():
        digest,name = line.split('  ',1)
        must(sha((ROOT/name).read_bytes()) == digest, 'New checksum mismatch: '+name)
    summed = {x.split('  ',1)[1] for x in (E/'SHA256SUMS').read_text().splitlines()}
    must(summed == set(paths)-{'RRM/evidence/M00-F/SHA256SUMS','RRM/evidence/M00-F/M00F_VALIDATION_RESULTS.json'}, 'Checksum coverage mismatch')
    passed('deliverables_scope_governance_checksums', f'{len(paths)} exact RRM-only paths; PROJECT_STATUS sole '
           'prior-file edit; new checksums valid; original criteria/history unchanged')
    return dict(result='PASS', checks=checks, changed_file_count=len(paths),
                execution_gate='NOT PASSED', target_designation='UNVERIFIED',
                runtime_isolation='NOT YET VERIFIED', recommendation='F1',
                warnings=['Static source observations establish configuration intent, not effective runtime isolation.',
                          'Nonprivileged GPU compatibility, exact images/assets, host policy and measurement overhead unverified.',
                          'A prior-validator rerun after creating the new snapshot correctly rejected the successor file horizon; '
                          'the new snapshot was temporarily withheld and the prior clean-horizon PASS was captured before other writes.',
                          'No immutable website build commit available; retrieval times recorded; no version-equivalence assumption.',
                          'Initial staged whitespace check found CSV CRLF and directive Markdown hard-break spaces; normalized new files to LF without trailing spaces and rechecked.'],
                skipped_checks=['All Docker/Compose/ROS/PX4/MAVROS/Isaac execution, image pulls, installs, service/container starts/restarts, '
                                'device reads, hardware interaction and active network probes: unauthorized and not performed.',
                                'Effective physical/network/GPU/host isolation and recorder failure survival: no execution scope/target.',
                                'F00 qualification/equivalence: missing authentic originals and admitted approved campaign evidence.',
                                'Original M00 has no SHA256SUMS; full protected snapshot preserves it.'],
                limitations=['Policy validator accepts only a closed non-executable JSON design schema; it is not a general Compose '
                             'resolver or deployed-policy auditor. Synthetic positives are not runnable configurations.',
                             'No active operations occurred in this milestone; this statement is based on the executed tool actions, '
                             'not a claim that file hashes can prove absence of runtime activity.',
                             'Roles are required reviewers, not appointed people. No contact or universal organizational prerequisite introduced.'])


if __name__ == '__main__':
    print(json.dumps(validate(),indent=2))
