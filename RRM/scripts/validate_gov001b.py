#!/usr/bin/env python3
"""Check the local GOV-001B coordination package; no deployment or runtime inspection."""
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
from collections import Counter

ROOT = Path(__file__).resolve().parents[2]
E = ROOT / 'RRM/evidence/GOV-001B'
V = ROOT / 'RRM/governance/GOV-001B'
PARENT = 'a72b8f412f2567e870903e2f4fec8082bc437e21'
SUBJECT = 'RRM-F00 GOV-001B: prepare ownership confirmation and evidence intake'
HOLD = 'EXTERNAL-INPUT HOLD — ARCHITECTURE C IMPLEMENTATION NOT AUTHORIZED'


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
    must(load(E / 'starting-integrity.json')['result'] == 'PASS', 'Prior GOV-001A validator not passing')
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
    must(inputs == load(ROOT / 'RRM/evidence/GOV-001A/input-inventory.json')['files'],
         'Controlled input changed from prior milestone')
    must({x['path'] for x in inputs} == {str(p.relative_to(ROOT)) for p in
         (ROOT / 'RRM/RRM-C001-F00-EXT').rglob('*') if p.is_file()}, 'Input set mismatch')
    pins = {'M00-B': '93b8196e835effc3da34cac5677eab4d30c1378a',
            'M00-C': 'b4ac9467b073b6ae7927a47b939f46e9407cb6c0',
            'M00-D': '5b11d4d348db399e0e3ae638e318da9d7079ace6',
            'M00-E1': '31e33893ef2fa0f724862a980f6b5db0d62509d2',
            'M00-E1A': 'df8ebd18a6e16909cca461e18dfba882b53c8120',
            'M00-E1B': '4326dd079a2d80518983a5b793b9bab7f751a5db',
            'M00-E1C': '9d60ee46b3348fbaa1811a5b234f80446b62d5bd', 'M00-E1D': 'e6f0c81eff15538809b9745cef665de04ba4e796', 'M00-E1E': '0fa876fd8191be61a3e6724e1a3c93fe64510f01', 'GOV-001A': PARENT}
    count = 0
    for milestone, pin in pins.items():
        for line in (ROOT / f'RRM/evidence/{milestone}/SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ', 1)
            must(sha(git('show', pin + ':' + name)) == digest, 'Sealed history mismatch: ' + name)
            count += 1
    passed('starting_and_protected_integrity', f'Clean pinned GOV-001A start; '
           f'{len(before["tracked_entries"])-1} protected entries, {count} pinned sealed checksums '
           f'and {len(inputs)} unchanged controlled input files')



    search = load(E / 'bounded-search.json')
    findings = rows(V / 'GOV001B_OWNER_RECORD_SEARCH.csv')
    must(len(findings) == 11 and len({x['search_id'] for x in findings}) == 11, 'Bounded source search scope')
    must(all(all(x.values()) and x['authority_established'] == 'NO' for x in findings), 'Unsupported authority')
    must(len(search['records']) == 7 and search['authoritative_organizational_records_found'] ==
         search['verified_accountable_owners'] == search['new_platform_attestations'] == 0
         and search['controlled_input_unchanged'], 'Search/closure overclaim')
    for x in search['records']:
        data = (ROOT / x['path']).read_bytes()
        must(sha(data) == x['sha256'] and len(data) == x['bytes']
             and data == git('show', PARENT + ':' + x['path']), 'Search source identity')
        if x['line']:
            must(data.decode().splitlines()[x['line']-1] == x['excerpt'], 'Search source locator')
    for path in ['CODEOWNERS', '.github/CODEOWNERS', 'docs/CODEOWNERS']:
        must(not (ROOT / path).exists(), 'Ownership-file discovery changed')
    structure = (V / 'GOV001B_MINIMUM_OWNER_STRUCTURE.md').read_text()
    for token in ['OR1', 'OR2', 'OR3', 'OR4', 'OR5', 'OR6', 'independent', 'delegation', 'OWNER UNVERIFIED',
                  'not an approved F00 independence rule']:
        must(token in structure, 'Owner structure/qualification missing: ' + token)
    passed('bounded_owner_search_and_minimum_roles', 'Six responsibilities/consolidation conditions '
           'with independent review;11 bounded source/location checks; zero accountable organizational owners '
           'verified; no contribution-to-authority inference or appointment')

    matrix = rows(V / 'GOV001B_DECISION_EVIDENCE_MATRIX.csv')
    must(len(matrix) == 6 and {x['request_id'] for x in matrix} == set('ABCDEF'), 'Request coverage')
    old = rows(ROOT / 'RRM/evidence/GOV-001A/GOV001A_PREREQUISITE_DELTA.csv')
    unresolved = {x['prerequisite_id'] for x in old if x['current_disposition'] != 'SATISFIED'}
    for row in matrix:
        must(all(row.values()) and row['state'] == 'REQUEST PREPARED', 'Request incomplete or falsely advanced')
        must({'D2', 'D4'} <= set(row['decision_ids'].split(';')) <= {f'D{i}' for i in range(1, 6)},
             'Missing governing decision link')
        must(set(row['prerequisite_ids'].split(';')) <= unresolved, 'Invalid unresolved CHK link')
        must(set(row['g0_blocker_ids'].split(';')) <= {f'M00-B{i:02d}' for i in range(1, 11)},
             'Unknown G0 blocker')
    must({item for row in matrix for item in row['prerequisite_ids'].split(';')} == unresolved,
         'Unresolved baseline gap omitted from requests')
    request = (V / 'GOV001B_ORGANIZATIONAL_REQUEST.md').read_text()
    must('PREPARED — NOT SENT' in request and 'Delivery is not authorized by GOV-001B' in request,
         'Unsent/authorization boundary missing')
    passed('targeted_request_and_decision_trace', 'A–F exact-artifact/authority/purpose/format/acceptance '
           'and consequence trace; all17 unresolved CHKs and original G0 IDs linked; requests prepared only')

    spec = importlib.util.spec_from_file_location('intake', ROOT / 'RRM/scripts/gov001b_intake_check.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    expected_states = ('REQUEST PREPARED', 'DELIVERY AUTHORIZED', 'REQUEST SENT', 'RESPONSE RECEIVED',
                       'PROVENANCE VERIFIED', 'TECHNICALLY REVIEWED', 'ACCEPTED', 'REJECTED OR INSUFFICIENT')
    must(module.STATES == expected_states, 'Intake states differ')
    intake = rows(V / 'GOV001B_EVIDENCE_INTAKE_REGISTER.csv')
    histories = load(E / 'request-history.json')
    must(len(intake) == 6 and set(histories) == {x['request_id'] for x in intake} == set('ABCDEF'),
         'Prospective intake count')
    for row in intake:
        result = module.check_record(row, histories[row['request_id']])
        must(row['state'] == 'REQUEST PREPARED' and row['review_disposition'] == 'NOT RECEIVED'
             and not result['authenticity_verified_by_checker'] and not result['operational_authority_conferred'],
             'Intake metadata confers false authority/receipt')
        must(not row['evidence_id'] and not row['date_received'] and not row['reviewer'], 'Fabricated received item')
    protocol = (V / 'GOV001B_EVIDENCE_INTAKE_PROTOCOL.md').read_text()
    for token in expected_states:
        must(token in protocol, 'Protocol state missing: ' + token)
    for token in ['Authentic', 'restricted', 'redacted', 'Unsolicited', 'quarantine',
                  'do not invent', 'independent', 'not received']:
        must(token.lower() in protocol.lower(), 'Protocol limit missing: ' + token)
    log = (E / 'offline-tests.txt').read_text()
    must('Ran 10 tests' in log and log.rstrip().endswith('OK') and 'FAILED' not in log,
         'Offline protocol tests not passing')
    import ast
    tree = ast.parse((ROOT / 'RRM/scripts/gov001b_intake_check.py').read_text())
    imports = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
    imports |= {alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
    must(imports <= {'datetime', 're'}, 'Intake checker contains runtime/delivery API')
    passed('intake_state_guards_and_offline_tests', 'Eight-state protocol; six prospective slots/zero '
           'received items; ten executed synthetic tests PASS for missing authority, skipped states, '
           'insufficient evidence, negative branches and restricted/independent acceptance; metadata only')

    holds = rows(V / 'GOV001B_EXTERNAL_HOLD_REGISTER.csv')
    must(len(holds) == 7 and len({x['hold_id'] for x in holds}) == 7, 'Hold coverage')
    for row in holds:
        must(all(row.values()) and row['current_status'] == 'OPEN_EXTERNAL_INPUT_HOLD'
             and row['schedule'] == 'EVENT_DRIVEN_ONLY_NO_RECURRING_WORK', 'Hold falsely closed or scheduled')
        must(set(row['request_ids'].split(';')) <= set('ABCDEF')
             and set(row['prerequisite_ids'].split(';')) <= unresolved, 'Hold trace mismatch')
    dispositions = load(ROOT / 'RRM/evidence/GOV-001A/dispositions.json')
    must([x['disposition'] for x in dispositions['decisions']] ==
         ['CONDITIONALLY APPROVED FOR FURTHER OFFLINE DESIGN.', 'DEFERRED.',
          'APPROVED AS PROJECT EVIDENCE POLICY.', 'PENDING VERIFICATION.',
          'APPROVED — PASSIVE AND OFFLINE WORK ONLY.'], 'Project dispositions changed')
    delta = rows(E / 'GOV001B_PREREQUISITE_DELTA.csv')
    must(len(delta) == len(old) == 20, 'Technical prerequisite count')
    for a, b in zip(old, delta):
        must(a['prerequisite_id'] == b['prerequisite_id'] and a['original_requirement'] == b['original_requirement']
             and a['current_disposition'] == b['prior_disposition'] == b['current_disposition'],
             'Prerequisite identity/status altered')
        must(all(b.values()) and b['disposition_changed'] == b['closed_by_gov001b'] ==
             b['execution_authorized'] == 'NO' and b['target_designation'] == 'UNVERIFIED', 'False closure/authority')
    must(Counter(x['current_disposition'] for x in delta) ==
         Counter(SATISFIED=3, UNSATISFIED=8, UNVERIFIED=9), 'Baseline totals changed')
    status = (ROOT / 'RRM/PROJECT_STATUS.md').read_text()
    for token in [PARENT, 'GOV-001B', HOLD, 'DEFERRED', 'PENDING VERIFICATION', 'PENDING PD REVIEW',
                  'WP01 NOT AUTHORIZED', 'BLOCKED BY EXTERNAL INPUT', '28 in-scope NOT_TESTED',
                  '1 OUT_OF_SCOPE', 'PREPARED — NOT SENT', 'zero whole closures']:
        must(token in status, 'Status control missing: ' + token)
    passed('event_driven_holds_and_governance', 'Seven open trigger-based holds, no scheduled recurrence; '
           '20 unchanged CHKs/3 satisfied/8 unsatisfied/9 unverified; D1–D5 and G0/RA01/WP01/Tier B controls retained')

    index = rows(E / 'GOV001B_EVIDENCE_INDEX.csv')
    must(len(index) == 12 and len({x['evidence_id'] for x in index}) == 12, 'Evidence index count')
    for row in index:
        data = (ROOT / row['path']).read_bytes()
        must(all(row.values()) and len(data) == int(row['bytes']) and sha(data) == row['sha256'],
             'Evidence index identity mismatch')
    audit = load(E / 'scope-audit.json')
    must(not audit['runtime_inspection_performed'] and not audit['construction_or_deployment_actions']
         and not audit['active_operations'] and audit['external_messages_sent'] == 0
         and not audit['credentials_read'] and not audit['push_performed']
         and audit['owners_appointed'] == audit['received_organizational_evidence_items'] ==
         audit['accepted_organizational_evidence_items'] == audit['new_authoritative_records'] ==
         audit['technical_prerequisite_closures'] == 0 and not audit['recurring_work_scheduled'],
         'Prohibited coordination/operation/closure recorded')
    required = ['GOV001B_MINIMUM_OWNER_STRUCTURE.md', 'GOV001B_OWNER_RECORD_SEARCH.csv',
                'GOV001B_ORGANIZATIONAL_REQUEST.md', 'GOV001B_EVIDENCE_INTAKE_PROTOCOL.md',
                'GOV001B_EVIDENCE_INTAKE_REGISTER.csv', 'GOV001B_DECISION_EVIDENCE_MATRIX.csv',
                'GOV001B_EXTERNAL_HOLD_REGISTER.csv', 'GOV001B_PD_ACTION_SUMMARY.md']
    must(all((V / name).is_file() for name in required), 'Required governance artifact missing')
    must((ROOT / 'RRM/handoffs/GOV001B_HANDOFF.md').is_file(), 'Handoff missing')
    paths = (E / 'GOV001B_CHANGED_FILES.txt').read_text().splitlines()
    must(paths == sorted(set(paths)), 'Changed list invalid')
    must(all(x.startswith('RRM/') and not x.startswith('RRM/RRM-C001-F00-EXT/') for x in paths),
         'Unauthorized scope')
    must(set(paths) & set(before['tracked_entries']) == {'RRM/PROJECT_STATUS.md'}, 'Old-file mutation')
    must(not any(x.endswith(('.sh', '.yaml', '.yml', '.service')) for x in paths), 'Runtime automation')
    actual = set(git('ls-files').decode().splitlines()) - set(before['tracked_entries'])
    actual |= set(git('ls-files', '--others', '--exclude-standard').decode().splitlines())
    must(actual == set(paths) - {'RRM/PROJECT_STATUS.md'}, 'Unexpected file set')
    staged = set(git('diff', '--cached', '--name-only').decode().splitlines())
    must(not staged or staged == set(paths), 'Staged set mismatch')
    if committed:
        must(not git('status', '--porcelain'), 'Postcommit dirty')
        must(set(git('diff-tree', '--no-commit-id', '--name-only', '-r', 'HEAD').decode().splitlines())
             == set(paths), 'Committed set mismatch')
    import re
    for name in paths:
        data = (ROOT / name).read_bytes()
        must(b'\r\n' not in data and all(line == line.rstrip() for line in data.decode().splitlines()),
             'Whitespace: ' + name)
        text = data.decode()
        must(re.search(r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----', text) is None
             and re.search(r'\b(?:AKIA|ASIA)[A-Z0-9]{16}\b', text) is None,
             'Potential credential material: ' + name)
    passed('provenance_scope_and_sensitive_material_review', '12 byte-bound evidence records distinguish '
           'inherited/source/synthetic records; no external artifacts/credentials read or included; '
           'high-confidence key-pattern scan plus scoped author review, not a complete secret-detection guarantee')
    hashed = set()
    for line in (E / 'SHA256SUMS').read_text().splitlines():
        digest, path = line.split('  ', 1)
        must(path not in hashed and sha((ROOT / path).read_bytes()) == digest, 'Package checksum: ' + path)
        hashed.add(path)
    must(hashed == set(paths) - {'RRM/evidence/GOV-001B/SHA256SUMS',
                               'RRM/evidence/GOV-001B/validation-results.json'}, 'Checksum coverage')
    subprocess.run(['git', 'diff', '--check'], cwd=ROOT, check=True)
    subprocess.run(['git', 'diff', '--cached', '--check'], cwd=ROOT, check=True)
    passed('package_hashes_and_repository_scope', f'{len(paths)} exact RRM-only paths; PROJECT_STATUS '
           'sole prior mutation; sums cover all except recursive sums/results, bound by final Git; '
           'staging/whitespace/protected history checks executed')
    return dict(milestone='GOV-001B', result='PASS', checks=checks, failures=[],
                offline_tests=dict(executed=10, passed=10, failed=0, fixtures='SYNTHETIC_METADATA_ONLY'),
                checksum_verification='PASS', repository_scope_verification='PASS',
                organizational_request_status='PREPARED — NOT SENT', verified_accountable_owners=0,
                newly_discovered_authoritative_organizational_records=0, actual_received_evidence_items=0,
                accepted_evidence_items=0, technical_prerequisite_changes=0, technical_prerequisite_closures=0,
                target_designation='UNVERIFIED', implementation_authorized=False, hold=HOLD,
                warnings=audit['warnings'], skipped_checks=audit['skipped_checks'],
                limitations=['Offline consistency/integrity/state checks cannot authenticate organizational '
                             'authority, verify restricted originals, certify effective isolation or approve operations.',
                             'Ten tests exercise synthetic metadata only; six prepared slots are not actual '
                             'received evidence. Intake acceptance never automatically closes a prerequisite or decision.',
                             'No external communication, owner appointment, deployment/runtime/hardware/active operation '
                             'or recurring work. Stop after GOV-001B, no push.'])


if __name__ == '__main__':
    print(json.dumps(validate(), indent=2))
