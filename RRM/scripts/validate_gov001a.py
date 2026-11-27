#!/usr/bin/env python3
"""Check the local GOV-001A governance package; no deployment or runtime inspection."""
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
from collections import Counter

ROOT = Path(__file__).resolve().parents[2]
E = ROOT / 'RRM/evidence/GOV-001A'
V = ROOT / 'RRM/governance/GOV-001A'
PARENT = '0fa876fd8191be61a3e6724e1a3c93fe64510f01'
SUBJECT = 'RRM-F00 GOV-001A: record PD dispositions and escalate owner evidence'
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
    must(load(E / 'starting-integrity.json')['result'] == 'PASS', 'Prior E1E validator not passing')
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
    must(inputs == load(ROOT / 'RRM/evidence/M00-E1E/input-inventory.json')['files'],
         'Controlled input changed from prior milestone')
    must({x['path'] for x in inputs} == {str(p.relative_to(ROOT)) for p in
         (ROOT / 'RRM/RRM-C001-F00-EXT').rglob('*') if p.is_file()}, 'Input set mismatch')
    pins = {'M00-B': '93b8196e835effc3da34cac5677eab4d30c1378a',
            'M00-C': 'b4ac9467b073b6ae7927a47b939f46e9407cb6c0',
            'M00-D': '5b11d4d348db399e0e3ae638e318da9d7079ace6',
            'M00-E1': '31e33893ef2fa0f724862a980f6b5db0d62509d2',
            'M00-E1A': 'df8ebd18a6e16909cca461e18dfba882b53c8120',
            'M00-E1B': '4326dd079a2d80518983a5b793b9bab7f751a5db',
            'M00-E1C': '9d60ee46b3348fbaa1811a5b234f80446b62d5bd', 'M00-E1D': 'e6f0c81eff15538809b9745cef665de04ba4e796', 'M00-E1E': PARENT}
    count = 0
    for milestone, pin in pins.items():
        for line in (ROOT / f'RRM/evidence/{milestone}/SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ', 1)
            must(sha(git('show', pin + ':' + name)) == digest, 'Sealed history mismatch: ' + name)
            count += 1
    passed('starting_and_protected_integrity', f'Clean pinned E1E start; '
           f'{len(before["tracked_entries"])-1} protected entries, {count} pinned sealed checksums '
           f'and {len(inputs)} unchanged controlled input files')


    directive = (E / 'EXECUTION_DIRECTIVE.md').read_text()
    record = (V / 'GOV001A_PD_DISPOSITION_RECORD.md').read_text()
    exact = directive[directive.index('### D1 —'):directive.index('\n## 4.')]
    must(exact in record, 'Directed decision text/limitations not reproduced exactly')
    expected = {'D1': 'CONDITIONALLY APPROVED FOR FURTHER OFFLINE DESIGN.',
                'D2': 'DEFERRED.', 'D3': 'APPROVED AS PROJECT EVIDENCE POLICY.',
                'D4': 'PENDING VERIFICATION.', 'D5': 'APPROVED — PASSIVE AND OFFLINE WORK ONLY.'}
    dispositions = load(E / 'dispositions.json')
    must({x['decision_id']: x['disposition'] for x in dispositions['decisions']} == expected,
         'Directed dispositions differ')
    must(dispositions['directive_identifier'] == 'RRM-C001-F00-GOV-001A'
         and dispositions['program_director_as_directed'] ==
         'ChatGPT — RRM engineering-program decision authority', 'Project authority altered')
    for name in ['signature', 'external_authorization_identifier']:
        must(dispositions[name] is None, 'Invented signature/external identifier')
    positives = {'architecture_c_offline_design_conditionally_approved',
                 'project_evidence_policy_approved', 'passive_offline_work_authorized'}
    for name in positives:
        must(dispositions[name] is True, 'Actual project approval not recorded: ' + name)
    for name, value in dispositions.items():
        if isinstance(value, bool) and name not in positives:
            must(value is False, 'Operational/material/owner approval inferred: ' + name)
    must(dispositions['target_designation'] == 'UNVERIFIED', 'Target assigned')
    receipt = load(E / 'directive-receipt.json')
    from datetime import datetime
    must(receipt['received_date'] == dispositions['receipt_date'] == '2026-11-27', 'Receipt date mismatch')
    must(datetime.fromisoformat(receipt['recorded_utc']).timestamp() ==
         datetime.fromisoformat(receipt['recorded_local']).timestamp(), 'Receipt clock representation mismatch')
    must(receipt['signature'] is None and receipt['external_authorization_identifier'] is None
         and 'Recording time' in receipt['timestamp_limit'], 'Receipt provenance overclaim')
    matrix = rows(V / 'GOV001A_DECISION_DEPENDENCY_MATRIX.csv')
    must(len(matrix) == 5 and {x['decision_id']: x['current_disposition'] for x in matrix} == expected,
         'Dependency dispositions differ')
    must(all(all(row.values()) for row in matrix), 'Decision dependencies incomplete')
    passed('directed_dispositions_and_limits', 'Exact directed D1–D5 text/conditions retained; '
           'D1 conditional offline/D3 evidence policy/D5 passive-offline approved; D2 deferred/D4 pending; '
           'receipt provenance explicit, no signature/external approval/operational authority invented')

    owners = rows(V / 'GOV001A_OWNER_DISCOVERY.csv')
    must(len(owners) == 9 and len({x['owner_id'] for x in owners}) == 9, 'Owner coverage')
    for row in owners:
        must(all(row.values()) and row['ownership_status'] == 'OWNER UNVERIFIED'
             and row['decision_authority_verified'] == 'NO', 'Organizational owner invented')
    review = load(E / 'passive-source-review.json')
    must(len(review['records']) == 8 and review['authoritative_infrastructure_owners_identified'] ==
         review['new_platform_attestations'] == review['external_requests_sent'] == 0, 'Discovery overclaim')
    for row in review['records']:
        data = (ROOT / row['path']).read_bytes()
        must(sha(data) == row['sha256'] and len(data) == row['bytes']
             and data == git('show', PARENT + ':' + row['path']), 'Source review identity')
        lines = data.decode().splitlines()
        for match in row['matches']:
            must(lines[match['line']-1] == match['text'], 'Source locator mismatch')
    for path, present in review['standard_codeowners_presence'].items():
        must((ROOT / path).exists() == present and not present, 'CODEOWNERS discovery mismatch')
    index = rows(E / 'GOV001A_EVIDENCE_INDEX.csv')
    must(len(index) == 14 and len({x['evidence_id'] for x in index}) == 14, 'Evidence index scope')
    for row in index:
        data = (ROOT / row['path']).read_bytes()
        must(all(row.values()) and len(data) == int(row['bytes']) and sha(data) == row['sha256'],
             'Indexed identity/limit mismatch')
    kinds = {x['evidence_kind'] for x in index}
    must('NEW_USER_DIRECTED_PROJECT_AUTHORITY' in kinds
         and 'NEW_REVIEW_OF_EXISTING_SOURCE' in kinds and 'INHERITED_HISTORICAL_ENVIRONMENT' in kinds,
         'Authority/source/historical evidence conflated')
    assessment = (V / 'GOV001A_PLATFORM_EVIDENCE_ASSESSMENT.md').read_text()
    for token in ['Previously available', 'Newly recorded', 'Inference', 'Missing authority',
                  'not current runtime', 'privilege', 'MAVLink', 'DDS', 'instrumentation']:
        must(token.lower() in assessment.lower(), 'Platform evidence distinction missing: ' + token)
    passed('ownership_platform_and_new_evidence', 'Nine candidate roles remain OWNER UNVERIFIED; '
           'eight exact source reads/14 indexed records separate new project authority, source intent '
           'and historical runtime; zero organizational appointment/platform attestation')

    requests = rows(E / 'request-lifecycle.csv')
    must(len(requests) == 4 and {x['request_id'] for x in requests} ==
         {f'R{i}' for i in range(1, 5)}, 'Request coverage')
    must({item for row in requests for item in row['e1e_request_ids'].split(';')} ==
         {f'Q{i}' for i in range(1, 7)}, 'Inherited request lost')
    for row in requests:
        must(row['state'] == 'PREPARED', 'Request falsely advanced')
        for key in ['authorized_for_delivery', 'sent', 'response_received', 'verified', 'accepted']:
            must(row[key] == 'NO', 'Unsupported request state')
        for key in ['delivery_reference', 'response_reference', 'acceptance_reference']:
            must(row[key] == '', 'Invented request reference')
        must(set(row['owner_candidate_ids'].split(';')) <= {x['owner_id'] for x in owners}, 'Unknown owner role')
    request = (V / 'GOV001A_EXTERNAL_EVIDENCE_REQUEST.md').read_text()
    for token in ['Confidentiality', 'provenance', 'If unavailable', 'EXTERNAL COMMUNICATION NOT AUTHORIZED']:
        must(token.lower() in request.lower(), 'Request field missing: ' + token)
    actions = (V / 'GOV001A_ESCALATION_ACTION_SHEET.md').read_text()
    for token in ['organizational', 'R1', 'Delivery gate', 'stop', 'D5']:
        must(token.lower() in actions.lower(), 'Escalation missing: ' + token)
    passed('prepared_external_escalation', 'R1–R4 exact-record request retains Q1–Q6; '
           'provenance/confidentiality/unavailable consequences and first organizational action explicit; '
           'no delivery authorization, message, response or acceptance claimed')

    prior = rows(ROOT / 'RRM/evidence/M00-E1E/M00E1E_PREREQUISITE_DELTA.csv')
    delta = rows(E / 'GOV001A_PREREQUISITE_DELTA.csv')
    must(len(prior) == len(delta) == 20, 'Original prerequisite count')
    for a, b in zip(prior, delta):
        must(a['prerequisite_id'] == b['prerequisite_id']
             and a['original_requirement'] == b['original_requirement']
             and a['current_disposition'] == b['prior_disposition'] == b['current_disposition'],
             'Technical identity/status changed')
        must(all(b.values()) and b['disposition_changed'] == b['closed_by_gov001a'] ==
             b['execution_authorized'] == 'NO' and b['target_designation'] == 'UNVERIFIED',
             'Project decision falsely closes technical prerequisite')
    must(Counter(x['current_disposition'] for x in delta) ==
         Counter(SATISFIED=3, UNSATISFIED=8, UNVERIFIED=9), 'Prerequisite totals changed')
    status = (ROOT / 'RRM/PROJECT_STATUS.md').read_text()
    for token in [PARENT, 'GOV-001A', HOLD, 'CONDITIONALLY APPROVED FOR FURTHER OFFLINE DESIGN',
                  'APPROVED AS PROJECT EVIDENCE POLICY', 'APPROVED — PASSIVE AND OFFLINE WORK ONLY',
                  'DEFERRED', 'PENDING VERIFICATION', 'BLOCKED BY EXTERNAL INPUT', 'PENDING PD REVIEW',
                  'WP01 NOT AUTHORIZED', '28 in-scope NOT_TESTED', '1 OUT_OF_SCOPE', 'zero whole closures']:
        must(token in status, 'Status governance missing: ' + token)
    passed('technical_prerequisite_and_gate_continuity', '20 original technical prerequisites unchanged '
           '3/8/9; operational CHK01 not closed by limited project authority; G0 blocked/RA01 pending/ '
           'WP01 and Tier B/C unauthorized, target undesignated; D2/D4 unresolved')

    audit = load(E / 'scope-audit.json')
    must(not audit['runtime_inspection_performed'] and not audit['construction_or_deployment_actions']
         and not audit['active_operations'] and audit['external_messages_sent'] == 0
         and not audit['credentials_read'] and not audit['push_performed'], 'Prohibited activity recorded')
    required = ['GOV001A_PD_DISPOSITION_RECORD.md', 'GOV001A_OWNER_DISCOVERY.csv',
                'GOV001A_PLATFORM_EVIDENCE_ASSESSMENT.md', 'GOV001A_EXTERNAL_EVIDENCE_REQUEST.md',
                'GOV001A_DECISION_DEPENDENCY_MATRIX.csv', 'GOV001A_ESCALATION_ACTION_SHEET.md',
                'GOV001A_G0_CONTINUITY.md']
    must(all((V / name).is_file() for name in required), 'Required governance artifact missing')
    must((ROOT / 'RRM/handoffs/GOV001A_HANDOFF.md').is_file(), 'Handoff missing')
    paths = (E / 'GOV001A_CHANGED_FILES.txt').read_text().splitlines()
    must(paths == sorted(set(paths)), 'Changed list invalid')
    must(all(x.startswith('RRM/') and not x.startswith('RRM/RRM-C001-F00-EXT/') for x in paths),
         'Unauthorized scope')
    must(set(paths) & set(before['tracked_entries']) == {'RRM/PROJECT_STATUS.md'}, 'Prior protected mutation')
    must(not any(x.endswith(('.sh', '.yaml', '.yml', '.service')) for x in paths), 'Runtime automation added')
    actual = set(git('ls-files').decode().splitlines()) - set(before['tracked_entries'])
    actual |= set(git('ls-files', '--others', '--exclude-standard').decode().splitlines())
    must(actual == set(paths) - {'RRM/PROJECT_STATUS.md'}, 'New file set mismatch')
    staged = set(git('diff', '--cached', '--name-only').decode().splitlines())
    must(not staged or staged == set(paths), 'Staged scope mismatch')
    if committed:
        must(not git('status', '--porcelain'), 'Postcommit dirty')
        must(set(git('diff-tree', '--no-commit-id', '--name-only', '-r', 'HEAD').decode().splitlines())
             == set(paths), 'Commit scope mismatch')
    hashed = set()
    for line in (E / 'SHA256SUMS').read_text().splitlines():
        digest, path = line.split('  ', 1)
        must(path not in hashed and sha((ROOT / path).read_bytes()) == digest, 'Package checksum: ' + path)
        hashed.add(path)
    must(hashed == set(paths) - {'RRM/evidence/GOV-001A/SHA256SUMS',
                               'RRM/evidence/GOV-001A/validation-results.json'}, 'Checksum coverage mismatch')
    for name in paths:
        data = (ROOT / name).read_bytes()
        must(b'\r\n' not in data and all(line == line.rstrip() for line in data.decode().splitlines()),
             'Whitespace: ' + name)
    subprocess.run(['git', 'diff', '--check'], cwd=ROOT, check=True)
    subprocess.run(['git', 'diff', '--cached', '--check'], cwd=ROOT, check=True)
    passed('package_scope_and_no_active_operations', f'{len(paths)} exact RRM-only paths; '
           'PROJECT_STATUS sole old-file mutation; no competing governance task or old-form rewrite; '
           'checksums complete excluding recursive sums/result, both bound by Git; no prohibited actions recorded')
    return dict(milestone='GOV-001A', result='PASS', checks=checks, failures=[],
                checksum_verification='PASS', repository_scope_verification='PASS',
                project_dispositions=expected, infrastructure_safety_owners_verified=0,
                new_platform_attestations=0, technical_prerequisite_changes=0, technical_prerequisite_closures=0,
                external_requests_state='PREPARED', external_requests_sent=0,
                implementation_authorized=False, target_designation='UNVERIFIED', hold=HOLD,
                warnings=audit['warnings'], skipped_checks=audit['skipped_checks'],
                limitations=['PASS means offline directive/provenance/consistency/integrity verification. '
                             'It does not certify platform feasibility, isolation, measurement integrity or qualification.',
                             'New project design/policy/offline authority is real, but organizational ownership, '
                             'exact D2 change and operational authority remain absent. No technical closure inferred.',
                             'No runtime/active tests performed; no behavioral unit tests needed for governance records. '
                             'Stop GOV-001A for Program Director technical review; no push.'])


if __name__ == '__main__':
    print(json.dumps(validate(), indent=2))
