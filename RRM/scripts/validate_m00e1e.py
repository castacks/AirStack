#!/usr/bin/env python3
"""Check the local E1E decision package; no deployment or runtime inspection."""
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
from collections import Counter

ROOT = Path(__file__).resolve().parents[2]
E = ROOT / 'RRM/evidence/M00-E1E'
V = ROOT / 'RRM/verification/reports/M00-E1E'
PARENT = 'e6f0c81eff15538809b9745cef665de04ba4e796'
SUBJECT = 'RRM-F00 M00-E1E: prepare Architecture C decisions and external evidence closure'
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
    must(load(E / 'starting-integrity.json')['result'] == 'PASS', 'Prior E1D validator not passing')
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
    must(inputs == load(ROOT / 'RRM/evidence/M00-E1D/input-inventory.json')['files'],
         'Controlled input changed from prior milestone')
    must({x['path'] for x in inputs} == {str(p.relative_to(ROOT)) for p in
         (ROOT / 'RRM/RRM-C001-F00-EXT').rglob('*') if p.is_file()}, 'Input set mismatch')
    pins = {'M00-B': '93b8196e835effc3da34cac5677eab4d30c1378a',
            'M00-C': 'b4ac9467b073b6ae7927a47b939f46e9407cb6c0',
            'M00-D': '5b11d4d348db399e0e3ae638e318da9d7079ace6',
            'M00-E1': '31e33893ef2fa0f724862a980f6b5db0d62509d2',
            'M00-E1A': 'df8ebd18a6e16909cca461e18dfba882b53c8120',
            'M00-E1B': '4326dd079a2d80518983a5b793b9bab7f751a5db',
            'M00-E1C': '9d60ee46b3348fbaa1811a5b234f80446b62d5bd', 'M00-E1D': PARENT}
    count = 0
    for milestone, pin in pins.items():
        for line in (ROOT / f'RRM/evidence/{milestone}/SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ', 1)
            must(sha(git('show', pin + ':' + name)) == digest, 'Sealed history mismatch: ' + name)
            count += 1
    passed('starting_and_protected_integrity', f'Clean pinned E1D start; '
           f'{len(before["tracked_entries"])-1} protected entries, {count} pinned sealed checksums '
           f'and {len(inputs)} unchanged controlled input files')

    review = load(E / 'passive-record-review.json')
    must(len(review['records']) == 10 and review['controlled_input_same_as_e1d'], 'Passive acquisition scope')
    must(review['authoritative_new_owner_assignments'] == review['new_platform_attestations'] ==
         review['new_pd_decisions'] == review['new_prerequisite_closures'] == 0, 'Unsupported closure')
    for record in review['records']:
        data = (ROOT / record['path']).read_bytes()
        must(sha(data) == record['sha256'] and len(data) == record['bytes'], 'Acquisition identity')
        must(data == git('show', PARENT + ':' + record['path']), 'Source not pre-existing parent bytes')
        must(record['kind'] == 'NEW_E1E_PASSIVE_READ_OF_PREEXISTING_REPOSITORY_SOURCE'
             and record['same_bytes_at_starting_commit'], 'New authority conflated with source reread')
        lines = data.decode().splitlines()
        for match in record['matches']:
            must(lines[match['line']-1] == match['text'], 'Line-located observation changed')
    for name, present in review['standard_codeowners_presence'].items():
        must((ROOT / name).exists() == present and not present, 'CODEOWNERS presence changed')
    index = rows(E / 'M00E1E_EVIDENCE_INDEX.csv')
    must(len(index) == 15 and len({x['evidence_id'] for x in index}) == 15, 'Evidence index coverage')
    for row in index:
        data = (ROOT / row['path']).read_bytes()
        must(all(row.values()) and sha(data) == row['sha256'] and len(data) == int(row['bytes']),
             'Evidence index identity/limit missing')
    must(any(x['evidence_kind'].startswith('INHERITED') for x in index)
         and any(x['evidence_kind'].startswith('NEW_PASSIVE') for x in index), 'Evidence provenance conflated')
    passed('bounded_passive_acquisition_and_provenance', 'Ten line-located existing source reads; '
           '15 byte-bound index records distinguish acquisition from inherited design/episode; '
           'zero new authoritative owner/platform/PD records, no global absence assertion')

    baseline = (V / 'M00E1E_ARCHITECTURE_C_DECISION_BASELINE.md').read_text()
    for token in ['Verified', 'Proposed', 'Unverified', 'not approved', 'INCOMPLETE', HOLD]:
        must(token.lower() in baseline.lower(), 'Baseline category/limit missing: ' + token)
    decisions = rows(V / 'M00E1E_PD_DECISION_MATRIX.csv')
    must(len(decisions) == 5 and {x['decision_id'] for x in decisions} ==
         {f'D{i}' for i in range(1, 6)}, 'Decision coverage')
    blanks = {'pd_decision', 'approval_reference', 'date', 'conditions'}
    for row in decisions:
        must(all(row[k] == '' for k in blanks), 'PD approval invented')
        must(all(value for key, value in row.items() if key not in blanks), 'Decision rationale/scope incomplete')
    package = (V / 'M00E1E_PD_DECISION_PACKAGE.md').read_text()
    must(package.count('PD decision: ____________________') == 5
         and package.count('Approval reference: ____________________') == 5
         and 'D1 cannot approve D2–D5' in package and 'INCOMPLETE' in package,
         'Decision fields or no-inheritance rule missing')
    owners = rows(V / 'M00E1E_OWNER_AUTHORITY_MATRIX.csv')
    must(len(owners) == 10 and len({x['owner_id'] for x in owners}) == 10, 'Owner subsystem coverage')
    for row in owners:
        must(all(row.values()) and row['verification_status'] == 'OWNER UNVERIFIED'
             and row['verified_named_owner'] == 'NONE_ESTABLISHED', 'Unsupported owner assignment')
    passed('baseline_independent_decisions_and_owners', 'C remains conditional/unapproved; '
           'five separately scoped decision forms blank; ten subsystem owners explicitly unverified; '
           'exact change and implementation readiness incomplete')

    requests = rows(E / 'request-lifecycle.csv')
    must(len(requests) == 6 and {x['request_id'] for x in requests} ==
         {f'Q{i}' for i in range(1, 7)}, 'Request scope')
    must({item for row in requests for item in row['e1c_request_ids'].split(';')} ==
         {f'RQ{i:02d}' for i in range(1, 15)}, 'Original owner request lost')
    for row in requests:
        must(row['request_state'] == 'REQUEST PREPARED' and row['request_prepared'] == 'YES',
             'Request falsely advanced')
        for key in ['request_authorized', 'request_sent', 'response_received', 'evidence_verified', 'evidence_accepted']:
            must(row[key] == 'NO', 'Unsupported request state: ' + key)
        for key in ['authorization_reference', 'delivery_reference', 'response_reference', 'acceptance_reference']:
            must(not row[key], 'Invented request reference')
        must(set(row['owner_ids'].split(';')) <= {x['owner_id'] for x in owners}, 'Unknown request owner role')
        must(set(row['decision_ids'].split(';')) <= {x['decision_id'] for x in decisions}, 'Unknown decision')
        for key in ['exact_requested_artifacts', 'purpose', 'acceptance_criteria']:
            must(row[key], 'Request not actionable')
    controls = rows(V / 'M00E1E_SAFETY_ENFORCEMENT_EVIDENCE.csv')
    must(len(controls) == 12 and {x['control_id'] for x in controls} ==
         {f'IC{i:02d}' for i in range(1, 13)}, 'Safety evidence coverage')
    for row in controls:
        must(all(row.values()) and row['evidence_status'] == 'INCOMPLETE_UNVERIFIED_NOT_IMPLEMENTED'
             and row['diagnostic_credit'] == 'NONE_FOR_INDEPENDENT_ENFORCEMENT'
             and row['criterion_authority'] == 'PROPOSED_ARCHITECTURE_C_ENTRY_CRITERION_NOT_APPROVED_F00_REQUIREMENT',
             'Control verification/criterion overclaim')
    passed('request_states_and_enforcement_evidence', 'Six exact-artifact requests prepared only; '
           'all14 original RQ links retained;12 proposed controls require technical verification '
           'in addition to scoped owner attestations; monitoring earns no enforcement credit')

    claims = (V / 'M00E1E_CLAIM_LIMITS.md').read_text()
    for token in ['Class 1', 'Class 2', 'Class 3', 'equivalence', 'qualification', 'root cause',
                  'repeatability', 'Tier B execution approval', 'automatically close G0', 'n=1',
                  '19.276162179', '74.228771713']:
        must(token.lower() in claims.lower(), 'Claim limit missing: ' + token)
    spec = importlib.util.spec_from_file_location('historical', ROOT / 'RRM/scripts/m00e1a_analyze.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    replay = module.analyze(load(ROOT / 'RRM/evidence/M00-C/startup-source-records.json'))
    must({x['measurement_id']: x['seconds'] for x in replay['measurements']} ==
         {'L11': '19.276162179', 'L03': '74.228771713'}, 'Historical arithmetic changed')
    entry = rows(V / 'M00E1E_IMPLEMENTATION_ENTRY_CHECKLIST.csv')
    must(len(entry) == 12 and {x['entry_id'] for x in entry} == {f'EN{i:02d}' for i in range(1, 13)},
         'Entry checklist coverage')
    for row in entry:
        must(row['status'] == 'NOT_SATISFIED_EXTERNAL_OR_FUTURE_SCOPE'
             and not row['acceptance_reference'] and row['required_evidence'], 'Entry falsely satisfied')
    boundaries = load(E / 'authorization-boundaries.json')
    must(len(boundaries['phases']) == 6 and {x['phase_id'] for x in boundaries['phases']} ==
         {f'AUTH{i}' for i in range(1, 7)}, 'Authorization boundary coverage')
    must(all(not x['authorized'] and x['approval_reference'] is None for x in boundaries['phases'])
         and boundaries['no_inheritance_between_phases'] and not boundaries['architecture_c_approved']
         and not boundaries['tier_c_authorized'] and not boundaries['wp01_authorized']
         and boundaries['target_designation'] == 'UNVERIFIED', 'Authorization inferred')
    passed('claim_limits_entry_boundaries_and_timing', 'Three claim classes and unsupported claims explicit; '
           '12 entry items unsatisfied; six distinct authority stages ungranted; Decimal n=1 replay unchanged')

    old = rows(ROOT / 'RRM/evidence/M00-E1D/M00E1D_PREREQUISITE_IMPACT.csv')
    delta = rows(E / 'M00E1E_PREREQUISITE_DELTA.csv')
    dependencies = rows(E / 'M00E1E_EXTERNAL_DEPENDENCY_REGISTER.csv')
    must(len(old) == len(delta) == 20, 'Prerequisite count')
    for prior, current in zip(old, delta):
        must(prior['prerequisite_id'] == current['prerequisite_id']
             and prior['original_requirement'] == current['original_requirement']
             and prior['current_disposition'] == current['prior_disposition'] == current['current_disposition'],
             'Prerequisite identity/requirement/status changed')
        must(all(current.values()) and current['disposition_changed'] == current['closed_by_e1e'] ==
             current['execution_authorized'] == 'NO' and current['target_designation'] == 'UNVERIFIED',
             'Prerequisite/target overclaim')
    must(Counter(x['current_disposition'] for x in delta) ==
         Counter(SATISFIED=3, UNSATISFIED=8, UNVERIFIED=9), 'Disposition count drift')
    unresolved = {x['prerequisite_id'] for x in delta if x['current_disposition'] != 'SATISFIED'}
    must(len(dependencies) == 17 and {x['related_prerequisite_ids'] for x in dependencies} == unresolved
         and len({x['dependency_id'] for x in dependencies}) == 17, 'Dependency double count or lost CHK')
    for row in dependencies:
        must(all(row.values()) and row['blocker_category'] in 'ABCDE'
             and row['current_status'] == 'OPEN_NO_NEW_AUTHORITATIVE_CLOSURE', 'Dependency status/category')
        must(set(row['request_ids'].split(';')) <= {x['request_id'] for x in requests}, 'Unknown RFI')
        must('MISSING_TARGET_EVIDENCE_OR_APPROVAL' in row['information_vs_noncompliance'],
             'Missing evidence conflated with actual target noncompliance')
    status = (ROOT / 'RRM/PROJECT_STATUS.md').read_text()
    for token in [PARENT, 'M00-E1E', HOLD, 'BLOCKED BY EXTERNAL INPUT', 'PENDING PD REVIEW',
                  'WP01 NOT AUTHORIZED', '28 in-scope NOT_TESTED', '1 OUT_OF_SCOPE',
                  'zero whole closures', 'America/New_York', 'REQUEST PREPARED']:
        must(token in status, 'Status governance missing: ' + token)
    passed('prerequisite_dependencies_and_g0', 'All20 original identities/requirements/dispositions '
           'unchanged;17 dependency drilldowns one per unresolved CHK; zero closures; '
           'G0 blocked/RA01 pending/WP01 and Tier B unapproved; external-input hold explicit')

    audit = load(E / 'scope-audit.json')
    must(not audit['runtime_inspection_performed'] and not audit['construction_or_deployment_actions']
         and not audit['active_operations'] and audit['external_messages_sent'] == 0
         and not audit['credentials_read'] and not audit['push_performed'], 'Prohibited action recorded')
    required = ['M00E1E_ARCHITECTURE_C_DECISION_BASELINE.md', 'M00E1E_PD_DECISION_MATRIX.csv',
                'M00E1E_PD_DECISION_PACKAGE.md', 'M00E1E_OWNER_AUTHORITY_MATRIX.csv',
                'M00E1E_EXTERNAL_EVIDENCE_REQUEST.md', 'M00E1E_SAFETY_ENFORCEMENT_EVIDENCE.csv',
                'M00E1E_CLAIM_LIMITS.md', 'M00E1E_IMPLEMENTATION_ENTRY_CHECKLIST.csv',
                'M00E1E_G0_CONTINUITY.md']
    must(all((V / name).is_file() for name in required), 'Missing required report')
    must((ROOT / 'RRM/handoffs/M00E1E_HANDOFF.md').is_file(), 'Missing handoff')
    paths = (E / 'M00E1E_CHANGED_FILES.txt').read_text().splitlines()
    must(paths == sorted(set(paths)), 'Invalid changed list')
    must(all(x.startswith('RRM/') and not x.startswith('RRM/RRM-C001-F00-EXT/') for x in paths),
         'Unauthorized file scope')
    must(set(paths) & set(before['tracked_entries']) == {'RRM/PROJECT_STATUS.md'}, 'Prior file mutation')
    must(not any(x.endswith(('.sh', '.yaml', '.yml', '.service')) for x in paths), 'Deployment artifact')
    actual_new = set(git('ls-files').decode().splitlines()) - set(before['tracked_entries'])
    actual_new |= set(git('ls-files', '--others', '--exclude-standard').decode().splitlines())
    must(actual_new == set(paths) - {'RRM/PROJECT_STATUS.md'}, 'Unexpected new/missing file')
    staged = set(git('diff', '--cached', '--name-only').decode().splitlines())
    must(not staged or staged == set(paths), 'Unexpected staged set')
    if committed:
        must(not git('status', '--porcelain'), 'Postcommit dirty')
        must(set(git('diff-tree', '--no-commit-id', '--name-only', '-r', 'HEAD').decode().splitlines()) ==
             set(paths), 'Committed set differs')
    hashed = set()
    for line in (E / 'SHA256SUMS').read_text().splitlines():
        digest, name = line.split('  ', 1)
        must(name not in hashed and sha((ROOT / name).read_bytes()) == digest, 'Package checksum: ' + name)
        hashed.add(name)
    must(hashed == set(paths) - {'RRM/evidence/M00-E1E/SHA256SUMS',
                               'RRM/evidence/M00-E1E/validation-results.json'}, 'Checksum coverage')
    for path in paths:
        data = (ROOT / path).read_bytes()
        must(b'\r\n' not in data, 'CRLF whitespace: ' + path)
        must(all(line == line.rstrip() for line in data.decode().splitlines()), 'Trailing whitespace: ' + path)
    subprocess.run(['git', 'diff', '--check'], cwd=ROOT, check=True)
    subprocess.run(['git', 'diff', '--cached', '--check'], cwd=ROOT, check=True)
    passed('no_active_operations_and_package_scope', f'{len(paths)} exact RRM-only changes; '
           'PROJECT_STATUS sole prior mutation; complete nonrecursive sums; no deployment/startup '
           'automation or active action recorded; offline validator uses local files/read-only Git')
    return dict(milestone='M00-E1E', result='PASS', checks=checks, checksum_verification='PASS',
                repository_scope_verification='PASS', hold=HOLD, architecture_c_approved=False,
                pd_decisions_approved=0, named_authoritative_owners_verified=0,
                new_authoritative_prerequisite_closures=0, prerequisite_status_changes=0,
                request_state='REQUEST PREPARED', requests_sent=0, target_designation='UNVERIFIED',
                implementation_authorized=False, tier_b_authorized=False, wp01_authorized=False,
                warnings=audit['warnings'], skipped_checks=audit['skipped_checks'],
                evidence_gaps=['No named delegated subsystem/operational owner or PD decision',
                               'No assigned compatible Architecture C target/effective isolation attestation',
                               'No exact approved configuration, native acquisition/clock/overhead/recorder proof',
                               'No authenticated full frozen originals/equivalence/qualified campaign evidence'],
                limitations=['PASS is offline document/provenance/integrity consistency only; owner-dependent '
                             'artifacts remain explicitly incomplete. No safe-to-operate or implementation-ready finding.',
                             'New acquisition rereads existing repository sources; it adds no authoritative '
                             'owner/platform/PD closure. No runtime observations, starts, active probes or external delivery.',
                             'No standalone behavioral tests required for documentation changes; executed '
                             'cross-reference checks and independent historical replay do not prove real controls.'])


if __name__ == '__main__':
    print(json.dumps(validate(), indent=2))
