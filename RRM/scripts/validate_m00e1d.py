#!/usr/bin/env python3
"""Offline design review checks; only local reads and read-only Git commands."""
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
from collections import Counter

ROOT = Path(__file__).resolve().parents[2]
E = ROOT / 'RRM/evidence/M00-E1D'
V = ROOT / 'RRM/verification/reports/M00-E1D'
PARENT = '9d60ee46b3348fbaa1811a5b234f80446b62d5bd'
SUBJECT = 'RRM-F00 M00-E1D: review guard host and configuration alternatives'
BOUNDARY = 'DESIGN REVIEW ONLY — NO CONSTRUCTION, DEPLOYMENT, OR TIER B EXECUTION AUTHORIZED'


def must(condition, reason):
    if not condition:
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
        checks.append(dict(check=name, result='PASS', detail=detail))

    before = load(E / 'repository-before.json')
    must(before['head'] == PARENT and before['branch'] == 'ore_proj'
         and not before['status_porcelain'], 'Starting checkout mismatch')
    must(git('branch', '--show-current').decode().strip() == 'ore_proj', 'Branch changed')
    head = git('rev-parse', 'HEAD').decode().strip()
    committed = head != PARENT
    if committed:
        must(git('rev-parse', 'HEAD^').decode().strip() == PARENT, 'Unexpected parent')
        must(git('log', '-1', '--format=%s').decode().strip() == SUBJECT, 'Unexpected subject')
    must(load(E / 'starting-integrity.json')['result'] == 'PASS', 'Prior validator failed')
    passed('starting_checkout', 'Pinned clean E1C start; nine prior validation groups passed before writes')

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
                 'Source/input changed: ' + entry['path'])
    must({x['path'] for x in load(E / 'input-inventory.json')['files']} ==
         {str(p.relative_to(ROOT)) for p in (ROOT / 'RRM/RRM-C001-F00-EXT').rglob('*') if p.is_file()},
         'Controlled input set changed')
    # Prior status bytes are mutable only in this successor; sealed sums bind pinned Git bytes.
    pins = {'M00': 'e772605ccfb6c3bfe515e8e07961cdddc0831968',
            'M00-B': '93b8196e835effc3da34cac5677eab4d30c1378a',
            'M00-C': 'b4ac9467b073b6ae7927a47b939f46e9407cb6c0',
            'M00-D': '5b11d4d348db399e0e3ae638e318da9d7079ace6',
            'M00-E1': '31e33893ef2fa0f724862a980f6b5db0d62509d2',
            'M00-E1A': 'df8ebd18a6e16909cca461e18dfba882b53c8120',
            'M00-E1B': '4326dd079a2d80518983a5b793b9bab7f751a5db', 'M00-E1C': PARENT}
    count = 0
    for milestone, pin in pins.items():
        sums = ROOT / f'RRM/evidence/{milestone}/SHA256SUMS'
        if milestone == 'M00':
            # Original M00 used its own inventory, not this later manifest convention.
            must(not sums.exists(), 'Unexpected original M00 manifest')
            continue
        for line in sums.read_text().splitlines():
            digest, name = line.split('  ', 1)
            must(sha(git('show', pin + ':' + name)) == digest, 'Sealed mismatch: ' + name)
            count += 1
    passed('protected_history_and_input', f'{len(before["tracked_entries"])-1} protected entries; '
           f'{count} pinned sealed checksum entries; controlled package unchanged')

    functions = rows(V / 'M00E1D_GUARD_HOST_FUNCTIONS.csv')
    must(len(functions) == 12 and {x['function_id'] for x in functions} ==
         {f'F{i:02d}' for i in range(1, 13)}, 'Function coverage')
    for row in functions:
        must(all(row.values()) and row['implementation_status'] == 'PROPOSED_NOT_IMPLEMENTED',
             'Function assumptions/status incomplete')
        must((ROOT / row['evidence']).is_file(), 'Function evidence path')
    must(all(x['function_class'] == 'SAFETY_ENFORCEMENT' for x in functions[:5]),
         'Enforcement misclassified')
    must(functions[5]['function_class'] == 'DIAGNOSTIC_WITH_LOCAL_SAFETY_RULES', 'Guard boundary conflation')
    alternatives = (V / 'M00E1D_ARCHITECTURE_ALTERNATIVES.md').read_text()
    for text in ['A —', 'B —', 'C —', 'not an additional physical machine',
                 'monitoring', 'UNVERIFIED', 'material']:
        must(text.lower() in alternatives.lower(), 'Alternative missing: ' + text)
    trade = rows(V / 'M00E1D_ARCHITECTURE_TRADE_STUDY.csv')
    must(len(trade) == 8 and len({x['criterion'] for x in trade}) == 8, 'Trade criteria missing')
    for row in trade:
        must(all(row.values()) and row['basis'] == 'QUALITATIVE_ENGINEERING_JUDGMENT_NOT_APPROVED_WEIGHTS',
             'Trade assumptions/authority incomplete')
    passed('functions_alternatives_and_trade', '12 functions distinguish enforcement/diagnostics; A/B/C '
           'and eight qualitative criteria; conditional preference and sensitivity, no measured ranking')

    risks = rows(V / 'M00E1D_MEASUREMENT_RISK_REGISTER.csv')
    failures = rows(V / 'M00E1D_SAFETY_FAILURE_ANALYSIS.csv')
    must(len(risks) == 11 and len({x['risk_id'] for x in risks}) == 11, 'Measurement coverage')
    must(len(failures) == 12 and len({x['failure_id'] for x in failures}) == 12, 'Hazard coverage')
    for row in risks:
        must(all(row.values()) and row['status'] == 'OPEN_NOT_TESTED', 'Risk falsely closed')
    for row in failures:
        must(all(row.values()) and row['authorization_impact'].startswith('OPEN:'), 'Hazard falsely closed')
    spec = importlib.util.spec_from_file_location('historical', ROOT / 'RRM/scripts/m00e1a_analyze.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    replay = module.analyze(load(ROOT / 'RRM/evidence/M00-C/startup-source-records.json'))
    must({x['measurement_id']: x['seconds'] for x in replay['measurements']} ==
         {'L11': '19.276162179', 'L03': '74.228771713'}, 'Historical arithmetic changed')
    measurement = (V / 'M00E1D_MEASUREMENT_INTEGRITY.md').read_text()
    for token in ['19.276162179', '74.228771713', 'n=1', 'UNMEASURABLE', '24', 'not authorized']:
        must(token.lower() in measurement.lower(), 'Measurement limitation missing: ' + token)
    passed('measurement_safety_and_historical_replay', '11 open measurement risks/12 preliminary failure '
           'modes; independent Decimal replay of two n=1 intervals unchanged; future tests not performed')

    gaps = rows(E / 'M00E1D_OWNER_EVIDENCE_GAPS.csv')
    must(len(gaps) == 6 and {x['gap_id'] for x in gaps} == {f'OG{i:02d}' for i in range(1, 7)},
         'Owner gap coverage')
    must({item for row in gaps for item in row['e1c_request_ids'].split(';')} ==
         {f'RQ{i:02d}' for i in range(1, 15)}, 'Lost original owner request')
    for row in gaps:
        must(all(row.values()) and row['named_owner'] == 'UNVERIFIED'
             and row['response_status'] == 'AWAITING_OWNER'
             and row['delivery_status'] == 'NOT_SENT_PD_ROUTING_REQUIRED', 'Unsupported owner/delivery claim')
    config = (V / 'M00E1D_CONFIGURATION_CHANGE_IMPACT.md').read_text()
    for token in ['material', 'executor', 'equivalence', 'qualification', 'MAP-018', 'MAP-021', 'MAP-023']:
        must(token.lower() in config.lower(), 'Configuration impact missing: ' + token)
    passed('configuration_and_owner_feasibility', 'Material diagnostic context change and claim limits; '
           'six unanswered owner groups retain all14 E1C requests; no implementation feasibility claimed')

    original = rows(ROOT / 'RRM/evidence/M00-E1C/M00E1C_PREREQUISITE_IMPACT.csv')
    impacts = rows(E / 'M00E1D_PREREQUISITE_IMPACT.csv')
    must(len(impacts) == len(original) == 20, 'Prerequisite count')
    for old, new in zip(original, impacts):
        must(old['prerequisite_id'] == new['prerequisite_id']
             and old['original_requirement'] == new['original_requirement']
             and old['current_disposition'] == new['prior_e1c_disposition'] == new['current_disposition'],
             'Original prerequisite changed')
        must(all(new.values()) and new['closed_by_e1d'] == 'NO'
             and new['target_designation'] == 'UNVERIFIED' and new['execution_authorized'] == 'NO',
             'Prerequisite/authority overclaim')
    must(Counter(x['current_disposition'] for x in impacts) ==
         Counter(SATISFIED=3, UNSATISFIED=8, UNVERIFIED=9), 'Prerequisite disposition drift')
    status = (ROOT / 'RRM/PROJECT_STATUS.md').read_text()
    for token in [PARENT, 'M00-E1D', 'M00-E1E', 'America/New_York', 'BLOCKED BY EXTERNAL INPUT',
                  'PENDING PD REVIEW', 'WP01 NOT AUTHORIZED', '28 in-scope NOT_TESTED', '1 OUT_OF_SCOPE',
                  'zero whole closures', BOUNDARY]:
        must(token in status, 'Status missing: ' + token)
    decision = (V / 'M00E1D_PD_CONFIGURATION_DECISION.md').read_text()
    must(decision.count('- [ ] ') == 4 and '- [x]' not in decision.lower(), 'Decision preselected')
    for letter in 'ABCDEFGHI':
        must('## ' + letter + '.' in decision, 'Missing PD section ' + letter)
    for field in ['PD decision', 'Approved scope', 'Conditions', 'Date (America/New_York or stated UTC)',
                  'Authorization reference']:
        must(field + ': ____________________' in decision, 'Approval field populated or missing')
    for path in V.glob('*.md'):
        must(BOUNDARY in path.read_text(), 'Review boundary missing: ' + str(path))
    pending = load(E / 'pending-decision.json')
    for key in ['selected_option', 'approved_scope', 'conditions', 'date', 'authorization_reference']:
        must(pending[key] is None, 'Pending decision populated')
    must(not any(value for key, value in pending.items() if key.endswith('_authorized')),
         'Operational authority inferred')
    must(pending['external_owner_requests_sent'] == pending['whole_prerequisite_closures'] == 0
         and pending['target_designation'] == 'UNVERIFIED', 'Decision governance changed')
    audit = load(E / 'scope-audit.json')
    must(not audit['runtime_inspection_performed'] and not audit['construction_or_deployment_actions']
         and not audit['active_operations'] and audit['external_messages_sent'] == 0
         and not audit['credentials_read'] and not audit['push_performed'], 'Out-of-scope activity recorded')
    passed('prerequisites_decision_and_governance', '20 original IDs/requirements/statuses; zero closures; '
           'four unselected options/blank PD fields; G0 blocked, RA01 pending, WP01 unauthorized; no active operations recorded')

    paths = (E / 'M00E1D_CHANGED_FILES.txt').read_text().splitlines()
    must(paths == sorted(set(paths)), 'Invalid changed list')
    must(all(p.startswith('RRM/') and not p.startswith('RRM/RRM-C001-F00-EXT/') for p in paths),
         'Unauthorized path')
    must(set(paths) & set(before['tracked_entries']) == {'RRM/PROJECT_STATUS.md'}, 'Protected prior mutation')
    must(not any(p.endswith(('.sh', '.yaml', '.yml', '.service')) for p in paths), 'Deployment artifact added')
    actual_new = set(git('ls-files').decode().splitlines()) - set(before['tracked_entries'])
    actual_new |= set(git('ls-files', '--others', '--exclude-standard').decode().splitlines())
    must(actual_new == set(paths) - {'RRM/PROJECT_STATUS.md'}, 'Unexpected new/missing file')
    staged = set(git('diff', '--cached', '--name-only').decode().splitlines())
    must(not staged or staged == set(paths), 'Unexpected staged scope')
    if committed:
        must(not git('status', '--porcelain'), 'Postcommit dirty')
        must(set(git('diff-tree', '--no-commit-id', '--name-only', '-r', 'HEAD').decode().splitlines())
             == set(paths), 'Commit file set differs')
    hashed = set()
    for line in (E / 'SHA256SUMS').read_text().splitlines():
        digest, name = line.split('  ', 1)
        must(name not in hashed and sha((ROOT / name).read_bytes()) == digest, 'Package checksum: ' + name)
        hashed.add(name)
    must(hashed == set(paths) - {'RRM/evidence/M00-E1D/SHA256SUMS',
                               'RRM/evidence/M00-E1D/validation-results.json'}, 'Checksum scope mismatch')
    for path in paths:
        if path.endswith(('.md', '.py')):
            must(all(line == line.rstrip() for line in (ROOT / path).read_text().splitlines()),
                 'Trailing whitespace: ' + path)
    subprocess.run(['git', 'diff', '--check'], cwd=ROOT, check=True)
    subprocess.run(['git', 'diff', '--cached', '--check'], cwd=ROOT, check=True)
    passed('package_hashes_and_git_scope', f'{len(paths)} exact authorized paths; status sole prior mutation; '
           'no runtime/deployment artifacts; results excluded from recursive sums and bound by Git')
    return dict(milestone='M00-E1D', result='PASS', checks=checks,
                recommendation=pending['recommendation'], target_designation='UNVERIFIED',
                pd_decision=None, whole_prerequisite_closures=0, active_operations_performed=False,
                warnings=audit['warnings'], skipped_checks=audit['checks_skipped'],
                limitations=['Offline consistency/integrity review cannot prove platform compatibility, '
                             'physical/network isolation, authentic event coverage, overhead, recorder survival '
                             'or qualification. No new startups or current runtime observations.',
                             'Architecture preference is conditional engineering judgment; C and A can converge. '
                             'All actual owner/PD decisions remain outstanding. No push.'])


if __name__ == '__main__':
    print(json.dumps(validate(), indent=2))
