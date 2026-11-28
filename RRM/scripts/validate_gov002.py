#!/usr/bin/env python3
"""Check the local GOV-002 authority/channel release package; no deployment or runtime inspection."""
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
from collections import Counter

ROOT = Path(__file__).resolve().parents[2]
E = ROOT / 'RRM/evidence/GOV-002'
V = ROOT / 'RRM/governance/GOV-002'
PARENT = '381db1c4dcdd1bb71d047176e681bd18c72e342d'
SUBJECT = 'RRM-F00 GOV-002: assess organizational authority and request release'
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
    must(load(E / 'starting-integrity.json')['result'] == 'PASS', 'Prior GOV-001B validator not passing')
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
            'M00-E1C': '9d60ee46b3348fbaa1811a5b234f80446b62d5bd', 'M00-E1D': 'e6f0c81eff15538809b9745cef665de04ba4e796', 'M00-E1E': '0fa876fd8191be61a3e6724e1a3c93fe64510f01', 'GOV-001A': 'a72b8f412f2567e870903e2f4fec8082bc437e21', 'GOV-001B': PARENT}
    count = 0
    for milestone, pin in pins.items():
        for line in (ROOT / f'RRM/evidence/{milestone}/SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ', 1)
            must(sha(git('show', pin + ':' + name)) == digest, 'Sealed history mismatch: ' + name)
            count += 1
    passed('starting_and_protected_integrity', f'Clean pinned GOV-001B start; '
           f'{len(before["tracked_entries"])-1} protected entries, {count} pinned sealed checksums '
           f'and {len(inputs)} unchanged controlled input files')




    public = load(E / 'public-source-observations.json')
    local = load(E / 'local-record-review.json')
    must(len(local['records']) == 7 and local['no_runtime_or_controlplane_interrogation']
         and local['no_external_message'], 'Bounded acquisition scope')
    for item in local['records']:
        data = (ROOT / item['path']).read_bytes()
        must(sha(data) == item['sha256'] and len(data) == item['bytes']
             and data == git('show', PARENT + ':' + item['path']), 'Local source identity')
    for path, present in local['standard_codeowners_presence'].items():
        must((ROOT / path).exists() == present and not present, 'CODEOWNERS condition changed')
    must(public['repo']['owner'] == 'castacks' and public['repo']['id'] == '812053723'
         and public['repo']['url'] == 'https://github.com/castacks/AirStack', 'Repository observation identity')
    must(public['remote_readme']['blob_sha'] == '45835a804bae6a0ece455c40a3d642a3cffdaacc'
         and 'main/README.md' in public['remote_readme']['url'], 'Remote README provenance')
    observations = {item['source_id']: item for item in public['observations']}
    must(set(observations) == {'PUB01', 'PUB02', 'PUB03', 'PUB04'}, 'Official source coverage')
    must(observations['PUB03']['email'] == observations['PUB04']['email'] == 'basti@andrew.cmu.edu'
         and observations['PUB03']['contact_name'] == observations['PUB04']['contact_name'] ==
         'Sebastian Scherer', 'Contact corroboration differs')
    from urllib.parse import urlparse
    must(urlparse(observations['PUB03']['url']).hostname == 'www.ri.cmu.edu'
         and urlparse(observations['PUB04']['url']).hostname == 'engineering.cmu.edu',
         'Contact sources not official institutional hosts')
    readme = (ROOT / 'README.md').read_text()
    must('Sebastian Scherer' in readme and 'contact the team' in readme and 'https://theairlab.org' in readme,
         'Project-to-organizational-route chain missing')
    sources = rows(V / 'GOV002_AUTHORITY_SOURCE_REGISTER.csv')
    must(len(sources) == 13 and len({x['source_id'] for x in sources}) == 13, 'Source registry count')
    must(all(all(x.values()) for x in sources), 'Source revision/finding/limitation absent')
    passed('bounded_discovery_and_public_provenance', 'Seven pinned local sources,13 registered sources; '
           'GitHub connector repo/README identities; four primary public pages, two official CMU contact '
           'records corroborate one professional routing address; no live service/mailbox test')

    recipients = rows(V / 'GOV002_RECIPIENT_CHANNEL_ASSESSMENT.csv')
    must(len(recipients) == 4 and len({x['candidate_id'] for x in recipients}) == 4, 'Candidate scope')
    classifications = {'VERIFIED AUTHORITY', 'VERIFIED ROUTING CHANNEL', 'CANDIDATE — UNVERIFIED', 'UNSUITABLE'}
    for row in recipients:
        must(all(row.values()) and row['classification'] in classifications and row['contact_made'] == 'NO',
             'Candidate classification/contact unsupported')
        for key in ['authority_to_appoint_owners', 'authority_to_approve_platform_change',
                    'authority_to_review_safety_isolation']:
            must(row[key] == 'NOT ESTABLISHED', 'Routing identity falsely promoted to decision authority')
        must(set(row['source_ids'].split(';')) <= {x['source_id'] for x in sources}, 'Candidate source trace')
    routes = [x for x in recipients if x['classification'] == 'VERIFIED ROUTING CHANNEL']
    must(len(routes) == 1 and routes[0]['candidate_id'] == 'RC01'
         and routes[0]['channel'] == observations['PUB03']['email']
         and 'R1' in routes[0]['qualified_scope'], 'Qualified route not scope-limited/provenance-bound')
    must(not any(x['classification'] == 'VERIFIED AUTHORITY' for x in recipients), 'Authority invented')
    release = load(E / 'release-state.json')
    must(release['outcome'] == 'A' and release['qualified_candidate_id'] == 'RC01'
         and release['verified_routing_channels'] == 1
         and release['r1_ready_to_request_pd_delivery_authorization'] is True
         and release['r2_platform_evidence_request_ready'] is False, 'Release outcome conflation')
    for key in ['selected_release_option', 'pd_delivery_authorization_reference', 'authorized_sender']:
        must(release[key] is None, 'PD delivery field invented')
    for key in ['external_communication_authorized', 'owner_appointed', 'construction_authorized',
                'deployment_authorized', 'active_verification_authorized', 'tier_b_authorized', 'wp01_authorized']:
        must(release[key] is False, 'Readiness treated as authorization')
    must(release['verified_organizational_approval_authorities'] == release['requests_sent'] ==
         release['technical_prerequisite_changes'] == 0 and release['target_designation'] == 'UNVERIFIED',
         'Owner/request/technical closure overclaim')
    passed('recipient_qualification_and_release_outcome', 'RC01 authentic published professional routing '
           'contact only; zero verified applicable decision authorities; Outcome A for seeking separate '
           'R1 approval, R2 not ready; sender and PD fields null; no contact or authority inferred')

    sheet = (V / 'GOV002_PD_RELEASE_DECISION.md').read_text()
    must(sheet.count('- [ ] ') == 4 and '- [x]' not in sheet.lower(), 'Release option preselected')
    for token in ['R1', 'R2', 'R3', 'R4', 'Preconditions', 'evidence', 'scope', 'Risks', 'limitations',
                  'GOV-002 itself authorizes neither R1 nor R2']:
        must(token.lower() in sheet.lower(), 'PD decision field/limit missing: ' + token)
    assessment = (V / 'GOV002_REQUEST_RELEASE_ASSESSMENT.md').read_text()
    strategy = (V / 'GOV002_MINIMUM_CONTACT_STRATEGY.md').read_text()
    for token in ['narrow', 'C1', 'C2', 'C3', 'C4', 'Sensitivity', 'Expected', 'Stop', 'PREPARED — NOT SENT']:
        must(token.lower() in assessment.lower(), 'Communication scope missing: ' + token)
    must('Sender name/project affiliation: [PD to supply and authorize]' in assessment,
         'Sender affiliation invented')
    for token in ['no attachments', 'separate PD R2', 'intake', 'No external communication authorized']:
        must(token.lower() in strategy.lower(), 'Minimum-contact guard missing: ' + token)
    passed('minimum_contact_and_unselected_pd_sheet', 'Narrow C1 ownership/referral-only local text; '
           'detailed A–F held and later C2/C3/C4 separated by authority/handling; four PD options '
           'unselected; no external draft/message or sender invented')

    dispositions = load(ROOT / 'RRM/evidence/GOV-001A/dispositions.json')
    must([x['disposition'] for x in dispositions['decisions']] ==
         ['CONDITIONALLY APPROVED FOR FURTHER OFFLINE DESIGN.', 'DEFERRED.',
          'APPROVED AS PROJECT EVIDENCE POLICY.', 'PENDING VERIFICATION.',
          'APPROVED — PASSIVE AND OFFLINE WORK ONLY.'], 'Project decision continuity changed')
    prior = rows(ROOT / 'RRM/evidence/GOV-001B/GOV001B_PREREQUISITE_DELTA.csv')
    continuity = rows(E / 'GOV002_PREREQUISITE_CONTINUITY.csv')
    must(len(prior) == len(continuity) == 20, 'Original prerequisite count')
    for a, b in zip(prior, continuity):
        must(a['prerequisite_id'] == b['prerequisite_id'] and a['original_requirement'] == b['original_requirement']
             and a['current_disposition'] == b['prior_disposition'] == b['current_disposition'],
             'Original prerequisite identity/status altered')
        must(all(b.values()) and b['disposition_changed'] == b['closed_by_gov002'] ==
             b['execution_authorized'] == 'NO' and b['target_designation'] == 'UNVERIFIED',
             'Contact discovery falsely closes prerequisite')
    must(Counter(x['current_disposition'] for x in continuity) ==
         Counter(SATISFIED=3, UNSATISFIED=8, UNVERIFIED=9), 'Technical counts changed')
    status = (ROOT / 'RRM/PROJECT_STATUS.md').read_text()
    for token in [PARENT, 'GOV-002', 'Outcome A', HOLD, 'R1 delivery not authorized', 'R2', 'DEFERRED',
                  'PENDING VERIFICATION', 'PENDING PD REVIEW', 'BLOCKED BY EXTERNAL INPUT',
                  'WP01 NOT AUTHORIZED', '28 in-scope NOT_TESTED', '1 OUT_OF_SCOPE', 'zero whole closures']:
        must(token in status, 'Current governance missing: ' + token)
    passed('technical_prerequisites_and_g0_continuity', 'All20 original identities/statuses unchanged '
           '3 satisfied/8 unsatisfied/9 unverified; no contact-to-closure inference; D1–D5/G0/RA01/WP01/ '
           'Tier B controls and implementation hold preserved')

    index = rows(E / 'GOV002_EVIDENCE_INDEX.csv')
    must(len(index) == 7 and len({x['evidence_id'] for x in index}) == 7, 'Evidence index scope')
    for row in index:
        data = (ROOT / row['path']).read_bytes()
        must(all(row.values()) and len(data) == int(row['bytes']) and sha(data) == row['sha256'],
             'Evidence identity/limit mismatch')
    audit = load(E / 'scope-audit.json')
    must(not audit['runtime_inspection_performed'] and not audit['construction_or_deployment_actions']
         and not audit['active_operations'] and audit['external_messages_sent'] ==
         audit['external_email_drafts_created'] == audit['owners_appointed'] ==
         audit['organizational_approval_authorities_verified'] == audit['received_owner_responses'] ==
         audit['technical_prerequisite_closures'] == 0 and not audit['credentials_read']
         and not audit['private_account_or_mailbox_access'] and not audit['push_performed'],
         'Prohibited operation/contact/authority recorded')
    passed('evidence_provenance_and_no_external_communications', 'Seven byte-bound index records '
           'distinguish public source observations/inherited project authority and technical baseline; '
           'no owner response, private account access, email draft, appointment or runtime action')

    required = ['GOV002_AUTHORITY_DISCOVERY.md', 'GOV002_AUTHORITY_SOURCE_REGISTER.csv',
                'GOV002_RECIPIENT_CHANNEL_ASSESSMENT.csv', 'GOV002_MINIMUM_CONTACT_STRATEGY.md',
                'GOV002_REQUEST_RELEASE_ASSESSMENT.md', 'GOV002_PD_RELEASE_DECISION.md',
                'GOV002_EXTERNAL_HOLD_DISPOSITION.md']
    must(all((V / name).is_file() for name in required), 'Required governance report missing')
    must((ROOT / 'RRM/handoffs/GOV002_HANDOFF.md').is_file(), 'Handoff missing')
    paths = (E / 'GOV002_CHANGED_FILES.txt').read_text().splitlines()
    must(paths == sorted(set(paths)), 'Changed-file list invalid')
    must(all(x.startswith('RRM/') and not x.startswith('RRM/RRM-C001-F00-EXT/') for x in paths),
         'Unauthorized path')
    must(set(paths) & set(before['tracked_entries']) == {'RRM/PROJECT_STATUS.md'}, 'Old protected mutation')
    must(not any(x.endswith(('.sh', '.yaml', '.yml', '.service')) for x in paths), 'Runtime automation')
    actual = set(git('ls-files').decode().splitlines()) - set(before['tracked_entries'])
    actual |= set(git('ls-files', '--others', '--exclude-standard').decode().splitlines())
    must(actual == set(paths) - {'RRM/PROJECT_STATUS.md'}, 'Unexpected file set')
    staged = set(git('diff', '--cached', '--name-only').decode().splitlines())
    must(not staged or staged == set(paths), 'Staged path scope mismatch')
    if committed:
        must(not git('status', '--porcelain'), 'Postcommit dirty')
        must(set(git('diff-tree', '--no-commit-id', '--name-only', '-r', 'HEAD').decode().splitlines())
             == set(paths), 'Commit path scope mismatch')
    hashed = set()
    for line in (E / 'SHA256SUMS').read_text().splitlines():
        digest, path = line.split('  ', 1)
        must(path not in hashed and sha((ROOT / path).read_bytes()) == digest, 'Package checksum: ' + path)
        hashed.add(path)
    must(hashed == set(paths) - {'RRM/evidence/GOV-002/SHA256SUMS',
                               'RRM/evidence/GOV-002/validation-results.json'}, 'Checksum scope mismatch')
    for name in paths:
        data = (ROOT / name).read_bytes()
        must(b'\r\n' not in data and all(line == line.rstrip() for line in data.decode().splitlines()),
             'Whitespace: ' + name)
    subprocess.run(['git', 'diff', '--check'], cwd=ROOT, check=True)
    subprocess.run(['git', 'diff', '--cached', '--check'], cwd=ROOT, check=True)
    passed('package_hashes_and_repository_scope', f'{len(paths)} exact RRM-only paths; PROJECT_STATUS '
           'sole prior mutation; sums cover all except recursive sums/results, bound by final Git; '
           'no deployment automation or historical edits')
    return dict(milestone='GOV-002', result='PASS', checks=checks, failures=[], outcome='A',
                verified_organizational_approval_authorities=0, verified_public_professional_routing_channels=1,
                qualified_candidate='RC01', ownership_confirmation_ready_for_separate_r1_decision=True,
                platform_evidence_request_ready=False, pd_release_selection=None,
                external_communication_authorized=False, requests_sent=0, technical_prerequisite_changes=0,
                checksum_verification='PASS', repository_scope_verification='PASS',
                target_designation='UNVERIFIED', implementation_authorized=False, hold=HOLD,
                warnings=audit['warnings'], skipped_checks=audit['skipped_checks'],
                limitations=['Offline checks bind captured source facts and document/decision scope; '
                             'they cannot authenticate organizational approval powers, prove email delivery '
                             'or safety, or establish actual routing success. Public pages may change.',
                             'Outcome A is a scoped engineering judgment from published project/lab/institution '
                             'contact records. Exact sender and separate PD R1 authority remain absent; R2 held.',
                             'No new behavioral tests needed for documentation; prior validator ran before writes. '
                             'No contact, runtime/active operations or push; stop GOV-002.'])


if __name__ == '__main__':
    print(json.dumps(validate(), indent=2))
