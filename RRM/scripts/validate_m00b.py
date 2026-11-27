#!/usr/bin/env python3
"""Validate M00-B evidence/traceability and protected boundaries, without runtime writes."""
import ast
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
WORK = ROOT / 'RRM'
EVIDENCE = WORK / 'evidence/M00-B'
MATRICES = WORK / 'verification/matrices/M00-B'
checks = []


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).rstrip('\n')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path):
    return json.loads(path.read_text())


def read_csv(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def check(name, passed, detail):
    checks.append({'check': name, 'passed': bool(passed), 'detail': detail})


def identity(path):
    if path.is_symlink():
        return {'type': 'symlink', 'target': os.readlink(path)}
    if path.is_file():
        return {'type': 'file', 'sha256': sha(path), 'mode': stat.S_IMODE(path.stat().st_mode)}
    if path.is_dir():
        return {'type': 'gitlink', 'head': subprocess.check_output(
            ['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()}
    return {'type': 'missing'}


def main():
    before = read_json(EVIDENCE / 'repository-before.json')
    check('authorized_repository_and_branch', git('remote', 'get-url', 'origin') ==
          before['repository'] == 'https://github.com/castacks/AirStack.git' and
          git('branch', '--show-current') == before['branch'] == 'ore_proj',
          'Origin/branch checked; no branch switch')
    head = git('rev-parse', 'HEAD')
    check('authorized_start_and_history', before['head'] ==
          'e772605ccfb6c3bfe515e8e07961cdddc0831968' and before['status_porcelain'] == '' and
          (head == before['head'] or git('rev-parse', 'HEAD^') == before['head']),
          'Clean required starting commit or one direct M00-B successor')
    drift = [p for p, expected in before['tracked_entries'].items()
             if identity(ROOT / p) != expected]
    check('all_preexisting_tracked_files_preserved', not drift,
          {'entries': len(before['tracked_entries']), 'changed': drift})
    initial = read_json(WORK / 'evidence/M00/repository-before.json')
    protected = {str(p.relative_to(ROOT)): identity(p)
                 for p in (ROOT / 'RRM-transfer').rglob('*')
                 if p.is_file() or p.is_symlink()}
    check('complete_reference_tree_preserved', protected == initial['protected_tree'],
          {'entries': len(protected)})
    outside = [line for line in git('status', '--porcelain=v1', '--untracked-files=all').splitlines()
               if not line[3:].startswith('RRM/')]
    check('outside_workspace_status', not outside and
          not git('diff', before['head'], '--', '.', ':(exclude)RRM'), {'outside': outside})
    package = read_json(WORK / 'evidence/M00/controlled-package-manifest.json')
    base = Path(package['supplied_path'])
    actual = {str(p.relative_to(base)): {'path': str(p.relative_to(base)),
              'bytes': p.stat().st_size, 'sha256': sha(p)}
              for p in base.rglob('*') if p.is_file()}
    check('controlled_input_unchanged', actual == {x['path']: x for x in package['files']},
          {'files': len(actual)})
    for directory, manifest in [(base, 'SHA256SUMS'),
                               (base / 'release_staging/F00X-REL-001', 'RELEASE_MANIFEST.sha256')]:
        result = subprocess.run(['sha256sum', '-c', manifest], cwd=directory,
                                text=True, capture_output=True)
        check('package_' + manifest, result.returncode == 0,
              {'ok_entries': result.stdout.count(': OK'), 'stderr': result.stderr})
    source = read_json(EVIDENCE / 'source-manifest.json')['sources']
    changed = [x['path'] for x in source if sha(ROOT / x['path']) != x['sha256']]
    check('investigated_source_integrity', not changed, {'changed': changed})
    original = read_csv(WORK / 'verification/matrices/M00_REQUIREMENTS_RECONCILIATION.csv')
    impacts = read_csv(MATRICES / 'M00B_REQUIREMENTS_IMPACT.csv')
    check('all_29_requirements_preserved', len(original) == len(impacts) == 29 and all(
          a['mapping_id'] == b['mapping_id'] and a['requirement_label'] == b['requirement_label']
          and a['kpi_id'] == b['kpi_id'] and a['ivv_id'] == b['ivv_id']
          and a['source_acceptance_exact'] == b['source_acceptance_unchanged']
          and hashlib.sha256(a['source_acceptance_exact'].encode()).hexdigest() ==
          b['source_acceptance_sha256'] for a, b in zip(original, impacts)),
          'MAP/KPI/IVV IDs and every exact D2 criterion retained')
    check('no_false_requirement_acceptance',
          sum(x['current_verdict'] == 'NOT_TESTED' for x in impacts) == 28 and
          sum(x['current_verdict'] == 'OUT_OF_SCOPE' for x in impacts) == 1 and
          all(x['characterization_substitution_allowed'] == 'NO' for x in impacts),
          '28 untested in-scope rows; one excluded scope trace; no diagnostic-as-pass')
    old_ids = {x['blocker_id'] for x in read_csv(WORK / 'verification/matrices/M00_BLOCKERS.csv')}
    blockers = read_csv(MATRICES / 'M00B_BLOCKER_REGISTER.csv')
    check('original_blocker_continuity', len(blockers) == len(old_ids) == 10 and
          old_ids == {x['blocker_id'] for x in blockers} and
          all(x['mandatory_blocker_closed'] == 'NO' and x['source_reference'] and
              x['evidence_location'] and x['closure_condition'] and
              x['responsible_decision_authority'] for x in blockers),
          'Original ten IDs, explicit authorities/evidence/closure; none falsely closed')
    configuration = read_csv(MATRICES / 'M00B_CONFIGURATION_EQUIVALENCE.csv')
    counts = {k: sum(x['classification'] == k for x in configuration) for k in
              ['VERIFIED_EQUIVALENT', 'NON_EQUIVALENT', 'UNRESOLVED', 'NOT_APPLICABLE']}
    check('configuration_classification', counts == {'VERIFIED_EQUIVALENT': 6,
          'NON_EQUIVALENT': 6, 'UNRESOLVED': 8, 'NOT_APPLICABLE': 2} and len(configuration) == 22,
          counts)
    records = read_csv(MATRICES / 'M00B_FROZEN_RECORDS.csv')
    check('unavailable_records_not_reconstructed', len(records) == 20 and
          all(x['authoritative_copy_located'] == 'NO' and x['required_identity'] and
              x['blocked_activity'] for x in records), '20 named records/sets retain access limits')
    live = read_json(EVIDENCE / 'live-readiness.json')
    ready = live['channels']['/robot_1/interface/actuation_ready']['samples']
    status = live['channels']['/robot_1/interface/actuation_status']['samples']
    parameter = live['queries'][0]['values'][0]
    analysis = read_json(EVIDENCE / 'readiness-analysis.json')
    check('readiness_diagnosis_consistency', len(ready) == len(status) == 60 and
          all(x['data'] is False for x in ready) and
          all(x['data'] == 'STARTUP_DEADLINE' for x in status) and
          parameter['name'] == 'thrust_scaling' and parameter['type'] == 3 and
          parameter['double'] == 1.0 and analysis['same_kernel_boot_id_verified'] and
          abs(analysis['heartbeat_after_deadline_s'] - 19.2761622) < 0.00001,
          'Actual guard rejection and correct child readback retained; timing characterized only')
    attempts = read_json(EVIDENCE / 'diagnostic-attempts.json')['attempts']
    mapping = read_json(EVIDENCE / 'parameters-and-map.json')
    check('diagnostic_failure_and_corrected_attempt_retained', attempts[0]['exit_code'] == 1 and
          mapping['attempt_id'] == 'MAP-PARAM-02' and all(
              x['count'] > 0 and x['last_points'] == 1503 for x in mapping['map_channels'].values()),
          'Failed dual-type collection retained; corrected map summaries do not qualify map accuracy')
    after = read_json(EVIDENCE / 'runtime-after.json')
    discovery = read_json(EVIDENCE / 'baseline-discovery.json')['results'][1]['stdout']
    initial_processes = []
    for filename in ['robot-identity.json', 'isaac-identity.json']:
        doc = read_json(EVIDENCE / filename)
        initial_processes.extend(dict(container=doc['container'], **p) for p in doc['result']['processes'])
    check('no_container_or_key_process_restart_observed', all(
          x['image'] in discovery and x['started'] in discovery for x in after['containers']) and all(
          any(a['container'] == b['container'] and a['pid'] == b['pid'] and
              a['proc_start_ticks'] == b['proc_start_ticks'] for b in initial_processes)
          for a in after['processes']) and after['plugin_sha256_after'] ==
          '5173ece556bed1d09911c17c690f3702d42d4bcb1b95c9322a8d8d01e45e667b',
          'Container starts/images, interface/PX4/Isaac process identities and plugin file unchanged')
    broken = []
    new_paths = (EVIDENCE / 'M00B_CHANGED_FILES.txt').read_text().splitlines()
    for name in new_paths:
        path = ROOT / name
        if path.suffix == '.md':
            for link in re.findall(r'\[[^\]]*\]\(([^)]+)\)', path.read_text()):
                if '://' not in link and not link.startswith('#') and not (
                        path.parent / link.split('#')[0]).exists():
                    broken.append({'file': name, 'link': link})
        if path.suffix == '.py':
            ast.parse(path.read_text(), filename=name)
    check('document_links_and_python_syntax', not broken, {'broken_links': broken})
    manifest = read_json(EVIDENCE / 'M00B_EVIDENCE_MANIFEST.json')
    failures = [x['path'] for x in manifest['artifacts'] if
                sha(ROOT / x['path']) != x['sha256'] or
                (ROOT / x['path']).stat().st_size != x['bytes']]
    covered = {x['path'] for x in manifest['artifacts']}
    excluded = {'RRM/evidence/M00-B/M00B_EVIDENCE_MANIFEST.json',
                'RRM/evidence/M00-B/SHA256SUMS', 'RRM/evidence/M00-B/validation-results.json'}
    check('artifact_manifest_integrity_and_coverage', not failures and
          covered == set(new_paths) - excluded, {'artifacts': len(covered), 'failed': failures})
    result = subprocess.run(['sha256sum', '-c', 'RRM/evidence/M00-B/SHA256SUMS'],
                            cwd=ROOT, text=True, capture_output=True)
    check('detached_checksums', result.returncode == 0,
          {'ok_entries': result.stdout.count(': OK'), 'stderr': result.stderr})
    staged = git('diff', '--cached', '--name-only').splitlines()
    check('explicit_staged_scope', not staged or set(staged) == set(new_paths),
          {'staged_files': len(staged)})
    result = subprocess.run(['git', '-C', str(ROOT), 'diff', '--cached', '--check'],
                            text=True, capture_output=True)
    check('staged_whitespace', result.returncode == 0, result.stdout + result.stderr)
    if head != before['head']:
        committed = git('diff-tree', '--no-commit-id', '--name-only', '-r', head).splitlines()
        check('committed_scope', set(committed) == set(new_paths) and
              all(p.startswith('RRM/') for p in committed),
              {'commit': head, 'files': len(committed)})
    passed = all(x['passed'] for x in checks)
    print(json.dumps({'all_passed': passed, 'scope': 'M00-B integrity/reconciliation checks; no functional requirement or independent IVV qualification',
                      'checks': checks}, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print(json.dumps({'all_passed': False, 'error': str(error), 'checks': checks}, indent=2))
        sys.exit(1)
