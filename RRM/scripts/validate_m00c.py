#!/usr/bin/env python3
"""Offline milestone integrity, traceability, clock replay and project-control checks."""
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
EVIDENCE = WORK / 'evidence/M00-C'
REPORTS = WORK / 'verification/reports/M00-C'
checks = []


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).rstrip('\n')


def doc(path):
    return json.loads(path.read_text())


def rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


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
    before = doc(EVIDENCE / 'repository-before.json')
    head = git('rev-parse', 'HEAD')
    check('authorized_repository_branch_start', git('remote', 'get-url', 'origin') ==
          before['repository'] == 'https://github.com/castacks/AirStack.git' and
          git('branch', '--show-current') == before['branch'] == 'ore_proj' and
          before['head'] == '93b8196e835effc3da34cac5677eab4d30c1378a' and
          before['status_porcelain'] == '', 'Required clean M00-B starting baseline')
    check('history_preserved', head == before['head'] or git('rev-parse', 'HEAD^') == before['head'],
          'At required parent or one direct local M00-C successor')
    changed = [p for p, value in before['tracked_entries'].items() if identity(ROOT / p) != value]
    check('all_preexisting_tracked_files_unchanged', not changed,
          {'tracked_entries': len(before['tracked_entries']), 'changed': changed})
    initial = doc(WORK / 'evidence/M00/repository-before.json')
    protected = {str(p.relative_to(ROOT)): identity(p)
                 for p in (ROOT / 'RRM-transfer').rglob('*') if p.is_file() or p.is_symlink()}
    check('complete_reference_tree_unchanged', protected == initial['protected_tree'],
          {'entries': len(protected), 'scope_limit': 'No submodule internal or ignored runtime tree hash claim'})
    outside = [s for s in git('status', '--porcelain=v1', '--untracked-files=all').splitlines()
               if not s[3:].startswith('RRM/')]
    check('outside_workspace_scope', not outside and not git(
          'diff', before['head'], '--', '.', ':(exclude)RRM'), {'outside': outside})
    package = doc(WORK / 'evidence/M00/controlled-package-manifest.json')
    base = Path(package['supplied_path'])
    actual = {str(p.relative_to(base)): {'path': str(p.relative_to(base)),
              'bytes': p.stat().st_size, 'sha256': sha(p)} for p in base.rglob('*') if p.is_file()}
    check('controlled_input_integrity', actual == {p['path']: p for p in package['files']},
          {'files': len(actual)})
    for folder, filename in [(base, 'SHA256SUMS'),
                             (base / 'release_staging/F00X-REL-001', 'RELEASE_MANIFEST.sha256')]:
        p = subprocess.run(['sha256sum', '-c', filename], cwd=folder,
                           text=True, capture_output=True)
        check('input_' + filename, p.returncode == 0,
              {'entries_ok': p.stdout.count(': OK'), 'stderr': p.stderr})
    sources = doc(EVIDENCE / 'source-manifest.json')['sources']
    changed = [s['path'] for s in sources if sha(ROOT / s['path']) != s['sha256']]
    check('source_evidence_hashes', not changed, {'sources': len(sources), 'changed': changed})
    p = subprocess.run([sys.executable, '-B', str(WORK / 'scripts/m00c_analyze_startup.py'), '--check'],
                       cwd=ROOT, text=True, capture_output=True)
    check('timeline_replay_exact', p.returncode == 0, {'exit_code': p.returncode, 'stderr': p.stderr})
    analysis = doc(EVIDENCE / 'timing-analysis.json')
    check('timing_domain_and_precision', analysis['heartbeat_after_deadline_s_exact_decimal'] ==
          '19.276162179' and analysis['px4_process_after_interface_s'] == '21.62' and
          analysis['prior_excerpt_lines_verified_in_original_logs'] and
          analysis['deadline_origin_numeric_observed'] is False,
          'Decimal replay of original logs; no numeric origin fabricated; kernel/producer/sink clocks separate')
    regression = (EVIDENCE / 'timing-regression-tests.txt').read_text()
    check('timing_regression_results', 'Ran 10 tests' in regression and regression.rstrip().endswith('OK'),
          'Ten offline cases: mixed clocks, boot IDs, wrong units/rates, missing/reversed/duplicate events and exact arithmetic')
    prior = rows(WORK / 'verification/matrices/M00-B/M00B_BLOCKER_REGISTER.csv')
    blockers = rows(EVIDENCE / 'M00C_BLOCKER_DISPOSITION.csv')
    check('all_original_blockers_dispositioned', len(blockers) == 10 and
          {r['blocker_id'] for r in prior} == {r['blocker_id'] for r in blockers} and
          all(r['closed'] == 'NO' and r['current_disposition'] and r['required_action'] and
              r['responsible_authority'] and r['original_closure_condition'] for r in blockers),
          'Ten original IDs, explicit actions/authorities/closure; no proposed remedy marked closed')
    old = rows(WORK / 'verification/matrices/M00-B/M00B_REQUIREMENTS_IMPACT.csv')
    impacts = rows(EVIDENCE / 'M00C_REQUIREMENTS_IMPACT.csv')
    check('no_acceptance_changes_or_false_passes', len(old) == len(impacts) == 29 and all(
          all(a[key] == b[key] for key in a) for a, b in zip(old, impacts)) and
          sum(r['current_verdict'] == 'NOT_TESTED' for r in impacts) == 28 and
          sum(r['current_verdict'] == 'OUT_OF_SCOPE' for r in impacts) == 1,
          'Every prior source acceptance/hash/MAP/KPI/IVV field preserved; diagnostics add no requirement verdict')
    actions = rows(EVIDENCE / 'M00C_CORRECTIVE_ACTIONS.csv')
    check('all_candidates_have_verification_and_approval', len(actions) == 5 and all(
          r['verification_method'] and r['affected_requirements_and_blockers'] and
          r['required_source_changes'] and r['safety_implications'] and r['configuration_impact'] and
          r['regression_risks_and_controls'] and r['baseline_change_approval'] for r in actions),
          {'candidate_count': len(actions), 'implemented': False})
    hypotheses = rows(REPORTS / 'M00C_HYPOTHESIS_MATRIX.csv')
    ids = {r['action_id'] for r in actions}
    check('hypotheses_trace_to_actions', len(hypotheses) == 10 and all(
          set(h['required_corrective_actions'].split(';')) <= ids and
          all(h[k] for k in ['supporting_evidence', 'contradictory_or_limiting_evidence',
                            'missing_measurements', 'diagnostic_method', 'root_cause_confidence'])
          for h in hypotheses), 'Ten competing hypotheses retain contrary evidence and missing measurements')
    catalog = rows(EVIDENCE / 'reference-catalog.csv')
    external = {r['location'] for r in catalog if r['kind'] == 'EXTERNAL_DEPENDENCY'}
    timeline = rows(REPORTS / 'M00C_STARTUP_TIMELINE.csv')
    missing = []
    for value in [r['source_reference'] for r in timeline] + [r['location'] for r in catalog]:
        if value.startswith('EXTERNAL:'):
            if value not in external:
                missing.append(value)
        else:
            path = re.split(r'[#:]| line', value)[0]
            if not (ROOT / path).is_file():
                missing.append(value)
    for r in hypotheses + blockers:
        for value in r['evidence_references'].split(';'):
            if not (ROOT / value).is_file():
                missing.append(value)
    check('evidence_references_resolve_or_are_declared_external', not missing,
          {'missing_references': missing, 'declared_external_dependencies': len(external)})
    audit = doc(EVIDENCE / 'diagnostic-scope-audit.json')
    passive = (WORK / 'scripts/m00b_passive_readiness.py').read_text()
    passive_ast = ast.parse(passive)
    forbidden = {'create_publisher', 'ActionClient', 'set_parameters', 'send_goal_async'}
    calls = {getattr(n.func, 'attr', getattr(n.func, 'id', ''))
             for n in ast.walk(passive_ast) if isinstance(n, ast.Call)}
    client_types = [ast.unparse(n.args[0]) for n in ast.walk(passive_ast)
                    if isinstance(n, ast.Call) and getattr(n.func, 'attr', '') == 'create_client']
    check('authorized_diagnostic_scope_audit', audit['active_operations_executed'] == [] and
          not (calls & forbidden) and client_types == ['GetParameters'],
          'Audited passive collector has subscriptions/GetParameters only; action inventory asserts no active tests; not an exhaustive external-runtime monitor')
    collected = doc(EVIDENCE / 'startup-source-records.json')
    after = doc(EVIDENCE / 'runtime-after.json')
    previous = [json.loads(s) for s in collected['commands'][0]['stdout'].splitlines()]
    proc_old = collected['original_ros_logs']['processes'] + collected['isaac_processes']['processes']
    check('runtime_key_identities_preserved', previous == after['containers'] and all(
          any(a['pid'] == b['pid'] and a['start_ticks'] == b['start_ticks'] and a['exe'] == b['exe']
              for b in proc_old) for a in after['processes']) and len(after['processes']) == 4,
          'Container start/image/restart counts and interface/MAVROS/Isaac/PX4 process identities unchanged')
    roadmap = (WORK / 'ROADMAP_MANDATE.md').read_text()
    status = (WORK / 'PROJECT_STATUS.md').read_text()
    g0 = (REPORTS / 'M00C_G0_REASSESSMENT.md').read_text()
    check('roadmap_status_consistency', all(x in roadmap for x in ['WP0' + str(i) for i in range(9)]) and
          all('G' + str(i) in roadmap for i in range(9)) and
          all('M00-C' in t and 'WP01' in t and 'NOT AUTHORIZED' in t for t in [roadmap, status, g0]) and
          all('RA01' in t and 'PROPOSED' in t for t in [roadmap, status]) and
          'PROVISIONAL' in roadmap and before['head'] in status and
          'BLOCKED BY EXTERNAL INPUT' in status and all(b['blocker_id'] in status for b in blockers),
          'WP00–08/G0–8 present; detailed allocation explicitly proposed; original IDs/history/coverage/gate/authorization agree; known full SHA is an as-of snapshot')
    files = (EVIDENCE / 'M00C_CHANGED_FILES.txt').read_text().splitlines()
    broken = []
    for name in files:
        path = ROOT / name
        if path.suffix == '.py':
            ast.parse(path.read_text())
        if path.suffix == '.md':
            for link in re.findall(r'\[[^\]]*\]\(([^)]+)\)', path.read_text()):
                if '://' not in link and not link.startswith('#') and not (
                        path.parent / link.split('#')[0]).exists():
                    broken.append({'file': name, 'link': link})
    check('new_document_links_and_script_syntax', not broken, {'broken': broken})
    manifest = doc(EVIDENCE / 'M00C_EVIDENCE_MANIFEST.json')
    bad = [x['path'] for x in manifest['artifacts'] if sha(ROOT / x['path']) != x['sha256'] or
           (ROOT / x['path']).stat().st_size != x['bytes']]
    excluded = set(manifest['excluded_tracked_paths'])
    check('evidence_manifest_coverage_and_hashes', not bad and
          set(files) - excluded == {x['path'] for x in manifest['artifacts']},
          {'hashed_artifacts': len(manifest['artifacts']), 'bad': bad})
    p = subprocess.run(['sha256sum', '-c', 'RRM/evidence/M00-C/SHA256SUMS'],
                       cwd=ROOT, text=True, capture_output=True)
    check('detached_sha256_manifest', p.returncode == 0,
          {'entries_ok': p.stdout.count(': OK'), 'stderr': p.stderr})
    staged = git('diff', '--cached', '--name-only').splitlines()
    check('staged_scope', (not staged or set(staged) == set(files)) and
          all(x.startswith('RRM/') and not x.startswith('RRM/RRM-C001-F00-EXT/') for x in files),
          {'staged_count': len(staged), 'authorized_files': len(files)})
    p = subprocess.run(['git', '-C', str(ROOT), 'diff', '--cached', '--check'],
                       text=True, capture_output=True)
    check('staged_whitespace', p.returncode == 0, p.stdout + p.stderr)
    if head != before['head']:
        committed = git('diff-tree', '--no-commit-id', '--name-only', '-r', head).splitlines()
        check('final_commit_and_status_together', set(committed) == set(files) and
              'RRM/PROJECT_STATUS.md' in committed and 'RRM/ROADMAP_MANDATE.md' in committed,
              {'commit': head, 'files': len(committed)})
    passed = all(x['passed'] for x in checks)
    print(json.dumps({'all_passed': passed, 'scope': 'Offline M00-C evidence/reconciliation/project-control integrity; not F00 qualification or independent V&V', 'checks': checks}, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print(json.dumps({'all_passed': False, 'error': str(error), 'checks': checks}, indent=2))
        sys.exit(1)
