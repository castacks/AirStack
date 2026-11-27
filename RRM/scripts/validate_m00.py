#!/usr/bin/env python3
"""Read-only M00 checks. Emits JSON; never stages, commits or changes services."""
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
EVIDENCE = WORK / 'evidence/M00'
checks = []


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).rstrip("\n")


def check(name, condition, detail):
    checks.append({'check': name, 'passed': bool(condition), 'detail': detail})


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def identity(path):
    if path.is_symlink():
        return {'type': 'symlink', 'target': os.readlink(path)}
    if path.is_file():
        return {'type': 'file', 'sha256': digest(path),
                'mode': stat.S_IMODE(path.stat().st_mode)}
    if path.is_dir():
        return {'type': 'gitlink', 'head': subprocess.check_output(
            ['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).rstrip("\n")}
    return {'type': 'missing'}


def read_csv(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def main():
    before = json.loads((EVIDENCE / 'repository-before.json').read_text())
    check('repository', git('remote', 'get-url', 'origin') == before['repository']
          == 'https://github.com/castacks/AirStack.git', 'Origin URL compared')
    check('branch', git('branch', '--show-current') == 'ore_proj', 'Existing ore_proj')
    head = git('rev-parse', 'HEAD')
    check('history', head == before['head'] or git('rev-parse', 'HEAD^') == before['head'],
          'At baseline or one direct successor commit; no rewrite')
    drift = [p for p, v in before['tracked_outside_rrm'].items()
             if identity(ROOT / p) != v]
    check('protected_tracked_content', not drift,
          {'entries': len(before['tracked_outside_rrm']), 'changed': drift})
    protected = {str(p.relative_to(ROOT)): identity(p)
                 for p in (ROOT / 'RRM-transfer').rglob('*')
                 if p.is_file() or p.is_symlink()}
    check('protected_reference_tree', protected == before['protected_tree'],
          {'before': len(before['protected_tree']), 'after': len(protected)})
    outside = [line for line in git('status', '--porcelain=v1', '--untracked-files=all').splitlines()
               if not line[3:].startswith('RRM/')]
    check('outside_workspace_status', not outside,
          {'initial_status': before['status_porcelain'], 'current_outside': outside})
    check('outside_workspace_diff', not git('diff', before['head'], '--', '.', ':(exclude)RRM'),
          'Tracked diff to baseline empty outside RRM')
    source = json.loads((EVIDENCE / 'source-manifest.json').read_text())
    changed = [x['path'] for x in source['sources'] if digest(ROOT / x['path']) != x['sha256']]
    check('reference_source_hashes', not changed, {'changed': changed})
    package = json.loads((EVIDENCE / 'controlled-package-manifest.json').read_text())
    base = Path(package['supplied_path'])
    actual = {str(p.relative_to(base)): {'path': str(p.relative_to(base)),
              'bytes': p.stat().st_size, 'sha256': digest(p)}
              for p in base.rglob('*') if p.is_file()}
    check('controlled_input_unchanged', actual == {x['path']: x for x in package['files']},
          {'input_files': len(actual), 'supplied_path': str(base)})
    for wd, manifest in [(base, 'SHA256SUMS'),
                         (base / 'release_staging/F00X-REL-001', 'RELEASE_MANIFEST.sha256')]:
        proc = subprocess.run(['sha256sum', '-c', manifest], cwd=wd,
                              text=True, capture_output=True)
        check('input_' + manifest, proc.returncode == 0,
              {'exit_code': proc.returncode, 'entries_ok': proc.stdout.count(': OK'),
               'stderr': proc.stderr})
    requirements = read_csv(WORK / 'verification/matrices/M00_REQUIREMENTS_RECONCILIATION.csv')
    kpis = read_csv(WORK / 'verification/matrices/M00_KPI_MATRIX.csv')
    ivvs = read_csv(WORK / 'verification/matrices/M00_IVV_MATRIX.csv')
    d2_rows = []
    header = None
    for line in (base / 'F00_EXT_REQUIREMENTS_TEST_MATRIX.md').read_text().splitlines():
        if line.startswith('## '):
            header = None
        if not line.startswith('|'):
            continue
        cells = [x.strip() for x in re.split(r'(?<!\\)\|', line)[1:-1]]
        if cells[0] == 'Req':
            header = cells
        elif header and not re.fullmatch(r'[-: ]+', cells[0]):
            d = dict(zip(header, cells))
            d2_rows.append((d['Req'], next(v for k, v in d.items() if k.startswith('Acceptance')),
                            d['Result']))
    mapped = [(d['requirement_label'], d['source_acceptance_exact'], d['source_result'])
              for d in requirements]
    check('d2_row_and_acceptance_coverage', mapped == d2_rows and len(mapped) == 29,
          {'source_rows': len(d2_rows), 'mapped_rows': len(mapped)})
    ids = {d['mapping_id'] for d in requirements}
    check('matrix_cross_references', len(ids) == len(requirements) == len(kpis) == len(ivvs)
          and ids == {d['mapping_id'] for d in kpis} == {d['mapping_id'] for d in ivvs}
          and all(a['kpi_id'] == b['kpi_id'] and a['ivv_id'] == c['ivv_id']
                  and a['source_acceptance_exact'] == b['acceptance_exact']
                  for a, b, c in zip(requirements, kpis, ivvs)),
          'One-to-one requirement, KPI and IVV mapping and exact criterion propagation')
    check('no_claimed_campaign_passes', all(d['current_verdict'] in ('NOT_TESTED', 'OUT_OF_SCOPE')
          and d['current_value'] == 'NOT_MEASURED' for d in kpis)
          and all(d['current_status'] in ('NOT_TESTED', 'OUT_OF_SCOPE') for d in ivvs),
          'All new measurements and IVV remain unperformed')
    broken = []
    for path in WORK.rglob('*.md'):
        if base in path.parents:
            continue
        for link in re.findall(r'\[[^\]]*\]\(([^)]+)\)', path.read_text()):
            if '://' in link or link.startswith('#'):
                continue
            target = link.split('#')[0]
            if not (path.parent / target).exists():
                broken.append({'file': str(path.relative_to(ROOT)), 'link': link})
    check('local_document_links', not broken, {'broken': broken})
    staged = git('diff', '--cached', '--name-only').splitlines()
    check('staged_path_scope', all(p.startswith('RRM/') and
          not p.startswith('RRM/RRM-C001-F00-EXT/') for p in staged), {'staged_files': len(staged)})
    diff_check = subprocess.run(['git', '-C', str(ROOT), 'diff', '--cached', '--check'],
                                capture_output=True, text=True)
    check('staged_whitespace', diff_check.returncode == 0, diff_check.stdout + diff_check.stderr)
    if head != before['head']:
        committed = git('diff-tree', '--no-commit-id', '--name-only', '-r', head).splitlines()
        check('committed_path_scope', bool(committed) and all(p.startswith('RRM/') and
              not p.startswith('RRM/RRM-C001-F00-EXT/') for p in committed),
              {'committed_files': len(committed), 'commit': head})
    result = {'scope': 'M00 repository/input/matrix integrity; no functional or independent IVV qualification',
              'all_passed': all(c['passed'] for c in checks), 'checks': checks}
    print(json.dumps(result, indent=2))
    return 0 if result['all_passed'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print(json.dumps({'all_passed': False, 'error': str(error), 'checks': checks}, indent=2))
        sys.exit(1)
