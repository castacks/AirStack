"""Check research entry-point budgets, links, catalog coverage and archive integrity."""
from pathlib import Path
import argparse
import hashlib
import re

# Budgets allow normal edits while keeping detailed progress in dated findings.
BUDGETS = {
    'README.md': (100, 6000),
    'AGENTS.md': (120, 8000),
    'MS_RESEARCH_HANDOFF.md': (150, 12000),
}
HISTORY_SHA256 = '084b2bf86d58d7b37213f71be1447839e799f11f18abbc6d73902aa8c45d35eb'


def check(root: Path) -> list[str]:
    errors = []
    pages = [*BUDGETS, 'docs/research/INDEX.md']
    for name in pages:
        path = root / name
        if not path.is_file():
            errors.append(f'{name}: missing page')
            continue
        body = path.read_text(encoding='utf-8')
        if name in BUDGETS:
            lines, size = BUDGETS[name]
            if len(body.splitlines()) > lines or len(path.read_bytes()) > size:
                errors.append(f'{name}: exceeds {lines} lines / {size} bytes; '
                              'put detailed evidence in a dated findings note and link it')
        for target in re.findall(r'\]\(([^)]+)\)', body):
            if re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:', target) or target.startswith('#'):
                continue
            target = target.split('#', 1)[0]
            if target and not (path.parent / target).exists():
                errors.append(f'{name}: missing local link target {target}')
    index_path = root / 'docs/research/INDEX.md'
    if index_path.is_file():
        targets = set(re.findall(r'\]\(([^)]+)\)', index_path.read_text(encoding='utf-8')))
        for record in sorted((root / 'experiments').glob('EXP-*.md')):
            if f'../../experiments/{record.name}' not in targets:
                errors.append(f'{record.name}: add declaration to docs/research/INDEX.md')
    history = root / 'MS_RESEARCH_HISTORY.md'
    if not history.is_file() or hashlib.sha256(history.read_bytes()).hexdigest() != HISTORY_SHA256:
        errors.append('MS_RESEARCH_HISTORY.md: historical snapshot changed; '
                      'write new evidence in dated notes, not the frozen archive')
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    errors = check(args.root)
    if errors:
        print('\n'.join(errors))
        return 1
    print('Research documentation checks passed.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
