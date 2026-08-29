#!/usr/bin/env python3
"""Pin the docs version-selector order: develop above main.

mike re-sorts versions.json on every deploy, and among non-numeric
("dev") versions it sorts reverse-lexically, so "main" always lands
above "develop". We want develop (which carries the higher, unreleased
version number) listed first while main stays the set-default landing
version. Every workflow that runs `mike deploy` must re-run this script
afterwards, since any mike write restores mike's own ordering.

Usage: docs_reorder_versions.py <path/to/versions.json>

Rewrites the file in place (mike's format: json indent=2, no trailing
newline). Idempotent — leaves the file byte-identical when already
ordered, so callers can use `git diff --quiet` to skip empty commits.
"""

import json
import sys


def reorder(entries):
    pinned_names = ('develop', 'main')
    pinned = [e for name in pinned_names for e in entries
              if e['version'] == name]
    rest = [e for e in entries if e['version'] not in pinned_names]
    return pinned + rest


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__.strip())
    path = sys.argv[1]
    with open(path) as f:
        entries = json.load(f)
    with open(path, 'w') as f:
        f.write(json.dumps(reorder(entries), indent=2))


if __name__ == '__main__':
    main()
