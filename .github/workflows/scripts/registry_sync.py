#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Sync trunk's committed module catalog from a registry checkout.

Module registration is TWO merges (registry PR + trunk PR — see the
extract-module skill). This script produces the trunk half mechanically from
a local checkout of castacks/airstack-modules-index:

1. mirror the registry's ``modules/*.yaml`` and ``stacks/*.yaml`` into the
   contract-test fixture ``tests/meta/fixtures/modules_index/`` (deleting
   entries the registry no longer has);
2. regenerate the committed ``docs/modules/`` pages with
   ``tools/gen_docs_catalog.py`` against that fixture and an EMPTY
   fetched-modules dir (the committed-page variant the docs-catalog contract
   test expects);
3. with ``--bump``, and only when step 1/2 changed anything: bump the ``.env``
   VERSION pre-release counter (the check-version-increment gate requires a
   strict bump on every PR) and add a release-notes bullet.

Run by ``.github/workflows/sync-modules-index.yml``, which turns a non-empty
diff into the trunk sync PR. Also runnable locally from the repo root:

    git clone --depth 1 https://github.com/castacks/airstack-modules-index /tmp/idx
    python3 .github/workflows/scripts/registry_sync.py --index /tmp/idx

stdlib only (gen_docs_catalog.py itself needs PyYAML).
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

TRUNK = Path(__file__).resolve().parents[3]
FIXTURE = TRUNK / "tests" / "meta" / "fixtures" / "modules_index"
GENERATOR = TRUNK / "tools" / "gen_docs_catalog.py"
ENV_FILE = TRUNK / ".env"
RELEASE_NOTES = TRUNK / "docs" / "release_notes" / "index.md"


def mirror_entries(index: Path) -> None:
    """Mirror registry modules/ and stacks/ YAMLs into the fixture."""
    for sub in ("modules", "stacks"):
        src, dst = index / sub, FIXTURE / sub
        if not src.is_dir():
            sys.exit(f"registry checkout has no {sub}/ directory: {src}")
        if dst.is_dir():
            shutil.rmtree(dst)
        dst.mkdir(parents=True)
        for entry in sorted(src.glob("*.yaml")):
            shutil.copy2(entry, dst / entry.name)


def regenerate_catalog() -> None:
    with tempfile.TemporaryDirectory() as empty:
        subprocess.run(
            [sys.executable, str(GENERATOR),
             "--index", str(FIXTURE),
             "--modules-dir", f"{empty}/no-fetched-modules"],
            check=True, cwd=TRUNK,
        )


def changed_paths() -> "list[str]":
    out = subprocess.run(
        ["git", "status", "--porcelain", "--",
         str(FIXTURE.relative_to(TRUNK)), "docs/modules"],
        check=True, capture_output=True, text=True, cwd=TRUNK,
    ).stdout
    return [line[3:].strip() for line in out.splitlines() if line.strip()]


def bump_version() -> "tuple[str, str]":
    """Increment the .env VERSION pre-release counter (or patch)."""
    text = ENV_FILE.read_text()
    m = re.search(r'^VERSION="([^"]+)"$', text, flags=re.M)
    if not m:
        sys.exit('.env has no VERSION="..." line')
    old = m.group(1)
    pre = re.fullmatch(r"(\d+\.\d+\.\d+-(?:alpha|beta|rc)\.)(\d+)", old)
    if pre:
        new = f"{pre.group(1)}{int(pre.group(2)) + 1}"
    else:
        rel = re.fullmatch(r"(\d+\.\d+\.)(\d+)", old)
        if not rel:
            sys.exit(f"unrecognized VERSION format: {old}")
        new = f"{rel.group(1)}{int(rel.group(2)) + 1}"
    ENV_FILE.write_text(text.replace(f'VERSION="{old}"', f'VERSION="{new}"', 1))
    return old, new


def add_release_note(entries: "list[str]") -> None:
    """Insert a bullet under the current '(Unreleased)' section heading."""
    lines = RELEASE_NOTES.read_text().splitlines(keepends=True)
    names = ", ".join(f"`{e}`" for e in entries) or "registry entries"
    bullet = (
        "- Module catalog synced from the registry "
        f"(`sync-modules-index`): {names}.\n"
    )
    for i, line in enumerate(lines):
        if re.match(r"^## .*\(Unreleased\)", line):
            lines.insert(i + 2, bullet)
            lines.insert(i + 3, "\n")
            RELEASE_NOTES.write_text("".join(lines))
            return
    print("warning: no '(Unreleased)' release-notes section — bullet skipped",
          file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--index", required=True, type=Path,
                        help="local checkout of castacks/airstack-modules-index")
    parser.add_argument("--bump", action="store_true",
                        help="on change, bump .env VERSION and add a "
                             "release-notes bullet (for the sync PR)")
    args = parser.parse_args()

    mirror_entries(args.index.resolve())
    regenerate_catalog()

    changed = changed_paths()
    if not changed:
        print("in-sync: committed catalog already matches the registry")
        return 0

    print("changed:")
    for path in changed:
        print(f"  {path}")
    if args.bump:
        # Only the registry-entry files (not the regenerated pages) make a
        # readable release note.
        entry_names = sorted(
            Path(p).stem for p in changed
            if "fixtures/modules_index" in p and p.endswith(".yaml")
        )
        old, new = bump_version()
        add_release_note(entry_names)
        print(f"VERSION {old} -> {new}; release-notes bullet added")
    return 0


if __name__ == "__main__":
    sys.exit(main())
