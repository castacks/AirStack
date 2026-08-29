"""MkDocs hook: trim the Release Notes page to the version being built.

The source file ``docs/release_notes/index.md`` keeps one ``##`` section per
version (newest first) so the full history lives in git, but each
mike-deployed docs version should only render its own notes — the site's
version selector (and the GitHub releases page) is the archive for the rest.

Docs versions are published under MAJOR.MINOR slugs (e.g. ``/0.20/``), with
hotfix patches republished in place. At build time this hook reads the
``VERSION=`` line from the repo-root ``.env`` and keeps every ``## X.Y.Z ...``
section sharing its MAJOR.MINOR — so a ``0.20`` build shows the ``## 0.20.0``
section plus any ``## 0.20.1`` hotfix sections, and pre-release suffixes are
ignored (``VERSION="0.21.0-dev.13"`` keeps ``## 0.21.0 (Unreleased)``).

If no section matches the current VERSION's MAJOR.MINOR, the page is left
unfiltered and a warning is logged rather than publishing an empty page.
"""

import logging
import re
from pathlib import Path

log = logging.getLogger("mkdocs.hooks.release_notes")

PAGE_SRC = "docs/release_notes/index.md"
SECTION_RE = re.compile(r"^## (\d+\.\d+\.\d+)\b", re.MULTILINE)
VERSION_RE = re.compile(r"^VERSION\s*=\s*[\"']?(\d+\.\d+\.\d+)", re.MULTILINE)


def _current_version(config) -> str | None:
    env_path = Path(config["config_file_path"]).parent / ".env"
    try:
        text = env_path.read_text()
    except OSError:
        return None
    match = VERSION_RE.search(text)
    return match.group(1) if match else None


def on_page_markdown(markdown, page, config, files):
    if page.file.src_uri != PAGE_SRC:
        return markdown

    current = _current_version(config)
    if current is None:
        log.warning(
            "release_notes hook: could not read VERSION= from .env; "
            "publishing the page unfiltered"
        )
        return markdown

    minor = ".".join(current.split(".")[:2])  # "0.20.2" -> "0.20"

    def matches(version: str) -> bool:
        return ".".join(version.split(".")[:2]) == minor

    sections = list(SECTION_RE.finditer(markdown))
    if not any(matches(m.group(1)) for m in sections):
        log.warning(
            "release_notes hook: no '## %s.x' section in %s; "
            "publishing the page unfiltered",
            minor,
            PAGE_SRC,
        )
        return markdown

    pieces = [markdown[: sections[0].start()]]
    for i, match in enumerate(sections):
        if not matches(match.group(1)):
            continue
        end = sections[i + 1].start() if i + 1 < len(sections) else len(markdown)
        pieces.append(markdown[match.start() : end])
    return "".join(pieces)
