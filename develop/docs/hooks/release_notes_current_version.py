"""MkDocs hook: trim the Release Notes page to the version being built.

The source file ``docs/release_notes/index.md`` keeps one ``##`` section per
version (newest first) so the full history lives in git, but each
mike-deployed docs version should only render its own notes — the site's
version selector (and the GitHub releases page) is the archive for the rest.

At build time this hook reads the ``VERSION=`` line from the repo-root
``.env`` and drops every ``## X.Y.Z ...`` section whose base semver does not
match (pre-release suffixes like ``-alpha.N`` are ignored for matching, so
``VERSION="0.20.0-alpha.13"`` keeps the ``## 0.20.0 (Unreleased)`` section).

If no section matches the current VERSION, the page is left unfiltered and a
warning is logged rather than publishing an empty page.
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

    sections = list(SECTION_RE.finditer(markdown))
    if not any(m.group(1) == current for m in sections):
        log.warning(
            "release_notes hook: no '## %s' section in %s; "
            "publishing the page unfiltered",
            current,
            PAGE_SRC,
        )
        return markdown

    pieces = [markdown[: sections[0].start()]]
    for i, match in enumerate(sections):
        if match.group(1) != current:
            continue
        end = sections[i + 1].start() if i + 1 < len(sections) else len(markdown)
        pieces.append(markdown[match.start() : end])
    return "".join(pieces)
