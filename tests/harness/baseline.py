"""Select a completed, configuration-identical simulation baseline."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

from harness.run_meta import classify_run, comparability_reason


def select_baseline(
    candidate_dirs: Iterable[Path],
    current_meta: dict,
) -> tuple[Path | None, list[str]]:
    """Return newest comparable candidate and human-readable rejection reasons."""
    matches: list[Path] = []
    rejected: list[str] = []
    for candidate in {Path(path) for path in candidate_dirs}:
        meta = classify_run(candidate)
        reason = comparability_reason(current_meta, meta)
        if reason:
            rejected.append(f"{candidate}: {reason}")
        else:
            matches.append(candidate)
    if not matches:
        return None, sorted(rejected)
    matches.sort(
        key=lambda path: (path / "run_meta.json").stat().st_mtime
        if (path / "run_meta.json").exists()
        else path.stat().st_mtime,
        reverse=True,
    )
    return matches[0], sorted(rejected)


def select_baseline_path(current_dir: Path, baseline_root: Path) -> Path | None:
    """Convenience API used by CI after downloading several artifacts."""
    current_meta = classify_run(Path(current_dir))
    candidates = [
        path.parent
        for path in Path(baseline_root).rglob("run_meta.json")
    ]
    selected, _ = select_baseline(candidates, current_meta)
    return selected
