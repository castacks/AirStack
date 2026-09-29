"""Terminal progress bar for the post-flight analysis scripts (stdlib only).

    prog = Progress("analyze_tigris_run", stages=["prior", "flown", ...])
    prog("flown", done, total)       # the callback the scoring functions take
    prog.finish()

The scoring functions (``mtl_metrics_logger.analysis.write_run_outputs``,
``tigris_search_planner.rewards.score_looks`` ...) accept an optional
``progress(stage, done, total)`` callable and call it from their long loops; this
class draws one line per stage. On a terminal the line redraws in place (at most
~5 times a second); in a log file (not a TTY) it prints one line per 10 %.
"""

from __future__ import annotations

import sys
import time


def _hms(s: float) -> str:
    s = max(0, int(round(s)))
    return f"{s // 3600:d}:{s % 3600 // 60:02d}:{s % 60:02d}" if s >= 3600 else f"{s // 60:d}:{s % 60:02d}"


class Progress:
    def __init__(self, tag: str, stages: list[str] | None = None, stream=None, width: int = 28) -> None:
        self.tag = tag
        self.stages = list(stages or [])
        self.stream = stream or sys.stderr
        self.tty = hasattr(self.stream, "isatty") and self.stream.isatty()
        self.width = width
        self.t_start = time.monotonic()
        self.stage = None
        self.t_stage = 0.0
        self.last_draw = 0.0
        self.last_decile = -1
        self.line_open = False

    # the callback ----------------------------------------------------------------
    def __call__(self, stage: str, done: float = 0, total: float = 0) -> None:
        now = time.monotonic()
        if stage != self.stage:
            self._close_stage(now)
            self.stage, self.t_stage, self.last_decile, self.last_draw = stage, now, -1, 0.0
            if stage not in self.stages:
                self.stages.append(stage)
        frac = min(max(done / total, 0.0), 1.0) if total and total > 0 else None
        if self.tty:
            if now - self.last_draw < 0.2 and frac not in (None, 1.0) and self.last_draw:
                return
            self.last_draw = now
            self.stream.write("\r" + self._line(frac, now) + "\033[K")
            self.stream.flush()
            self.line_open = True
        else:
            decile = -1 if frac is None else int(frac * 10)
            if decile != self.last_decile:
                self.last_decile = decile
                self.stream.write(self._line(frac, now) + "\n")
                self.stream.flush()

    def end_stage(self) -> None:
        """Close the current stage's line (call before printing anything else)."""
        self._close_stage(time.monotonic())

    def finish(self) -> None:
        self._close_stage(time.monotonic())
        self.stream.write(f"[{self.tag}] analysis done in {_hms(time.monotonic() - self.t_start)}\n")
        self.stream.flush()

    # drawing ---------------------------------------------------------------------
    def _line(self, frac, now: float) -> str:
        k = self.stages.index(self.stage) + 1 if self.stage in self.stages else 0
        head = f"[{self.tag}] step {k}/{len(self.stages)} {self.stage}"
        el = now - self.t_stage
        if frac is None:
            return f"{head}  ... {_hms(el)}"
        n = int(round(frac * self.width))
        bar = "#" * n + "-" * (self.width - n)
        eta = f", ~{_hms(el * (1.0 - frac) / frac)} left" if 0.0 < frac < 1.0 and el > 1.0 else ""
        return f"{head} [{bar}] {100 * frac:5.1f}%  {_hms(el)}{eta}"

    def _close_stage(self, now: float) -> None:
        if self.stage is None:
            return
        line = self._line(1.0, now) if self.tty or self.last_decile < 10 else None
        if self.tty:
            self.stream.write("\r" + line + "\033[K\n")
        elif line is not None:
            self.stream.write(line + "\n")
        self.stream.flush()
        self.line_open = False
        self.stage = None
