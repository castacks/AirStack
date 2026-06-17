"""M-of-N temporal confirmation for detections rebuilt from scratch each tick.

A detection must be seen across `confirm_hits` ticks before it's confirmed; a
confirmed track then survives up to `max_misses` dropped ticks. Generic over the
payload via a caller-supplied `match` predicate and optional `merge`.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Generic, List, Optional, TypeVar

T = TypeVar('T')


@dataclass
class _Track(Generic[T]):
    payload: T
    hits: int
    misses: int = 0
    confirmed: bool = False


class TemporalConfirmer(Generic[T]):
    """confirm_hits: hits to confirm. max_misses: misses to drop. hit_cap:
    counter ceiling so a stale track still ages out fast (default confirm_hits)."""

    def __init__(self, confirm_hits: int = 3, max_misses: int = 4,
                 hit_cap: Optional[int] = None):
        self.confirm_hits = max(1, int(confirm_hits))
        self.max_misses = max(1, int(max_misses))
        self.hit_cap = (self.confirm_hits if hit_cap is None
                        else max(self.confirm_hits, int(hit_cap)))
        self._tracks: List[_Track[T]] = []

    def reset(self) -> None:
        self._tracks.clear()

    def update(self, detections: List[T],
               match: Callable[[T, T], bool],
               merge: Optional[Callable[[T, T], T]] = None) -> List[T]:
        """Fold this tick's detections into the tracks, age unmatched tracks,
        and return the payloads of every currently-confirmed track. Call once
        per tick even with an empty list so dropouts age correctly."""
        matched = [False] * len(self._tracks)
        for det in detections:
            ti = next((i for i, tr in enumerate(self._tracks)
                       if not matched[i] and match(tr.payload, det)), None)
            if ti is None:
                self._tracks.append(_Track(
                    payload=det, hits=1, confirmed=self.confirm_hits <= 1))
                matched.append(True)
                continue
            tr = self._tracks[ti]
            tr.payload = merge(tr.payload, det) if merge else det
            tr.hits = min(self.hit_cap, tr.hits + 1)
            tr.misses = 0
            tr.confirmed = tr.confirmed or tr.hits >= self.confirm_hits
            matched[ti] = True

        survivors: List[_Track[T]] = []
        for i, tr in enumerate(self._tracks):
            if matched[i]:
                survivors.append(tr)
                continue
            tr.misses += 1
            tr.hits = max(0, tr.hits - 1)
            if tr.misses < self.max_misses:
                survivors.append(tr)
        self._tracks = survivors
        return [tr.payload for tr in self._tracks if tr.confirmed]
