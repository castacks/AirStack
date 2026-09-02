"""LVLM-guided search — numpy port of the original RAVEN behaviour.

Sources merged:
  RayFronts_raven/rayfronts/behaviors/lvlm_behavior.py   (the behaviour)
  LVLM/internvl3.py                                      (the VLM node)

In the paper these were two processes: the behaviour raised `/lvlm_trigger`,
a separate InternVL3-2B node saw the trigger, ran ONE prompt against the
latest FPV frame and answered on `/lvlm_output`. Here the behaviour keeps the
same shape but the model call is an HTTP request to an OpenAI-compatible
endpoint (`raven_nav/lvlm_client.py`) issued by the node on a worker thread,
and `/<robot>/raven_nav/lvlm_output` still accepts an externally supplied
answer for tests and manual use.

The loop:
  targets exist, neither Voxel nor Ray fired
  -> publish `lvlm_trigger`, and at most once every `lvlm_request_interval_s`
     ask the VLM for three clue objects near the target
  -> clean the answer (OG `set_guiding_objects`) into guiding objects
  -> register them with rayfronts as queries; they come back as extra score
     columns
  -> fire when any ray scores > 0.9 on a guiding column; fly to the mean of
     those rays: path = [mean origin, mean origin + 5 * unit(mean dir)]
  -> the behaviour never locks a waypoint and clears the target pair (OG:124)
"""
from __future__ import annotations

from typing import List, Optional

import numpy as np

from raven_nav.behaviors.common import BehaviorOutput, TickContext, columns_for

# ── OG constants ────────────────────────────────────────────────────────────
# OG LVLM/internvl3.py:35 throttled the VLM node itself at 30 s; OG
# lvlm_behavior.py:16 additionally throttled the trigger at 20 s. One throttle
# is enough now that both live in one process, and the node's is the binding
# one, so 30 s it is.
REQUEST_INTERVAL_S = 30.0
RAY_THRESHOLD = 0.9             # OG lvlm_behavior.py:78
MAGNITUDE_M = 5.0               # OG lvlm_behavior.py:105
N_GUIDING_OBJECTS = 3           # OG LVLM/internvl3.py:78 ("three unique objects")

# OG LVLM/internvl3.py:77-79, minus the InternVL-specific '<image>\n' prefix
# (the image is a separate content part in an OpenAI-compatible request).
PROMPT_TEMPLATE = (
    'Find {targets}. '
    'List three unique objects or areas that are most helpful as clues or '
    'context to locate the {targets}. '
    'Write ONLY the object or area names as a plain comma-separated list.'
)


def build_prompt(target_objects) -> str:
    """OG LVLM/internvl3.py:77-79. `_target_objects` there was the raw
    comma-separated, lower-cased prompt string, so join the same way."""
    targets = ', '.join(str(t).strip() for t in target_objects if str(t).strip())
    return PROMPT_TEMPLATE.format(targets=targets)


def clean_guiding_objects(objects_string: str) -> List[str]:
    """OG lvlm_behavior.py:19-37, verbatim: split on commas, lower-case, strip
    a leading article, strip trailing punctuation, dedupe, drop empties."""
    raw_objects = [o.strip().lower() for o in str(objects_string).split(',')]
    cleaned: List[str] = []
    seen = set()
    for obj in raw_objects:
        if obj.startswith('a '):
            obj = obj[2:]
        elif obj.startswith('an '):
            obj = obj[3:]
        elif obj.startswith('the '):
            obj = obj[4:]
        obj = obj.rstrip('.,').strip()
        if obj and obj not in seen:
            cleaned.append(obj)
            seen.add(obj)
    return cleaned


class LvlmBehavior:
    name = 'LVLM-guided'

    def __init__(self, request_interval_s: float = REQUEST_INTERVAL_S,
                 ray_threshold: float = RAY_THRESHOLD,
                 enabled: bool = True) -> None:
        self.request_interval_s = float(request_interval_s)
        self.ray_threshold = float(ray_threshold)
        self.enabled = bool(enabled)
        self.guiding_objects: List[str] = []
        self.last_request_time: Optional[float] = None
        # Set by condition_check; the node reads them and does the ROS work.
        self.want_trigger = False       # publish /raven_nav/lvlm_trigger
        self.want_request = False       # fire one VLM request this tick
        self.guiding_changed = False    # re-publish the rayfronts queries
        self._indices: np.ndarray = np.zeros((0,), dtype=int)
        self._columns: List[int] = []

    # ── guiding objects ─────────────────────────────────────────────────────
    def set_guiding_objects(self, objects_string: str) -> List[str]:
        """OG lvlm_behavior.py:19-37 + OG mapping_server_rosnode.py:504-506."""
        cleaned = clean_guiding_objects(objects_string)
        self.guiding_changed = cleaned != self.guiding_objects
        self.guiding_objects = cleaned
        return self.guiding_objects

    def stale_queries(self, query_labels, target_objects,
                      background_objects=()) -> List[str]:
        """OG lvlm_behavior.py:42-43 + OG mapping_server_rosnode.py:282-288 —
        registered labels that are neither a target, a background label, nor a
        current guiding object, i.e. guiding labels from an earlier answer that
        rayfronts should drop."""
        keep = {str(x).strip().lower()
                for x in list(target_objects) + list(background_objects)
                + list(self.guiding_objects)}
        return [str(l) for l in query_labels
                if str(l).strip().lower() not in keep]

    # ── condition ───────────────────────────────────────────────────────────
    def condition_check(self, ctx: TickContext) -> bool:
        self.want_trigger = False
        self.want_request = False
        self._indices = np.zeros((0,), dtype=int)
        self._columns = []
        if not self.enabled:
            return False
        if not ctx.target_objects or not ctx.query_labels:
            return False           # OG lvlm_behavior.py:45-48

        # OG lvlm_behavior.py:50-55 — the trigger goes out every tick; the
        # request itself is throttled.
        self.want_trigger = True
        now = float(ctx.now)
        if (self.last_request_time is None
                or (now - self.last_request_time) > self.request_interval_s):
            self.want_request = True
            self.last_request_time = now

        if not self.guiding_objects:
            return False           # OG lvlm_behavior.py:57-58

        cols = columns_for(ctx.query_labels, self.guiding_objects)
        if not cols:
            # rayfronts has not registered the guiding labels yet.
            return False
        self._columns = cols
        o, s = ctx.ray_origins, ctx.ray_scores
        if o is None or s is None or len(o) == 0:
            return False
        s = np.asarray(s, dtype=np.float64)
        if s.ndim != 2 or max(cols) >= s.shape[1]:
            return False
        mask = (s[:, cols] > self.ray_threshold).any(axis=1)   # OG:78-80
        idx = np.nonzero(mask)[0]
        if idx.size == 0:
            return False
        self._indices = idx
        return True

    # ── execution (OG lvlm_behavior.py:89-124) ──────────────────────────────
    def execute(self, ctx: TickContext) -> BehaviorOutput:
        # OG:124 returns (waypoint_locked, None, None) — the guided hop never
        # locks and clears the waypoint pair.
        out = BehaviorOutput(waypoint_locked=ctx.waypoint_locked,
                             target_waypoint=None, target_waypoint2=None)
        if self._indices.size == 0:
            out.note = 'no guiding rays'
            return out
        fo = np.asarray(ctx.ray_origins, dtype=np.float64)[self._indices]
        fd = np.asarray(ctx.ray_dirs, dtype=np.float64)[self._indices]
        origin = fo.mean(axis=0)                       # OG:97
        direction = fd.mean(axis=0)                    # OG:98
        nrm = float(np.linalg.norm(direction))
        if nrm < 1e-9:
            out.note = 'degenerate guiding direction'
            return out
        unit = direction / nrm
        target = origin + unit * MAGNITUDE_M           # OG:110
        start = ctx.clamp(origin)
        end = ctx.clamp(target)
        # DEVIATION 2 — never route outside the mission polygon.
        if not (ctx.inside_area(start) and ctx.inside_area(end)):
            out.note = 'guiding hop outside search_area'
            return out
        out.path = [start, end]                        # OG:112-120
        return out

    def guiding_rays(self):
        """Origins/dirs of the rays that fired, for viz."""
        return self._indices, list(self._columns)
