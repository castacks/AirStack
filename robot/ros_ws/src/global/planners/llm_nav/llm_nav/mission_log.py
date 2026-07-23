"""Structured JSONL event logging for llm_nav.

Every LLM interaction (full prompt, raw response, parse result, latency),
every commitment transition, and every call into the digest/waypoint helpers
(via @log_call) lands as one JSON line in
{results_dir}/llm_nav_{robot}_events.jsonl. The file is append-only and
line-buffered so `tail -f` follows a live run, and the trace doubles as
fine-tuning data later.
"""

import functools
import json
import os
import threading
import time


def _jsonable(obj, max_str=2000):
    """Best-effort compact JSON conversion for arbitrary event payloads."""
    import numpy as np
    if obj is None or isinstance(obj, (bool, int, float)):
        return obj
    if isinstance(obj, str):
        return obj if len(obj) <= max_str else obj[:max_str] + f'...(+{len(obj) - max_str} chars)'
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    if isinstance(obj, np.ndarray):
        if obj.size <= 8:
            return [round(float(v), 3) for v in obj.flatten()]
        return f'ndarray{obj.shape}'
    if isinstance(obj, dict):
        return {str(k): _jsonable(v, max_str) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set)):
        seq = list(obj)
        if len(seq) > 50:
            return [_jsonable(v, max_str) for v in seq[:50]] + [f'...(+{len(seq) - 50} items)']
        return [_jsonable(v, max_str) for v in seq]
    return repr(obj)[:max_str]


class MissionLog:
    """Append-only JSONL log; thread-safe (LLM worker + executor callbacks)."""

    def __init__(self, path: str, robot_name: str, sim_clock_fn=None):
        self._path = path
        self._robot = robot_name
        self._sim_clock_fn = sim_clock_fn
        self._lock = threading.Lock()
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self._fh = open(path, 'a', buffering=1)
        self.event('log_opened', path=path)

    def event(self, event_type: str, **fields):
        rec = {'t_wall': round(time.time(), 3), 'event': event_type,
               'robot': self._robot}
        if self._sim_clock_fn is not None:
            try:
                rec['t_sim'] = round(float(self._sim_clock_fn()), 3)
            except Exception:
                pass
        rec.update({k: _jsonable(v) for k, v in fields.items()})
        line = json.dumps(rec, ensure_ascii=False)
        with self._lock:
            try:
                self._fh.write(line + '\n')
            except (OSError, ValueError):
                pass

    def close(self):
        with self._lock:
            try:
                self._fh.close()
            except (OSError, ValueError):
                pass


# Module-level active log so @log_call works on free functions/methods in
# digest.py without threading a logger through every signature.
_active_log: 'MissionLog | None' = None


def set_active_log(log: 'MissionLog | None'):
    global _active_log
    _active_log = log


def _summarize_arg(a):
    import numpy as np
    if isinstance(a, np.ndarray):
        return f'ndarray{a.shape}'
    if isinstance(a, (list, tuple, set, dict)):
        return f'{type(a).__name__}[{len(a)}]'
    if isinstance(a, str):
        return a if len(a) <= 60 else a[:60] + '...'
    if isinstance(a, (bool, int, float)) or a is None:
        return a
    return type(a).__name__


def log_call(fn):
    """Log function name, arg summary, and duration to the active MissionLog."""
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        t0 = time.monotonic()
        err = None
        try:
            return fn(*args, **kwargs)
        except Exception as e:
            err = f'{type(e).__name__}: {e}'
            raise
        finally:
            if _active_log is not None:
                # Skip `self` for methods so the summary stays readable.
                shown = args[1:] if args and hasattr(args[0], fn.__name__) else args
                rec = dict(
                    fn=fn.__qualname__,
                    duration_ms=round((time.monotonic() - t0) * 1000.0, 2),
                    args=[_summarize_arg(a) for a in shown],
                    kwargs={k: _summarize_arg(v) for k, v in kwargs.items()},
                )
                if err:
                    rec['error'] = err
                _active_log.event('fn_call', **rec)
    return wrapper
