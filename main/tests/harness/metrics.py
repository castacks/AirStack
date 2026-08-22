"""Per-run metrics recording (``metrics.json``)."""
import json
import os
import threading

from harness.commands import _nodeid_dotted
from harness.session import current_item, run_dir


class MetricsRecorder:
    def __init__(self, path):
        self._path = path
        self._data = json.loads(path.read_text()) if path.exists() else {}
        self._lock = threading.Lock()

    def _flush(self):
        tmp = self._path.with_suffix(self._path.suffix + ".tmp")
        tmp.write_text(json.dumps(self._data, indent=2))
        os.replace(tmp, self._path)

    def record(self, test_name, key, value, unit="", direction="lower_is_better", **extra):
        with self._lock:
            if test_name not in self._data:
                self._data[test_name] = {}
            entry = {"value": value, "unit": unit, "direction": direction}
            entry.update(extra)
            self._data[test_name][key] = entry
            self._flush()

    def record_list(self, test_name, key, values):
        """Store a raw list (time series) — not scored by parse_metrics."""
        with self._lock:
            if test_name not in self._data:
                self._data[test_name] = {}
            self._data[test_name][key] = {"samples": values}
            self._flush()


_METRICS = None


def get_metrics():
    global _METRICS
    if _METRICS is None:
        _METRICS = MetricsRecorder(run_dir() / "metrics.json")
    return _METRICS


def current_test_id():
    """Test id used as the metrics.json key. Matches JUnit XML's classname.name
    format so parse_metrics.py can merge results.xml and metrics.json entries."""
    item = current_item()
    if item is None:
        return "unknown"
    return _nodeid_dotted(item.nodeid)
