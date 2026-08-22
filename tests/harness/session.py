"""Session-scoped mutable state shared between conftest hooks and harness helpers.

pytest hooks (in conftest.py) *write* this state — ``init_run_dir`` in
``pytest_configure``, ``set_current_item`` in ``pytest_runtest_setup``/``teardown``,
``record_cmd_output`` from the subprocess helpers. Helpers *read* it — ``run_dir``,
``current_item``, ``last_cmd_output``. Keeping it here means helper modules never reach
back into conftest globals.
"""
import logging
from datetime import datetime
from pathlib import Path

# Shared logger used across the harness and the tests.
logger = logging.getLogger("airstack")
logger.setLevel(logging.INFO)

_DEFAULT_LOG_KEY = "_last"

_run_dir = None
_current_item = None
_last_cmd_output: dict[str, str] = {}


def init_run_dir(airstack_root) -> Path:
    """Create and record this session's timestamped results dir; return it."""
    global _run_dir
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    _run_dir = Path(airstack_root) / "tests" / "results" / timestamp
    _run_dir.mkdir(parents=True, exist_ok=True)
    return _run_dir


def run_dir():
    """This session's results dir (``None`` before ``pytest_configure`` runs)."""
    return _run_dir


def set_current_item(item):
    """Record the currently-running pytest item (``None`` between tests)."""
    global _current_item
    _current_item = item


def current_item():
    """The currently-running pytest item, or ``None`` outside a test."""
    return _current_item


def record_cmd_output(text, log_name=None):
    """Store the latest subprocess output, keyed by ``log_name`` and as the default."""
    key = log_name or _DEFAULT_LOG_KEY
    _last_cmd_output[key] = text
    _last_cmd_output[_DEFAULT_LOG_KEY] = text


def last_cmd_output(log_name=None) -> str:
    """The most recent subprocess output for ``log_name`` (or the default)."""
    key = log_name or _DEFAULT_LOG_KEY
    return _last_cmd_output.get(key) or _last_cmd_output.get(_DEFAULT_LOG_KEY, "")
