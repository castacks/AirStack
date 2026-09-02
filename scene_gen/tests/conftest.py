"""conftest.py — pytest configuration for the scene_gen host-side tests.

Registers the `slow` marker so a suite can separate the arithmetic gates
(milliseconds) from the ones that build a whole 1 km city (40-90 s each):

    python3 -m pytest -q scene_gen/tests -m "not slow"   # the fast gates
    python3 -m pytest -q scene_gen/tests -m slow         # the layout builds
"""


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "slow: builds a full city layout (tens of seconds); deselect with "
        "-m 'not slow'")
