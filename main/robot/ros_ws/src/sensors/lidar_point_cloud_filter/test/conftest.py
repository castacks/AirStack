# Skip ament linter modules during pytest/colcon test.
# PYTEST_ADDOPTS -m is not forwarded by ament pytest; collect_ignore is.
collect_ignore = [
    "test_copyright.py",
    "test_flake8.py",
    "test_pep257.py",
]
