# Robot-side unit tests

Unit-test **source is co-located** with each ROS 2 package (the standard colcon
convention) and is collected by `pytest tests/`:

```
robot/ros_ws/src/<layer>/<package>/test/test_<name>.py   ← source of truth
```

[`../colcon_unit_test_packages.yaml`](../colcon_unit_test_packages.yaml) lists which
packages have unit tests; `tests/conftest.py` resolves each to its `test/` dir and
collects the non-linter `test_*.py` files under `--import-mode=importlib`, tagging each
`@pytest.mark.unit`. Both `airstack test -m unit` and `colcon test --packages-select <pkg>`
run the same source.

To add a package's unit tests, list it under `robot.packages` in the YAML — see the
`add-unit-tests` agent skill. The per-layer subdirectories here hold only documentation.
