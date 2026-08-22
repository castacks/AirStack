# Robot-side unit tests

Unit-test **source is co-located** with each ROS 2 package (the standard colcon
convention):

```
robot/ros_ws/src/<layer>/<package>/test/test_<name>.py   ← source of truth
```

[`../colcon_unit_test_packages.yaml`](../colcon_unit_test_packages.yaml) lists which
packages have unit tests; `tests/conftest.py` resolves each to its `test/` dir and
collects the non-linter `test_*.py` files under `--import-mode=importlib`, tagging each
`@pytest.mark.unit` by path — you do not write the mark yourself.

Run them with `airstack test -m unit`, or `cd tests && pytest -m unit`. C++ gtests in the
same `test/` dir run under `colcon test --packages-select <pkg>`.

To add a package's unit tests, list it under `robot.packages` in the YAML — see the
`add-unit-tests` agent skill.
