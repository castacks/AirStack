# Simulation-side unit tests

Unit-test **source is co-located** with each simulation component, the same way the
robot workspace works:

```
simulation/**/<extension>/test/test_<name>.py   ← source of truth
```

[`../colcon_unit_test_packages.yaml`](../colcon_unit_test_packages.yaml) lists which
components have unit tests, under the `sim:` key; `tests/conftest.py` resolves each to
its `test/` dir and tags the collected items `@pytest.mark.unit` by path.

Run them with `airstack test -m unit`, or `cd tests && pytest -m unit`. These components
are not part of the onboard ROS workspace, so `colcon test` does not run them.

Tests needing a GPU, a full sim, or Docker belong in [`../system/`](../system/) instead.

Currently listed: none. The OptiTrack NatNet emulator moved to the
[asm_optitrack module](https://github.com/castacks/asm_optitrack), whose unit tests run
in the module repo's CI. Add new sim-side components under the `sim:` key when they
gain a co-located `test/` dir.
