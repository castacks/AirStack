"""pytest configuration for svg_ground_control/test.

functional_*_test.py are closed-loop scripts, not unit tests: each needs a
live swarm_commander and is run by hand as `python3 test/functional_<x>_test.py`
(see its docstring). Their names match pytest's default `*_test.py` pattern,
so without this `pytest test/` tried to import them and stopped with
collection errors.
"""

collect_ignore_glob = ["functional_*_test.py"]
