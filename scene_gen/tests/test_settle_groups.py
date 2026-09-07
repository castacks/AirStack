"""The settle's per-group verdict — the piece a batched bake stands on.

`settle.run` used to stop when the busiest body in the WHOLE scene was at
rest, which is correct for one wreck and useless for several: a single piece
still tumbling anywhere held every other pile to the step ceiling. That is
what a batched settle failed on before — 5,064 bodies, 975 s, 4,825 still
moving and nothing dropped.

None of this needs PhysX. `_measure` is arithmetic over three position dicts,
and it is the thing a group and the whole scene must be asked identically.
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from disaster import settle as S        # noqa: E402


def _positions(spec):
    return {k: np.array(v, dtype=float) for k, v in spec.items()}


def test_measure_on_a_subset_matches_measuring_that_subset_alone():
    """A group's numbers must not depend on who it was measured beside."""
    before = _positions({"a/0": (0, 0, 10), "a/1": (1, 0, 10),
                         "b/0": (40, 0, 30), "b/1": (41, 0, 30)})
    after = _positions({"a/0": (0, 0, 2), "a/1": (1.2, 0, 3),
                        "b/0": (40, 0, 0), "b/1": (48, 0, 0)})
    settled = after

    whole = S._measure(before, after, settled)
    only_a = S._measure({k: v for k, v in before.items() if k.startswith("a/")},
                        {k: v for k, v in after.items() if k.startswith("a/")},
                        {k: v for k, v in settled.items() if k.startswith("a/")})
    as_group = S._measure(before, after, settled, keys=["a/0", "a/1"])

    for key in ("drop_median", "drop_mean", "spread_max", "moved_max",
                "still_moving"):
        assert as_group[key] == pytest.approx(only_a[key]), key
    # And group A alone is NOT the whole scene — b/1 is thrown 8 m sideways.
    assert whole["spread_max"] > as_group["spread_max"]


def test_measure_reports_the_drop_and_the_throw_separately():
    """A collapse is large -Z and small XY; an explosion is large XY."""
    before = _positions({"p": (0, 0, 10), "q": (0, 0, 10)})
    dropped = _positions({"p": (0, 0, 0), "q": (0, 0, 0)})
    thrown = _positions({"p": (30, 0, 10), "q": (-30, 0, 10)})

    fell = S._measure(before, dropped, dropped)
    blew = S._measure(before, thrown, thrown)
    assert fell["drop_median"] == pytest.approx(-10.0)
    assert fell["spread_max"] == pytest.approx(0.0)
    assert blew["drop_median"] == pytest.approx(0.0)
    assert blew["spread_max"] == pytest.approx(30.0)


def test_measure_of_nothing_is_zero_not_an_error():
    """A group whose bodies all got thrown clear still has to report."""
    got = S._measure({}, {}, {}, keys=[])
    assert got["still_moving"] == 0
    assert got["drop_median"] == 0.0
    assert got["spread_max"] == 0.0


def test_run_accepts_groups_and_defaults_to_one():
    """The signature is the contract the batched baker calls through."""
    import inspect
    sig = inspect.signature(S.run)
    assert "groups" in sig.parameters
    assert sig.parameters["groups"].default is None
