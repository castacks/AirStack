#!/usr/bin/env python3
"""test_hurricane_washaway_blockers.py — `blocked=` is a CALLABLE, and this
file exists because that has now been got wrong twice in the same launcher.

WHAT BROKE, BOTH TIMES. `washaway.shift_spec`, `washaway.collapse_spec` and
`washaway.car_shift_spec` all march a drift outward step by step and ask a
predicate whether anything is in the way:

    tag = blocked(x, y)          # -> a tag string, or None

Handing them a LIST of `(x, y, footprint)` tuples instead raises
`TypeError: 'list' object is not callable` INSIDE the march. Every one of
the three call sites in `suburb_hurricane_launch_script.py` sits inside a
broad `except Exception`, so the raise never reaches a human as a failure —
it degrades to a single warning line and the entire pass silently does
nothing:

  * CARS, first: "[hurricane] cars in water: piled 1 (0 moved)". Fixed in
    the launcher, and the fix left a comment there describing the trap.
  * HOUSES, second (D6, 2026-09-01, the neighbour-aware drift): the same
    list was passed to `shift_spec`/`collapse_spec`, printing
    "[hurricane] washaway FAILED on /World/stage/generated/inst/h_47:
    'list' object is not callable". Every `shifted` and `collapsed` house
    on the plate silently kept its original pose, so the surge did nothing
    to any building it was supposed to move — while the scene still
    reported those houses as shifted/collapsed in its tally.

The correct value is what `tornado.car_blockers(...)` RETURNS, which is the
predicate itself, not the lists you feed that function.

WHY A SOURCE READ FOR THE LAUNCHER. `suburb_hurricane_launch_script.py`
calls `isaacsim.SimulationApp(...)` at MODULE SCOPE, so it cannot be
imported without a licensed Isaac Sim — the same constraint
`test_hurricane_tornado_parity_launcher.py` documents and the same
technique it uses. The BEHAVIOUR half (that a list really does raise, and a
predicate really does not) is exercised for real against `washaway` itself,
which imports cleanly.

    python3 scene_gen/tests/test_hurricane_washaway_blockers.py
    pytest -q scene_gen/tests/test_hurricane_washaway_blockers.py
"""
import os
import random
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
_LAUNCHER = os.path.normpath(os.path.join(
    _SCENE_GEN, "..", "simulation", "isaac-sim", "launch_scripts",
    "suburb_hurricane_launch_script.py"))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import tornado as tn                            # noqa: E402
from disaster import washaway as wash                          # noqa: E402


def _src():
    with open(_LAUNCHER) as fh:
        return fh.read()


_HOUSES = [(0.0, 0.0, 21.1), (60.0, 0.0, 21.1), (0.0, 70.0, 12.8)]


# ---------------------------------------------------------------------------
# (a) the behaviour: a list raises, a predicate does not
# ---------------------------------------------------------------------------

def test_a_list_still_raises_so_this_test_is_not_vacuous():
    """The control, inline. If `washaway` is ever changed to accept a list
    directly this test fails LOUDLY and the guard below can be retired on
    purpose rather than rotting into a no-op."""
    for name, call in (
            ("shift_spec",
             lambda b: wash.shift_spec(1.8, 325.0, random.Random(1),
                                       x=0.0, y=0.0, own_fp_m=21.1,
                                       blocked=b)),
            ("collapse_spec",
             lambda b: wash.collapse_spec(1.8, random.Random(1),
                                          x=0.0, y=0.0, own_fp_m=21.1,
                                          blocked=b)),
            ("car_shift_spec",
             lambda b: wash.car_shift_spec(1.2, 325.0, random.Random(1),
                                           x=0.0, y=0.0, blocked=b))):
        try:
            call(list(_HOUSES))
        except TypeError as exc:
            assert "not callable" in str(exc), (name, str(exc))
            continue
        raise AssertionError(
            "%s accepted a LIST for `blocked=` — if that is now supported, "
            "delete this test deliberately; until then it is the bug that "
            "silently disabled the whole pass twice" % name)


def test_the_predicate_from_car_blockers_works_everywhere():
    """`tornado.car_blockers` RETURNS the callable the three spec functions
    want. All three must run clean with it and return a usable spec."""
    blocked = tn.car_blockers(standing=list(_HOUSES))

    s = wash.shift_spec(1.8, 325.0, random.Random(1), x=0.0, y=0.0,
                        own_fp_m=21.1, blocked=blocked)
    assert {"dx", "dy", "d_m", "arrested_by"} <= set(s)
    assert isinstance(s["d_m"], float)

    c = wash.collapse_spec(1.8, random.Random(1), x=0.0, y=0.0,
                           own_fp_m=21.1, blocked=blocked)
    assert {"dx", "dy", "arrested_by"} <= set(c)

    v = wash.car_shift_spec(1.2, 325.0, random.Random(1), x=0.0, y=0.0,
                            blocked=blocked)
    assert "arrested_by" in v


def test_the_march_actually_arrests_on_a_neighbour():
    """Not just "does not raise": the predicate must really stop the drift,
    or passing one would be indistinguishable from passing None and the D6
    "house flying onto another house" fix would still be inert."""
    near = [(14.0, 0.0, 21.1)]          # a neighbour squarely downflow
    blocked = tn.car_blockers(standing=near)
    stopped = wash.shift_spec(1.8, 0.0, random.Random(3), x=0.0, y=0.0,
                              own_fp_m=21.1, blocked=blocked)
    free = wash.shift_spec(1.8, 0.0, random.Random(3), x=0.0, y=0.0,
                           own_fp_m=21.1, blocked=None)
    assert stopped["arrested_by"] is not None, (
        "the drift marched straight through a neighbour 14 m downflow")
    assert stopped["d_m"] <= free["d_m"] + 1e-9, (
        "an arrested drift (%.2f m) travelled further than a free one "
        "(%.2f m)" % (stopped["d_m"], free["d_m"]))


# ---------------------------------------------------------------------------
# (b) the launcher passes a callable at BOTH call sites
# ---------------------------------------------------------------------------

def test_launcher_house_pass_builds_a_predicate():
    src = _src()
    m = re.search(r'_blocked\s*=\s*([^\n]*)', src)
    assert m, "the house pass no longer defines `_blocked`"
    assert "car_blockers" in m.group(1), (
        "the house pass builds `_blocked` as %r — `shift_spec`/"
        "`collapse_spec` call it as `blocked(x, y)`, so a list silently "
        "disables the whole surge-displacement pass" % (m.group(1).strip(),))


def test_launcher_car_pass_still_builds_a_predicate():
    """The first occurrence of this bug, kept under guard so the fix cannot
    be undone by someone tidying the two call sites into one style."""
    src = _src()
    assert re.search(r'_blockers\s*=\s*tn\.car_blockers\(', src), (
        "the car pass no longer builds its blocker predicate with "
        "tornado.car_blockers")
    assert "blocked=_blockers" in src


def test_no_call_site_passes_a_bare_list_comprehension():
    """The specific shape that broke twice: `blocked=` receiving something
    built inline as a list."""
    src = _src()
    # `(?<![A-Za-z0-9_])` so this does not also match the TAIL of
    # `_blocked = tn.car_blockers(...)`, which it did on first writing and
    # reported a false failure against correct code.
    for m in re.finditer(
            r'(?<![A-Za-z0-9_])blocked\s*=\s*([A-Za-z_][A-Za-z0-9_]*)', src):
        name = m.group(1)
        assert name in ("_blocked", "_blockers", "None", "blocked"), (
            "unexpected `blocked=%s`; check it is a callable" % name)
    assert not re.search(r'(?<![A-Za-z0-9_])blocked\s*=\s*\[', src), (
        "a call site passes a literal list to `blocked=`")


if __name__ == "__main__":
    fails = 0
    for _n, _f in sorted(globals().items()):
        if not _n.startswith("test_") or not callable(_f):
            continue
        try:
            _f()
            print("PASS  %s" % _n)
        except Exception as _e:
            fails += 1
            print("FAIL  %s: %s" % (_n, _e))
    print("ALL PASS" if not fails else "%d FAILURE(S)" % fails)
    sys.exit(1 if fails else 0)
