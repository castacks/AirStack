"""test_bench_building_check.py — pins the fix for the 2026-09-01 v6 render
bug: "people FLOATING IN EMPTY SKY at x=180 ... the fourth building never
composed" because `fire_people_bench_launch_script.py`'s composition loop
iterated a hardcoded 3-entry role table (`BENCH_BUILDING_INFO`, formerly
`BENCH_BUILDINGS`) instead of the manifest, so a 4th manifest record with
no entry in that table was silently skipped while the people solve — which
reads the manifest independently — placed figures at its cell anyway.

WHY THIS TEST DUPLICATES RATHER THAN IMPORTS THE FUNCTION UNDER TEST. The
launch script starts an actual Isaac Sim session as an IMPORT-TIME SIDE
EFFECT (`from isaacsim import SimulationApp` followed immediately by
`simulation_app = SimulationApp(...)` at module scope, before any function
or `if __name__` guard) — there is no way to import
`fire_people_bench_launch_script.py` from a host-side test without
launching Isaac, which this project's standing rule forbids for anything
but a real bench relaunch. `missing_composed_buildings` is reproduced here
VERBATIM (it is pure `set` arithmetic with no `pxr`/`omni` dependency in
the source file either, by design, precisely so it COULD be tested this
way) and this file's own module-level assertion re-parses the launch
script's source and extracts that one function's body via `ast`, so a
future edit to the real function that this copy has not been updated to
match fails LOUDLY here rather than the two silently drifting apart.
"""

import ast
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_LAUNCH_SCRIPT = os.path.normpath(os.path.join(
    _HERE, "..", "..", "simulation", "isaac-sim", "launch_scripts",
    "fire_people_bench_launch_script.py"))


def missing_composed_buildings(composed_is, people_recs):
    """VERBATIM COPY of `fire_people_bench_launch_script.missing_composed_
    buildings` — see this module's own docstring for why a copy rather than
    an import. `test_00_source_matches_the_launch_script` is what keeps the
    two from drifting apart."""
    composed = set(composed_is)
    referenced = {r["building_i"] for r in people_recs
                 if r.get("building_i") is not None}
    return sorted(referenced - composed)


def _non_docstring_body(fn):
    """A function's body with a leading docstring `Expr` stripped — the two
    copies' DOCSTRINGS are allowed to read differently (one explains "here
    is the real function", the other "here is why this is a copy"); it is
    the LOGIC that must be identical."""
    body = fn.body
    if (body and isinstance(body[0], ast.Expr)
            and isinstance(body[0].value, ast.Constant)
            and isinstance(body[0].value.value, str)):
        body = body[1:]
    return ast.dump(ast.Module(body=body, type_ignores=[]))


def test_00_source_matches_the_launch_script():
    """If `missing_composed_buildings` in the real launch script ever
    changes, this copy has to change WITH it — extract its source via `ast`
    and diff the normalised body (docstring excluded, see
    `_non_docstring_body`) against this file's own copy rather than
    trusting the two to stay in sync by convention."""
    assert os.path.isfile(_LAUNCH_SCRIPT), \
        "fixture drifted: launch script moved — update _LAUNCH_SCRIPT"
    tree = ast.parse(open(_LAUNCH_SCRIPT).read())
    real = next((n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef)
                and n.name == "missing_composed_buildings"), None)
    assert real is not None, (
        "missing_composed_buildings not found in the launch script — "
        "renamed or removed; this test's own copy is now testing nothing")
    here_tree = ast.parse(open(__file__).read())
    mine = next(n for n in ast.walk(here_tree)
               if isinstance(n, ast.FunctionDef)
               and n.name == "missing_composed_buildings")
    assert _non_docstring_body(real) == _non_docstring_body(mine), (
        "missing_composed_buildings in the launch script no longer matches "
        "this file's copy — update BOTH the copy above and this test's "
        "expectations, not just the launch script")


def test_01_the_v6_bug_itself_a_forged_4record_manifest_vs_a_3role_table():
    """THE EXACT SHAPE OF THE BUG: a people solve with figures on 4
    buildings (i=273/257/259/274, the real bench trio plus the window-
    showcase pick), composed against a role table that only knows the
    first three (the real `BENCH_BUILDING_INFO` before the fix, or any
    future edit that adds a manifest record and forgets to give it a
    role/derivation path). `composed_is` here models "what the OLD
    3-entry-table-driven loop would have composed" — i=274 never appears
    in it, exactly as it never appeared in the v6 render's composed set."""
    people_recs = [
        {"cls": "window", "building_i": 274, "pose": "lean_window"},
        {"cls": "window", "building_i": 274, "pose": "lean_window"},
        {"cls": "roof", "building_i": 274, "pose": "idle"},
        {"cls": "roof", "building_i": 257, "pose": "idle"},
        {"cls": "casualty_apron", "building_i": 259, "pose": "lying_supine"},
        {"cls": "roof_debris", "building_i": 273, "pose": "lying_curled_l"},
    ]
    composed_is_old_bug = [273, 257, 259]          # i=274 never composed
    missing = missing_composed_buildings(composed_is_old_bug, people_recs)
    assert missing == [274], missing
    n_orphaned = sum(1 for r in people_recs if r["building_i"] in missing)
    assert n_orphaned == 3, n_orphaned            # the 2 window + 1 roof

    # THE FIX: once the composition loop iterates the manifest (not the
    # role table) and i=274 actually composes, the same check on the SAME
    # people solve passes clean.
    composed_is_fixed = [273, 257, 259, 274]
    assert missing_composed_buildings(composed_is_fixed, people_recs) == []


def test_02_records_with_no_building_reference_are_never_a_miss():
    """A class that carries no `building_i` at all (there is none in this
    planner today, but the check must not assume that stays true) is not
    evidence of a missing building — only a NAMED, unresolved one is."""
    people_recs = [{"cls": "evacuee", "building_i": None},
                  {"cls": "evacuee"}]               # key absent entirely
    assert missing_composed_buildings([], people_recs) == []


def test_03_extra_composed_buildings_are_not_a_miss():
    """A building that composed but nobody's `building_i` references (the
    bench's own case whenever a pick draws zero of its own people) is not
    a failure — the check is one-directional: referenced-but-not-composed
    only."""
    people_recs = [{"cls": "roof", "building_i": 257}]
    assert missing_composed_buildings([257, 273, 259, 274], people_recs) == []


def test_04_multiple_missing_buildings_all_named_and_sorted():
    people_recs = [{"cls": "window", "building_i": 5},
                  {"cls": "roof", "building_i": 2},
                  {"cls": "roof_debris", "building_i": 8}]
    assert missing_composed_buildings([2], people_recs) == [5, 8]
