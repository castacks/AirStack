#!/usr/bin/env python3
"""test_aec_variety.py — the offline gate for per-unit brownstone variance.

    python3 -m pytest -q scene_gen/tests/test_aec_variety.py

No stage, no Kit, no GPU: everything above `author_variety` in
`disaster/aec_variety.py` is arithmetic over the unit index, and that is the
part that decides whether a terrace reads as twelve houses or as one house
twelve times.

WHAT IT GUARDS

  1. NO TWO ADJACENT UNITS SHARE A BRICK TONE. This is the whole point --
     `Reference_Brownstone12Row` is one 6.67 m facade merged twelve times, so
     without this every neighbour is identical by construction.
  2. DETERMINISM, and specifically determinism that does not depend on
     PYTHONHASHSEED. The palette is keyed with `zlib.crc32`, not `hash()`;
     this pipeline has desynced on interpreter hash salting before.
  3. TWO DIFFERENT ROWS GET DIFFERENT SEQUENCES. Otherwise the repetition
     just moves up a level: every brownstone row in the city would carry the
     same twelve colours in the same order.
  4. The mesh classifier routes a door to the door colour and a facade to the
     brick tone, and does not claim meshes it has no business tinting.
"""
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import aec_variety as av  # noqa: E402


@pytest.mark.parametrize("n", [2, 5, 6, 8, 10, 11, 12])
def test_no_adjacent_unit_shares_a_brick_tone(n):
    """Every shipped row length. 2 is the tightest case: with one neighbour
    the exclusion has to still hold."""
    pal = av.assign_palette("/World/row_a", n)
    assert len(pal) == n
    for a, b in zip(pal, pal[1:]):
        assert a["brick"] != b["brick"], (
            "units %d and %d share brick tone %r — adjacent identical facades "
            "are the exact artefact this exists to remove"
            % (a["unit"], b["unit"], a["brick"]))


def test_palette_is_deterministic():
    a = av.assign_palette("/World/row_a", 8)
    b = av.assign_palette("/World/row_a", 8)
    assert a == b


def test_palette_does_not_depend_on_interpreter_hash_salt():
    """`hash()` is salted per interpreter; `zlib.crc32` is not. A palette that
    moved with PYTHONHASHSEED would make a row look different between the
    layout stage and the bake stage, which is the desync class this pipeline
    has already paid for."""
    import subprocess
    code = (
        "import sys; sys.path.insert(0, %r);"
        "from disaster import aec_variety as av;"
        "print([p['brick'] for p in av.assign_palette('/World/row_a', 8)])"
        % _SCENE_GEN)
    outs = set()
    for salt in ("0", "1", "12345"):
        env = dict(os.environ, PYTHONHASHSEED=salt)
        outs.add(subprocess.check_output([sys.executable, "-c", code],
                                         env=env).decode().strip())
    assert len(outs) == 1, "palette moved with PYTHONHASHSEED: %s" % outs


def test_two_rows_get_different_sequences():
    a = [p["brick"] for p in av.assign_palette("/World/row_a", 10)]
    b = [p["brick"] for p in av.assign_palette("/World/row_b", 10)]
    assert a != b, ("two rows produced the same colour sequence — the "
                    "repetition would just move from unit to row")


def test_a_long_row_uses_several_tones():
    """A twelve-unit row cycling two tones would satisfy the adjacency rule
    and still read as a chequerboard."""
    pal = av.assign_palette("/World/row_long", 12)
    assert len({p["brick"] for p in pal}) >= 4


def test_classifier_routes_meshes():
    assert av.classify("Walls_ExteriorFacade") == "facade"
    assert av.classify("Walls_Exterior_123") == "facade"
    assert av.classify("Doors_441396_18939") == "door"
    assert av.classify("Windows_88") == "trim"
    assert av.classify("Railings_588755_0") == "trim"
    # a door must NOT be swallowed by the facade rule even though the row
    # asset names some doors inside wall categories
    assert av.classify("Walls_Exterior_Door_2") == "door"
    # things that are not skin at all
    assert av.classify("Structural_Framing_12") is None
    assert av.classify("Lighting_Fixtures_3") is None


def test_tones_are_multipliers_not_absolute_colours():
    """The brick tint multiplies the MDL brick rather than replacing it, so
    every tone has to sit near 1.0 — a value near 0 would render the facade
    black and lose the courses and mortar the map carries."""
    for name, rgb in av.BRICK_TONES:
        for c in rgb:
            assert 0.6 <= c <= 1.3, "%s has an out-of-range channel %.2f" % (
                name, c)
