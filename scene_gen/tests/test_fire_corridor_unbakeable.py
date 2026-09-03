#!/usr/bin/env python3
"""test_fire_corridor_unbakeable.py — regression for the 2026-09-02
`unbakeable()` false-positive: `urban_fire_city.bake_kind()`'s own
`('slice', None)` refusal for an unregistered pack (`standalone/buildings/
...` and similar) reads "... fire_bake.KINDS=(...) has no bake kind for it
... must be REFUSED, not silently dropped" — its own substring "no bake"
false-matched `fire_corridor._BAKEABLE_HINTS`, so `unbakeable()` returned
`False` for a refusal that names itself as permanent in its own text. That
put every such placement into a corridor's `needs_bake: true` set with
`kind: null` instead of excluding it via `permanent_bad` before the
corridor placement search ran — measured 44 of 61 L1 records on one real
run, silently shipping most of a "burning" corridor undamaged while every
count downstream still looked plausible.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import fire_corridor as fc  # noqa: E402

# The exact message shape from urban_fire_city.bake_kind()'s ('slice', None)
# case for an unregistered pack (standalone/buildings/..., or any pack
# kit_substitute.route() cannot resolve to gac/dtc/aec/kit).
_UNREGISTERED_PACK_REFUSAL = (
    "kit_substitute.route() says 'slice' (a pack with real, unnamed parts) "
    "but fire_bake.KINDS=('gac', 'dtc', 'aec', 'kit') has no bake kind for "
    "it -- standalone/buildings/... and any other unregistered pack land "
    "here and must be REFUSED, not silently dropped: "
    "omniverse://.../standalone/buildings/intact/midrise/block_05/block_05.usdc"
)


def test_unregistered_pack_refusal_is_permanent():
    """The bug: this refusal's own "no bake kind for it" substring must not
    be read as one of the BAKEABLE hints — it is the exact opposite."""
    assert fc.unbakeable(_UNREGISTERED_PACK_REFUSAL) is True


def test_generic_missing_bake_stays_temporary():
    """A real "no bake exists yet" reason must still classify as bakeable —
    the fix must not turn every refusal permanent."""
    assert fc.unbakeable("no bake exists yet for this building") is False


def test_pack_blacklist_refusal_stays_permanent():
    assert fc.unbakeable(
        "refused as a firebreak, never picked for ignition") is True


def test_height_cap_refusal_stays_permanent():
    assert fc.unbakeable(
        "231.4 m tall -- taller than the fire-height cap") is True


def test_none_reason_is_permanent():
    assert fc.unbakeable(None) is True
