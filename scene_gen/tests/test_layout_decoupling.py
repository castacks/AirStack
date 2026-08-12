"""
Guardrails for the three-stage refactor (layout -> detail -> damage).

Run from `scene_gen/` with the system python3, which has `pxr`:

    python3 -m pytest tests/ -v

These are pure-Python layout tests: no Isaac Sim, no Nucleus, no stage.
A full run is a couple of seconds.

WHAT EACH TEST IS FOR
---------------------
* `test_deterministic`          — a seed means a scene. Guards the RNG plumbing.
* `test_matches_baseline`       — nothing moved that wasn't meant to. This is
                                  the refactor's "preserve behavior" contract;
                                  when a change is intended, re-baseline with
                                  `python3 tests/snapshot.py --write` and let
                                  the committed diff show what changed.
* `test_geometry_*`             — the street plan must not depend on severity.
                                  Already true today; kept so it stays true.
* `test_layout_*`               — every prop's pose must not depend on severity.
                                  **Expected to fail until Phase 2 lands.**

See `snapshot.py`'s docstring for why the signatures are split three ways.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import snapshot as S  # noqa: E402


# Severity sweeps at a fixed (preset, seed). Each must produce one layout.
SWEEPS = [
    ("earthquake", 42, [0.0, 0.4, 0.8]),
    ("tornado", 42, [0.0, 0.8]),
]


@pytest.mark.parametrize("preset,seed,severity", S.DEFAULT_CASES)
def test_matches_baseline(preset, seed, severity):
    """The committed snapshot still describes what the generator produces."""
    try:
        want = S.load(preset, seed, severity)
    except FileNotFoundError:
        pytest.skip(f"no baseline for {S.case_name(preset, seed, severity)}; "
                    "run `python3 tests/snapshot.py --write`")
    got = S.snapshot(preset, seed, severity)
    for key in ("geometry", "layout", "full"):
        assert got["digests"][key] == want["digests"][key], (
            f"{key} signature changed for "
            f"{S.case_name(preset, seed, severity)}. If intended, re-run "
            f"`python3 tests/snapshot.py --write` and commit the diff.")


def test_deterministic():
    """Same preset, seed and severity -> byte-identical scene, twice running."""
    a = S.snapshot("earthquake", 42, 0.6)
    b = S.snapshot("earthquake", 42, 0.6)
    assert a["digests"] == b["digests"]


def test_seed_actually_varies_the_scene():
    """Guards against the snapshot silently comparing constants."""
    a = S.snapshot("earthquake", 42, 0.6)
    b = S.snapshot("earthquake", 7, 0.6)
    assert a["digests"]["geometry"] != b["digests"]["geometry"]


@pytest.mark.parametrize("preset,seed,severities", SWEEPS)
def test_geometry_invariant_to_severity(preset, seed, severities):
    """The street plan is a function of locale and seed alone.

    Already holds — blocks and road corridors are laid out before any damage
    knob is read. This test exists to keep it that way through the refactor.
    """
    base = S.snapshot(preset, seed, severities[0])["digests"]["geometry"]
    for sev in severities[1:]:
        got = S.snapshot(preset, seed, sev)["digests"]["geometry"]
        assert got == base, (
            f"{preset} street plan changed between severity {severities[0]} "
            f"and {sev} — layout must not depend on damage")


@pytest.mark.xfail(
    strict=True,
    reason="Requirement 4a, not yet implemented. Damage currently perturbs "
           "layout two ways: _anchor_ok (scene_generator.py:1613) gates "
           "building anchoring on local damage intensity, and — the larger "
           "effect — damage decisions draw from the same RNG stream as "
           "placement, so every subsequent prop shifts. Phase 2 fixes both. "
           "Remove this marker when it starts passing.")
@pytest.mark.parametrize("preset,seed,severities", SWEEPS)
def test_layout_invariant_to_severity(preset, seed, severities):
    """Raising severity must add damage, not move the pristine scene.

    This is the headline requirement: a locale and a seed fully specify the
    layout, so a severity sweep yields the same city at different damage
    levels — which is what makes search-algorithm comparisons across severity
    mean anything.
    """
    base = S.snapshot(preset, seed, severities[0])["digests"]["layout"]
    for sev in severities[1:]:
        got = S.snapshot(preset, seed, sev)["digests"]["layout"]
        assert got == base, (
            f"{preset} layout changed between severity {severities[0]} and "
            f"{sev} — damage must be additive")
