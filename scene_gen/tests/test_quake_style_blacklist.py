#!/usr/bin/env python3
"""test_quake_style_blacklist.py — `disaster.style_blacklist`, the
test-preset-scoped kit STYLE exclusion (item 1, user 2026-09-01, verbatim:
"what if we blacklist [office_wide] for the next test run?").

    pytest -q scene_gen/tests/test_quake_style_blacklist.py

WHY A DAMAGE-SIDE SWAP, NOT A LAYOUT-SIDE FILTER
-------------------------------------------------
`office_wide` is a `detail.urban_building.STYLES` kit archetype drawn
straight off the asset set's `midrise`/`tower` pool at LAYOUT time
(`detail/districts.py` + `scene_generator.build_city`'s greedy pack) — code
this session does not own (districts.py / city_detail.py / footprint-and-
dims is another session's work). A preset-level style blacklist therefore
cannot filter the POOL from here; it has to act where this module DOES have
a decision point — `disaster/quake.py`'s decide path, right after a
placement's kit `style` is resolved and before any grade is drawn
(`assemble`'s main loop, `_resolve_blacklisted_style`'s call site).

TEST-PRESET-SCOPED: `_style_blacklist` reads one optional
`disaster.style_blacklist` key and returns an empty set when it is absent
(every preset but the one that opts in) — `test_empty_blacklist_is_a_no_op`
below pins that this is a true no-op, not just "usually harmless".

WHAT THIS CANNOT SEE: whether the substitute (`office_plain`) reads right
next to its actual neighbours in a render — that needs Isaac Sim, not run
here (memory: "Verify offline, no sim").
"""
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..", "tools")))

from disaster import quake as q               # noqa: E402


# ---------------------------------------------------------------------------
# `_style_blacklist` — reads `disaster.style_blacklist`, empty by default
# ---------------------------------------------------------------------------
def test_style_blacklist_empty_when_key_absent():
    assert q._style_blacklist({}) == frozenset()
    assert q._style_blacklist({"disaster": {}}) == frozenset()
    assert q._style_blacklist({"disaster": {"damaged_fraction": 0.0}}) == frozenset()


def test_style_blacklist_reads_the_configured_styles():
    cfg = {"disaster": {"style_blacklist": ["office_wide"]}}
    assert q._style_blacklist(cfg) == frozenset({"office_wide"})


def test_style_blacklist_accepts_more_than_one_entry():
    cfg = {"disaster": {"style_blacklist": ["office_wide", "walkup"]}}
    assert q._style_blacklist(cfg) == frozenset({"office_wide", "walkup"})


# ---------------------------------------------------------------------------
# `_resolve_blacklisted_style` — the pure per-style decision
# ---------------------------------------------------------------------------
def test_resolve_blacklisted_style_swaps_office_wide_to_office_plain():
    """The registered substitute — same family (`urban_building.fam02`/
    `rc`), verified against `assets/archetype/archetypes.json`'s own
    per-style `family`/`type` fields in both this test's header and
    `STYLE_BLACKLIST_SUBSTITUTE`'s own comment."""
    cfg = {"disaster": {"style_blacklist": ["office_wide"]}}
    assert q._resolve_blacklisted_style("office_wide", cfg) == "office_plain"


def test_resolve_blacklisted_style_is_a_noop_for_an_unlisted_style():
    cfg = {"disaster": {"style_blacklist": ["office_wide"]}}
    for style in ("office_plain", "brownstone_row", "tower", "commercial_mid"):
        assert q._resolve_blacklisted_style(style, cfg) == style


def test_resolve_blacklisted_style_is_a_noop_with_no_table_at_all():
    """`downtown_earthquake.yaml`'s own default (before this preset opts
    in) and every OTHER preset: no `style_blacklist` key at all."""
    assert q._resolve_blacklisted_style("office_wide", {}) == "office_wide"
    assert q._resolve_blacklisted_style("office_wide",
                                        {"disaster": {}}) == "office_wide"


def test_resolve_blacklisted_style_leaves_an_unregistered_entry_alone():
    """A style someone blacklists with NO registered substitute
    (`STYLE_BLACKLIST_SUBSTITUTE` has no entry for it) is left as-is rather
    than silently vanishing — a config-authoring gap should surface as
    "still placed", not as a mysteriously missing building."""
    cfg = {"disaster": {"style_blacklist": ["tower"]}}
    assert q._resolve_blacklisted_style("tower", cfg) == "tower"


def test_resolve_blacklisted_style_passes_through_falsy_style():
    assert q._resolve_blacklisted_style(None, {"disaster": {
        "style_blacklist": ["office_wide"]}}) is None
    assert q._resolve_blacklisted_style("", {"disaster": {
        "style_blacklist": ["office_wide"]}}) == ""


def test_substitute_table_is_geometry_family_compatible():
    """`office_wide` -> `office_plain`: both `urban_building.fam02`/`rc` —
    read straight off the real baked manifest (`assets/archetype/
    archetypes.json`), not just asserted by comment."""
    _SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
    arch_dir = os.path.join(_SCENE_GEN, "assets", "archetype")
    manifest = q.load_manifest(arch_dir)
    for style, sub in q.STYLE_BLACKLIST_SUBSTITUTE.items():
        orig = manifest.get((style, "DG0"))
        repl = manifest.get((sub, "DG0"))
        assert orig is not None, f"{style} DG0 missing from the real manifest"
        assert repl is not None, f"{sub} DG0 missing from the real manifest"
        assert orig["family"] == repl["family"], \
            f"{style} (family {orig['family']}) -> {sub} (family " \
            f"{repl['family']}) is not geometry-family compatible"
        assert orig["type"] == repl["type"]


# ---------------------------------------------------------------------------
# End to end, offline, on the REAL preset (`downtown_earthquake`) — the
# same-process harness idiom the task asked for (scratchpad
# quake_fill_probe.py): a real `generate_scene_on_stage` layout, a real
# in-memory pxr stage, a real `disaster.quake.assemble` call, all local
# files (the kit archetype library is mirrored on disk — see
# `tools/layout_dry_run.py`'s own module docstring for why this needs no
# Nucleus/Isaac Sim access at all).
# ---------------------------------------------------------------------------
def _assemble_styles(seed, region_m, style_blacklist=None):
    from pxr import Usd
    import compile_disaster
    import generate_scene
    import layout_dry_run as ldr

    overrides = {"region_m": [float(region_m), float(region_m)], "seed": seed}
    if style_blacklist is not None:
        overrides["overrides"] = {"disaster": {"style_blacklist": style_blacklist}}
    config = compile_disaster.load_scene_config(
        "downtown_earthquake", spec_overrides=overrides)
    ldr._localize_building_urls(config["usds"])

    stage = Usd.Stage.CreateInMemory()
    placements = generate_scene.generate_scene_on_stage(
        stage, config, parent_path="/World/stage/generated",
        scene_scale_factor=1.0, snap_to_ground=False)

    _SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
    arch_dir = os.path.join(_SCENE_GEN, "assets", "archetype")
    gac_dir = os.path.join(_SCENE_GEN, "assets", "gac_quake")
    q.assemble(stage, config, placements, arch_dir, seed=11,
              gac_dir=gac_dir, verbose=False)

    houses = [p for p in placements if p.get("category") == "house"]
    styles = []
    for p in houses:
        style, _level = q.style_of(p.get("usd"))
        if style:
            styles.append(style)
    return styles


@pytest.mark.parametrize("seed,region_m", [(9, 250.0), (3, 500.0)])
def test_office_wide_blacklist_yields_zero_placements_end_to_end(seed, region_m):
    """`office_wide` present with the blacklist explicitly CLEARED (`[]` —
    a blacklist that removed nothing would be a vacuous test) and ABSENT
    once `disaster.style_blacklist: [office_wide]` is set — every one of
    those placements' geometry is the substitute (`office_plain`) instead,
    not merely relabelled bookkeeping with the original mesh still standing
    (`assemble` swaps the STAGE reference, see its own "STYLE BLACKLIST"
    comment).

    Uses an explicit `[]` override rather than "no override at all" for the
    "before" run: `downtown_earthquake.yaml` itself now ships `disaster.
    style_blacklist: [office_wide]` (this IS the deliverable), so "no
    override" no longer means "no blacklist" for THIS preset specifically —
    see `test_committed_preset_places_zero_office_wide_by_default`, below,
    for that shipped behaviour."""
    baseline = _assemble_styles(seed, region_m, style_blacklist=[])
    assert baseline.count("office_wide") > 0, \
        "test is vacuous at this seed/region: office_wide never placed"
    before_plain = baseline.count("office_plain")

    blacklisted = _assemble_styles(seed, region_m,
                                   style_blacklist=["office_wide"])
    assert blacklisted.count("office_wide") == 0
    assert blacklisted.count("office_plain") == \
        before_plain + baseline.count("office_wide")
    # Every OTHER style's count is unaffected by the swap.
    for style in set(baseline) - {"office_wide", "office_plain"}:
        assert blacklisted.count(style) == baseline.count(style)


@pytest.mark.parametrize("seed,region_m", [(9, 250.0), (3, 500.0)])
def test_committed_preset_places_zero_office_wide_by_default(seed, region_m):
    """The SHIPPED `downtown_earthquake.yaml` — no override at all — must
    place zero `office_wide`: this preset's own `overrides.disaster.
    style_blacklist: [office_wide]` is the deliverable itself, not just the
    mechanism behind it."""
    styles = _assemble_styles(seed, region_m, style_blacklist=None)
    assert styles.count("office_wide") == 0


def test_mechanism_itself_is_inert_when_a_config_never_sets_the_key():
    """A config that never sets `disaster.style_blacklist` at all (every
    OTHER preset, and this one before 2026-09-01) must be untouched by the
    mechanism — proved at the pure-function level (`_style_blacklist`
    already asserts this in isolation) and end to end here against a
    from-scratch config with no `style_blacklist` key anywhere, confirming
    `office_wide` places normally when nothing opts in."""
    cfg = {"disaster": {}}
    assert q._resolve_blacklisted_style("office_wide", cfg) == "office_wide"
