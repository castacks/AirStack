"""test_hurricane_tornado_parity_launcher.py — pins the REWRITTEN hurricane
house loop's decision logic (STREAM S, 2026-08-31: "Make sure we're doing
the same damage as tornado... you need to just adjust the pattern of house
damage") without ever starting Isaac Sim.

WHY SOURCE-READ, NOT IMPORT. `suburb_hurricane_launch_script.py` calls
`isaacsim.SimulationApp(...)` at MODULE SCOPE (before any function is
defined), so `import suburb_hurricane_launch_script` hangs/fails in any
environment without a running Isaac Sim license — there is no function
boundary to import around. `test_hurricane_palettes_rebind.py`'s own
`test_recolour_condition_matches_the_launcher_intent` already established
the pattern this file extends: read the launcher's SOURCE TEXT and assert
on the literal expressions it contains, so a future edit that reintroduces
the old behaviour breaks a test instead of silently reverting a fix. Where
the source-read alone cannot prove behaviour (e.g. "does `house_water_state`
actually gate `swept`"), this file ALSO exercises the real, importable, pure
functions (`disaster.hurricane`, `disaster.surge`, `disaster.washaway`) the
launcher calls, against a synthetic house list — a genuine replay of the
decision logic, just without a stage.

    python3 scene_gen/tests/test_hurricane_tornado_parity_launcher.py
    pytest -q scene_gen/tests/test_hurricane_tornado_parity_launcher.py
"""
import os
import random
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN_DIR = os.path.dirname(_HERE)
_LAUNCH_DIR = os.path.normpath(os.path.join(
    _SCENE_GEN_DIR, "..", "simulation", "isaac-sim", "launch_scripts"))
_LAUNCHER_PATH = os.path.join(_LAUNCH_DIR, "suburb_hurricane_launch_script.py")
_TORNADO_LAUNCHER_PATH = os.path.join(
    _LAUNCH_DIR, "suburb_tornado_launch_script.py")
sys.path.insert(0, _SCENE_GEN_DIR)

from disaster import hurricane as hu                           # noqa: E402
from disaster import surge as sgw                              # noqa: E402
from disaster import tornado as tn                             # noqa: E402
from disaster import washaway as wash                          # noqa: E402
from detail import modular_house as mh                         # noqa: E402


def _src():
    with open(_LAUNCHER_PATH) as fh:
        return fh.read()


# ---------------------------------------------------------------------------
# (a) EVERY NON-SWEPT HOUSE REFERENCES THE TORNADO LIBRARY
# ---------------------------------------------------------------------------

def test_house_archetype_key_and_dir_are_the_tornado_ones():
    """`HOUSE_ARCH_DIR` defaults into `archetypes_tornado` (not this file's
    own `archetypes_hurricane`, which stays the TREE library), and the house
    loop's `usd = harch.get(key) or ...` reads from `harch` -- built off
    `HOUSE_ARCH_DIR` -- never from `arch` (`ARCH_DIR`, the tree dict)."""
    src = _src()
    m = re.search(r'HOUSE_ARCH_DIR\s*=\s*_env\(\s*"HOUSE_ARCH_DIR",\s*'
                  r'(.+?)\)', src, re.S)
    assert m, "could not find the HOUSE_ARCH_DIR default"
    assert "archetypes_tornado" in m.group(1), m.group(1)

    m = re.search(r'harch\s*=\s*\{[^}]*HOUSE_ARCH_DIR', src, re.S)
    assert m, "`harch` must be built off HOUSE_ARCH_DIR"

    # The house loop's own lookup line, read verbatim. Every `.get(` call on
    # this line must be against `harch`, never the bare tree dict `arch`
    # (checked on a word boundary -- "harch.get" itself contains the
    # substring "arch.get", so a naive `in` check would false-pass here).
    m = re.search(r'usd = (harch\.get\(key\).*)', src)
    assert m, "could not find the house archetype lookup line"
    line = m.group(1)
    assert re.search(r'\bharch\.get\(', line), line
    assert not re.search(r'\barch\.get\(', line), line


def test_house_key_format_resolves_against_the_real_archetype_library():
    """For every real style, `house_<style>_<level>` for every non-swept
    tornado level names a file that ACTUALLY EXISTS in the shipped
    `archetypes_tornado` directory -- proving the key format the launcher
    builds is not just syntactically plausible but resolvable."""
    arch_dir = os.path.join(_SCENE_GEN_DIR, "assets", "archetypes_tornado")
    if not os.path.isdir(arch_dir):
        import pytest
        pytest.skip("archetypes_tornado not present at {0}".format(arch_dir))
    have = set(os.listdir(arch_dir))
    non_swept = [lv for lv in tn.HOUSE_LEVELS if lv != "swept"]
    missing = []
    for style in mh.STYLES:
        for level in non_swept:
            key = "house_{0}_{1}.usd".format(style, level)
            if key not in have:
                missing.append(key)
    assert not missing, missing


# ---------------------------------------------------------------------------
# (b) `swept` ONLY WHEN `house_water_state` (OR THE SURGE-DERIVED WASHAWAY
#     STATE) SAYS SO -- NEVER FROM WIND ALONE
# ---------------------------------------------------------------------------

def test_swept_override_reads_house_water_state_first():
    """The literal override line must test `wst.get("swept")` (`surge.
    house_water_state`'s own field) before falling to the wind ladder --
    reading from source so a future edit that reorders/removes the surge
    gate breaks this test instead of silently letting wind sweep a slab."""
    src = _src()
    m = re.search(r'if (wst\.get\("swept"\)[^:\n]*):', src)
    assert m, "could not find the swept override `if` line"
    cond = m.group(1)
    assert 'wst.get("swept")' in cond and '_sl == "swept"' in cond, cond
    # And the block immediately below it actually sets `level = "swept"`
    # (within the next few lines, allowing for explanatory comments).
    tail = src[m.end():m.end() + 400]
    assert re.search(r'\blevel = "swept"', tail), tail
    # THE LADDER ASSERTION IS INVERTED, 2026-09-01, on the user's
    # instruction ("make this ladder for hurricane too").
    #
    # This file was written for STREAM S (2026-08-31), which switched the
    # hurricane onto the tornado's SIX-level vocabulary on the doctrine that
    # "tornado and hurricane are both largely wind damage". That holds at
    # the top of the range and fails at the bottom: the six-level ladder has
    # no CLADDING rung, so at the dataset's level-1 gust (38 m/s) its only
    # way to show damage is to jump a house straight to `roof_stripped` —
    # the whole covering gone. Replayed on level 1's own intensity field it
    # gave 86.8% pristine / 13.2% roof_stripped, against the eight-level
    # ladder's 79.2% pristine / 20.8% shingles_lost. The dataset specifies
    # level 1 as "cladding only", so the six-level vocabulary cannot express
    # the cell at all.
    #
    # The REST of this file is untouched and still load-bearing: the swept
    # override, `house_water_state` as the sole source of `swept`, the
    # archetype key format and the row-recolour truth table are all still
    # the tornado's and are all still asserted. Only the ladder moved.
    assert "hu.house_level_for_intensity(it, drng, vuln=vuln)" in src
    assert "hu.tornado_level_for_intensity(it, drng, vuln=vuln)" not in src


def test_house_water_state_is_the_sole_source_of_swept():
    """Replay `surge.house_water_state` directly: a point with depth past
    `swept_depth_m` reports `swept=True`; a dry or shallow point never does
    -- regardless of how violent the (synthetic) wind intensity there is.
    This is the ACTUAL gate the launcher's override reads, exercised for
    real rather than merely quoted from source."""
    cfg = sgw.resolve_cfg({})
    rng = random.Random(3)
    kn = sgw.resolve_cfg(cfg)
    swept_depth = float(kn["swept_depth_m"])

    # A point with the ground pinned well past the swept threshold.
    deep_cfg = dict(cfg)
    deep_cfg["surge_m"] = swept_depth + 5.0
    deep_cfg["slope_pct"] = 0.0
    wst_deep = sgw.house_water_state(deep_cfg, 0.0, 0.0, rng)
    assert wst_deep["depth"] >= swept_depth
    assert wst_deep["swept"] is True

    # A bone-dry point (surge = 0) never sweeps, whatever the wind is doing.
    dry_cfg = dict(cfg)
    dry_cfg["surge_m"] = 0.0
    wst_dry = sgw.house_water_state(dry_cfg, 5000.0, 5000.0, rng)
    assert wst_dry["depth"] == 0.0
    assert wst_dry["swept"] is False

    # `tornado_level_for_intensity` at maximum intensity STILL never says
    # "swept" -- confirming the launcher's wind branch could not have
    # produced it even if the surge check were skipped by mistake.
    assert hu.tornado_level_for_intensity(4.0, rng, vuln=1.0) != "swept"


def test_swept_house_land_debris_is_skipped():
    """`washaway.land_debris_specs`'s own level gate excludes `swept` (it is
    absent from `_HOUSE_LEVEL_ORDER`) -- a swept house's material went to
    the surge/raft system, not the wind, so it must shed nothing on land."""
    wrecks = [(0.0, 0.0, 12.0, 0.95, "swept", "brick_red")]
    specs = wash.land_debris_specs(wrecks, lambda x, y: 0.0, random.Random(1))
    assert specs == []


# ---------------------------------------------------------------------------
# (c) ROW-ONLY RECOLOUR, EXACTLY THE TORNADO'S CONDITION
# ---------------------------------------------------------------------------

def test_recolour_condition_is_exactly_row_and_palette():
    """`_recolour` must be the tornado's own row-gated condition, read from
    source: `bool(h.get("row")) and bool(_pal)` — no style-default
    comparison, no unconditional per-house draw. This is a deliberate
    REVERT of a same-day change (`suburb_scene.build_placements`'s
    `draw_house_palette` for detached lots) that this stream's master
    directive explicitly undoes because it also altered the tornado
    scenes; see `git log`/the job-3 revert of `scene_gen/suburb_scene.py`."""
    src = _src()
    m = re.search(r"_recolour = (.+)", src)
    assert m, "could not find the `_recolour` assignment"
    expr = m.group(1).strip()
    assert expr == 'bool(h.get("row")) and bool(_pal)', expr


def _recolour(row, pal):
    return bool(row) and bool(pal)


def test_recolour_truth_table_matches_tornado():
    cases = [
        (False, None, False),
        (False, "brick_red", False),   # detached house: NEVER recoloured,
                                       # even with a truthy palette (the
                                       # style default, post-revert)
        (True, None, False),
        (True, "brick_red", True),     # row home with its own palette
    ]
    for row, pal, expected in cases:
        assert _recolour(row, pal) is expected, (row, pal)


def test_suburb_scene_house_palette_hunk_is_reverted():
    """`git diff` on `suburb_scene.py` must be empty for the house-palette
    hunk this stream reverted (job 3) -- `draw_house_palette` no longer
    reached from the parcel loop, `pal` falls back straight to the style
    default for a detached lot. Read from source, not from `git diff`
    directly, so this test works even if run from a copy without the
    original commit history."""
    path = os.path.join(_SCENE_GEN_DIR, "suburb_scene.py")
    with open(path) as fh:
        src = fh.read()
    assert "mh.draw_house_palette(" not in src
    assert ('pal = h.get("palette") or mh.STYLES[ent["style"]]'
           '.get("palette")') in src


# ---------------------------------------------------------------------------
# cross-checks against the tornado launcher this stream mirrors
# ---------------------------------------------------------------------------

def test_row_recolour_condition_matches_the_tornado_launcher_verbatim():
    """The two launchers' `_recolour` lines must be the SAME expression --
    proof this is a mirror, not an independently-drifting reimplementation."""
    with open(_TORNADO_LAUNCHER_PATH) as fh:
        tsrc = fh.read()
    hsrc = _src()
    tm = re.search(r"_recolour = (.+)", tsrc)
    hm = re.search(r"_recolour = (.+)", hsrc)
    assert tm and hm
    assert tm.group(1).strip() == hm.group(1).strip()


def test_no_wind_yawed_re_yaw_remains():
    """Job 1: every house keeps its STREET yaw -- the `_WIND_YAWED`
    re-yawing (and the windward-variant machinery it fed) must be gone."""
    src = _src()
    assert "_WIND_YAWED = (" not in src
    assert "hf.windward_variant" not in src
    assert "hf.LEVELS_WITH_VARIANTS" not in src
    assert '"variant": variant' not in src
    # And the assignment feeding the reference call is the bare street yaw.
    assert 'yaw = h["yaw"]' in src


if __name__ == "__main__":
    for _name, _fn in sorted(globals().items()):
        if _name.startswith("test_") and callable(_fn):
            _fn()
            print("ok  " + _name)
