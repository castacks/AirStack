"""Stage A's addressing and plan — the half that needs no Isaac Sim.

The bake itself needs Kit (fracture is CPU, settle is PhysX), so it cannot run
here. What CAN be checked on the host is the part that silently ruins a bake
you have already paid an hour for: whether the baker and the assembler agree on
what a file is called, and whether the plan covers everything the assembler
will ask for. Both are pure functions.
"""

import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import compile_disaster as cd                                   # noqa: E402
from archetypes import library as lib                           # noqa: E402
from archetypes import plan as P                                # noqa: E402
from disaster import levels as L                                # noqa: E402


# --------------------------------------------------------------------------
# Naming: the baker writes it, the assembler reads it
# --------------------------------------------------------------------------

@pytest.mark.parametrize("raw,want", [
    ("objaverse://7c64a6d90567428eb511f41036fe2d88",
     "7c64a6d90567428eb511f41036fe2d88"),
    ("/Library/Stages/Muyang/Bungalow The Chase.usd", "Bungalow_The_Chase"),
    ("ranch", "ranch"),
    ("Black_Oak", "Black_Oak"),
    ("a/b/SM_prop-rail_a.usd", "SM_prop_rail_a"),
])
def test_type_slug(raw, want):
    assert lib.type_slug(raw) == want


def test_type_slug_is_stable_and_filename_safe():
    """A slug must survive a manifest and a shell, and must not change between
    runs — one derived from a pool index renames every archetype the moment an
    asset is added, invalidating the whole library."""
    for raw in ("objaverse://abc", "X Y/Z W.usd", "  spaces  ", "///"):
        s = lib.type_slug(raw)
        assert s and s == lib.type_slug(raw)
        assert all(c.isalnum() or c == "_" for c in s), s


def test_archetype_path_puts_the_disaster_in_the_directory():
    p = lib.archetype_path("/sg", "fire", "Bungalow The Chase", "burned_out")
    assert p.endswith(os.path.join("archetypes", "fire",
                                   "Bungalow_The_Chase_burned_out.usd"))


# --------------------------------------------------------------------------
# The library read side
# --------------------------------------------------------------------------

def _doc(*rows):
    return {"disaster": "fire",
            "archetypes": [{"type": t, "level": lv, "usd": f"{t}_{lv}.usd"}
                           for t, lv in rows]}


def test_library_resolves_exact_hits():
    L_ = lib.Library(doc=_doc(("ranch", "burned_out")))
    assert L_.get("ranch", "burned_out")["usd"] == "ranch_burned_out.usd"
    assert L_.types() == ["ranch"]
    assert len(L_) == 1


def test_library_falls_back_down_the_ladder_never_up():
    """A missing level must degrade to a LESS damaged archetype, never a more
    damaged one — placing a `rubble` asset where the field asked for
    `scorched` is visibly wrong in the direction nobody would look for."""
    names = L.level_names("fire")
    L_ = lib.Library(doc=_doc(("ranch", "pristine"), ("ranch", "scorched")))
    got = L_.resolve("ranch", "burned_out", names)
    assert got["level"] == "scorched"
    assert L_.misses, "a fallback must be recorded, not silently taken"


def test_library_returns_empty_rather_than_raising():
    """A partial bake must still assemble. Dying because one archetype failed
    to export turns a mostly-good library into no scene at all."""
    L_ = lib.Library(doc=_doc(("ranch", "pristine")))
    assert L_.resolve("nosuchtype", "rubble", L.level_names("fire")) == {}


def test_manifest_round_trips(tmp_path):
    p = str(tmp_path / "manifest.json")
    recs = [{"type": "ranch", "level": "rubble", "usd": "x.usd"}]
    lib.write_manifest(p, recs, {"disaster": "fire"})
    doc = lib.read_manifest(p)
    assert doc["disaster"] == "fire" and doc["archetypes"] == recs
    assert lib.Library(p).get("ranch", "rubble")["usd"] == "x.usd"


def test_read_manifest_of_a_missing_file_is_empty():
    assert lib.read_manifest("/no/such/manifest.json") == {}


# --------------------------------------------------------------------------
# The plan
# --------------------------------------------------------------------------

CONFIGS = ["urban", "suburb"]


@pytest.fixture(scope="module")
def configs():
    return {n: cd.load_scene_config(n) for n in CONFIGS}


@pytest.mark.parametrize("name", CONFIGS)
@pytest.mark.parametrize("dtype", sorted(cd.DISASTERS))
def test_plan_covers_every_level_the_ladder_can_ask_for(configs, name, dtype):
    """Stage B resolves (type, level); Stage A must have planned that pair."""
    items = P.build_plan(configs[name], dtype)
    assert items, f"{name}/{dtype} plans nothing to bake"
    for it in items:
        assert it.levels == L.bake_levels(dtype, it.kind)


@pytest.mark.parametrize("name", CONFIGS)
def test_plan_has_both_structures_and_vegetation(configs, name):
    kinds = {i.kind for i in P.build_plan(configs[name], "fire")}
    assert L.STRUCTURE in kinds
    assert L.VEGETATION in kinds, (
        f"{name} planned no vegetation — a wildfire library with no burnt "
        "trees in it is the failure this test exists for")


@pytest.mark.parametrize("name", CONFIGS)
def test_plan_slugs_are_unique(configs, name):
    """Two different USDs must never collapse into one archetype. Basenames
    collide across packs (`Trees/Oak.usd` vs `Vegetation/Oak.usd`), and baking
    one while placing it for both reads as every tree being identical."""
    items = P.build_plan(configs[name], "fire")
    keys = [(i.type, i.kind) for i in items]
    assert len(keys) == len(set(keys))
    sources = [(i.source, i.kind) for i in items]
    assert len(sources) == len(set(sources))


@pytest.mark.parametrize("name", CONFIGS)
def test_plan_excludes_pre_authored_ruins(configs, name):
    """`buildings.damaged` / `.destroyed` are somebody's authored ruin. Baking
    a ruin's ruin is meaningless, and it doubles the library for nothing."""
    cfg = configs[name]
    ruins = {t[0] for t in P.grouped_pools(cfg.get("usds") or {}, "buildings",
                                           ("damaged", "destroyed"))}
    planned = {i.source for i in P.build_plan(cfg, "fire")}
    assert not (ruins & planned)


def test_plan_is_deterministic(configs):
    a = P.build_plan(configs["urban"], "fire")
    b = P.build_plan(configs["urban"], "fire")
    assert [(i.type, i.kind, i.source) for i in a] == \
           [(i.type, i.kind, i.source) for i in b]


def test_pristine_disaster_plans_only_pristine(configs):
    for it in P.build_plan(configs["urban"], "none"):
        assert it.levels == ["pristine"]


# --------------------------------------------------------------------------
# Manifest paths must survive being moved
# --------------------------------------------------------------------------

def test_manifest_paths_are_relative_to_the_manifest(tmp_path):
    """A library is self-describing: the manifest sits beside its USDs.

    Records were briefly anchored to `scene_gen`, which looked fine for the
    default output root and produced `../../../../../tmp/...` the moment
    `--out` pointed outside the tree — which Stage B then pasted into an
    `airstack://` URL.
    """
    d = tmp_path / "fire"
    d.mkdir()
    (d / "ranch_rubble.usd").write_text("")
    p = str(d / lib.MANIFEST_NAME)
    lib.write_manifest(p, [{"type": "ranch", "level": "rubble",
                            "usd": "ranch_rubble.usd"}], {"disaster": "fire"})
    L_ = lib.Library(p)
    got = L_.usd_path(L_.get("ranch", "rubble"))
    assert os.path.isabs(got) and os.path.exists(got)
    assert ".." not in got


def test_library_survives_being_moved(tmp_path):
    src = tmp_path / "a" / "fire"
    src.mkdir(parents=True)
    (src / "ranch_rubble.usd").write_text("")
    lib.write_manifest(str(src / lib.MANIFEST_NAME),
                       [{"type": "ranch", "level": "rubble",
                         "usd": "ranch_rubble.usd"}], {"disaster": "fire"})
    import shutil
    dst = tmp_path / "b" / "fire"
    dst.parent.mkdir()
    shutil.copytree(src, dst)
    L_ = lib.Library(str(dst / lib.MANIFEST_NAME))
    assert os.path.exists(L_.usd_path(L_.get("ranch", "rubble")))


def test_a_partial_bake_merges_into_the_manifest(tmp_path):
    """A `--used-only` bake must not un-bake every type it did not touch —
    a tiny bake left the showcase with 2 of its 10 building types."""
    path = str(tmp_path / lib.MANIFEST_NAME)
    old = [{"type": "A", "level": "pristine", "usd": "A_pristine.usd"},
           {"type": "A", "level": "pancaked", "usd": "A_pancaked.usd"},
           {"type": "B", "level": "pristine", "usd": "B_pristine.usd"}]
    lib.write_manifest(path, old, {"disaster": "earthquake"})
    new = [{"type": "A", "level": "pancaked", "usd": "A_pancaked.usd",
            "meshes": 99}]
    lib.merge_manifest(path, new, {"disaster": "earthquake", "seed": 7})
    doc = lib.read_manifest(path)
    recs = {(r["type"], r["level"]): r for r in doc["archetypes"]}
    assert set(recs) == {("A", "pristine"), ("A", "pancaked"), ("B", "pristine")}
    assert recs[("A", "pancaked")]["meshes"] == 99          # the new one won
    assert doc["seed"] == 7
