"""`quake.load_manifest` rebases every record's `usd` onto the arch_dir the
caller passed — the bake records the absolute path of wherever it exported,
and that directory was renamed and mirrored to Nucleus after the bake."""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from disaster import quake  # noqa: E402


def _write(tmp_path, recs):
    p = tmp_path / "archetypes.json"
    p.write_text(json.dumps(recs))
    return str(tmp_path)


def test_stale_container_path_is_rebased_onto_arch_dir(tmp_path):
    recs = [{"usd": "/isaac-sim/AirStack/scene_gen/assets/archetypes_quake/bld_office_DG3.usd",
             "style": "office", "level": "DG3", "W": 32.0, "D": 16.0, "H": 19.0},
            {"usd": "/isaac-sim/AirStack/scene_gen/assets/archetypes_quake/bld_office_DG3_v2.usd",
             "style": "office", "level": "DG3_v2"}]
    d = _write(tmp_path, recs)
    m = quake.load_manifest(d)
    assert m[("office", "DG3")]["usd"] == os.path.join(d, "bld_office_DG3.usd")
    assert m[("office", "DG3_v2")]["usd"] == os.path.join(d, "bld_office_DG3_v2.usd")
    # the other fields ride along untouched
    assert m[("office", "DG3")]["H"] == 19.0
    # and the source list is not mutated
    assert recs[0]["usd"].endswith("archetypes_quake/bld_office_DG3.usd")


def test_omniverse_arch_dir_joins_as_a_url():
    a = "omniverse://host:443/Projects/x/archetype"
    assert quake._arch_join(a, "bld_a_DG1.usd") == a + "/bld_a_DG1.usd"
    assert quake._arch_join(a + "/", "archetypes.json") == a + "/archetypes.json"
    assert quake._arch_join("/local/dir", "f.usd") == "/local/dir/f.usd"


def test_record_without_usd_is_kept(tmp_path):
    d = _write(tmp_path, [{"style": "tower", "level": "DG0"}])
    m = quake.load_manifest(d)
    assert ("tower", "DG0") in m and "usd" not in m[("tower", "DG0")]
