import importlib.util
import json
from pathlib import Path


_PATH = Path(__file__).parents[1] / "tools" / "urban_fire_fast_preflight.py"
_SPEC = importlib.util.spec_from_file_location("urban_fire_fast_preflight", _PATH)
guard = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(guard)


def _files(tmp_path, usd="omniverse://server/building.usd", kind="gac"):
    dump = tmp_path / "dump.json"
    dump.write_text(json.dumps({"preset": "p", "seed": 3}))
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps({
        "preset": "p", "seed": 3,
        "placements_dump": {"sha256": guard.sha(dump)},
        "records": [{"kind": kind, "usd": usd, "asset": "SM_A"}],
    }))
    return dump, manifest


def test_coherent_plan_passes(monkeypatch, tmp_path):
    dump, manifest = _files(tmp_path)
    monkeypatch.setattr(guard, "source_fingerprint", lambda *_: ("abc", 7))
    proof, errors, count = guard.inspect(tmp_path, dump, manifest)
    assert errors == []
    assert count == 1
    assert proof["offline_dump_sha256"] == guard.sha(dump)


def test_stale_dump_and_host_local_aec_fail(monkeypatch, tmp_path):
    dump, manifest = _files(tmp_path, "/host/repo/scene_gen/assets/aec/SM_A.usd", "gac")
    doc = json.loads(manifest.read_text())
    doc["placements_dump"]["sha256"] = "stale"
    manifest.write_text(json.dumps(doc))
    monkeypatch.setattr(guard, "source_fingerprint", lambda *_: ("abc", 7))
    _, errors, _ = guard.inspect(tmp_path, dump, manifest)
    assert any("sha256 is stale" in e for e in errors)
    assert any("host-local absolute USD" in e for e in errors)
    assert any("AEC asset routed" in e for e in errors)


def test_proof_changes_when_manifest_changes(monkeypatch, tmp_path):
    dump, manifest = _files(tmp_path)
    monkeypatch.setattr(guard, "source_fingerprint", lambda *_: ("abc", 7))
    before, errors, _ = guard.inspect(tmp_path, dump, manifest)
    assert not errors
    doc = json.loads(manifest.read_text())
    doc["note"] = "changed"
    manifest.write_text(json.dumps(doc))
    after, errors, _ = guard.inspect(tmp_path, dump, manifest)
    assert not errors
    assert before != after
