# Copyright (c) 2026 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Contract tests for the module manifest schema + validator (RFC #379 §2, Phase P1).

``tools/validate_module.py`` interprets ``common/module_schema/module.schema.json``
with a generic walker — these tests pin the contract both sides must keep: the
required fields and their shapes, the custom format hooks (semver ranges, path
safety), dir-mode cross-file checks, and the stable JSON verdict emitted for
scripts. The valid fixture module lives at ``tests/fixtures/modules/hello_module``.

The validator is exercised both through its Python API (fast, most cases) and as a
subprocess (the CLI contract: verdict on stdout, exit code).
"""
import copy
import importlib.util
import json
import os
import subprocess
import sys

import pytest
import yaml

from harness.discovery import repo_path

pytestmark = pytest.mark.unit

VALIDATOR = repo_path("tools", "validate_module.py")
SCHEMA = repo_path("common", "module_schema", "module.schema.json")
FIXTURE = repo_path("tests", "fixtures", "modules", "hello_module")


def _load_validator():
    spec = importlib.util.spec_from_file_location("airstack_validate_module", VALIDATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def vm():
    return _load_validator()


@pytest.fixture()
def manifest():
    """A fresh copy of the valid fixture manifest, ready to mutate."""
    with (FIXTURE / "module.yaml").open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def _validate(vm, tmp_path, data, as_dir=True):
    """Write a manifest into tmp_path and validate; return (verdict, warnings)."""
    (tmp_path / "module.yaml").write_text(yaml.safe_dump(data), encoding="utf-8")
    target = tmp_path if as_dir else tmp_path / "module.yaml"
    return vm.validate_module(target)


def _error_paths(verdict):
    return {e["path"] for e in verdict["errors"]}


# ── the valid fixture passes ───────────────────────────────────────────────

def test_valid_fixture_passes_via_import(vm):
    verdict, warnings = vm.validate_module(FIXTURE)
    assert verdict["valid"] is True, verdict["errors"]
    assert verdict["errors"] == []
    assert warnings == []  # tests.packages: [hello_module] matches a real directory


def test_valid_fixture_passes_via_subprocess():
    assert os.access(VALIDATOR, os.X_OK), "tools/validate_module.py must be executable"
    result = subprocess.run(
        [sys.executable, str(VALIDATOR), str(FIXTURE)],
        capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, result.stderr
    verdict = json.loads(result.stdout)
    assert verdict == {"valid": True, "errors": []}


# ── schema violations ──────────────────────────────────────────────────────

def test_missing_maintainer_fails_naming_the_path(vm, tmp_path, manifest):
    del manifest["maintainer"]
    verdict, _ = _validate(vm, tmp_path, manifest)
    assert verdict["valid"] is False
    assert "maintainer" in _error_paths(verdict)


def test_bad_type_enum_fails(vm, tmp_path, manifest):
    manifest["type"] = "launch_bundle"
    verdict, _ = _validate(vm, tmp_path, manifest)
    assert verdict["valid"] is False
    assert "type" in _error_paths(verdict)


@pytest.mark.parametrize("bad_range", ["main", "latest", "0.19"])
def test_malformed_airstack_compat_fails(vm, tmp_path, manifest, bad_range):
    manifest["airstack_compat"] = bad_range
    verdict, _ = _validate(vm, tmp_path, manifest)
    assert verdict["valid"] is False
    assert "airstack_compat" in _error_paths(verdict)


@pytest.mark.parametrize(
    "good_range",
    [">=0.19.0 <0.21.0", ">=0.19.0-alpha.18 <0.20.0", "0.20.0", "^0.19.0"],
)
def test_wellformed_airstack_compat_passes(vm, tmp_path, manifest, good_range):
    manifest["airstack_compat"] = good_range
    verdict, _ = _validate(vm, tmp_path, manifest)
    assert verdict["valid"] is True, verdict["errors"]


def test_asset_with_http_url_fails(vm, tmp_path, manifest):
    manifest["assets"] = [{
        "url": "http://example.com/asset.tar",
        "sha256": "0" * 64,
        "dest": "assets/asset.tar",
    }]
    verdict, _ = _validate(vm, tmp_path, manifest)
    assert verdict["valid"] is False
    assert "assets[0].url" in _error_paths(verdict)


def test_asset_dest_with_traversal_fails(vm, tmp_path, manifest):
    manifest["assets"] = [{
        "url": "https://example.com/asset.tar",
        "sha256": "0" * 64,
        "dest": "../outside/asset.tar",
    }]
    verdict, _ = _validate(vm, tmp_path, manifest)
    assert verdict["valid"] is False
    assert "assets[0].dest" in _error_paths(verdict)


def test_unknown_top_level_key_fails(vm, tmp_path, manifest):
    manifest["slot"] = "local_planner"  # wiring metadata is deliberately absent
    verdict, _ = _validate(vm, tmp_path, manifest)
    assert verdict["valid"] is False
    assert "slot" in _error_paths(verdict)


def test_unknown_mark_fails(vm, tmp_path, manifest):
    manifest["tests"] = {"packages": [], "marks": ["hover_forever"]}
    verdict, _ = _validate(vm, tmp_path, manifest)
    assert verdict["valid"] is False
    assert "tests.marks[0]" in _error_paths(verdict)


def test_empty_targets_fails(vm, tmp_path, manifest):
    manifest["targets"] = []
    verdict, _ = _validate(vm, tmp_path, manifest)
    assert verdict["valid"] is False
    assert "targets" in _error_paths(verdict)


# ── dir-mode cross-file checks ─────────────────────────────────────────────

def test_dockerfile_pointing_at_nonexistent_file_fails_in_dir_mode(vm, tmp_path, manifest):
    manifest["dockerfile"] = "Dockerfile.module"  # declared but never created
    verdict, _ = _validate(vm, tmp_path, manifest, as_dir=True)
    assert verdict["valid"] is False
    assert "dockerfile" in _error_paths(verdict)


def test_dockerfile_declared_and_present_passes(vm, tmp_path, manifest):
    manifest["dockerfile"] = "Dockerfile.module"
    (tmp_path / "Dockerfile.module").write_text("ARG BASE_IMAGE\nFROM ${BASE_IMAGE}\n")
    verdict, _ = _validate(vm, tmp_path, manifest, as_dir=True)
    assert verdict["valid"] is True, verdict["errors"]


def test_file_mode_skips_cross_file_checks(vm, tmp_path, manifest):
    """Given a bare module.yaml (not a dir), declared paths are not probed."""
    manifest["dockerfile"] = "Dockerfile.module"
    verdict, _ = _validate(vm, tmp_path, manifest, as_dir=False)
    assert verdict["valid"] is True, verdict["errors"]


def test_missing_tests_package_dir_warns_but_stays_valid(vm, tmp_path, manifest):
    manifest["tests"] = {"packages": ["no_such_package"], "marks": []}
    verdict, warnings = _validate(vm, tmp_path, manifest, as_dir=True)
    assert verdict["valid"] is True, verdict["errors"]
    assert warnings and "no_such_package" in warnings[0]


# ── verdict shape ──────────────────────────────────────────────────────────

def test_json_verdict_shape_is_stable(vm, tmp_path, manifest):
    """Scripts parse the stdout verdict: exactly {valid, errors:[{path,message}]}."""
    broken = copy.deepcopy(manifest)
    del broken["maintainer"]
    for data in (manifest, broken):
        verdict, _ = _validate(vm, tmp_path, data)
        assert set(verdict.keys()) == {"valid", "errors"}
        assert isinstance(verdict["valid"], bool)
        assert isinstance(verdict["errors"], list)
        for error in verdict["errors"]:
            assert set(error.keys()) == {"path", "message"}


def test_invalid_manifest_exits_1_via_subprocess(tmp_path, manifest):
    del manifest["maintainer"]
    (tmp_path / "module.yaml").write_text(yaml.safe_dump(manifest), encoding="utf-8")
    result = subprocess.run(
        [sys.executable, str(VALIDATOR), str(tmp_path)],
        capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 1
    verdict = json.loads(result.stdout)
    assert verdict["valid"] is False
    assert "maintainer" in result.stderr  # human-readable error names the path


def test_schema_keeps_to_the_interpreted_subset():
    """The walker only interprets a fixed keyword subset — the schema must not
    quietly grow keywords (oneOf, $ref, ...) the validator would silently ignore."""
    allowed = {
        "$schema", "title", "description", "default",
        "type", "required", "properties", "enum", "pattern", "items",
        "additionalProperties", "minLength", "minItems",
        "x-airstack-format", "x-airstack-check-exists", "x-airstack-warn-missing-dir",
    }

    def keys(node):
        if isinstance(node, dict):
            for key, value in node.items():
                yield key
                if key in ("properties",):
                    for sub in value.values():
                        yield from keys(sub)
                elif key in ("items", "additionalProperties"):
                    yield from keys(value)
        return

    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    unknown = set(keys(schema)) - allowed
    assert not unknown, f"schema uses keywords the validator does not interpret: {unknown}"
