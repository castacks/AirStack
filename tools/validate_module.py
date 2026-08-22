#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Validate an AirStack module manifest (``module.yaml``) against the module schema.

The manifest contract is RFC #379 §2 — a *thin* manifest: deps, identity, and test
metadata only, never wiring. The schema lives at
``common/module_schema/module.schema.json`` and this validator interprets it with a
small generic walker (python3 stdlib + PyYAML — deliberately no ``jsonschema``
dependency, so module authors and CI need nothing beyond what the repo already uses).

The walker understands the draft-07 subset the schema keeps to — ``type``,
``required``, ``properties``, ``enum``, ``pattern``, ``items``,
``additionalProperties``, ``minLength``, ``minItems`` — so schema evolution does not
require code changes here. Only two kinds of checks are custom, keyed off
``x-airstack-*`` annotations in the schema rather than hardcoded field names:

- ``x-airstack-format``: ``semver-range`` (airstack_compat) and
  ``safe-relative-path`` (no absolute paths, no ``..`` escapes).
- ``x-airstack-check-exists`` / ``x-airstack-warn-missing-dir``: cross-file facts,
  applied only when the target is a module *directory* — declared dockerfile /
  compose / hooks / docs paths must exist (error); ``tests.packages`` entries that
  name no directory in the module are a warning, not an error.

CLI::

    validate_module.py <module_dir_or_module.yaml>

Human-readable errors and warnings go to stderr; a JSON verdict
``{"valid": bool, "errors": [{"path", "message"}]}`` goes to stdout; exit 0/1.
"""
import argparse
import json
import re
import sys
from pathlib import Path

import yaml

DEFAULT_SCHEMA_PATH = (
    Path(__file__).resolve().parent.parent / "common" / "module_schema" / "module.schema.json"
)

MANIFEST_NAME = "module.yaml"

# ── generic walker ─────────────────────────────────────────────────────────

_JSON_TYPES = {
    "object": dict,
    "array": list,
    "string": str,
    "integer": int,
    "number": (int, float),
    "boolean": bool,
    "null": type(None),
}

_SEMVER = (
    r"(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)"
    r"(?:-[0-9A-Za-z][0-9A-Za-z.-]*)?"
    r"(?:\+[0-9A-Za-z][0-9A-Za-z.-]*)?"
)
_COMPARATOR_RE = re.compile(r"^(?:>=|<=|>|<|==|=|\^|~)?" + _SEMVER + r"$")


def _check_semver_range(value):
    """Validate a semver range like ``>=0.19.0 <0.21.0`` or ``>=0.19.0-alpha.18 <0.20.0``.

    Space-separated comparators; each is an optional operator followed by a *full*
    ``X.Y.Z`` semver (prerelease/build allowed). Branch names (``main``, ``latest``)
    and partial versions (``0.19``) are invalid.
    """
    tokens = value.split()
    if not tokens:
        return "empty semver range"
    for token in tokens:
        if not _COMPARATOR_RE.match(token):
            return (
                f"invalid semver range component {token!r} — expected an optional "
                "operator (>=, <=, >, <, ==, =, ^, ~) followed by a full X.Y.Z "
                "semver, e.g. \">=0.19.0 <0.21.0\""
            )
    return None


def _check_safe_relative_path(value):
    """A repo/module-relative path: non-empty, not absolute, no ``..`` escapes."""
    if not value:
        return "path must be non-empty"
    if value.startswith(("/", "\\", "~")) or re.match(r"^[A-Za-z]:[/\\]", value):
        return f"path must be relative, got {value!r}"
    if ".." in Path(value).parts:
        return f"path must not escape the module checkout via '..', got {value!r}"
    return None


_FORMAT_CHECKS = {
    "semver-range": _check_semver_range,
    "safe-relative-path": _check_safe_relative_path,
}


def _join(path, key):
    return f"{path}.{key}" if path else str(key)


def _type_ok(value, type_spec):
    types = type_spec if isinstance(type_spec, list) else [type_spec]
    for name in types:
        expected = _JSON_TYPES.get(name)
        if expected is None:
            continue
        # bool is a subclass of int in Python; keep JSON semantics strict.
        if isinstance(value, bool) and name in ("integer", "number"):
            continue
        if isinstance(value, expected):
            return True
    return False


def _type_names(type_spec):
    return type_spec if isinstance(type_spec, list) else [type_spec]


class _Context:
    """Collects errors, warnings, and cross-file annotations during the walk."""

    def __init__(self):
        self.errors = []
        self.warnings = []
        self.existence_checks = []  # (path, relative_path) — must exist in the module dir
        self.dir_warnings = []      # (path, name) — warn when no matching directory

    def error(self, path, message):
        self.errors.append({"path": path or "(root)", "message": message})


def _walk(value, schema, path, ctx):
    """Apply one schema node to one instance node, recursing into properties/items."""
    if "type" in schema and not _type_ok(value, schema["type"]):
        ctx.error(
            path,
            f"expected type {' or '.join(_type_names(schema['type']))}, "
            f"got {type(value).__name__}",
        )
        return
    if value is None:
        return  # a nullable field left null — string/object keywords do not apply

    if "enum" in schema and value not in schema["enum"]:
        ctx.error(
            path,
            f"{value!r} is not one of {schema['enum']}",
        )
        return

    if isinstance(value, str):
        ok = True
        if "minLength" in schema and len(value) < schema["minLength"]:
            ctx.error(path, f"must be at least {schema['minLength']} characters")
            ok = False
        if "pattern" in schema and not re.search(schema["pattern"], value):
            ctx.error(path, f"{value!r} does not match pattern {schema['pattern']!r}")
            ok = False
        fmt = schema.get("x-airstack-format")
        if fmt:
            msg = _FORMAT_CHECKS[fmt](value)
            if msg:
                ctx.error(path, msg)
                ok = False
        if ok:
            # Cross-file annotations only make sense for values that passed the
            # syntactic checks — a path with '..' must never reach a filesystem probe.
            if schema.get("x-airstack-check-exists"):
                ctx.existence_checks.append((path, value))
            if schema.get("x-airstack-warn-missing-dir"):
                ctx.dir_warnings.append((path, value))

    elif isinstance(value, dict):
        properties = schema.get("properties", {})
        for key in schema.get("required", []):
            if key not in value:
                ctx.error(_join(path, key), "required property is missing")
        if schema.get("additionalProperties") is False:
            for key in value:
                if key not in properties:
                    ctx.error(_join(path, key), "unknown property (additionalProperties: false)")
        for key, subschema in properties.items():
            if key in value:
                _walk(value[key], subschema, _join(path, key), ctx)

    elif isinstance(value, list):
        if "minItems" in schema and len(value) < schema["minItems"]:
            ctx.error(path, f"must have at least {schema['minItems']} item(s)")
        if "items" in schema:
            for i, item in enumerate(value):
                _walk(item, schema["items"], f"{path}[{i}]", ctx)


# ── entry points ───────────────────────────────────────────────────────────

def load_schema(schema_path=None):
    with open(schema_path or DEFAULT_SCHEMA_PATH, encoding="utf-8") as f:
        return json.load(f)


def validate_manifest(data, schema):
    """Schema-only validation of a parsed manifest. Returns a ``_Context``."""
    ctx = _Context()
    _walk(data, schema, "", ctx)
    return ctx


def validate_module(target, schema_path=None):
    """Validate a module dir (schema + cross-file facts) or a bare module.yaml.

    Returns ``(verdict, warnings)`` where ``verdict`` is the stable JSON shape
    ``{"valid": bool, "errors": [{"path", "message"}]}`` and ``warnings`` is a
    list of human-readable strings (never affecting validity).
    """
    target = Path(target)
    ctx = _Context()

    if target.is_dir():
        module_dir, manifest_path = target, target / MANIFEST_NAME
    else:
        module_dir, manifest_path = None, target

    if not manifest_path.is_file():
        ctx.error("(file)", f"manifest not found: {manifest_path}")
        return {"valid": False, "errors": ctx.errors}, ctx.warnings

    try:
        with manifest_path.open(encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except yaml.YAMLError as exc:
        ctx.error("(file)", f"invalid YAML: {exc}")
        return {"valid": False, "errors": ctx.errors}, ctx.warnings

    if not isinstance(data, dict):
        ctx.error("(root)", "manifest top level must be a mapping")
        return {"valid": False, "errors": ctx.errors}, ctx.warnings

    schema = load_schema(schema_path)
    ctx = validate_manifest(data, schema)

    if module_dir is not None:
        for path, rel in ctx.existence_checks:
            if not (module_dir / rel).exists():
                ctx.error(path, f"declared path does not exist in the module: {rel!r}")
        for path, name in ctx.dir_warnings:
            if not (module_dir / name).is_dir():
                ctx.warnings.append(
                    f"{path}: {name!r} names no directory in the module — "
                    "tests.packages entries usually match a package directory"
                )

    return {"valid": not ctx.errors, "errors": ctx.errors}, ctx.warnings


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Validate an AirStack module.yaml against the module manifest schema."
    )
    parser.add_argument(
        "target",
        help="module directory (validates module.yaml + cross-file facts) "
             "or a module.yaml path (schema-only)",
    )
    parser.add_argument(
        "--schema",
        default=str(DEFAULT_SCHEMA_PATH),
        help="schema path (default: common/module_schema/module.schema.json)",
    )
    args = parser.parse_args(argv)

    verdict, warnings = validate_module(Path(args.target), Path(args.schema))
    for warning in warnings:
        print(f"warning: {warning}", file=sys.stderr)
    for error in verdict["errors"]:
        print(f"error: {error['path']}: {error['message']}", file=sys.stderr)
    if verdict["valid"]:
        print(f"{args.target}: module manifest is valid", file=sys.stderr)
    print(json.dumps(verdict, indent=2))
    return 0 if verdict["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
