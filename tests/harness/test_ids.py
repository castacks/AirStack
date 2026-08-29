"""Canonical test identifiers shared by collection, metadata, and reporting."""

import re


def canonical_test_id(name: str) -> str:
    """Unify pytest node-id path slashes with JUnit classname dots.

    ``metrics.json`` keys start with paths such as
    ``system/test_liveliness.Class.test`` while JUnit uses
    ``system.test_liveliness.Class.test``.
    """
    value = str(name).replace("\\", "/")
    value = value.replace(".py::", ".").replace("::", ".")
    value = value.replace(".py.", ".")
    return value.replace("/", ".").lstrip(".")


def normalize_csv(value, cast=str) -> list:
    """Normalize a comma-separated pytest option into a stable sorted list."""
    if value is None:
        return []
    if isinstance(value, (list, tuple, set)):
        parts = value
    else:
        parts = str(value).split(",")
    normalized = [cast(str(part).strip()) for part in parts if str(part).strip()]
    return sorted(normalized)


def base_iteration_test_id(name: str) -> str:
    """Canonical test ID with only the generated stress-iteration suffix removed."""
    return re.sub(r"-iter\d+(?=\])", "", canonical_test_id(name))
