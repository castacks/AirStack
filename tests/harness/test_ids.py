"""Canonical test identifiers shared by metrics and summary reporting."""


def canonical_test_id(name: str) -> str:
    """Unify pytest node-id path slashes with JUnit classname dots.

    ``metrics.json`` keys start with paths such as
    ``system/test_liveliness.Class.test`` while JUnit uses
    ``system.test_liveliness.Class.test``.
    """
    head, dot, rest = name.partition(".")
    if "/" in head:
        head = head.replace("/", ".")
        return head + dot + rest if dot else head
    return name
