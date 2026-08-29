# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""CLI help truthfulness contract.

``airstack help <cmd>`` text drifts from the code because nothing ties the
two together. This test does — at grep level, matching flag/subcommand
TOKENS (not prose), so wording can evolve freely while lies cannot:

- every ``parse_launch_intent`` flag appears in the ``up`` help arm
- every pytest ``addoption`` long-name appears in the ``test`` help arm
- every dispatched subcommand of the ``module``/``fleet``/``stack`` command
  groups appears in its help arm
"""
import re

import pytest

from harness.discovery import repo_path

pytestmark = pytest.mark.unit

REPO = repo_path()
AIRSTACK_SH = (REPO / "airstack.sh").read_text(encoding="utf-8")
CONFTEST = (REPO / "tests" / "conftest.py").read_text(encoding="utf-8")


def _function_body(text, name):
    """Body of a bash ``function <name> {`` ... ``}`` (closing brace at col 0)."""
    marker = f"function {name} {{"
    start = text.index(marker)
    end = text.index("\n}", start)
    return text[start:end]


def _help_arm(command):
    """The ``<command>)`` arm of print_command_help's case statement."""
    body = _function_body(AIRSTACK_SH, "print_command_help")
    match = re.search(
        rf"^        {re.escape(command)}\)\n(.*?)^\s*;;", body,
        re.MULTILINE | re.DOTALL,
    )
    assert match, f"print_command_help has no '{command})' help arm"
    return match.group(1)


def _dispatch_subcommands(module_file, dispatch_fn):
    """Word tokens of a dispatcher's ``case "$sub" in`` arms (help/* excluded)."""
    text = (REPO / ".airstack" / "modules" / module_file).read_text(encoding="utf-8")
    body = _function_body(text, dispatch_fn)
    case_body = body[body.index('case "$sub" in'):]
    subs = set()
    for arm in re.findall(r"^\s*([A-Za-z0-9_|-]+)\)", case_body, re.MULTILINE):
        for token in arm.split("|"):
            if token and token not in {"help", "-h", "--help", "*"}:
                subs.add(token)
    return subs


# ── up: every launch-intent flag is documented ───────────────────────────────

def test_up_help_names_every_parse_launch_intent_flag():
    body = _function_body(AIRSTACK_SH, "parse_launch_intent")
    flags = set(re.findall(r"(--[a-z][a-z-]*)(?:=\*)?\)", body))
    assert flags, "no flags extracted from parse_launch_intent — parser drifted?"
    arm = _help_arm("up")
    missing = sorted(f for f in flags if f not in arm)
    assert not missing, f"'airstack help up' does not mention: {missing}"


# ── test: every pytest addoption long-name is documented ─────────────────────

def test_test_help_names_every_pytest_addoption():
    body = CONFTEST[CONFTEST.index("def pytest_addoption"):]
    body = body[:body.index("\ndef ", 1)]
    options = set(re.findall(r"addoption\(\s*\"(--[a-z][a-z-]*)\"", body))
    assert options, "no addoptions extracted from tests/conftest.py — drifted?"
    arm = _help_arm("test")
    missing = sorted(o for o in options if o not in arm)
    assert not missing, f"'airstack help test' does not mention: {missing}"


def test_test_help_names_every_pytest_ini_mark():
    ini = (REPO / "tests" / "pytest.ini").read_text(encoding="utf-8")
    marks_block = ini[ini.index("markers ="):ini.index("testpaths")]
    marks = set(re.findall(r"^\s{4}(\w+):", marks_block, re.MULTILINE))
    assert marks, "no marks extracted from tests/pytest.ini — drifted?"
    arm = _help_arm("test")
    missing = sorted(m for m in marks if not re.search(rf"\b{m}\b", arm))
    assert not missing, f"'airstack help test' does not list marks: {missing}"


# ── command groups: every dispatched subcommand is documented ────────────────

@pytest.mark.parametrize(
    ("help_command", "module_file", "dispatch_fn"),
    [
        ("module", "module.sh", "cmd_module_dispatch"),
        ("fleet", "fleet.sh", "cmd_fleet_dispatch"),
        ("stack", "stack.sh", "cmd_stack_dispatch"),
    ],
)
def test_group_help_names_every_dispatched_subcommand(
        help_command, module_file, dispatch_fn):
    subs = _dispatch_subcommands(module_file, dispatch_fn)
    assert subs, f"no subcommands extracted from {dispatch_fn} — drifted?"
    arm = _help_arm(help_command)
    missing = sorted(s for s in subs if not re.search(rf"\b{re.escape(s)}\b", arm))
    assert not missing, (
        f"'airstack help {help_command}' does not mention subcommand(s): {missing}"
    )
