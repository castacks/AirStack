#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Generate a DDS-router config from a split stack's ``bridge.yaml`` (RFC #380 §2).

A split stack carries one launch entry point per host role plus a ``bridge.yaml``
explicitly listing every topic/service/action crossing the machine boundary.
That list is authoritative and human-readable; this tool derives the eProsima
DDS Router allowlist config from it — the same format as
``autonomy_bringup/onboard_local_offboard_global/config/dds_router.yaml`` and
consumed by the same ``interpolate_dds_router.launch.py`` (``$(env ROBOT_NAME)``
/ ``$(var gcs_domain)`` tokens are resolved at launch, per-robot).

Output is **deterministic**: a pure function of ``bridge.yaml`` (no timestamps,
no absolute paths), written to ``.airstack/generated/dds_router.<stack>.yaml``
so identical inputs regenerate byte-identical configs.

``--check`` validates the ``bridge.yaml`` schema and enforces **doctor hard
gate #2** (RFC #379 §4 / RFC #380 §2): ``control_setpoint`` and
trajectory-group names (``trajectory_override``, ``trajectory_segment_to_add``,
``set_trajectory_mode``, ``tracking_point``, ``look_ahead`` — the
``trajectory_controller/*`` group) must never appear in a bridge list. Command
authority stays onboard; link loss must leave the vehicle able to failsafe.
Violations exit 1 naming each offending entry.

CLI::

    gen_dds_router.py <bridge.yaml> [--out PATH] [--check] [--project-root DIR]

Human-readable errors go to stderr; a JSON verdict
``{"valid": bool, "errors": [{"path", "message"}]}`` goes to stdout in
``--check`` mode; exit 0/1.
"""
import argparse
import json
import re
import sys
from pathlib import Path

import yaml

GENERATED_REL = Path(".airstack") / "generated"

DIRECTIONS = ("onboard_to_offboard", "offboard_to_onboard")
QOS_VALUES = ("reliable", "best_effort")
ENTRY_KINDS = ("topic", "service", "action")

_TYPE_RE = re.compile(r"^[a-z][a-z0-9_]*/(msg|srv|action)/[A-Za-z][A-Za-z0-9_]*$")
_NAME_RE = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_/]*$")

# ── Doctor hard gate #2 (RFC #379 §4, RFC #380 §2) ──────────────────────────
# The enumerated trajectory-group names plus the control-setpoint interchange.
# Matched against the entry name's final path segment.
FORBIDDEN_BASENAMES = frozenset({
    "trajectory_override",
    "trajectory_segment_to_add",
    "set_trajectory_mode",
    "tracking_point",
    "look_ahead",
    "control_setpoint",
})
# The whole trajectory_controller/* group stays off the bridge, and so do the
# concrete control-setpoint topics (the interface's cmd_* command inputs).
FORBIDDEN_SEGMENTS = frozenset({"trajectory_controller"})
_FORBIDDEN_BASENAME_PREFIXES = ("cmd_",)

RFC_CITE = "RFC #379 §4 / RFC #380 §2 (doctor hard gate: command authority stays onboard)"


def check_hard_gate(name):
    """Return an error message when *name* violates the placement gate, else None."""
    segments = [s for s in name.split("/") if s]
    if not segments:
        return None
    basename = segments[-1]
    if basename in FORBIDDEN_BASENAMES:
        return (
            f"{name!r} is a control-setpoint/trajectory-group name "
            f"({basename!r}) and must not cross a machine boundary — {RFC_CITE}"
        )
    if basename.startswith(_FORBIDDEN_BASENAME_PREFIXES):
        return (
            f"{name!r} looks like a control-setpoint command input "
            f"({basename!r}) and must not cross a machine boundary — {RFC_CITE}"
        )
    hit = FORBIDDEN_SEGMENTS.intersection(segments)
    if hit:
        return (
            f"{name!r} is under the {sorted(hit)[0]}/* group, which stays off "
            f"the bridge wholesale — {RFC_CITE}"
        )
    return None


# ── schema validation ────────────────────────────────────────────────────────

def validate_bridge(data):
    """Validate a parsed bridge.yaml. Returns a list of {path, message} errors."""
    errors = []

    def err(path, message):
        errors.append({"path": path, "message": message})

    if not isinstance(data, dict):
        err("(root)", "bridge.yaml top level must be a mapping")
        return errors

    stack = data.get("stack")
    if stack is not None and (not isinstance(stack, str) or not stack.strip()):
        err("stack", "when present, stack: must be a non-empty string")

    entries = data.get("bridge")
    if not isinstance(entries, list):
        err("bridge", "bridge: must be a list of boundary entries")
        return errors

    seen = set()
    for i, entry in enumerate(entries):
        path = f"bridge[{i}]"
        if not isinstance(entry, dict):
            err(path, "entry must be a mapping")
            continue

        kinds = [k for k in ENTRY_KINDS if k in entry]
        if len(kinds) != 1:
            err(path, f"entry must have exactly one of {'/'.join(ENTRY_KINDS)} "
                      f"(got {kinds or 'none'})")
            continue
        kind = kinds[0]
        name = entry[kind]

        if not isinstance(name, str) or not name.strip():
            err(f"{path}.{kind}", "name must be a non-empty string")
            continue
        if name.startswith("/"):
            err(f"{path}.{kind}",
                f"{name!r} must be relative to the robot namespace — the "
                "generator prefixes $(env ROBOT_NAME) itself")
        elif "$(" in name:
            err(f"{path}.{kind}",
                f"{name!r} must not carry substitution tokens — interpolation "
                "conventions live in the generated config, not the bridge list")
        elif not _NAME_RE.match(name):
            err(f"{path}.{kind}", f"{name!r} is not a valid relative ROS name")

        if (kind, name) in seen:
            err(f"{path}.{kind}", f"duplicate entry for {name!r}")
        seen.add((kind, name))

        type_name = entry.get("type")
        if not isinstance(type_name, str) or not _TYPE_RE.match(type_name):
            err(f"{path}.type",
                f"{type_name!r} is not a pkg/(msg|srv|action)/Name interface type")
        else:
            expected_ns = {"topic": "msg", "service": "srv", "action": "action"}[kind]
            if f"/{expected_ns}/" not in type_name:
                err(f"{path}.type",
                    f"{type_name!r} does not match the entry kind {kind!r} "
                    f"(expected a */{expected_ns}/* type)")

        direction = entry.get("direction")
        if direction not in DIRECTIONS:
            err(f"{path}.direction",
                f"{direction!r} is not one of {list(DIRECTIONS)}")

        qos = entry.get("qos")
        if kind == "topic":
            if qos not in QOS_VALUES:
                err(f"{path}.qos",
                    f"{qos!r} is not one of {list(QOS_VALUES)} (required for topics)")
        elif qos is not None and qos not in QOS_VALUES:
            err(f"{path}.qos", f"{qos!r} is not one of {list(QOS_VALUES)}")

        unknown = set(entry) - {kind, "type", "direction", "qos"}
        if unknown:
            err(path, f"unknown fields: {sorted(unknown)}")

        # Hard gate #2 — checked here so schema-valid-but-unsafe still fails.
        if isinstance(name, str):
            gate = check_hard_gate(name)
            if gate:
                err(f"{path}.{kind}", gate)

    return errors


# ── config generation ────────────────────────────────────────────────────────

def _allowlist_lines(kind, name):
    """DDS endpoint allowlist entries for one bridge entry (relative name).

    Topic prefixes per the DDS/ROS 2 mapping documented in
    onboard_all/config/dds_router.yaml: rt/ topics, rq/…Request + rr/…Reply
    service pairs, and the five action sub-endpoints.
    """
    ns = "$(env ROBOT_NAME)"
    if kind == "topic":
        return [f"rt/{ns}/{name}"]
    if kind == "service":
        return [f"rq/{ns}/{name}Request", f"rr/{ns}/{name}Reply"]
    # action: goal/cancel/result services + feedback/status topics
    base = f"{ns}/{name}/_action"
    return [
        f"rq/{base}/send_goalRequest", f"rr/{base}/send_goalReply",
        f"rq/{base}/cancel_goalRequest", f"rr/{base}/cancel_goalReply",
        f"rq/{base}/get_resultRequest", f"rr/{base}/get_resultReply",
        f"rt/{base}/feedback", f"rt/{base}/status",
    ]


def render_router_config(data, source_rel="bridge.yaml"):
    """Render the DDS-router YAML text for a validated bridge mapping.

    Deterministic: pure function of the input (entries grouped by direction in
    input order; no timestamps, no absolute paths). The output format mirrors
    autonomy_bringup/onboard_local_offboard_global/config/dds_router.yaml —
    participants on $(env ROS_DOMAIN_ID) / $(var gcs_domain), an rt/rq/rr
    allowlist with $(env ROBOT_NAME) interpolation — so the same
    interpolate_dds_router.launch.py consumes it unchanged.
    """
    stack = data.get("stack", "unknown")
    groups = {d: [] for d in DIRECTIONS}
    for entry in data.get("bridge") or []:
        kind = next(k for k in ENTRY_KINDS if k in entry)
        groups[entry["direction"]].append((kind, entry[kind], entry.get("qos")))

    lines = [
        f"# GENERATED by tools/gen_dds_router.py from {source_rel} — do not edit.",
        f"# Split stack: {stack} (RFC #380 S2). Edit the bridge.yaml and regenerate.",
        "#",
        "# The DDS router bridges allowlisted endpoints bidirectionally; the",
        "# direction comments below document intent (from bridge.yaml).",
        "# $(env ...) / $(var ...) tokens are resolved per-robot at launch by",
        "# interpolate_dds_router.launch.py.",
        "participants:",
        '  - name: "onboard"',
        '    kind: "local"',
        "    domain: $(env ROS_DOMAIN_ID)",
        '  - name: "offboard"',
        '    kind: "local"',
        "    domain: $(var gcs_domain)",
        "allowlist:",
    ]
    headers = {
        "onboard_to_offboard": "  # ===== onboard --> offboard =====",
        "offboard_to_onboard": "  # ===== offboard --> onboard =====",
    }
    for direction in DIRECTIONS:
        entries = groups[direction]
        if not entries:
            continue
        lines.append(headers[direction])
        for kind, name, qos in entries:
            annotation = f"{kind}: {name}" + (f" ({qos})" if qos else "")
            lines.append(f"  # {annotation}")
            for endpoint in _allowlist_lines(kind, name):
                lines.append(f'  - name: "{endpoint}"')
    return "\n".join(lines) + "\n"


# ── entry points ─────────────────────────────────────────────────────────────

def load_bridge(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def default_out_path(bridge_path, project_root, stack=None):
    name = stack or _stack_name(bridge_path)
    return Path(project_root) / GENERATED_REL / f"dds_router.{name}.yaml"


def _stack_name(bridge_path):
    bridge_path = Path(bridge_path)
    try:
        data = load_bridge(bridge_path)
        if isinstance(data, dict) and isinstance(data.get("stack"), str):
            return data["stack"]
    except (OSError, yaml.YAMLError):
        pass
    return bridge_path.resolve().parent.name


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Generate (or --check) a DDS-router config from a split "
                    "stack's bridge.yaml (RFC #380 §2).",
    )
    parser.add_argument("bridge", help="path to the stack's bridge.yaml")
    parser.add_argument(
        "--out",
        help="output path (default: <project-root>/.airstack/generated/"
             "dds_router.<stack>.yaml)",
    )
    parser.add_argument(
        "--check", action="store_true",
        help="validate schema + the control/trajectory hard gate; write nothing",
    )
    parser.add_argument(
        "--project-root",
        default=str(Path(__file__).resolve().parent.parent),
        help="AirStack checkout root (default: parent of tools/)",
    )
    args = parser.parse_args(argv)

    bridge_path = Path(args.bridge)
    if not bridge_path.is_file():
        print(f"error: bridge file not found: {bridge_path}", file=sys.stderr)
        if args.check:
            print(json.dumps({"valid": False, "errors": [
                {"path": "(file)", "message": f"not found: {bridge_path}"}]},
                indent=2))
        return 1

    try:
        data = load_bridge(bridge_path)
    except yaml.YAMLError as exc:
        print(f"error: invalid YAML in {bridge_path}: {exc}", file=sys.stderr)
        if args.check:
            print(json.dumps({"valid": False, "errors": [
                {"path": "(file)", "message": f"invalid YAML: {exc}"}]}, indent=2))
        return 1

    errors = validate_bridge(data)
    if args.check:
        for error in errors:
            print(f"error: {error['path']}: {error['message']}", file=sys.stderr)
        if not errors:
            print(f"{bridge_path}: bridge.yaml is valid and passes the "
                  f"placement hard gate ({RFC_CITE})", file=sys.stderr)
        print(json.dumps({"valid": not errors, "errors": errors}, indent=2))
        return 0 if not errors else 1

    if errors:
        for error in errors:
            print(f"error: {error['path']}: {error['message']}", file=sys.stderr)
        print(f"error: refusing to generate from an invalid bridge.yaml "
              f"({len(errors)} error(s) above)", file=sys.stderr)
        return 1

    project_root = Path(args.project_root)
    out_path = Path(args.out) if args.out else default_out_path(
        bridge_path, project_root, stack=(data or {}).get("stack"))

    try:
        source_rel = bridge_path.resolve().relative_to(project_root.resolve())
    except ValueError:
        source_rel = bridge_path.name  # keep the output free of absolute paths

    text = render_router_config(data, source_rel=str(source_rel))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text, encoding="utf-8")
    print(f"wrote {out_path}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
