#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Resolve a robot's whole fleet entry from a fleet file (RFC #380 §2).

The fleet-file successor of ``robot/docker/robot_name_map/resolve_robot_name.py``:
where the legacy resolver maps a container/host name to ``ROBOT_NAME`` +
``ROS_DOMAIN_ID`` only, this one resolves the **whole fleet entry** — name,
domain, stack placement (dir + entry point), vehicle, URDF, and per-unit
calibration overlay — from a ``config/fleets/*.yaml`` file.

Opt-in: the robot container's ``.bashrc`` calls this ONLY when
``FLEET_CONFIG_FILE`` is set (``airstack up --fleet <name>``); otherwise the
legacy resolver runs and behavior is byte-identical to before.

Usage (mirrors resolve_robot_name.py's eval-able stdout contract)::

    resolve_fleet.py <fleet.yaml> --name airstack-robot-desktop-2   # exports
    resolve_fleet.py <fleet.yaml> --index 2                         # exports
    resolve_fleet.py <fleet.yaml> --robot robot_2                   # exports
    resolve_fleet.py <fleet.yaml> --validate      # whole-fleet schema check
    resolve_fleet.py <fleet.yaml> --json          # full resolved fleet (machine)
    resolve_fleet.py <fleet.yaml> --table         # resolved robot table (human)

Export mode prints ``NAME=value`` lines (eval'd by ``robot/docker/.bashrc``)::

    ROBOT_NAME=robot_1
    ROS_DOMAIN_ID=1
    AIRSTACK_STACK_DIR=/root/AirStack/stacks/full_default
    AIRSTACK_STACK_ENTRY=stack
    URDF_FILE=robot_descriptions/iris/urdf/iris_with_sensors.pegasus.robot.urdf
    VEHICLE=quad_default
    CALIBRATION_DIR=

Identity resolution for ``--name`` (top-down, first match wins):
  1. exact robot key match (``--name wanda`` → robot ``wanda``)
  2. trailing replica/host index (``airstack-robot-desktop-2`` / ``robot-2``
     → the 2nd robot in file order) — same convention as the legacy
     ``default_robot_name_map.yaml`` rule ``.*robot-\\D*(\\d+)``.

``network.domain_policy: auto`` (the only policy implemented) assigns robot N
(1-based file order) → domain N — today's rule, so a fleet whose robots are
named ``robot_1..robot_N`` resolves identically to the legacy resolver.

All resolution is stdlib + PyYAML; paths in exports are rooted at the fleet
file's checkout root (``<root>/config/fleets/<f>.yaml`` → ``<root>``), which is
``/root/AirStack`` inside robot containers and the checkout on the host.
"""
import argparse
import json
import re
import sys
from pathlib import Path

import yaml

DEFAULT_ENTRY = "stack"
ONBOARD_ENTRY = "onboard"
SPAWN_DEFAULT = [0.0, 0.0, 0.07]

# Robot-level keys the schema accepts. Unknown keys are named errors so typos
# (``vehicel:``) fail loudly instead of silently applying defaults.
ROBOT_KEYS = {"vehicle", "unit", "stack", "spawn", "hosts", "overrides"}
GROUND_KEYS = {"stack"}
FLEET_KEYS = {"defaults", "robots", "ground", "sim", "network"}
NETWORK_KEYS = {"domain_policy", "gossip_domain", "gcs_domain"}

_TRAILING_INDEX_RE = re.compile(r"(\d+)$")


class FleetError(Exception):
    """A named fleet-file problem (schema or resolution)."""


# ── loading ──────────────────────────────────────────────────────────────────

def project_root_of(fleet_path):
    """Checkout root for a fleet file at ``<root>/config/fleets/<name>.yaml``.

    Falls back to the file's grandparent's parent regardless of naming, so a
    fleet file elsewhere still resolves relative to a sensible root.
    """
    return Path(fleet_path).resolve().parents[2]


def load_fleet(fleet_path):
    path = Path(fleet_path)
    if not path.is_file():
        raise FleetError(f"fleet file not found: {path}")
    try:
        with path.open(encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except yaml.YAMLError as exc:
        raise FleetError(f"fleet file is not valid YAML: {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise FleetError(f"fleet file must be a YAML mapping: {path}")
    return data


def load_vehicle(root, name):
    """Load ``config/vehicles/<name>/vehicle.yaml`` under ``root``."""
    manifest = Path(root) / "config" / "vehicles" / name / "vehicle.yaml"
    if not manifest.is_file():
        raise FleetError(
            f"vehicle '{name}' has no manifest at config/vehicles/{name}/vehicle.yaml"
        )
    with manifest.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise FleetError(f"vehicle manifest must be a YAML mapping: {manifest}")
    return data


# ── resolution helpers ───────────────────────────────────────────────────────

def _robots(fleet):
    robots = fleet.get("robots")
    if not isinstance(robots, dict) or not robots:
        raise FleetError("fleet has no robots: — 'robots:' must be a non-empty mapping")
    return list(robots.items())


def resolve_stack_path(root, stack_ref):
    """Resolve a fleet ``stack:`` value to a checkout-relative stack dir.

    Order (RFC #380 §3): a path in the checkout first (``stacks/<name>``),
    then ``<alias>/<stack>`` against external stack repos fetched by
    ``airstack sync`` into ``stacks/.external/<alias>/``.
    """
    root = Path(root)
    if not isinstance(stack_ref, str) or not stack_ref:
        raise FleetError(f"stack reference must be a non-empty string (got {stack_ref!r})")
    if (root / stack_ref).is_dir():
        return stack_ref
    if "/" in stack_ref and not stack_ref.startswith("stacks/"):
        alias, _, name = stack_ref.partition("/")
        external = Path("stacks") / ".external" / alias / name
        if (root / external).is_dir():
            return str(external)
        raise FleetError(
            f"stack '{stack_ref}' not found: no {stack_ref} in the checkout and no "
            f"external checkout at {external} — declare the repo under 'stacks:' in "
            f"airstack.yaml and run 'airstack sync'"
        )
    raise FleetError(f"stack '{stack_ref}' not found under {root}")


def stack_entries(root, stack_rel):
    launch_dir = Path(root) / stack_rel / "launch"
    if not launch_dir.is_dir():
        raise FleetError(f"stack '{stack_rel}' has no launch/ directory")
    return sorted(
        p.name[: -len(".launch.xml")]
        for p in launch_dir.glob("*.launch.xml")
    )


def _domain_policy(fleet):
    network = fleet.get("network") or {}
    if not isinstance(network, dict):
        raise FleetError("'network:' must be a mapping")
    policy = network.get("domain_policy", "auto")
    if policy != "auto":
        raise FleetError(
            f"network.domain_policy '{policy}' is not implemented — only 'auto' "
            f"(robot N → domain N) is"
        )
    return policy


def resolve_robot(fleet, root, key):
    """Resolve one robot's full entry. Returns a plain dict (JSON-safe)."""
    robots = _robots(fleet)
    keys = [k for k, _ in robots]
    if key not in keys:
        raise FleetError(f"no robot '{key}' in fleet (robots: {', '.join(keys)})")
    index = keys.index(key) + 1
    entry = dict(robots[index - 1][1] or {})
    unknown = set(entry) - ROBOT_KEYS
    if unknown:
        raise FleetError(
            f"robot '{key}' has unknown key(s): {', '.join(sorted(unknown))} "
            f"(allowed: {', '.join(sorted(ROBOT_KEYS))})"
        )

    defaults = fleet.get("defaults") or {}
    _domain_policy(fleet)  # auto: robot N → domain N

    vehicle = entry.get("vehicle", defaults.get("vehicle"))
    if not vehicle:
        raise FleetError(f"robot '{key}' has no vehicle and the fleet declares no defaults.vehicle")
    vehicle_manifest = load_vehicle(root, vehicle)
    urdf = ((vehicle_manifest.get("airframe") or {}).get("base_urdf")) or ""
    if not urdf:
        raise FleetError(f"vehicle '{vehicle}' declares no airframe.base_urdf")

    stack_ref = entry.get("stack", defaults.get("stack"))
    if not stack_ref:
        raise FleetError(f"robot '{key}' has no stack and the fleet declares no defaults.stack")
    stack_rel = resolve_stack_path(root, stack_ref)
    entries = stack_entries(root, stack_rel)

    hosts = entry.get("hosts") or {}
    if hosts and not isinstance(hosts, dict):
        raise FleetError(f"robot '{key}': 'hosts:' must be a mapping of role → ground host")
    ground = fleet.get("ground") or {}
    if hosts:
        if ONBOARD_ENTRY not in entries:
            raise FleetError(
                f"robot '{key}' names hosts: but stack '{stack_ref}' has no "
                f"launch/onboard.launch.xml entry point (entries: {', '.join(entries)})"
            )
        for role, host in hosts.items():
            if role == ONBOARD_ENTRY:
                raise FleetError(
                    f"robot '{key}': hosts role 'onboard' is the robot itself — "
                    f"name only offboard roles"
                )
            if role not in entries:
                raise FleetError(
                    f"robot '{key}': hosts role '{role}' has no matching entry point "
                    f"launch/{role}.launch.xml in stack '{stack_ref}' "
                    f"(entries: {', '.join(entries)})"
                )
            if host not in ground:
                raise FleetError(
                    f"robot '{key}': hosts.{role} names ground host '{host}' but the "
                    f"fleet declares no ground.{host} entry "
                    f"(ground hosts: {', '.join(ground) or '<none>'})"
                )
        launch_entry = ONBOARD_ENTRY
    else:
        if DEFAULT_ENTRY not in entries:
            raise FleetError(
                f"robot '{key}': stack '{stack_ref}' is a split stack "
                f"(entries: {', '.join(entries)}) — a robot using it must declare "
                f"hosts: {{<role>: <ground-host>}} placement"
            )
        launch_entry = DEFAULT_ENTRY

    spawn = entry.get("spawn", SPAWN_DEFAULT)
    if (not isinstance(spawn, (list, tuple)) or len(spawn) != 3
            or not all(isinstance(v, (int, float)) for v in spawn)):
        raise FleetError(f"robot '{key}': spawn must be [x, y, z] numbers (got {spawn!r})")

    unit = entry.get("unit")
    calibration_rel = f"config/local/calibration/{unit}" if unit else ""

    overrides = entry.get("overrides") or {}
    if overrides and not isinstance(overrides, dict):
        raise FleetError(f"robot '{key}': 'overrides:' must be a mapping of leaf values")

    return {
        "robot_name": key,
        "index": index,
        "domain_id": index,  # domain_policy auto: robot N → domain N
        "vehicle": vehicle,
        "urdf_file": urdf,
        "stack": stack_rel,
        "stack_ref": stack_ref,
        "entry": launch_entry,
        "hosts": dict(hosts),
        "spawn": [float(v) for v in spawn],
        "unit": unit,
        "calibration_dir": calibration_rel,
        "overrides": overrides,
        "lidar": vehicle_has_lidar(vehicle_manifest),
    }


def vehicle_has_lidar(vehicle_manifest):
    """True when the vehicle's sensor list carries any lidar entry."""
    for sensor in vehicle_manifest.get("sensors") or []:
        if isinstance(sensor, dict) and "lidar" in str(sensor.get("type", "")):
            return True
    return False


def resolve_fleet(fleet, root):
    """Resolve every robot + ground host. Returns the full machine-readable view."""
    robots = [resolve_robot(fleet, root, key) for key, _ in _robots(fleet)]
    ground = {}
    for host, cfg in (fleet.get("ground") or {}).items():
        cfg = cfg or {}
        unknown = set(cfg) - GROUND_KEYS
        if unknown:
            raise FleetError(
                f"ground host '{host}' has unknown key(s): {', '.join(sorted(unknown))}"
            )
        tenants = [
            {"robot_name": r["robot_name"], "role": role, "stack": r["stack"],
             "domain_id": r["domain_id"]}
            for r in robots
            for role, h in r["hosts"].items()
            if h == host
        ]
        ground[host] = {"stack": cfg.get("stack"), "tenants": tenants}
    network = fleet.get("network") or {}
    return {
        "robots": robots,
        "ground": ground,
        "sim": fleet.get("sim") or {},
        "network": {
            "domain_policy": network.get("domain_policy", "auto"),
            "gossip_domain": network.get("gossip_domain", 99),
            "gcs_domain": network.get("gcs_domain", 0),
        },
    }


def fleet_is_homogeneous(fleet, root):
    """True when deploy.replicas can stamp this fleet: every robot runs the same
    vehicle and stack, none has hosts: placement, and there are no ground hosts."""
    resolved = resolve_fleet(fleet, root)
    robots = resolved["robots"]
    if resolved["ground"]:
        return False
    first = robots[0]
    return all(
        r["vehicle"] == first["vehicle"]
        and r["stack"] == first["stack"]
        and not r["hosts"]
        for r in robots
    )


def validate_fleet(fleet, root):
    """Return a list of named error strings (empty = valid)."""
    errors = []
    unknown = set(fleet) - FLEET_KEYS
    if unknown:
        errors.append(
            f"unknown top-level key(s): {', '.join(sorted(unknown))} "
            f"(allowed: {', '.join(sorted(FLEET_KEYS))})"
        )
    network = fleet.get("network") or {}
    if isinstance(network, dict):
        unknown_net = set(network) - NETWORK_KEYS
        if unknown_net:
            errors.append(f"unknown network key(s): {', '.join(sorted(unknown_net))}")
    try:
        resolve_fleet(fleet, root)
    except FleetError as exc:
        errors.append(str(exc))
    return errors


def resolve_identity(fleet, name):
    """Map a container/host name to a robot key (see module docstring)."""
    keys = [k for k, _ in _robots(fleet)]
    if name in keys:
        return name
    match = _TRAILING_INDEX_RE.search(name)
    if match:
        index = int(match.group(1))
        if 1 <= index <= len(keys):
            return keys[index - 1]
        raise FleetError(
            f"'{name}' resolves to index {index} but the fleet has only "
            f"{len(keys)} robot(s)"
        )
    raise FleetError(
        f"no robot identity for '{name}': not a robot key "
        f"({', '.join(keys)}) and no trailing index"
    )


# ── output modes ─────────────────────────────────────────────────────────────

def print_exports(resolved, root):
    root = Path(root)
    stack_dir = str(root / resolved["stack"])
    cal = str(root / resolved["calibration_dir"]) if resolved["calibration_dir"] else ""
    print(f"ROBOT_NAME={resolved['robot_name']}")
    print(f"ROS_DOMAIN_ID={resolved['domain_id']}")
    print(f"AIRSTACK_STACK_DIR={stack_dir}")
    print(f"AIRSTACK_STACK_ENTRY={resolved['entry']}")
    print(f"URDF_FILE={resolved['urdf_file']}")
    print(f"VEHICLE={resolved['vehicle']}")
    print(f"CALIBRATION_DIR={cal}")


def print_table(resolved_fleet):
    rows = [
        (
            r["robot_name"], str(r["domain_id"]), r["vehicle"], r["stack_ref"],
            r["entry"],
            ",".join(f"{role}:{host}" for role, host in r["hosts"].items()) or "-",
            "[" + ", ".join(f"{v:g}" for v in r["spawn"]) + "]",
        )
        for r in resolved_fleet["robots"]
    ]
    for host, cfg in resolved_fleet["ground"].items():
        for tenant in cfg["tenants"]:
            rows.append((
                f"{host} (ground)", str(resolved_fleet["network"]["gcs_domain"]),
                "-", tenant["stack"], tenant["role"],
                f"serves:{tenant['robot_name']}", "-",
            ))
    headers = ("ROBOT", "DOMAIN", "VEHICLE", "STACK", "ENTRY", "HOSTS", "SPAWN")
    widths = [max(len(r[i]) for r in rows + [headers]) for i in range(len(headers))]
    fmt = "  ".join("{:<%d}" % w for w in widths)
    print(fmt.format(*headers))
    for row in rows:
        print(fmt.format(*row))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Resolve robot identity/placement from a fleet file (RFC #380 §2)."
    )
    parser.add_argument("fleet_file", help="path to config/fleets/<name>.yaml")
    parser.add_argument("--project-root", default=None,
                        help="checkout root (default: derived from the fleet file "
                             "path — <root>/config/fleets/<f>.yaml)")
    who = parser.add_mutually_exclusive_group()
    who.add_argument("--name", help="container or host name to resolve")
    who.add_argument("--index", type=int, help="1-based robot index to resolve")
    who.add_argument("--robot", help="explicit robot key to resolve")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--validate", action="store_true",
                      help="validate the whole fleet; named errors, exit 1 on any")
    mode.add_argument("--json", action="store_true",
                      help="dump the fully resolved fleet as JSON")
    mode.add_argument("--table", action="store_true",
                      help="print the resolved robot table (human)")
    args = parser.parse_args(argv)

    root = Path(args.project_root) if args.project_root else project_root_of(args.fleet_file)

    try:
        fleet = load_fleet(args.fleet_file)
        if args.validate:
            errors = validate_fleet(fleet, root)
            if errors:
                for err in errors:
                    print(f"Error: {err}", file=sys.stderr)
                return 1
            n = len(_robots(fleet))
            homogeneous = fleet_is_homogeneous(fleet, root)
            print(f"OK: {n} robot(s), "
                  f"{'homogeneous' if homogeneous else 'heterogeneous'} fleet")
            return 0
        if args.json:
            print(json.dumps(resolve_fleet(fleet, root), indent=2, sort_keys=True))
            return 0
        if args.table:
            print_table(resolve_fleet(fleet, root))
            return 0

        if args.robot:
            key = args.robot
        elif args.index is not None:
            keys = [k for k, _ in _robots(fleet)]
            if not 1 <= args.index <= len(keys):
                raise FleetError(
                    f"--index {args.index} out of range (fleet has {len(keys)} robot(s))"
                )
            key = keys[args.index - 1]
        elif args.name:
            key = resolve_identity(fleet, args.name)
        else:
            parser.error("one of --name/--index/--robot (or a mode flag) is required")
        print_exports(resolve_robot(fleet, root, key), root)
        return 0
    except FleetError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
