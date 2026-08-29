#!/usr/bin/env python3
"""Offline check: the five canonical module launch files produce the same ROS
graph as the monolithic local_bringup/launch/local.launch.xml.

No ROS, no roslaunch, no simulator: the launch XML is parsed with minidom and
the substitutions are resolved textually, which is enough because every
endpoint on both sides is either a literal, a `$(env ROBOT_NAME)` expansion, or
a `$(var ...)` whose default is declared in the same file.

For each of the five nodes it compares the tuple

    (namespace, pkg, exec, {remap from -> to}, {param name: value})

between local.launch.xml (the legacy source of truth on this branch) and the
canonical file for that module.  Effective parameter values are compared, so a
`<param name=... value=...>` written inline in the legacy file must equal the
key of the same name in the canonical file's `config/<pkg>.yaml`.

A parameter that appears ONLY in the canonical YAML is accepted only when its
value equals the compiled default that the node's own C++ reads for it
(`airstack::get_param(this, "name", <default>)`) -- i.e. writing it down did
not change the value the node runs with.  Anything else is a failure.

Usage:  python3 check_canonical_launch.py     (exit 0 = graphs identical)
"""

import os
import re
import sys
import xml.dom.minidom as minidom

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("PyYAML is required: pip install pyyaml")

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))  # ros_ws/src
LEGACY = os.path.join(SRC_ROOT, "local", "local_bringup", "launch", "local.launch.xml")

# module key -> (canonical launch file, (pkg, exec) identity of the node)
CANONICAL = {
    "droan_gl": (
        "local/planners/droan_gl/launch/droan_gl.launch.xml",
        ("droan_gl", "droan_gl_node"),
    ),
    "takeoff_landing_planner": (
        "local/planners/takeoff_landing_planner/launch/takeoff_landing_planner.launch.xml",
        ("takeoff_landing_planner", "takeoff_landing_task"),
    ),
    "fixed_trajectory_task": (
        "local/controls/trajectory_controller/launch/fixed_trajectory_task.launch.xml",
        ("trajectory_controller", "fixed_trajectory_task"),
    ),
    "trajectory_controller": (
        "local/controls/trajectory_controller/launch/trajectory_controller.launch.xml",
        ("trajectory_controller", "trajectory_controller"),
    ),
    "pid_controller": (
        "local/controls/pid_controller/launch/pid_controller.launch.xml",
        ("pid_controller", "pid_controller"),
    ),
}

ROBOT = "${ROBOT_NAME}"  # both sides expand $(env ROBOT_NAME) to this placeholder


# --------------------------------------------------------------------------
# package index (stands in for find-pkg-share; share/ mirrors the source dir)
# --------------------------------------------------------------------------
def index_packages(root):
    pkgs = {}
    for dirpath, dirnames, filenames in os.walk(root):
        if "package.xml" in filenames:
            try:
                dom = minidom.parse(os.path.join(dirpath, "package.xml"))
                name = dom.getElementsByTagName("name")[0].firstChild.data.strip()
            except Exception:
                continue
            pkgs[name] = dirpath
            dirnames[:] = []
    return pkgs


PKGS = index_packages(SRC_ROOT)

MISSING_PATHS = []


def substitute(text, args):
    """Resolve $(env ROBOT_NAME), $(var X) and $(find-pkg-share P) textually."""
    if text is None:
        return None
    out = text
    for _ in range(10):
        prev = out
        out = re.sub(r"\$\(env ROBOT_NAME\)", ROBOT.strip("${}"), out)
        out = re.sub(r"\$\(env ROBOT_NAME [^)]*\)", ROBOT.strip("${}"), out)

        def var(m):
            name = m.group(1)
            if name not in args:
                raise KeyError("undeclared launch arg $(var %s)" % name)
            return args[name]

        out = re.sub(r"\$\(var ([A-Za-z0-9_]+)\)", var, out)

        def share(m):
            pkg = m.group(1)
            if pkg not in PKGS:
                MISSING_PATHS.append("find-pkg-share %s : package not in %s" % (pkg, SRC_ROOT))
                return "<MISSING-PKG:%s>" % pkg
            return PKGS[pkg]

        out = re.sub(r"\$\(find-pkg-share ([A-Za-z0-9_]+)\)", share, out)
        if out == prev:
            break
    return out


def norm_value(v):
    """Normalise a param value to (kind, value) so 'true'/True and '2.'/2.0 match."""
    if isinstance(v, bool):
        return ("bool", v)
    if isinstance(v, int):
        return ("int", v)
    if isinstance(v, float):
        return ("float", v)
    s = str(v).strip()
    if s.lower() in ("true", "false"):
        return ("bool", s.lower() == "true")
    try:
        return ("int", int(s))
    except ValueError:
        pass
    try:
        return ("float", float(s))
    except ValueError:
        pass
    return ("str", s)


def load_yaml_params(path):
    if not os.path.exists(path):
        MISSING_PATHS.append("param file does not exist: %s" % path)
        return {}
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    params = {}
    for key, body in doc.items():
        if isinstance(body, dict) and "ros__parameters" in body:
            params.update(body["ros__parameters"])
    return {k: norm_value(v) for k, v in params.items()}


def elements(node):
    return [c for c in node.childNodes if c.nodeType == c.ELEMENT_NODE]


def collect_nodes(path):
    """Walk a launch file, returning [{namespace, pkg, exec, remaps, params, ...}]."""
    dom = minidom.parse(path)
    launch = dom.getElementsByTagName("launch")[0]
    args = {}
    found = []

    def walk(el, ns, remaps):
        for child in elements(el):
            tag = child.tagName
            if tag == "arg":
                name = child.getAttribute("name")
                default = child.getAttribute("default")
                args[name] = substitute(default, args)
            elif tag == "group":
                gns, gremaps = list(ns), dict(remaps)
                for gchild in elements(child):
                    if gchild.tagName == "push-ros-namespace":
                        gns.append(substitute(gchild.getAttribute("namespace"), args))
                    elif gchild.tagName == "set_remap":
                        gremaps[substitute(gchild.getAttribute("from"), args)] = substitute(
                            gchild.getAttribute("to"), args
                        )
                walk(child, gns, gremaps)
            elif tag == "node":
                nns = list(ns)
                if child.getAttribute("namespace"):
                    nns.append(substitute(child.getAttribute("namespace"), args))
                nremaps = dict(remaps)
                nparams = {}
                for nchild in elements(child):
                    if nchild.tagName == "remap":
                        nremaps[substitute(nchild.getAttribute("from"), args)] = substitute(
                            nchild.getAttribute("to"), args
                        )
                    elif nchild.tagName == "param":
                        if nchild.getAttribute("from"):
                            yaml_path = substitute(nchild.getAttribute("from"), args)
                            nparams.update(load_yaml_params(yaml_path))
                        elif nchild.getAttribute("name"):
                            nparams[nchild.getAttribute("name")] = norm_value(
                                nchild.getAttribute("value")
                            )
                found.append(
                    {
                        "namespace": "/" + "/".join(n for n in nns if n),
                        "pkg": child.getAttribute("pkg"),
                        "exec": child.getAttribute("exec"),
                        "remaps": nremaps,
                        "params": nparams,
                        "respawn": child.getAttribute("respawn") or "",
                        "respawn_delay": child.getAttribute("respawn_delay") or "",
                    }
                )
            elif tag in ("include",):
                continue
            else:
                walk(child, ns, remaps)

    walk(launch, [], {})
    return found


COMPILED_DEFAULT_RE = re.compile(
    r'get_param\(\s*(?:this|node)\s*,\s*"%s"\s*,\s*([^)]+?)\s*\)'
)


def compiled_default(pkg, param):
    """Find `airstack::get_param(this|node, "param", <default>)` in a package's C++."""
    pkg_dir = PKGS.get(pkg)
    if not pkg_dir:
        return None, None
    rx = re.compile(COMPILED_DEFAULT_RE.pattern % re.escape(param))
    for sub in ("src", "include"):
        base = os.path.join(pkg_dir, sub)
        for dirpath, _dirnames, filenames in os.walk(base):
            for fn in filenames:
                if not fn.endswith((".cpp", ".hpp", ".h", ".cc")):
                    continue
                fp = os.path.join(dirpath, fn)
                with open(fp, errors="replace") as f:
                    text = f.read()
                m = rx.search(text)
                if m:
                    raw = m.group(1).strip()
                    raw = raw.replace("std::string(", "").rstrip(")").strip().strip('"')
                    return norm_value(raw), os.path.relpath(fp, SRC_ROOT)
    return None, None


def compare(module, legacy, canon):
    problems = []
    notes = []
    for field in ("namespace", "pkg", "exec"):
        if legacy[field] != canon[field]:
            problems.append(
                "  %-14s legacy=%r canonical=%r" % (field, legacy[field], canon[field])
            )

    lr, cr = legacy["remaps"], canon["remaps"]
    for key in sorted(set(lr) | set(cr)):
        if lr.get(key) != cr.get(key):
            problems.append(
                "  remap %-28s legacy=%r canonical=%r" % (key, lr.get(key), cr.get(key))
            )

    lp, cp = legacy["params"], canon["params"]
    for key in sorted(set(lp) | set(cp)):
        if key in lp and key in cp:
            if lp[key] != cp[key]:
                problems.append(
                    "  param %-34s legacy=%r canonical=%r" % (key, lp[key][1], cp[key][1])
                )
        elif key in cp:
            dflt, where = compiled_default(canon["pkg"], key)
            if dflt is not None and dflt == cp[key]:
                notes.append(
                    "  param %-34s canonical-only, equals compiled default %r (%s)"
                    % (key, cp[key][1], where)
                )
            else:
                problems.append(
                    "  param %-34s canonical-only=%r, compiled default=%r -> CHANGES BEHAVIOUR"
                    % (key, cp[key][1], dflt[1] if dflt else None)
                )
        else:
            problems.append("  param %-34s legacy-only=%r, DROPPED" % (key, lp[key][1]))
    return problems, notes


def main():
    if not os.path.exists(LEGACY):
        sys.exit("legacy launch file not found: %s" % LEGACY)
    legacy_nodes = collect_nodes(LEGACY)
    legacy_by_id = {(n["pkg"], n["exec"]): n for n in legacy_nodes}

    print("legacy   : %s" % os.path.relpath(LEGACY, SRC_ROOT))
    print("packages : %d indexed under %s" % (len(PKGS), SRC_ROOT))
    print()

    failures = 0
    for module, (rel, ident) in CANONICAL.items():
        path = os.path.join(SRC_ROOT, rel)
        print("=== %s" % module)
        print("  file        : %s" % rel)
        if not os.path.exists(path):
            print("  FAIL        : canonical launch file missing")
            failures += 1
            continue
        try:
            minidom.parse(path)
        except Exception as exc:
            print("  FAIL        : XML does not parse: %s" % exc)
            failures += 1
            continue
        canon_nodes = [n for n in collect_nodes(path) if (n["pkg"], n["exec"]) == ident]
        if len(canon_nodes) != 1:
            print("  FAIL        : expected 1 %s/%s node, found %d" % (ident + (len(canon_nodes),)))
            failures += 1
            continue
        if ident not in legacy_by_id:
            print("  FAIL        : %s/%s is not launched by local.launch.xml" % ident)
            failures += 1
            continue
        canon = canon_nodes[0]
        legacy = legacy_by_id[ident]
        problems, notes = compare(module, legacy, canon)
        print("  namespace   : %s" % canon["namespace"])
        print("  node        : %s / %s" % (canon["pkg"], canon["exec"]))
        print("  remaps      : %d" % len(canon["remaps"]))
        print("  params      : %d" % len(canon["params"]))
        if canon["respawn"] or legacy["respawn"]:
            print(
                "  respawn     : legacy=%r/%r canonical=%r/%r"
                % (
                    legacy["respawn"],
                    legacy["respawn_delay"],
                    canon["respawn"],
                    canon["respawn_delay"],
                )
            )
        for note in notes:
            print("  NOTE" + note[1:])
        if problems:
            failures += 1
            print("  FAIL        : %d difference(s) from local.launch.xml" % len(problems))
            for p in problems:
                print("   " + p)
        else:
            print("  OK          : graph identical to local.launch.xml")
        print()

    if MISSING_PATHS:
        failures += 1
        print("MISSING PATHS:")
        for m in sorted(set(MISSING_PATHS)):
            print("  " + m)
        print()

    if failures:
        print("RESULT: FAIL (%d)" % failures)
        return 1
    print("RESULT: PASS - all 5 canonical launch files reproduce local.launch.xml's graph")
    return 0


if __name__ == "__main__":
    sys.exit(main())
