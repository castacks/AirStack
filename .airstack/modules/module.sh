#!/usr/bin/env bash

# module.sh — `airstack module` command group (RFC #379 Phase P2).
#
# Manages AirStack *modules*: thin external repos (or local checkouts) declared
# in ./modules.repos (vcstool/vcs2l format, pinned to tags/SHAs — never
# branches), synced into the gitignored ./modules/ directory, and overlaid into
# the trunk checkout by tools/module_overlay.py (colcon symlinks, Isaac launch
# scripts + Kit extension mounts, generated compose override).
#
# Subcommands: add | remove | list | sync | create | lock | doctor | help
# See `airstack help module` and docs/development/modules.md.
#
# Dispatcher note: airstack.sh's fallback argument scan filters ALL occurrences
# of the matched command token, so an argument literally named "module"
# (e.g. `airstack module remove module`) would be swallowed. Don't name a
# module "module".

MODULE_REPOS_FILE="${PROJECT_ROOT}/modules.repos"
MODULE_CHECKOUT_DIR="${PROJECT_ROOT}/modules"
MODULE_OVERLAY_TOOL="${PROJECT_ROOT}/tools/module_overlay.py"
MODULE_VALIDATOR_TOOL="${PROJECT_ROOT}/tools/validate_module.py"
# (P4) Docker layer composition planner — RFC #379 §6; see docs/development/modules.md
MODULE_LAYER_TOOL="${PROJECT_ROOT}/tools/compose_module_layers.py"
MODULE_GENERATED_COMPOSE=".airstack/generated/docker-compose.modules.yaml"
MODULE_INTREE_DIR="${PROJECT_ROOT}/robot/ros_ws/src/modules"

# Refs that look like branches — the RFC #379 §3 pinning rule: a branch ref
# rots silently, so `module add` refuses them. Pin a tag or a commit SHA.
MODULE_BRANCHLIKE_REFS="main master develop devel dev latest trunk head HEAD rolling"

# python/PyYAML availability check: shared _require_python_yaml (_lib.sh).
function _module_check_python {
    _require_python_yaml "'airstack module' commands"
}

# Ensure a `vcs` binary exists. vcs2l is the MAINTAINED successor of vcstool
# (https://github.com/ros-infrastructure/vcs2l) — never install the legacy
# dirk-thomas vcstool. An already-present `vcs` binary is used as-is, but we
# print which provider it comes from so surprises are debuggable.
function _module_ensure_vcs {
    if command -v vcs >/dev/null 2>&1; then
        local provider="unknown"
        if pip3 show vcs2l >/dev/null 2>&1; then
            provider="vcs2l"
        elif pip3 show vcstool >/dev/null 2>&1; then
            provider="vcstool (legacy — consider migrating to vcs2l)"
        fi
        log_info "Using vcs binary: $(command -v vcs) (provider: ${provider})"
        return 0
    fi
    log_info "No 'vcs' binary found — installing vcs2l (maintained vcstool successor) via pip3 --user..."
    if ! pip3 install --user vcs2l; then
        log_error "pip3 install --user vcs2l failed. Install it manually and re-run."
        return 1
    fi
    export PATH="$HOME/.local/bin:$PATH"
    if ! command -v vcs >/dev/null 2>&1; then
        log_error "vcs2l installed but 'vcs' is still not on PATH (expected ~/.local/bin/vcs)."
        return 1
    fi
    log_info "Using vcs binary: $(command -v vcs) (provider: vcs2l)"
}

# ── modules.repos helpers (python heredocs; values passed via environment) ──

function _module_repos_upsert {
    # env: MODULE_ENTRY_NAME, MODULE_ENTRY_KIND (git|local), MODULE_ENTRY_URL,
    #      MODULE_ENTRY_VERSION, MODULE_ENTRY_PATH
    MODULE_REPOS_FILE="$MODULE_REPOS_FILE" python3 - <<'PY'
import os, yaml

path = os.environ["MODULE_REPOS_FILE"]
name = os.environ["MODULE_ENTRY_NAME"]
kind = os.environ["MODULE_ENTRY_KIND"]

data = {}
if os.path.exists(path):
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

repos = data.get("repositories") or {}
local = data.get("x-local-modules") or []

# an entry lives in exactly one of the two lists
local = [e for e in local if e.get("name") != name]
repos.pop(name, None)

if kind == "git":
    repos[name] = {
        "type": "git",
        "url": os.environ["MODULE_ENTRY_URL"],
        "version": os.environ["MODULE_ENTRY_VERSION"],
    }
else:
    local.append({"name": name, "path": os.environ["MODULE_ENTRY_PATH"]})

data["repositories"] = repos
data["x-local-modules"] = local

header = (
    "# AirStack modules (RFC #379 §3) — managed by `airstack module add/remove`.\n"
    "# `repositories:` is vcstool/vcs2l format, PINNED to tags/SHAs (never branches).\n"
    "# `x-local-modules:` records local-path modules; vcs tools ignore this key.\n"
)
with open(path, "w", encoding="utf-8") as f:
    f.write(header)
    yaml.safe_dump(data, f, sort_keys=True, default_flow_style=False)
PY
}

function _module_repos_remove {
    # env: MODULE_ENTRY_NAME. Prints "removed" or "absent".
    MODULE_REPOS_FILE="$MODULE_REPOS_FILE" python3 - <<'PY'
import os, yaml

path = os.environ["MODULE_REPOS_FILE"]
name = os.environ["MODULE_ENTRY_NAME"]

if not os.path.exists(path):
    print("absent")
    raise SystemExit(0)

with open(path, encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
repos = data.get("repositories") or {}
local = data.get("x-local-modules") or []

found = name in repos or any(e.get("name") == name for e in local)
repos.pop(name, None)
local = [e for e in local if e.get("name") != name]

extra_keys = {k for k in data if k not in ("repositories", "x-local-modules")}
if not repos and not local and not extra_keys:
    os.remove(path)
else:
    data["repositories"] = repos
    data["x-local-modules"] = local
    header = (
        "# AirStack modules (RFC #379 §3) — managed by `airstack module add/remove`.\n"
        "# `repositories:` is vcstool/vcs2l format, PINNED to tags/SHAs (never branches).\n"
        "# `x-local-modules:` records local-path modules; vcs tools ignore this key.\n"
    )
    with open(path, "w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(data, f, sort_keys=True, default_flow_style=False)

print("removed" if found else "absent")
PY
}

# Prints "<name>\t<abs_path>" per local module; nothing when file absent.
function _module_local_entries {
    [ -f "$MODULE_REPOS_FILE" ] || return 0
    MODULE_PROJECT_ROOT="$PROJECT_ROOT" MODULE_REPOS_FILE="$MODULE_REPOS_FILE" python3 - <<'PY'
import os, yaml

root = os.environ["MODULE_PROJECT_ROOT"]
with open(os.environ["MODULE_REPOS_FILE"], encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
for entry in data.get("x-local-modules") or []:
    path = entry.get("path", "")
    if not os.path.isabs(path):
        path = os.path.join(root, path)
    print(f"{entry.get('name')}\t{os.path.normpath(path)}")
PY
}

function _module_git_count {
    if [ ! -f "$MODULE_REPOS_FILE" ]; then
        echo 0
        return 0
    fi
    MODULE_REPOS_FILE="$MODULE_REPOS_FILE" python3 - <<'PY'
import os, yaml
with open(os.environ["MODULE_REPOS_FILE"], encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
print(len(data.get("repositories") or {}))
PY
}

# ── subcommands ──────────────────────────────────────────────────────────────

function cmd_module_add {
    _module_check_python || return 1

    local src="" version="" no_hooks=false
    while [ $# -gt 0 ]; do
        case "$1" in
            --version)
                [ $# -ge 2 ] || { log_error "--version requires a value (tag or commit SHA)"; return 1; }
                version="$2"; shift 2 ;;
            --no-hooks) no_hooks=true; shift ;;
            -*) log_error "Unknown option for 'module add': $1"; return 1 ;;
            *)
                if [ -n "$src" ]; then
                    log_error "'module add' takes exactly one <git-url|local-path> (got '$src' and '$1')"
                    return 1
                fi
                src="$1"; shift ;;
        esac
    done
    if [ -z "$src" ]; then
        log_error "Usage: airstack module add <git-url|local-path> [--version <tag-or-sha>] [--no-hooks]"
        return 1
    fi

    local name
    if [ -d "$src" ]; then
        # ── local-path module ────────────────────────────────────────────
        if [ ! -f "$src/module.yaml" ]; then
            log_error "'$src' has no module.yaml — not an AirStack module."
            return 1
        fi
        local abs_path record_path
        abs_path="$(cd "$src" && pwd)"
        name="$(basename "$abs_path")"
        # record relative to the project root when inside it (portable across
        # machines sharing the checkout layout), absolute otherwise
        record_path="$(MODULE_ABS="$abs_path" MODULE_ROOT="$PROJECT_ROOT" python3 -c '
import os
abs_path = os.environ["MODULE_ABS"]; root = os.environ["MODULE_ROOT"]
rel = os.path.relpath(abs_path, root)
print(abs_path if rel.startswith("..") else rel)')"
        log_info "Adding local module '${name}' from ${record_path}"
        MODULE_ENTRY_NAME="$name" MODULE_ENTRY_KIND="local" \
            MODULE_ENTRY_PATH="$record_path" _module_repos_upsert || return 1
    else
        # ── git-url module ───────────────────────────────────────────────
        if [ -z "$version" ]; then
            log_error "Remote modules must be pinned: --version <tag-or-sha> is required."
            log_error "  (RFC #379 §3: pinned .repos files are non-negotiable — a branch ref rots silently.)"
            return 1
        fi
        local ref
        for ref in $MODULE_BRANCHLIKE_REFS; do
            if [ "$version" = "$ref" ]; then
                log_error "'${version}' looks like a branch, not a pin. Pin a tag (v0.1.0) or a commit SHA."
                log_error "  (RFC #379 §3 pinning rule: never track branches in modules.repos.)"
                return 1
            fi
        done
        name="$(basename "$src")"
        name="${name%.git}"
        log_info "Adding module '${name}' from ${src} @ ${version}"
        MODULE_ENTRY_NAME="$name" MODULE_ENTRY_KIND="git" \
            MODULE_ENTRY_URL="$src" MODULE_ENTRY_VERSION="$version" _module_repos_upsert || return 1
    fi

    local sync_args=()
    [ "$no_hooks" = true ] && sync_args+=(--no-hooks)
    cmd_module_sync "${sync_args[@]}"
}

function cmd_module_sync {
    _module_check_python || return 1

    local no_hooks=false
    while [ $# -gt 0 ]; do
        case "$1" in
            --no-hooks) no_hooks=true; shift ;;
            *) log_error "Unknown option for 'module sync': $1"; return 1 ;;
        esac
    done

    if [ ! -f "$MODULE_REPOS_FILE" ] && [ ! -d "$MODULE_CHECKOUT_DIR" ]; then
        log_info "No modules.repos and no modules/ directory — nothing to sync."
        # Still let the overlay clean any leftover artifacts (links, compose).
        python3 "$MODULE_OVERLAY_TOOL" --project-root "$PROJECT_ROOT" || return 1
        # (P4) …and the layer planner clean stale layer_plan.json / modules.lock.
        python3 "$MODULE_LAYER_TOOL" --project-root "$PROJECT_ROOT" || return 1
        return 0
    fi

    mkdir -p "$MODULE_CHECKOUT_DIR"

    # 1. git modules via vcs import (recursive → submodules too)
    local git_count
    git_count="$(_module_git_count)"
    if [ "$git_count" -gt 0 ]; then
        _module_ensure_vcs || return 1
        # Self-heal: a managed checkout dir without .git (interrupted clone /
        # partial remove) makes `vcs import` fail with "destination path
        # already exists" — clear it so the import can reclone.
        local _mod_name _mod_dir
        while IFS= read -r _mod_name; do
            _mod_dir="$MODULE_CHECKOUT_DIR/$_mod_name"
            if [ -d "$_mod_dir" ] && [ ! -e "$_mod_dir/.git" ]; then
                log_warn "modules/${_mod_name} exists without .git (stale partial checkout) — clearing for reclone"
                rm -rf "$_mod_dir"
            fi
        done < <(MODULE_REPOS_FILE="$MODULE_REPOS_FILE" python3 - <<'PY'
import os, yaml
with open(os.environ["MODULE_REPOS_FILE"], encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
for name in (data.get("repositories") or {}):
    print(name)
PY
)
        log_info "Importing ${git_count} pinned module repo(s) into modules/ (vcs import --recursive)..."
        if ! vcs import "$MODULE_CHECKOUT_DIR" --input "$MODULE_REPOS_FILE" --recursive; then
            log_error "vcs import failed — check the URLs/pins in modules.repos."
            return 1
        fi
    fi

    # 2. local-path modules → symlink into modules/<name>
    local line lname lpath
    while IFS=$'\t' read -r lname lpath; do
        [ -n "$lname" ] || continue
        if [ ! -d "$lpath" ]; then
            log_error "Local module '${lname}' points at missing directory: ${lpath}"
            return 1
        fi
        ln -sfn "$lpath" "$MODULE_CHECKOUT_DIR/$lname"
        log_info "Linked local module '${lname}' -> ${lpath}"
    done < <(_module_local_entries)

    # 3. validate every synced manifest (tools/validate_module.py — P1 contract)
    local module_dir failed=0 have_any=0
    for module_dir in "$MODULE_CHECKOUT_DIR"/*/; do
        [ -d "$module_dir" ] || continue
        have_any=1
        if ! python3 "$MODULE_VALIDATOR_TOOL" "$module_dir" >/dev/null; then
            log_error "Invalid module manifest: ${module_dir}module.yaml (errors above)"
            failed=1
        fi
    done
    if [ "$failed" -ne 0 ]; then
        log_error "Module manifest validation failed — fix the errors above and re-run 'airstack module sync'."
        return 1
    fi

    # 4. overlay: colcon symlinks, isaac launch scripts, generated compose
    python3 "$MODULE_OVERLAY_TOOL" --project-root "$PROJECT_ROOT" || return 1

    # 4b. (P4) Docker layer composition (RFC #379 §6): dep-conflict hard gate,
    #     then per-host layer plan + modules.lock (plan-only — no docker calls).
    if ! python3 "$MODULE_LAYER_TOOL" --check-conflicts --project-root "$PROJECT_ROOT"; then
        log_error "Module dependency conflict — sync refuses to compose a broken image (RFC #379 §6)."
        return 1
    fi
    python3 "$MODULE_LAYER_TOOL" --project-root "$PROJECT_ROOT" || return 1
    # Plan-only regeneration DROPS any image: overrides a prior
    # `module lock --build` wrote into the generated compose — containers
    # would silently run the base image without the module's dep layers
    # (e.g. macvo without torch). Warn loudly when that just happened.
    if [ -f "$PROJECT_ROOT/.airstack/generated/layer_plan.json" ] && \
       ! grep -q "image:" "$PROJECT_ROOT/$MODULE_GENERATED_COMPOSE" 2>/dev/null && \
       python3 -c "
import json,sys
plan = json.load(open('$PROJECT_ROOT/.airstack/generated/layer_plan.json'))
sys.exit(0 if any(h.get('steps') for h in plan.values()) else 1)" 2>/dev/null; then
        log_warn "docker-relevant module layers exist but the generated compose has NO image overrides —"
        log_warn "containers will run the BASE image without module dep layers. Run: airstack module lock --build"
    fi

    # 5. host_setup hooks (idempotent, no sudo, write only inside the module)
    if [ "$no_hooks" = true ]; then
        log_info "Skipping host_setup hooks (--no-hooks)."
    else
        for module_dir in "$MODULE_CHECKOUT_DIR"/*/; do
            [ -f "${module_dir}module.yaml" ] || continue
            local hook
            hook="$(MODULE_MANIFEST="${module_dir}module.yaml" python3 -c '
import os, yaml
with open(os.environ["MODULE_MANIFEST"], encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
print((data.get("hooks") or {}).get("host_setup") or "")')"
            if [ -n "$hook" ]; then
                log_info "Running host_setup hook for $(basename "$module_dir"): ${hook}"
                if ! (cd "$module_dir" && bash "$hook"); then
                    log_error "host_setup hook failed for $(basename "$module_dir") (${hook})."
                    return 1
                fi
            fi
        done
    fi

    if [ "$have_any" -eq 1 ]; then
        log_info "Module sync complete."
        if [ -f "$PROJECT_ROOT/$MODULE_GENERATED_COMPOSE" ]; then
            log_info "Module mounts are included automatically by 'airstack up' (${MODULE_GENERATED_COMPOSE})."
            log_info "(opt out with AIRSTACK_NO_MODULE_COMPOSE=1 — see docs/development/modules.md)"
        fi
        if ls "$PROJECT_ROOT/simulation/isaac-sim/launch_scripts/modules" >/dev/null 2>&1; then
            log_info "Isaac module launch scripts are addressable as ISAAC_SIM_SCRIPT_NAME=modules/<module>/<script>.py"
        fi
    else
        log_info "No modules checked out — overlay cleaned."
    fi
}

function cmd_module_list {
    _module_check_python || return 1
    MODULE_PROJECT_ROOT="$PROJECT_ROOT" \
    MODULE_VALIDATOR="$MODULE_VALIDATOR_TOOL" python3 - <<'PY'
import importlib.util, os, yaml

root = os.environ["MODULE_PROJECT_ROOT"]
repos_file = os.path.join(root, "modules.repos")
checkout_dir = os.path.join(root, "modules")

spec = importlib.util.spec_from_file_location("airstack_validate_module", os.environ["MODULE_VALIDATOR"])
validator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validator)

entries = {}   # name -> dict(version=..., source=...)
if os.path.exists(repos_file):
    with open(repos_file, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    for name, repo in (data.get("repositories") or {}).items():
        entries[name] = {"version": repo.get("version", "?"), "source": repo.get("url", "?")}
    for entry in data.get("x-local-modules") or []:
        entries[entry["name"]] = {"version": "local", "source": entry.get("path", "?")}
# checkouts not tracked in modules.repos still show up (flagged)
if os.path.isdir(checkout_dir):
    for child in sorted(os.listdir(checkout_dir)):
        if os.path.isdir(os.path.join(checkout_dir, child)) and child not in entries:
            entries[child] = {"version": "(untracked)", "source": "(not in modules.repos)"}

if not entries:
    print("No modules. Add one with: airstack module add <git-url|local-path> [--version <pin>]")
    raise SystemExit(0)

rows = []
for name in sorted(entries):
    meta = entries[name]
    module_dir = os.path.join(checkout_dir, name)
    if not os.path.isdir(module_dir):
        rows.append((name, "?", meta["version"], "?", "not synced"))
        continue
    manifest_path = os.path.join(module_dir, "module.yaml")
    mtype, targets = "?", "?"
    if os.path.isfile(manifest_path):
        try:
            with open(manifest_path, encoding="utf-8") as f:
                manifest = yaml.safe_load(f) or {}
            mtype = manifest.get("type", "?")
            targets = ",".join(manifest.get("targets") or []) or "?"
        except yaml.YAMLError:
            pass
    verdict, _warnings = validator.validate_module(module_dir)
    valid = "yes" if verdict["valid"] else "NO"
    rows.append((name, mtype, meta["version"], targets, valid))

headers = ("NAME", "TYPE", "VERSION/PIN", "TARGETS", "VALID")
widths = [max(len(str(r[i])) for r in rows + [headers]) for i in range(5)]
fmt = "  ".join("{:<%d}" % w for w in widths)
print(fmt.format(*headers))
for row in rows:
    print(fmt.format(*row))
PY
}

function cmd_module_remove {
    _module_check_python || return 1

    local name="${1:-}"
    if [ -z "$name" ] || [ $# -gt 1 ]; then
        log_error "Usage: airstack module remove <name>"
        return 1
    fi

    local status
    status="$(MODULE_ENTRY_NAME="$name" _module_repos_remove)" || return 1
    local had_checkout=false
    if [ -L "$MODULE_CHECKOUT_DIR/$name" ] || [ -d "$MODULE_CHECKOUT_DIR/$name" ]; then
        had_checkout=true
    fi
    if [ "$status" = "absent" ] && [ "$had_checkout" = false ]; then
        log_error "Module '${name}' is not in modules.repos and has no checkout under modules/."
        return 1
    fi

    # drop the checkout (symlink for local modules, real clone for git ones)
    if [ -L "$MODULE_CHECKOUT_DIR/$name" ]; then
        rm "$MODULE_CHECKOUT_DIR/$name"
    elif [ -d "$MODULE_CHECKOUT_DIR/$name" ]; then
        # Containers drop root-owned artifacts (__pycache__, build debris)
        # into mounted module checkouts; a plain rm then fails and leaves a
        # partial dir that breaks the next `vcs import`. Retry via a
        # throwaway container when that happens.
        if ! rm -rf "$MODULE_CHECKOUT_DIR/$name" 2>/dev/null; then
            log_warn "checkout has root-owned files (created in-container) — removing via docker..."
            if ! docker run --rm -v "$MODULE_CHECKOUT_DIR:/m" ubuntu:24.04 \
                    bash -c "rm -rf /m/$name"; then
                log_error "could not remove modules/$name — remove it manually, e.g.:"
                log_error "  docker run --rm -v \"$MODULE_CHECKOUT_DIR:/m\" ubuntu:24.04 rm -rf /m/$name"
                return 1
            fi
        fi
    fi

    # regenerate the overlay without it (also prunes links + compose entries)
    python3 "$MODULE_OVERLAY_TOOL" --project-root "$PROJECT_ROOT" --remove "$name" || return 1

    # (P4) regenerate the layer plan + modules.lock without it (removes both
    # when no modules remain, so remove still restores a clean tree)
    python3 "$MODULE_LAYER_TOOL" --project-root "$PROJECT_ROOT" || return 1

    rmdir "$MODULE_CHECKOUT_DIR" 2>/dev/null || true
    log_info "Removed module '${name}' (modules.repos entry, checkout, and overlay artifacts)."
}

function cmd_module_create {
    _module_check_python || return 1

    local in_tree=false name=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --in-tree) in_tree=true; shift ;;
            -*) log_error "Unknown option for 'module create': $1"; return 1 ;;
            *)
                if [ -n "$name" ]; then
                    log_error "'module create' takes exactly one <name>"
                    return 1
                fi
                name="$1"; shift ;;
        esac
    done
    if [ "$in_tree" != true ] || [ -z "$name" ]; then
        log_error "Usage: airstack module create --in-tree <name>"
        log_error "  (standalone module-repo scaffolding is Phase P4 — 'module extract' graduates an in-tree module)"
        return 1
    fi
    if ! [[ "$name" =~ ^[a-z][a-z0-9_]*$ ]]; then
        log_error "Module name must be lowercase snake_case starting with a letter: '$name'"
        return 1
    fi

    local dest="$MODULE_INTREE_DIR/$name"
    if [ -e "$dest" ]; then
        log_error "Already exists: $dest"
        return 1
    fi

    local maintainer version compat
    maintainer="$(git config user.email 2>/dev/null || true)"
    [ -n "$maintainer" ] || maintainer="todo@example.com"
    version="$(get_VERSION)"
    compat="$(MODULE_VERSION="$version" python3 -c '
import os, re
v = os.environ["MODULE_VERSION"]
m = re.match(r"^(\d+)\.(\d+)\.(\d+)", v)
if m:
    major, minor = int(m.group(1)), int(m.group(2))
    print(f">={v} <{major}.{minor + 2}.0")
else:
    print(">=0.19.0 <0.21.0")')"

    log_info "Scaffolding in-tree module at robot/ros_ws/src/modules/${name}/ ..."
    mkdir -p "$dest/$name/$name" "$dest/$name/resource" "$dest/$name/launch" "$dest/$name/test"

    cat > "$dest/module.yaml" <<EOF
# AirStack module manifest (RFC #379 §2) — deps, identity, and test metadata
# only. Wiring (topics, remaps) deliberately does NOT live here: the module's
# interface is its launch file's declared args + the interface conventions.
# Validate with: python3 tools/validate_module.py robot/ros_ws/src/modules/${name}
name: ${name}
description: "TODO: one-line description of ${name} (min 8 chars)"
maintainer: ${maintainer}
license: TODO
type: ros_package                # isaac_extension | ros_package | data | platform
airstack_compat: "${compat}"
targets: [robot]                 # host containers touched: robot | gcs | isaac-sim | ms-airsim

deps: {apt: [], pip: []}         # dep tier 1; rosdep keys live in package.xml
dockerfile: null                 # tier 2: Dockerfile.module using ARG BASE_IMAGE
overlay_image: null              # tier 3: prebuilt overlay image ref
compose: null                    # optional compose fragment (mounts, env)

assets: []
docs: {readme: README.md}
foxglove: null

tests:
  packages: [${name}]
  marks: []
EOF

    cat > "$dest/README.md" <<EOF
# ${name}

TODO: what this module does, its inputs/outputs (launch args with canonical
defaults), and how to run it. See docs/development/modules.md.
EOF

    cat > "$dest/$name/package.xml" <<EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>${name}</name>
  <version>0.1.0</version>
  <description>TODO: package description</description>
  <maintainer email="${maintainer}">TODO</maintainer>
  <license>TODO</license>

  <exec_depend>rclpy</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

    cat > "$dest/$name/setup.py" <<EOF
from setuptools import setup

package_name = "${name}"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/${name}.launch.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="${maintainer}",
    description="TODO: package description",
    license="TODO",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "${name}_node = ${name}.${name}_node:main",
        ],
    },
)
EOF

    cat > "$dest/$name/setup.cfg" <<EOF
[develop]
script_dir=\$base/lib/${name}
[install]
install_scripts=\$base/lib/${name}
EOF

    touch "$dest/$name/resource/$name"

    cat > "$dest/$name/$name/__init__.py" <<EOF
__version__ = "0.1.0"
EOF

    cat > "$dest/$name/$name/${name}_node.py" <<EOF
"""TODO: implement ${name}.

Topic endpoints arrive as parameters set from the launch file's declared args
(canonical defaults) — never hardcode topic names in node code.
"""
import rclpy
from rclpy.node import Node


class ${name^}Node(Node):  # TODO: rename to a CamelCase class name you like
    def __init__(self):
        super().__init__("${name}")
        # Canonical-default topic endpoints (RFC #379 §2): the launch file
        # declares these as args and passes them down as parameters.
        self.declare_parameter("odometry_topic", "odometry")
        # TODO: subscriptions/publishers, e.g.:
        # from nav_msgs.msg import Odometry
        # topic = self.get_parameter("odometry_topic").value
        # self.create_subscription(Odometry, topic, self.on_odometry, 10)


def main(args=None):
    rclpy.init(args=args)
    node = ${name^}Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
EOF

    cat > "$dest/$name/launch/${name}.launch.xml" <<EOF
<?xml version="1.0"?>
<!-- Module launch file (RFC #379 §2/§4): every topic endpoint is a declared
     launch arg DEFAULTING to the canonical name from the interface
     conventions. NO <remap> here and no hardcoded topics — cross-module
     wiring overrides live only in the stack's entry launch file. -->
<launch>
  <arg name="robot_name" default="\$(env ROBOT_NAME robot_1)" description="Robot namespace"/>
  <arg name="odometry_topic" default="odometry"
       description="Input: state estimate (nav_msgs/Odometry); canonical /\$(var robot_name)/odometry"/>

  <node pkg="${name}" exec="${name}_node" name="${name}"
        namespace="\$(var robot_name)" output="screen">
    <param name="odometry_topic" value="\$(var odometry_topic)"/>
  </node>
</launch>
EOF

    cat > "$dest/$name/test/test_import.py" <<EOF
"""Import smoke tests for ${name} (colcon co-located test/ convention)."""
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

_PKG_ROOT = Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))


def test_package_imports():
    import ${name}

    assert ${name}.__version__


def test_node_module_imports_when_rclpy_available():
    pytest.importorskip("rclpy")
    from ${name} import ${name}_node

    assert callable(${name}_node.main)
EOF

    log_info "Scaffolded ${dest}"
    if ! python3 "$MODULE_VALIDATOR_TOOL" "$dest" >/dev/null; then
        log_warn "Stub manifest needs the TODOs filled before it validates (errors above)."
    fi
    log_warn "Note: robot/ros_ws/src/modules/ is gitignored in trunk (overlay symlinks live there)."
    log_warn "  In your research fork, commit the module with 'git add -f robot/ros_ws/src/modules/${name}'"
    log_warn "  or un-ignore the path in your fork's .gitignore."
    log_info "Next steps:"
    log_info "  1. Fill the TODOs in module.yaml, package.xml, setup.py"
    log_info "  2. Build in the robot container: docker exec airstack-robot-desktop-1 bash -c 'bws --packages-select ${name}'"
    log_info "  3. Track extraction debt as you work: airstack module doctor --drift"
}

# (P4) `airstack module lock` — alias for the Docker layer planner: recompute
# .airstack/generated/layer_plan.json + modules.lock (RFC #379 §6). Extra flags
# pass through (--check-conflicts, --build).
function cmd_module_lock {
    _module_check_python || return 1
    python3 "$MODULE_LAYER_TOOL" --project-root "$PROJECT_ROOT" "$@"
}

function cmd_module_doctor {
    _module_check_python || return 1

    local drift=false
    while [ $# -gt 0 ]; do
        case "$1" in
            --drift) drift=true; shift ;;
            *) log_error "Unknown option for 'module doctor': $1"; return 1 ;;
        esac
    done

    if [ "$drift" = true ]; then
        # RFC #379 §11: classify changes vs the pinned base — informs, never blocks.
        local base="" ref
        for ref in origin/develop develop origin/main main; do
            if base="$(git -C "$PROJECT_ROOT" merge-base HEAD "$ref" 2>/dev/null)" && [ -n "$base" ]; then
                break
            fi
        done
        if [ -z "$base" ]; then
            log_warn "Could not find a merge-base with origin/develop (or develop/main) — no drift report."
            return 0
        fi
        local contained=() debt=() file
        while IFS= read -r file; do
            [ -n "$file" ] || continue
            case "$file" in
                robot/ros_ws/src/modules/*|modules/*) contained+=("$file") ;;
                *) debt+=("$file") ;;
            esac
        done < <(git -C "$PROJECT_ROOT" diff --name-only "$base" HEAD)

        echo ""
        log_info "Drift report vs merge-base ${base:0:12} (informs, never blocks — RFC #379 §11):"
        echo ""
        echo "  Contained in module directories (${#contained[@]} file(s)) — fine:"
        if [ ${#contained[@]} -gt 0 ]; then printf '    %s\n' "${contained[@]}"; else echo "    (none)"; fi
        echo ""
        echo "  Trunk edits = extraction debt (${#debt[@]} file(s)) — upstream, carry, or propose a convention:"
        if [ ${#debt[@]} -gt 0 ]; then printf '    %s\n' "${debt[@]}"; else echo "    (none)"; fi
        echo ""
        return 0
    fi

    # default: validate all manifests + overlay integrity
    local module_dir manifests_ok=true
    for module_dir in "$MODULE_CHECKOUT_DIR"/*/ "$MODULE_INTREE_DIR"/*/; do
        [ -d "$module_dir" ] || continue
        # in-tree dir: skip overlay symlinks (those are the modules/ checkouts,
        # already validated by the first glob)
        case "$module_dir" in
            "$MODULE_INTREE_DIR"/*) [ -L "${module_dir%/}" ] && continue ;;
        esac
        [ -f "${module_dir}module.yaml" ] || continue
        if python3 "$MODULE_VALIDATOR_TOOL" "$module_dir" >/dev/null; then
            log_info "manifest OK: ${module_dir}module.yaml"
        else
            log_error "manifest INVALID: ${module_dir}module.yaml (errors above)"
            manifests_ok=false
        fi
    done
    [ "$manifests_ok" = true ] || log_warn "Manifest problems reported above (doctor informs; sync is what fails on them)."

    if ! python3 "$MODULE_OVERLAY_TOOL" --check --project-root "$PROJECT_ROOT"; then
        log_error "Overlay is broken or stale — run 'airstack module sync'."
        return 1
    fi
    return 0
}

# Dispatcher for the `module` command group.
function cmd_module_dispatch {
    local sub="${1:-help}"
    if [ $# -gt 0 ]; then shift; fi
    case "$sub" in
        add)    cmd_module_add "$@" ;;
        remove) cmd_module_remove "$@" ;;
        list)   cmd_module_list "$@" ;;
        sync)   cmd_module_sync "$@" ;;
        create) cmd_module_create "$@" ;;
        lock)   cmd_module_lock "$@" ;;   # (P4) layer plan + modules.lock
        doctor) cmd_module_doctor "$@" ;;
        help|-h|--help) print_command_help module ;;
        *)
            log_error "Unknown module subcommand: '$sub'"
            print_command_help module
            return 1
            ;;
    esac
}

# Register commands from this module.
function register_module_commands {
    COMMANDS["module"]="cmd_module_dispatch"
    COMMAND_HELP["module"]="Manage AirStack modules: add|remove|list|sync|create|lock|doctor (RFC #379; see 'airstack help module')"
}
