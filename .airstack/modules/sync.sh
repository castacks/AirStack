#!/usr/bin/env bash

# sync.sh — `airstack sync` (RFC #380 §3, Phase P6).
#
# The checkout-level sync driven by ./airstack.yaml (the one hand-edited entry
# point). In order:
#   1. read airstack.yaml (absent file = plain module sync, nothing more)
#   2. upsert its `modules:` additions into modules.repos (naming every
#      deviation; bare {version:} without repo: is a named error — registry
#      resolution is future work)
#   3. run the module sync (modules.repos → modules/ → overlay → layer plan)
#   4. fetch declared external stack repos into gitignored stacks/.external/
#      <alias>/ (same vcs machinery as the module sync; pinned refs only)
#   5. validate the declared fleet file (tools/fleet/resolve_fleet.py)
#   6. write .airstack/generated/effective_sources.yaml — what this sync
#      actually resolved
#
# Deliberately NOT done (documented future work): rewriting .env (it stays
# hand-edited; airstack.yaml layers on top) and release-set pin resolution.
#
# Loads after module.sh (alphabetical module load order), so the module
# command group's helpers are available.

AIRSTACK_YAML_FILE="${PROJECT_ROOT}/airstack.yaml"
EXTERNAL_STACKS_DIR="${PROJECT_ROOT}/stacks/.external"
EFFECTIVE_SOURCES_FILE="${PROJECT_ROOT}/.airstack/generated/effective_sources.yaml"
FLEET_RESOLVER_TOOL="${PROJECT_ROOT}/tools/fleet/resolve_fleet.py"

# Read a scalar key from airstack.yaml (empty when absent).
function _sync_yaml_scalar {
    AIRSTACK_YAML_FILE="$AIRSTACK_YAML_FILE" SYNC_KEY="$1" python3 - <<'PY'
import os, yaml
path = os.environ["AIRSTACK_YAML_FILE"]
with open(path, encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
value = data.get(os.environ["SYNC_KEY"])
print(value if isinstance(value, (str, int, float)) else "")
PY
}

# Emit tab-separated rows for a mapping key: name<TAB>field1<TAB>field2.
# modules: name  kind(path|repo|version-only)  value  version
# stacks:  alias repo ref
function _sync_module_rows {
    AIRSTACK_YAML_FILE="$AIRSTACK_YAML_FILE" python3 - <<'PY'
import os, sys, yaml
with open(os.environ["AIRSTACK_YAML_FILE"], encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
for name, entry in (data.get("modules") or {}).items():
    entry = entry or {}
    if not isinstance(entry, dict):
        print(f"{name}\tinvalid\t\t")
        continue
    if entry.get("path"):
        print(f"{name}\tpath\t{entry['path']}\t")
    elif entry.get("repo"):
        print(f"{name}\trepo\t{entry['repo']}\t{entry.get('version', '')}")
    else:
        print(f"{name}\tversion-only\t\t{entry.get('version', '')}")
PY
}

function _sync_stack_rows {
    AIRSTACK_YAML_FILE="$AIRSTACK_YAML_FILE" python3 - <<'PY'
import os, yaml
with open(os.environ["AIRSTACK_YAML_FILE"], encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
for alias, entry in (data.get("stacks") or {}).items():
    entry = entry or {}
    print(f"{alias}\t{entry.get('repo', '')}\t{entry.get('ref', '')}")
PY
}

function cmd_sync {
    _require_python_yaml "'airstack sync'" || return 1
    local no_hooks=false
    while [ $# -gt 0 ]; do
        case "$1" in
            --no-hooks) no_hooks=true; shift ;;
            *)
                log_error "Usage: airstack sync [--no-hooks]   (configuration lives in airstack.yaml)"
                return 1
                ;;
        esac
    done

    local have_yaml=false
    if [ -f "$AIRSTACK_YAML_FILE" ]; then
        have_yaml=true
    else
        log_warn "No airstack.yaml at ${AIRSTACK_YAML_FILE} — running plain module sync only."
    fi

    local errors=0

    # ── 2. modules: additions → modules.repos (named deviations) ────────────
    if [ "$have_yaml" = true ]; then
        local name kind value version
        while IFS=$'\t' read -r name kind value version; do
            [ -n "$name" ] || continue
            case "$kind" in
                path)
                    local mod_path="$value"
                    [[ "$mod_path" != /* ]] && mod_path="$PROJECT_ROOT/$mod_path"
                    if [ ! -d "$mod_path" ]; then
                        log_error "airstack.yaml modules.${name}: path '$value' does not exist"
                        errors=1; continue
                    fi
                    log_info "airstack.yaml override: module '${name}' from local path ${value}"
                    MODULE_ENTRY_NAME="$name" MODULE_ENTRY_KIND="local" \
                        MODULE_ENTRY_PATH="$value" _module_repos_upsert || errors=1
                    ;;
                repo)
                    if [ -z "$version" ]; then
                        log_error "airstack.yaml modules.${name}: repo entries must pin a version: (tag or SHA)"
                        errors=1; continue
                    fi
                    local ref
                    for ref in $MODULE_BRANCHLIKE_REFS; do
                        if [ "$version" = "$ref" ]; then
                            log_error "airstack.yaml modules.${name}: '${version}' looks like a branch — pin a tag or SHA (RFC #379 §3)"
                            errors=1; continue 2
                        fi
                    done
                    log_info "airstack.yaml override: module '${name}' from ${value} @ ${version}"
                    MODULE_ENTRY_NAME="$name" MODULE_ENTRY_KIND="git" \
                        MODULE_ENTRY_URL="$value" MODULE_ENTRY_VERSION="$version" \
                        _module_repos_upsert || errors=1
                    ;;
                version-only)
                    log_error "airstack.yaml modules.${name}: {version: ${version:-?}} without repo: needs registry resolution — not implemented yet (RFC #379 §7). Give repo: or path:."
                    errors=1
                    ;;
                *)
                    log_error "airstack.yaml modules.${name}: entry must be a mapping with path: or repo:+version:"
                    errors=1
                    ;;
            esac
        done < <(_sync_module_rows)
    fi

    # ── 3. module sync (modules.repos → checkouts → overlay → layer plan) ───
    if declare -f cmd_module_sync >/dev/null; then
        local module_sync_args=()
        [ "$no_hooks" = true ] && module_sync_args+=(--no-hooks)
        cmd_module_sync "${module_sync_args[@]}" || errors=1
    else
        log_warn "module command group not loaded — skipping module sync."
    fi

    # ── 4. external stack repos → stacks/.external/<alias>/ ─────────────────
    local stack_count=0
    if [ "$have_yaml" = true ]; then
        # vcs availability is a per-sync precondition, not a per-row one —
        # check it once before the loop.
        local stack_rows
        stack_rows="$(_sync_stack_rows)"
        if [ -n "$stack_rows" ] && ! _module_ensure_vcs; then
            errors=1
            stack_rows=""
        fi
        local alias repo sref
        while IFS=$'\t' read -r alias repo sref; do
            [ -n "$alias" ] || continue
            if [ -z "$repo" ] || [ -z "$sref" ]; then
                log_error "airstack.yaml stacks.${alias}: needs repo: and a pinned ref:"
                errors=1; continue
            fi
            local bref
            for bref in $MODULE_BRANCHLIKE_REFS; do
                if [ "$sref" = "$bref" ]; then
                    log_error "airstack.yaml stacks.${alias}: ref '${sref}' looks like a branch — pin a tag or SHA (RFC #380 §3)"
                    errors=1; continue 2
                fi
            done
            mkdir -p "$EXTERNAL_STACKS_DIR"
            # Same self-heal as the module sync: a checkout without .git makes
            # vcs import refuse; clear it for a clean reclone.
            if [ -d "$EXTERNAL_STACKS_DIR/$alias" ] && [ ! -e "$EXTERNAL_STACKS_DIR/$alias/.git" ]; then
                log_warn "stacks/.external/${alias} exists without .git — clearing for reclone"
                rm -rf "${EXTERNAL_STACKS_DIR:?}/$alias"
            fi
            local tmp_repos
            tmp_repos="$(mktemp)"
            SYNC_ALIAS="$alias" SYNC_REPO="$repo" SYNC_REF="$sref" \
                TMP_REPOS="$tmp_repos" python3 - <<'PY'
import os, yaml
with open(os.environ["TMP_REPOS"], "w", encoding="utf-8") as f:
    yaml.safe_dump({"repositories": {os.environ["SYNC_ALIAS"]: {
        "type": "git",
        "url": os.environ["SYNC_REPO"],
        "version": os.environ["SYNC_REF"],
    }}}, f)
PY
            log_info "Fetching external stack repo '${alias}' (${repo} @ ${sref}) into stacks/.external/..."
            if ! vcs import "$EXTERNAL_STACKS_DIR" --input "$tmp_repos" --recursive; then
                log_error "vcs import failed for external stack repo '${alias}'."
                errors=1
            else
                stack_count=$((stack_count + 1))
            fi
            rm -f "$tmp_repos"
        done <<< "$stack_rows"
    fi

    # ── 5. validate the declared fleet ───────────────────────────────────────
    local fleet_file=""
    if [ "$have_yaml" = true ]; then
        fleet_file="$(_sync_yaml_scalar fleet)"
        if [ -n "$fleet_file" ]; then
            local fleet_host="$fleet_file"
            [[ "$fleet_host" != /* ]] && fleet_host="$PROJECT_ROOT/$fleet_file"
            if python3 "$FLEET_RESOLVER_TOOL" "$fleet_host" --project-root "$PROJECT_ROOT" --validate; then
                log_info "Fleet OK: ${fleet_file}"
            else
                log_error "Declared fleet failed validation: ${fleet_file} (errors above)"
                errors=1
            fi
        fi
        local sim_sel
        sim_sel="$(_sync_yaml_scalar sim)"
        case "$sim_sel" in
            ""|isaacsim|msairsim|none) ;;
            *) log_error "airstack.yaml sim: '${sim_sel}' (expected isaacsim | msairsim | none)"; errors=1;;
        esac
    fi

    # ── 6. record what this sync resolved ───────────────────────────────────
    mkdir -p "$(dirname "$EFFECTIVE_SOURCES_FILE")"
    AIRSTACK_YAML_FILE="$AIRSTACK_YAML_FILE" \
    PROJECT_ROOT_ENV="$PROJECT_ROOT" \
    EFFECTIVE_SOURCES_FILE="$EFFECTIVE_SOURCES_FILE" \
    SYNC_HAVE_YAML="$have_yaml" python3 - <<'PY'
import os, yaml

root = os.environ["PROJECT_ROOT_ENV"]
out_path = os.environ["EFFECTIVE_SOURCES_FILE"]

top = {}
if os.environ["SYNC_HAVE_YAML"] == "true":
    with open(os.environ["AIRSTACK_YAML_FILE"], encoding="utf-8") as f:
        top = yaml.safe_load(f) or {}

modules = {}
repos_path = os.path.join(root, "modules.repos")
if os.path.exists(repos_path):
    with open(repos_path, encoding="utf-8") as f:
        repos = yaml.safe_load(f) or {}
    for name, entry in (repos.get("repositories") or {}).items():
        modules[name] = {"source": entry.get("url"), "pin": entry.get("version")}
    for entry in repos.get("x-local-modules") or []:
        modules[entry.get("name")] = {"source": entry.get("path"), "pin": "local"}

stacks = {}
for alias, entry in (top.get("stacks") or {}).items():
    entry = entry or {}
    stacks[alias] = {
        "repo": entry.get("repo"),
        "ref": entry.get("ref"),
        "path": f"stacks/.external/{alias}",
    }

record = {
    "release": top.get("release"),
    "fleet": top.get("fleet"),
    "sim": top.get("sim"),
    "modules": modules,
    "external_stacks": stacks,
}
header = (
    "# GENERATED by `airstack sync` — what this checkout's sources resolved to\n"
    "# (airstack.yaml -> modules.repos / stacks/.external / fleet). DO NOT EDIT.\n"
)
with open(out_path, "w", encoding="utf-8") as f:
    f.write(header)
    yaml.safe_dump(record, f, sort_keys=True, default_flow_style=False)
print(f"Wrote {out_path}")
PY

    if [ "$errors" -ne 0 ]; then
        log_error "airstack sync finished with errors (named above)."
        return 1
    fi
    log_info "Sync complete."
    if [ "$stack_count" -gt 0 ]; then
        log_info "External stacks are addressable in fleets as <alias>/<stack> (stacks/.external/)."
    fi
    return 0
}

# Register commands from this module.
function register_sync_commands {
    COMMANDS["sync"]="cmd_sync"
    COMMAND_HELP["sync"]="Sync the checkout from airstack.yaml: modules, external stack repos, fleet validation (RFC #380 §3)"
}
