#!/usr/bin/env bash

# stack.sh — `airstack stack` command group (RFC #379 §3, Phase P5-E3).
#
# Manages AirStack *stacks*: self-contained folders under stacks/ composing
# modules into a running system (pinned modules.repos, launch/ entry points,
# docker-compose stub, generated wiring.md, README).
#
# Subcommands: list | new | diff | help
# See `airstack help stack` and docs/development/stacks.md.

STACKS_DIR="${PROJECT_ROOT}/stacks"
STACK_DIFF_TOOL="${PROJECT_ROOT}/tools/stack_diff.py"

function _stack_check_python {
    if ! command -v python3 >/dev/null 2>&1; then
        log_error "python3 is required for 'airstack stack' commands."
        return 1
    fi
    if ! python3 -c 'import yaml' 2>/dev/null; then
        log_error "PyYAML is required (pip3 install --user pyyaml)."
        return 1
    fi
}

# ── subcommands ──────────────────────────────────────────────────────────────

function cmd_stack_list {
    _stack_check_python || return 1
    if [ ! -d "$STACKS_DIR" ]; then
        log_error "No stacks/ directory at ${STACKS_DIR}."
        return 1
    fi
    STACKS_DIR="$STACKS_DIR" python3 - <<'PY'
import os, yaml

stacks_dir = os.environ["STACKS_DIR"]
rows = []
for name in sorted(os.listdir(stacks_dir)):
    stack = os.path.join(stacks_dir, name)
    if not os.path.isdir(stack) or name.startswith("."):
        continue  # .external holds fetched third-party stack repos
    launch_dir = os.path.join(stack, "launch")
    entries = sorted(
        f[: -len(".launch.xml")]
        for f in (os.listdir(launch_dir) if os.path.isdir(launch_dir) else [])
        if f.endswith(".launch.xml")
    )
    compat = "?"
    repos = os.path.join(stack, "modules.repos")
    if os.path.isfile(repos):
        try:
            with open(repos, encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            compat = data.get("airstack_compat") or "?"
        except yaml.YAMLError:
            compat = "(invalid yaml)"
    wiring = "yes" if os.path.isfile(os.path.join(stack, "wiring.md")) else "-"
    bridge = os.path.isfile(os.path.join(stack, "bridge.yaml"))
    entry_text = ",".join(entries) or "(none)"
    if bridge:
        entry_text += " +bridge"
    rows.append((name, entry_text, wiring, compat))

if not rows:
    print("No stacks. Copy a reference: airstack stack new full_default <name>")
    raise SystemExit(0)

headers = ("NAME", "ENTRY POINTS", "WIRING.MD", "AIRSTACK_COMPAT")
widths = [max(len(str(r[i])) for r in rows + [headers]) for i in range(4)]
fmt = "  ".join("{:<%d}" % w for w in widths)
print(fmt.format(*headers))
for row in rows:
    print(fmt.format(*row))
PY
}

function cmd_stack_new {
    local src="${1:-}" dest="${2:-}"
    if [ -z "$src" ] || [ -z "$dest" ] || [ $# -gt 2 ]; then
        log_error "Usage: airstack stack new <source-stack> <new-stack>"
        log_error "  e.g. airstack stack new full_default my_experiment"
        return 1
    fi
    local src_dir="$STACKS_DIR/$src"
    local dest_dir="$STACKS_DIR/$dest"
    if [ ! -d "$src_dir" ]; then
        log_error "Source stack not found: ${src_dir}"
        log_error "Available stacks:"
        cmd_stack_list
        return 1
    fi
    if [ -e "$dest_dir" ]; then
        log_error "Refusing to overwrite: ${dest_dir} already exists."
        return 1
    fi
    if ! [[ "$dest" =~ ^[a-z][a-z0-9_]*$ ]]; then
        log_error "Stack name must be lowercase snake_case starting with a letter: '$dest'"
        return 1
    fi

    cp -r "$src_dir" "$dest_dir"
    # wiring.md is the SOURCE stack's observed graph — it would lie about the
    # copy the moment an include changes. New stacks bootstrap their own.
    rm -f "$dest_dir/wiring.md"

    log_info "Created ${dest_dir} from ${src}."
    log_info "Next steps:"
    log_info "  1. Edit ${dest_dir}/launch/*.launch.xml — swap/add module <include>s"
    log_info "  2. Update ${dest_dir}/README.md (purpose, how to run, known limits)"
    log_info "  3. Run it:   airstack up --stack ${dest} --sim isaac"
    log_info "  4. Regenerate the wiring baseline (wiring.md was NOT copied):"
    log_info "       airstack test -m wiring --stack ${dest}"
    log_info "     then validate the observed snapshot and commit it as wiring.md."
    log_info "  5. Check it: airstack doctor   (and 'airstack test -m unit' for the lint)"
    if [ -f "$dest_dir/bridge.yaml" ]; then
        log_warn "Copied a SPLIT stack: review ${dest_dir}/bridge.yaml and regenerate"
        log_warn "  the router config: python3 tools/gen_dds_router.py ${dest_dir#$PROJECT_ROOT/}/bridge.yaml"
    fi
}

function cmd_stack_diff {
    _stack_check_python || return 1
    if [ $# -lt 2 ]; then
        log_error "Usage: airstack stack diff <stack-a> <stack-b> [--json]"
        return 1
    fi
    python3 "$STACK_DIFF_TOOL" --stacks-dir "$STACKS_DIR" "$@"
}

# Dispatcher for the `stack` command group.
function cmd_stack_dispatch {
    local sub="${1:-help}"
    if [ $# -gt 0 ]; then shift; fi
    case "$sub" in
        list)   cmd_stack_list "$@" ;;
        new)    cmd_stack_new "$@" ;;
        diff)   cmd_stack_diff "$@" ;;
        help|-h|--help) print_command_help stack ;;
        *)
            log_error "Unknown stack subcommand: '$sub'"
            print_command_help stack
            return 1
            ;;
    esac
}

# Register commands from this module.
function register_stack_commands {
    COMMANDS["stack"]="cmd_stack_dispatch"
    COMMAND_HELP["stack"]="Manage stack folders: list|new|diff (RFC #379 §3; see 'airstack help stack')"
}
