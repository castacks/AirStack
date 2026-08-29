#!/usr/bin/env bash

# _lib.sh — shared helpers for airstack.sh and its command modules.
#
# Sourced EXPLICITLY by airstack.sh BEFORE load_command_modules runs (the
# loader skips _*.sh files), so both the built-in commands and every command
# module may rely on these being defined. Only function definitions live here
# — nothing executes at source time.
#
# tools/doctor/checks.py carries the python twin of the python/PyYAML check
# (cross-language dedupe deferred) — keep the two in sync.

# Fail with a named error unless python3 + PyYAML are available.
# Usage: _require_python_yaml "'airstack fleet' commands"
function _require_python_yaml {
    local ctx="${1:-this command}"
    if ! command -v python3 >/dev/null 2>&1; then
        log_error "python3 is required for ${ctx}."
        return 1
    fi
    if ! python3 -c 'import yaml' 2>/dev/null; then
        log_error "PyYAML is required (pip3 install --user pyyaml)."
        return 1
    fi
}

# Value of NAME in an env file, stripped of quotes and trailing comments.
# Usage: _env_value NAME FILE
function _env_value {
    local name="$1" file="$2" line
    [ -f "$file" ] || return 0
    line=$(grep -E "^${name}=" "$file" 2>/dev/null | tail -1)
    [[ -z "$line" ]] && return 0
    line="${line#*=}"
    line="${line%%#*}"
    # trim whitespace, then surrounding quotes
    line="${line#"${line%%[![:space:]]*}"}"
    line="${line%"${line##*[![:space:]]}"}"
    line="${line%\"}"; line="${line#\"}"
    line="${line%\'}"; line="${line#\'}"
    echo "$line"
}

# List running robot containers, one per line: compose replicas
# (airstack-robot-desktop-N) AND fleet-generated per-robot services
# (airstack-robot_N-1). Ground hosts (gcs-robot_N tenants) are NOT robots —
# they never run MAVROS/PX4 — so exclude them.
function _robot_containers {
    docker ps --format '{{.Names}}' | grep -E -- '-robot[-_]' | grep -v 'gcs-' | sort
}

# Resolve a container's ROBOT_NAME + ROS_DOMAIN_ID in ONE docker exec, via the
# same login-shell resolution robot/docker/.bashrc performs at startup. The
# unique prefix lets us grep the values out of .bashrc echo noise.
# Prints "ROBOT_NAME<TAB>ROS_DOMAIN_ID"; returns 1 when unresolvable.
function _container_identity {
    local container="$1" vars
    vars=$(docker exec "$container" bash --login -c \
        'printf "AIRSTACK_VARS:%s:%s\n" "$ROBOT_NAME" "$ROS_DOMAIN_ID"' 2>/dev/null \
        | grep "^AIRSTACK_VARS:" | tail -1)
    [ -z "$vars" ] && return 1
    vars="${vars#AIRSTACK_VARS:}"
    printf '%s\t%s\n' "${vars%%:*}" "${vars##*:}"
}
