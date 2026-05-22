#!/usr/bin/env bash

# osmo.sh — AirStack-on-OSMO convenience commands.
#
# Wraps `osmo workflow submit/port-forward/logs/cancel` for the
# osmo/workflows/airstack-dev.yaml workflow so a Mac/Windows student doesn't
# have to memorize the WebRTC port range or the entry-script path.
#
# This module is pure bash + the cross-platform `osmo` CLI — no Docker
# dependency. Safe to run on a laptop with no AirStack runtime.
#
# Most commands need a workflow id. `osmo:up` saves the id to
# $OSMO_STATE_FILE; the other commands read it from there. You can also
# override it for a single invocation by exporting AIRSTACK_OSMO_WF.

# State directory and file: ~/.airstack/osmo-state stores the most recent
# workflow id submitted with `airstack osmo:up`.
OSMO_STATE_DIR="${HOME}/.airstack"
OSMO_STATE_FILE="${OSMO_STATE_DIR}/osmo-state"

# WebRTC livestream ports — must match the ports published by the
# isaac-sim-livestream service in
# simulation/isaac-sim/docker/docker-compose.yaml AND the
# app.livestream.fixedHostPort setting pinned in the Pegasus launch script
# (simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py).
#
# Two ports total:
#   TCP 49100 — omni.kit.livestream.webrtc WebSocket signaling
#   UDP 49099 — SRTP media (pinned; Kit 107 otherwise picks dynamically and
#               escapes both the compose-published and CLI-forwarded ranges)
OSMO_WEBRTC_TCP="49100"
OSMO_WEBRTC_UDP="49099"

# GCS Foxglove websocket: container 8765 → host 8766 (per
# gcs/docker/docker-compose.yaml).
OSMO_FOXGLOVE_PORT="8766:8766"

# SSH port-forward: local 2200 → pod 22.
OSMO_SSH_PORT="2200:22"

# Default `osmo workflow port-forward` connect-timeout (24h).
OSMO_PF_TIMEOUT="${OSMO_PF_TIMEOUT:-86400}"

# Helper: ensure the osmo CLI is on PATH.
function _osmo_check_cli {
    if ! command -v osmo >/dev/null 2>&1; then
        log_error "osmo CLI not found on PATH. Install from https://github.com/NVIDIA/OSMO and run 'osmo login'."
        return 1
    fi
}

# Helper: strip leading/trailing whitespace + CR/NUL bytes from the
# variable named in $1.
#
# Why this exists: bracket-paste mode and cross-OS clipboards (RDP, VNC,
# Windows-side note apps) routinely smuggle invisible bytes around long
# pastes — Nucleus API tokens (JWT, ~1 KB) and SSH keys are the usual
# victims. Nucleus's auth endpoint silently `DENIES` a token that has
# one extra trailing byte, with no actionable error from the client side.
# Stripping defensively at prompt time saves an entire round-trip of
# "regenerate token → still denied → check auth-service log" debugging.
function _osmo_trim {
    local var_name="$1"
    local val="${!var_name}"
    local original_len="${#val}"
    val="${val//$'\r'/}"
    val="${val//$'\0'/}"
    val="${val#"${val%%[![:space:]]*}"}"
    val="${val%"${val##*[![:space:]]}"}"
    if [ "${#val}" -ne "$original_len" ]; then
        log_warn "Stripped $((original_len - ${#val})) whitespace/control byte(s) from ${var_name}."
    fi
    printf -v "$var_name" '%s' "$val"
}

# Helper: read a value with prompt; supports -s for silent (passwords).
#
# Visible prompts switch the TTY out of canonical mode for the duration of
# the read. Without this, macOS caps each input line at MAX_CANON = 1024
# bytes (per <sys/syslimits.h>) and rings the terminal bell on Enter when
# the buffer overflows. Nucleus API tokens are JWTs ~950 bytes long, so
# `Nucleus API token: <jwt>` lands right at the cap. `stty -icanon` makes
# the kernel deliver bytes to bash as they're typed, with no line-buffer
# limit; bash's `read` still terminates on newline normally.
#
# We use a trap to guarantee the saved stty is restored if the user Ctrl-Cs
# mid-paste — otherwise the shell would be left in raw mode.
#
# After reading we always run _osmo_trim — see comment there.
function _osmo_prompt {
    local var_name="$1"
    local prompt_text="$2"
    local silent="${3:-false}"
    local saved_stty=""

    if [ "$silent" = "true" ]; then
        # Passwords are short — canonical-mode cap is fine here.
        read -r -s -p "${prompt_text}: " "$var_name"
        printf "\n" >&2
    else
        if [ -t 0 ]; then
            saved_stty="$(stty -g 2>/dev/null || true)"
            if [ -n "$saved_stty" ]; then
                trap 'stty "$saved_stty" 2>/dev/null; trap - INT' INT
                stty -icanon 2>/dev/null
            fi
        fi
        read -r -p "${prompt_text}: " "$var_name"
        if [ -n "$saved_stty" ]; then
            stty "$saved_stty" 2>/dev/null
            trap - INT
        fi
    fi

    _osmo_trim "$var_name"

    if [ -z "${!var_name}" ]; then
        log_error "Empty input for ${var_name}; aborting."
        return 1
    fi
}

# osmo:setup — interactively register the three OSMO credentials AirStack
# needs (airlab-docker-registry, airlab-docker-login, airlab-nucleus).
# Idempotent — re-running rotates the credentials.
function cmd_osmo_setup {
    _osmo_check_cli || return 1

    cat >&2 <<'EOF'

This sets up the three per-user OSMO credentials AirStack-on-OSMO needs:

  1. airlab-docker-registry  (REGISTRY) — for OSMO to pull the workspace image
  2. airlab-docker-login     (GENERIC)  — for the inner dockerd to pull AirStack images
  3. airlab-nucleus          (GENERIC)  — for Isaac Sim Nucleus access

You'll be asked for:

  - your Andrew ID (no @andrew.cmu.edu suffix)
  - your AirLab Docker password (same as your Andrew password)
  - your Nucleus API token (https://airlab-nucleus.andrew.cmu.edu/omni/web3/
    → right-click cloud → API Tokens). NOT your Andrew password.

Values go directly to OSMO; nothing is written to disk locally.

EOF

    local andrew_id andrew_password nucleus_token
    _osmo_prompt andrew_id        "Andrew ID"                       false || return 1
    _osmo_prompt andrew_password  "AirLab Docker password (hidden)" true  || return 1
    _osmo_prompt nucleus_token    "Nucleus API token"               false || return 1

    # Sanity-check the Nucleus token shape. Nucleus issues RS256 JWTs:
    # base64url(header).base64url(payload).base64url(signature), with the
    # header always starting `eyJ` (base64url of `{"`). Catching a wrong
    # paste here (e.g. Andrew password, or token without the trailing
    # signature segment) saves the user from a silent `InternalCredentials
    # .auth: DENIED` round-trip later on. We do not validate the signature.
    case "$nucleus_token" in
        eyJ*.*.*) ;;  # looks like a 3-segment JWT
        *)
            log_error "That doesn't look like a Nucleus API token."
            log_error "  - Expected: a JWT of the form eyJ…<dot>…<dot>… (~1 KB long)"
            log_error "  - Got:      ${#nucleus_token} chars, prefix '$(printf '%s' "$nucleus_token" | head -c 8)…'"
            log_error "  Generate one at https://airlab-nucleus.andrew.cmu.edu/omni/web3/"
            log_error "  → right-click cloud icon → API Tokens → Create."
            return 1
            ;;
    esac

    local omni_server="${OMNI_SERVER:-omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1}"
    local airlab_registry="${AIRLAB_REGISTRY:-airlab-docker.andrew.cmu.edu}"

    # `osmo credential set` is NOT an upsert for GENERIC credentials — re-setting
    # one that already exists fails with `400 duplicate key value violates unique
    # constraint "credential_pkey"`. Delete first so re-running osmo:setup
    # (e.g. to rotate a Nucleus token) is idempotent. The `|| true` swallows the
    # "credential not found" case on a first-time run.
    log_info "Refreshing airlab-docker-registry (REGISTRY)..."
    osmo credential delete airlab-docker-registry >/dev/null 2>&1 || true
    osmo credential set airlab-docker-registry \
        --type REGISTRY \
        --payload "registry=${airlab_registry}" \
                  "username=${andrew_id}" \
                  "auth=${andrew_password}" \
        || { log_error "osmo credential set airlab-docker-registry failed"; return 1; }

    log_info "Refreshing airlab-docker-login (GENERIC)..."
    osmo credential delete airlab-docker-login >/dev/null 2>&1 || true
    osmo credential set airlab-docker-login \
        --type GENERIC \
        --payload "username=${andrew_id}" \
                  "password=${andrew_password}" \
        || { log_error "osmo credential set airlab-docker-login failed"; return 1; }

    log_info "Refreshing airlab-nucleus (GENERIC)..."
    osmo credential delete airlab-nucleus >/dev/null 2>&1 || true
    osmo credential set airlab-nucleus \
        --type GENERIC \
        --payload "omni_user=${andrew_id}" \
                  "omni_pass=${nucleus_token}" \
                  "omni_server=${omni_server}" \
        || { log_error "osmo credential set airlab-nucleus failed"; return 1; }

    log_info "All three credentials registered. List them with: osmo credential list"
    log_info "Next: airstack osmo:up [--pool POOL]"
}

# Helper: pick the first existing SSH public key on the host.
function _osmo_pick_pubkey {
    local candidates=(
        "${HOME}/.ssh/id_ed25519.pub"
        "${HOME}/.ssh/id_ecdsa.pub"
        "${HOME}/.ssh/id_rsa.pub"
    )
    for k in "${candidates[@]}"; do
        if [ -f "$k" ]; then
            echo "$k"
            return 0
        fi
    done
    return 1
}

# Helper: get the active workflow id (env override first, then state file).
#
# The state file persists across shell sessions, so it can easily go stale
# (e.g. a previous airstack-dev-N is now FAILED/CANCELED). To avoid the
# confusing "Workflow airstack-dev-10 is not running!" 410 error from the
# downstream osmo command, this helper verifies the saved id is still in a
# live state (PENDING / RUNNING) before returning it.
function _osmo_wf_id {
    local wf
    if [ -n "${AIRSTACK_OSMO_WF:-}" ]; then
        wf="${AIRSTACK_OSMO_WF}"
    elif [ -f "${OSMO_STATE_FILE}" ]; then
        wf="$(cat "${OSMO_STATE_FILE}")"
    else
        log_error "No workflow id found. Run 'airstack osmo:up' first, or export AIRSTACK_OSMO_WF=<id>."
        return 1
    fi

    # Validate the workflow is still alive (only when osmo CLI is available).
    if command -v osmo >/dev/null 2>&1; then
        local status
        status="$(osmo workflow query "${wf}" 2>/dev/null | awk -F': +' '/^Status/ {print $2; exit}' | tr -d ' \r\n')"
        case "${status}" in
            PENDING|RUNNING|"")
                # "" means we couldn't reach osmo; let the downstream
                # command surface the real error rather than failing here.
                ;;
            *)
                log_error "Saved workflow '${wf}' is ${status}, not running."
                log_warn  "Run 'airstack osmo:up' to launch a fresh one, or:"
                log_warn  "  rm ${OSMO_STATE_FILE}"
                log_warn  "  export AIRSTACK_OSMO_WF=<id-of-a-running-workflow>"
                return 1
                ;;
        esac
    fi

    echo "${wf}"
    return 0
}

# Helper: persist the workflow id.
function _osmo_save_wf_id {
    mkdir -p "${OSMO_STATE_DIR}"
    echo "$1" > "${OSMO_STATE_FILE}"
    log_info "Saved workflow id '$1' to ${OSMO_STATE_FILE}"
}

# Helper: best-effort detection of the user's current AirStack branch so
# `airstack osmo:up` can default --branch to whatever the user is editing
# locally. Returns the branch name on stdout, or empty if we shouldn't
# auto-pin (detached HEAD, not a git repo, etc.).
#
# Why default to the local branch: the pod's entrypoint clones AirStack
# fresh from GitHub on every workflow start (the pod fs is ephemeral, so
# nothing else makes sense). If we don't tell it which branch, it
# defaults to `main` — and any developer testing branch-only OSMO
# changes (compose services, entrypoint tweaks, workflow yaml edits)
# silently runs against stale `main` code instead of their work.
# Defaulting to the local branch makes "edit on laptop, push, osmo:up"
# the natural workflow.
function _osmo_local_branch {
    if ! command -v git >/dev/null 2>&1; then
        return 0
    fi
    local b
    b="$(git -C "${PROJECT_ROOT}" rev-parse --abbrev-ref HEAD 2>/dev/null)" || return 0
    case "$b" in
        ""|HEAD) return 0 ;;  # detached HEAD or empty
    esac
    echo "$b"
}

# Helper: warn if the about-to-submit branch isn't safely pushed. The
# pod clones from GitHub, so unpushed commits / dirty working tree don't
# make it into the pod even if the user thinks they did. Catching this
# before submit avoids a 60-90s "wait for pod, then realize" round trip.
function _osmo_check_branch_pushed {
    local branch="$1"
    command -v git >/dev/null 2>&1 || return 0
    local repo="${PROJECT_ROOT}"
    [ -d "${repo}/.git" ] || return 0

    local local_sha upstream_sha
    local_sha="$(git -C "$repo" rev-parse "${branch}" 2>/dev/null)" || return 0

    # Look for a remote-tracking branch first (the explicit upstream
    # set by `git push -u`); fall back to origin/<branch>.
    upstream_sha="$(git -C "$repo" rev-parse "${branch}@{upstream}" 2>/dev/null)"
    if [ -z "$upstream_sha" ]; then
        upstream_sha="$(git -C "$repo" rev-parse "origin/${branch}" 2>/dev/null)"
    fi

    if [ -z "$upstream_sha" ]; then
        log_warn "Branch '${branch}' has no upstream on origin — the pod's clone will fail. Run: git push -u origin ${branch}"
        return 0
    fi

    if [ "$local_sha" != "$upstream_sha" ]; then
        local ahead behind
        ahead="$(git -C "$repo" rev-list --count "${upstream_sha}..${local_sha}" 2>/dev/null)"
        behind="$(git -C "$repo" rev-list --count "${local_sha}..${upstream_sha}" 2>/dev/null)"
        if [ "${ahead:-0}" -gt 0 ]; then
            log_warn "Local '${branch}' is ${ahead} commit(s) ahead of origin/${branch} — the pod will clone the older origin tip. Run: git push"
        fi
        if [ "${behind:-0}" -gt 0 ]; then
            log_info "Local '${branch}' is ${behind} commit(s) behind origin/${branch} (pod will clone the newer origin tip)."
        fi
    fi

    if [ -n "$(git -C "$repo" status --porcelain 2>/dev/null)" ]; then
        log_warn "Working tree has uncommitted changes — the pod will not see them. Commit + push first if you want the pod to pick them up."
    fi
}

# osmo:up — submit airstack-dev.yaml with the local pubkey injected.
#
# Usage: airstack osmo:up [--pool POOL] [--key PATH] [--branch BRANCH]
#
# --branch defaults to the local repo's current branch (or `main` if we
# can't detect one), and is passed through as AIRSTACK_BRANCH so the
# pod's entrypoint clones the matching code. Pass `--branch main`
# explicitly to override.
function cmd_osmo_up {
    _osmo_check_cli || return 1

    local pool="${OSMO_POOL:-}"
    local pubkey_file=""
    local branch=""
    local branch_explicit=false
    local extra_args=()

    while [ $# -gt 0 ]; do
        case "$1" in
            --pool)   pool="$2"; shift 2 ;;
            --key)    pubkey_file="$2"; shift 2 ;;
            --branch) branch="$2"; branch_explicit=true; shift 2 ;;
            *)        extra_args+=("$1"); shift ;;
        esac
    done

    if [ -z "$pubkey_file" ]; then
        if ! pubkey_file="$(_osmo_pick_pubkey)"; then
            log_error "No SSH public key found in ~/.ssh. Generate one with: ssh-keygen -t ed25519"
            return 1
        fi
    fi
    log_info "Using SSH public key: ${pubkey_file}"

    local workflow_yaml="${PROJECT_ROOT}/osmo/workflows/airstack-dev.yaml"
    if [ ! -f "$workflow_yaml" ]; then
        log_error "Workflow file not found: ${workflow_yaml}"
        return 1
    fi

    # Auto-pin --branch to the local checkout if the user didn't pass one.
    if [ "$branch_explicit" = false ] && [ -z "$branch" ]; then
        branch="$(_osmo_local_branch)"
        if [ -n "$branch" ]; then
            log_info "Auto-detected local branch '${branch}'; pod will clone from origin/${branch} (override with --branch main)."
        else
            log_info "Could not detect local branch (detached HEAD?); pod will clone from origin/main."
        fi
    fi
    if [ -n "$branch" ]; then
        _osmo_check_branch_pushed "$branch"
    fi

    local cmd=(osmo workflow submit "$workflow_yaml")
    if [ -n "$pool" ]; then
        cmd+=(--pool "$pool")
    else
        log_warn "No --pool provided and OSMO_POOL is unset; using your osmo profile's default pool."
    fi
    # IMPORTANT: `osmo workflow submit --set-env` is variadic. Passing two
    # separate `--set-env A=1 --set-env B=2` silently drops the first one
    # (only the last `--set-env` flag's values are kept). We collect all
    # K=V pairs and pass them under a single `--set-env`.
    local env_kvs=("SSH_PUB_KEY=$(cat "$pubkey_file")")
    if [ -n "$branch" ]; then
        env_kvs+=("AIRSTACK_BRANCH=${branch}")
    fi
    cmd+=(--set-env "${env_kvs[@]}")
    if [ ${#extra_args[@]} -gt 0 ]; then
        cmd+=("${extra_args[@]}")
    fi

    log_info "Submitting: ${cmd[*]}"
    local output
    if ! output="$("${cmd[@]}" 2>&1)"; then
        echo "$output" >&2
        log_error "osmo workflow submit failed."
        return 1
    fi
    echo "$output"

    # Parse the workflow id out of the submit output. The cookbook examples
    # show "Workflow ID        - <id>" formatted output (see OSMO
    # submission.rst). Match that line.
    local wf_id
    wf_id="$(echo "$output" | awk -F'- ' '/^Workflow ID/ {print $2; exit}' | tr -d ' \r\n')"
    if [ -z "$wf_id" ]; then
        log_warn "Could not parse workflow id from submit output. Set it manually:"
        log_warn "  echo <wf-id> > ${OSMO_STATE_FILE}"
        return 0
    fi
    _osmo_save_wf_id "$wf_id"

    log_info "Next steps:"
    log_info "  airstack osmo:logs       # follow startup until 'sshd listening'"
    log_info "  airstack osmo:ide        # port-forward sshd + open VS Code"
    log_info "  airstack osmo:webrtc     # forward Isaac Sim WebRTC ports"
    log_info "  airstack osmo:foxglove   # forward GCS Foxglove websocket"
    log_info "  airstack osmo:down       # cancel the workflow"
}

# osmo:logs — follow the workspace task logs.
#
# Despite the `osmo workflow logs --help` output advertising only `-n
# LAST_N_LINES` (no `--follow`), the CLI in fact streams the tail and keeps
# the connection open as new lines arrive — i.e. it already behaves like
# `tail -f`. We just exec it in the foreground so the user sees output
# immediately and can Ctrl+C to stop. (An earlier implementation wrapped
# this in `out=$(osmo workflow logs ...)`; command substitution waits for
# the process to exit, which never happened, so nothing was ever printed.)
function cmd_osmo_logs {
    _osmo_check_cli || return 1
    local wf; wf="$(_osmo_wf_id)" || return 1

    local task="${OSMO_LOGS_TASK:-workspace}"
    local lines="${OSMO_LOGS_TAIL:-500}"

    log_info "Following ${task} logs for ${wf} (last ${lines} lines, then live; Ctrl+C to stop)"

    # Filter stderr for the same OSMOUserError-when-workflow-dies case
    # the port-forward path hits — same noisy asyncio Traceback +
    # "Task exception was never retrieved" header. _osmo_pf_filter
    # collapses it into one clean log line.
    osmo workflow logs "${wf}" -t "${task}" -n "${lines}" \
        2> >(_osmo_pf_filter "${wf}")
}

# osmo:ide — port-forward sshd + (optionally) launch VS Code/Cursor on the
# `airstack-osmo` host. Runs the port-forward in the foreground so closing
# the terminal closes the tunnel.
#
# Usage: airstack osmo:ide [--no-open] [code|cursor]
function cmd_osmo_ide {
    _osmo_check_cli || return 1
    local wf; wf="$(_osmo_wf_id)" || return 1

    local open_ide=true
    local ide_cmd=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --no-open) open_ide=false; shift ;;
            code|cursor) ide_cmd="$1"; shift ;;
            *) log_warn "Ignoring unknown osmo:ide arg: $1"; shift ;;
        esac
    done

    if [ -z "$ide_cmd" ]; then
        if command -v cursor >/dev/null 2>&1; then
            ide_cmd="cursor"
        elif command -v code >/dev/null 2>&1; then
            ide_cmd="code"
        else
            log_warn "Neither 'cursor' nor 'code' found on PATH; will only port-forward (open the IDE manually and Connect to Host airstack-osmo)."
            open_ide=false
        fi
    fi

    log_info "Make sure ~/.ssh/config has a 'Host airstack-osmo' entry pointing at localhost:2200, User root."

    # Local TCP port the user's IDE will connect to (the local side of the
    # `--port LOCAL:REMOTE` mapping).
    local local_port="${OSMO_SSH_PORT%%:*}"

    # Every fresh OSMO pod ships a new sshd host key. If the user's
    # ~/.ssh/known_hosts still has an entry for [localhost]:${local_port}
    # from a previous workflow, ssh aborts with "Host key for [localhost]
    # :${local_port} has changed and you have requested strict checking",
    # which the IDE surfaces as a generic "could not connect" error.
    #
    # The recommended ~/.ssh/config block for `airstack-osmo` uses
    # `UserKnownHostsFile /dev/null`, which sidesteps this entirely — but
    # users who set up before that change still have a stale entry on
    # disk. Scrub it defensively on every osmo:ide invocation. ssh-keygen
    # -R is idempotent: a no-op if the entry doesn't exist.
    if command -v ssh-keygen >/dev/null 2>&1; then
        ssh-keygen -R "[localhost]:${local_port}" >/dev/null 2>&1 || true
    fi

    # Reuse an existing forward if one is already listening (the user might
    # have run this from a second terminal, or osmo:foxglove already opened
    # a multi-port forward). Otherwise spawn one in the background and wait
    # for it to bind before launching the IDE — this avoids the race where
    # Cursor/VS Code tries to SSH before the tunnel exists and dies with
    # "connect to host localhost port 2200: Connection refused".
    local pf_pid=""
    if nc -z localhost "$local_port" 2>/dev/null; then
        log_info "Port ${local_port} is already listening; reusing existing port-forward."
    else
        log_info "osmo workflow port-forward ${wf} workspace --port ${OSMO_SSH_PORT} --connect-timeout ${OSMO_PF_TIMEOUT}"
        osmo workflow port-forward "$wf" workspace --port "$OSMO_SSH_PORT" --connect-timeout "$OSMO_PF_TIMEOUT" \
            > "${OSMO_STATE_DIR}/ssh-pf.log" 2>&1 &
        pf_pid=$!
        # Wait up to 30s for the tunnel to start accepting connections.
        local waited=0
        until nc -z localhost "$local_port" 2>/dev/null; do
            sleep 1; waited=$((waited+1))
            if [ "$waited" -ge 30 ]; then
                log_error "Timed out waiting for port-forward on :${local_port} after ${waited}s."
                log_error "  port-forward log: ${OSMO_STATE_DIR}/ssh-pf.log"
                kill "$pf_pid" 2>/dev/null
                return 1
            fi
            if ! kill -0 "$pf_pid" 2>/dev/null; then
                log_error "port-forward exited early. Tail:"
                tail -10 "${OSMO_STATE_DIR}/ssh-pf.log" >&2
                return 1
            fi
        done
        log_info "Port-forward established on localhost:${local_port} (pid ${pf_pid})."
    fi

    if [ "$open_ide" = true ]; then
        # vscode-remote URI launches the IDE pre-attached to the remote host.
        local uri="vscode-remote://ssh-remote+airstack-osmo/root/AirStack"
        log_info "Launching ${ide_cmd} → ${uri}"
        ( "$ide_cmd" --folder-uri "$uri" >/dev/null 2>&1 || \
          "$ide_cmd" "$uri" >/dev/null 2>&1 || \
          log_warn "Could not launch ${ide_cmd} automatically; open it and pick airstack-osmo from Remote-SSH manually." ) &
    fi

    if [ -n "$pf_pid" ]; then
        log_info "Leave this terminal running for the length of your session (Ctrl+C to disconnect)."
        # Forward Ctrl+C to the port-forward and clean up.
        trap 'kill "$pf_pid" 2>/dev/null; exit 0' INT TERM
        wait "$pf_pid"
    else
        log_info "Existing port-forward owns the tunnel; this command will exit immediately."
        log_info "Stop the tunnel with: pkill -f 'osmo workflow port-forward'  or  airstack osmo:down"
    fi
}

# Helper: filter `osmo workflow port-forward` stderr through awk to
# suppress the asyncio traceback that erupts whenever the workflow gets
# canceled mid-flight (e.g. via osmo:down in another shell, or because
# OSMO timed it out). The CLI raises OSMOUserError("Workflow X is not
# running!") from inside an asyncio Task, which then prints "Task
# exception was never retrieved" + a multi-line Traceback that obscures
# the actual one-line cause. We translate that into a single clean log
# line and drop everything else.
function _osmo_pf_filter {
    local wf="$1"
    awk -v WF="$wf" '
        /^Task exception was never retrieved/        { skipping=1; next }
        /^future:/                                    { skipping=1; next }
        /^Traceback \(most recent call last\):/      { skipping=1; next }
        /^  File "/                                   { next }
        /^src\.lib\.utils\.osmo_errors\.OSMOUserError/ {
            sub(/^src\.lib\.utils\.osmo_errors\.OSMOUserError: */, "")
            printf "\033[0;31m[ERROR]\033[0m %s (run `airstack osmo:up` to start a new workflow)\n", $0
            next
        }
        /OSMOUserError: Workflow .* is not running!/ {
            printf "\033[0;31m[ERROR]\033[0m Workflow %s is no longer running (run `airstack osmo:up` to start a new one).\n", WF
            next
        }
        skipping && /^$/                              { skipping=0; next }
        skipping                                      { next }
        { print }
    ' >&2
}

# Helper: run `osmo workflow port-forward` with the noise filter
# attached. Returns the underlying exit code so callers can decide
# whether to retry / fail. Args after the helper name are passed to
# `osmo workflow port-forward` verbatim.
function _osmo_run_port_forward {
    osmo workflow port-forward "$@" 2> >(_osmo_pf_filter "$1")
}

# osmo:webrtc — forward both Isaac Sim WebRTC port ranges (TCP in this
# terminal, spawn UDP in the background). Cleans up the UDP child on
# exit (Ctrl+C, foreground TCP failure, or the workflow disappearing
# mid-stream) so we don't leak a port-forward into the user's process
# table.
function cmd_osmo_webrtc {
    _osmo_check_cli || return 1
    local wf; wf="$(_osmo_wf_id)" || return 1

    log_info "Spawning UDP port-forward in background: ${OSMO_WEBRTC_UDP}"
    nohup osmo workflow port-forward "$wf" workspace \
        --port "$OSMO_WEBRTC_UDP" --udp \
        --connect-timeout "$OSMO_PF_TIMEOUT" \
        > "${OSMO_STATE_DIR}/webrtc-udp.log" 2>&1 &
    local udp_pid=$!
    log_info "  UDP log: ${OSMO_STATE_DIR}/webrtc-udp.log (pid ${udp_pid})"

    # Tear the UDP fork down when this function exits, by any path.
    # Without this, hitting Ctrl+C on the TCP foreground (or the
    # workflow being canceled, which surfaces as the foreground exiting
    # non-zero) leaves the UDP `osmo workflow port-forward` running
    # against a dead workflow until the user notices and pkill's it.
    trap '
        if kill -0 "'"${udp_pid}"'" 2>/dev/null; then
            kill "'"${udp_pid}"'" 2>/dev/null
            wait "'"${udp_pid}"'" 2>/dev/null
        fi
        trap - EXIT INT TERM
    ' EXIT INT TERM

    log_info "Foreground TCP port-forward: ${OSMO_WEBRTC_TCP}"
    log_info "Open the Omniverse Streaming Client / WebRTC client at http://localhost"
    _osmo_run_port_forward "$wf" workspace \
        --port "$OSMO_WEBRTC_TCP" \
        --connect-timeout "$OSMO_PF_TIMEOUT"
}

# osmo:foxglove — install the AirStack Foxglove extensions into the local
# Foxglove Desktop user-extensions dir, then forward the GCS Foxglove
# websocket.
#
# The extension install is the same script the GCS container runs on
# startup — gcs/foxglove_extensions/install.py — invoked with env-var
# overrides that point at the local laptop dirs. Default destination on
# Linux/macOS is ~/.foxglove-studio/extensions (Foxglove's canonical user
# extensions path; the macOS rebrand still reads from here). Override
# with OSMO_FOXGLOVE_EXT_DIR, or skip the install entirely with
# OSMO_FOXGLOVE_SKIP_EXTENSIONS=1 (e.g. when using app.foxglove.dev
# which doesn't load local extensions anyway).
function cmd_osmo_foxglove {
    _osmo_check_cli || return 1
    local wf; wf="$(_osmo_wf_id)" || return 1

    local ext_src="${PROJECT_ROOT}/gcs/foxglove_extensions"
    local ext_dst="${OSMO_FOXGLOVE_EXT_DIR:-${HOME}/.foxglove-studio/extensions}"

    if [ "${OSMO_FOXGLOVE_SKIP_EXTENSIONS:-0}" != "1" ] && [ -d "${ext_src}" ]; then
        if command -v python3 >/dev/null 2>&1; then
            log_info "Installing Foxglove extensions to ${ext_dst}"
            FOXGLOVE_EXT_SRC="${ext_src}" FOXGLOVE_EXT_DST="${ext_dst}" \
                python3 "${ext_src}/install.py" \
                || log_warn "Foxglove extension install failed; panels like 'Robot Tasks' may show as 'Unknown panel type' in Foxglove"
        else
            log_warn "python3 not found on PATH — skipping Foxglove extension install."
            log_warn "  Custom panels (Robot Tasks, Waypoint Editor, Polygon Editor) will show as 'Unknown panel type'."
            log_warn "  Install python3 (e.g. 'brew install python') or copy ${ext_src}/* manually to ${ext_dst}."
        fi
    elif [ "${OSMO_FOXGLOVE_SKIP_EXTENSIONS:-0}" = "1" ]; then
        log_info "Skipping Foxglove extension install (OSMO_FOXGLOVE_SKIP_EXTENSIONS=1)."
    fi

    log_info "osmo workflow port-forward ${wf} workspace --port ${OSMO_FOXGLOVE_PORT} --connect-timeout ${OSMO_PF_TIMEOUT}"
    log_info "Then in Foxglove Desktop: Open connection → ws://localhost:8766"
    log_info "  Layouts → Import from file → ${ext_src}/airstack_default.json"
    log_info "  (Restart Foxglove Desktop once if newly-installed panels still show as 'Unknown panel type'.)"
    _osmo_run_port_forward "$wf" workspace \
        --port "$OSMO_FOXGLOVE_PORT" \
        --connect-timeout "$OSMO_PF_TIMEOUT"
}

# osmo:down — cancel the active workflow. Reminds you to push first.
function cmd_osmo_down {
    _osmo_check_cli || return 1
    local wf; wf="$(_osmo_wf_id)" || return 1

    log_warn "About to cancel workflow '${wf}'."
    log_warn "Anything not pushed to git in /root/AirStack inside the pod will be LOST."
    log_warn "Hit Ctrl-C in the next 5 seconds to abort."
    sleep 5
    osmo workflow cancel "$wf"
    rm -f "${OSMO_STATE_FILE}"
}

# Register commands from this module.
function register_osmo_commands {
    COMMANDS["osmo:setup"]="cmd_osmo_setup"
    COMMANDS["osmo:up"]="cmd_osmo_up"
    COMMANDS["osmo:logs"]="cmd_osmo_logs"
    COMMANDS["osmo:ide"]="cmd_osmo_ide"
    COMMANDS["osmo:webrtc"]="cmd_osmo_webrtc"
    COMMANDS["osmo:foxglove"]="cmd_osmo_foxglove"
    COMMANDS["osmo:down"]="cmd_osmo_down"

    COMMAND_HELP["osmo:setup"]="One-time per-user OSMO credential setup (airlab-docker-registry, airlab-docker-login, airlab-nucleus)"
    COMMAND_HELP["osmo:up"]="Submit osmo/workflows/airstack-dev.yaml with your SSH pubkey injected (--pool POOL, --key PATH, --branch BRANCH)"
    COMMAND_HELP["osmo:logs"]="Follow the workspace task logs (osmo workflow logs <id> -t workspace -n 500; OSMO_LOGS_TASK / OSMO_LOGS_TAIL override)"
    COMMAND_HELP["osmo:ide"]="Port-forward sshd (2200:22) and open VS Code/Cursor on Host airstack-osmo"
    COMMAND_HELP["osmo:webrtc"]="Port-forward Isaac Sim WebRTC ranges (TCP foreground + UDP background)"
    COMMAND_HELP["osmo:foxglove"]="Install AirStack Foxglove extensions locally, then port-forward GCS Foxglove websocket (8766:8766). Override target dir with OSMO_FOXGLOVE_EXT_DIR; skip install with OSMO_FOXGLOVE_SKIP_EXTENSIONS=1."
    COMMAND_HELP["osmo:down"]="Cancel the active workflow (push to git before running this)"
}
