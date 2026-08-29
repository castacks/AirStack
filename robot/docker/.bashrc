# ========== BASHRC FOR ROBOT DOCKER CONTAINER ==========
# /.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# --- ROS2 workspace setup ---

# Define the ROS2 workspace directory
ROS2_WS_DIR="$HOME/AirStack/robot/ros_ws"
# needed for communication with Isaac Sim ROS2  # https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#enabling-the-ros-bridge-extension
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROS2_WS_DIR/src/fastdds.xml"

# fix ROS2 jazzy setuptools deprecation warning https://robotics.stackexchange.com/questions/24230/setuptoolsdeprecationwarning-in-ros2-humble/24349#24349
PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated"
export PYTHONWARNINGS

# Convenience functions for ROS2 workspace

function bws(){
    # Since multiple robot containers may mount a shared volume, we need to use flock to prevent race conditions.
    echo "Running \`colcon build $@\` in $ROS2_WS_DIR"
    COLCON_LOG_PATH="$ROS2_WS_DIR"/log flock "$ROS2_WS_DIR/.build.lock" \
        colcon build --symlink-install --base-paths "$ROS2_WS_DIR"/ --build-base "$ROS2_WS_DIR"/build/ --install-base "$ROS2_WS_DIR"/install/ "$@"
}
function sws(){
    if [ -f "$ROS2_WS_DIR/install/local_setup.bash" ]; then
        echo "Sourcing $ROS2_WS_DIR/install/local_setup.bash"
        source "$ROS2_WS_DIR/install/local_setup.bash"
    else
        echo "Workspace not built yet. Please make sure to build first with 'bws'"
    fi
}

# Function to prompt user for confirmation
confirm_cws() {
    while true; do
        read -p "Are you sure you want to clean the ROS2 workspace under $ROS2_WS_DIR? (y/N): " yn
        yn=${yn:-no} # Default to 'no' if no answer is given
        case $yn in
            [Yy] | [Yy][Ee][Ss] ) return 0;;
            [Nn] | [Nn][Oo] ) return 1;;
            * ) echo "Please answer yes or no.";;
        esac
    done
}
function cws(){
    # Call the confirmation function
    if confirm_cws; then
        echo "Cleaning ROS2 workspace..."
        set -x
        # Remove build, install, and log directories
        if ! rm -rf "$ROS2_WS_DIR"/build/ "$ROS2_WS_DIR"/install/ "$ROS2_WS_DIR"/log/; then
            { set +x; } 2>/dev/null
            echo "Error: Failed to remove ROS2 workspace directories."
            exit 1
        fi

        # Set environment variables
        export AMENT_PREFIX_PATH="/opt/ros/jazzy"
        export CMAKE_PREFIX_PATH=""

        { set +x; } 2>/dev/null  # set +x w/out it being printed
        echo "ROS2 workspace has been cleaned successfully."
    else
        echo "Operation cancelled."
    fi
}

# Build → source → launch, with an unmissable banner when a stage fails.
# Used by the docker-compose AUTOLAUNCH tmux commands: a bare
# `bws && sws && ros2 launch ...` dies silently on a build failure — the tmux
# pane just returns to a prompt and `docker logs` shows nothing — so bringup
# failures went unnoticed. Also fine to use interactively.
function _autolaunch_banner(){
    local line='!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
    printf '\n\033[1;97;41m%s\033[0m\n' "$line" "  AUTOLAUNCH FAILED on $(hostname) (ROBOT_NAME=${ROBOT_NAME:-unset})" "  $1" "  Scroll up in this tmux pane (airstack connect) or 'airstack logs'" "  for the first error." "$line"
    # Plain repeat so the message survives log processors that strip ANSI.
    printf '%s\n' "AUTOLAUNCH FAILED: $1"
}
function autolaunch(){
    if ! bws; then
        _autolaunch_banner "colcon build (bws) failed — the stack was NOT launched"
        return 1
    fi
    sws
    ros2 launch "$@"
    local rc=$?
    if [ $rc -ne 0 ]; then
        _autolaunch_banner "ros2 launch $* exited with code $rc — the stack is DOWN"
        return $rc
    fi
}

source /opt/ros/jazzy/setup.bash
sws # source the ROS2 workspace by default

# Resolve this container's docker compose container name from inside the
# container: reverse-DNS the container IP (hostname -> IP -> PTR record),
# then strip the network suffix. Docker's embedded DNS serves the PTR record
# with the compose container name (e.g. airstack-robot-desktop-1).
# https://wiki.psuter.ch/doku.php?id=get_docker_container_name_from_within_the_container
# WARNING: this technique ONLY works with docker version 29 and up.
_resolve_container_name() {
    host $(host $(hostname) | awk '{print $NF}') | awk '{print $NF}' | awk -F . '{print $1}'
}

# --- Fleet resolution (RFC #380 §2, OPT-IN) ---
# When FLEET_CONFIG_FILE is set (airstack up --fleet <name>), resolve this
# container's WHOLE fleet entry — name, domain, stack placement, vehicle,
# calibration overlay — via tools/fleet/resolve_fleet.py. Contract:
#   - pre-set ROBOT_NAME skips resolution entirely (same guard as the legacy
#     branch below; heterogeneous-fleet services set it explicitly);
#   - pre-set non-empty ROS_DOMAIN_ID / AIRSTACK_STACK_DIR(+ENTRY) / URDF_FILE
#     win over the fleet's values (leaf-value precedence);
#   - resolution failure warns and falls through to the legacy resolver.
# FLEET_CONFIG_FILE unset or empty = byte-identical legacy behavior.
if [ -n "${FLEET_CONFIG_FILE:-}" ] && [ -z "${ROBOT_NAME:-}" ]; then
    if [ "$ROBOT_NAME_SOURCE" == "hostname" ]; then
        fleet_identity=$(hostname)
    else
        # container-name resolution (shared helper; needs docker >= 29)
        fleet_identity=$(_resolve_container_name)
        CONTAINER_NAME=":$fleet_identity"
    fi
    fleet_resolver="$HOME/AirStack/tools/fleet/resolve_fleet.py"
    if [ -f "$fleet_resolver" ]; then
        _fleet_prev_domain="${ROS_DOMAIN_ID:-}"
        _fleet_prev_stack_dir="${AIRSTACK_STACK_DIR:-}"
        _fleet_prev_stack_entry="${AIRSTACK_STACK_ENTRY:-}"
        _fleet_prev_urdf="${URDF_FILE:-}"
        _fleet_exports=$(python3 "$fleet_resolver" "$FLEET_CONFIG_FILE" --name "$fleet_identity")
        if [ $? -eq 0 ] && [ -n "$_fleet_exports" ]; then
            eval "$_fleet_exports"
            export ROBOT_NAME VEHICLE CALIBRATION_DIR
            # pre-set env wins per variable
            [ -n "$_fleet_prev_domain" ] && ROS_DOMAIN_ID="$_fleet_prev_domain"
            export ROS_DOMAIN_ID
            if [ -n "$_fleet_prev_stack_dir" ]; then
                AIRSTACK_STACK_DIR="$_fleet_prev_stack_dir"
                AIRSTACK_STACK_ENTRY="$_fleet_prev_stack_entry"
            fi
            export AIRSTACK_STACK_DIR AIRSTACK_STACK_ENTRY
            [ -n "$_fleet_prev_urdf" ] && URDF_FILE="$_fleet_prev_urdf"
            export URDF_FILE
        else
            echo "WARNING: fleet resolution failed for '$fleet_identity' via" \
                 "$FLEET_CONFIG_FILE (resolver error above) — falling back to the" \
                 "legacy robot_name_map resolver."
        fi
        unset _fleet_prev_domain _fleet_prev_stack_dir _fleet_prev_stack_entry \
              _fleet_prev_urdf _fleet_exports
    else
        echo "WARNING: FLEET_CONFIG_FILE=$FLEET_CONFIG_FILE is set but $fleet_resolver" \
             "is missing (tools/fleet should be bind-mounted) — falling back to the" \
             "legacy robot_name_map resolver."
    fi
fi

# If ROBOT_NAME is pre-set (e.g. via docker compose), keep it.
# Otherwise extract robot name and ROS domain ID from the container/hostname mapping.
if [ -z "${ROBOT_NAME:-}" ]; then
    if [ "$ROBOT_NAME_SOURCE" == "container_name" ]; then
        # container-name resolution (shared helper; needs docker >= 29)
        name_to_map=$(_resolve_container_name)
        CONTAINER_NAME=":$name_to_map"
    elif [ "$ROBOT_NAME_SOURCE" == "hostname" ]; then
        name_to_map=$(hostname)
    else
        echo "Warning: ROBOT_NAME_SOURCE=$ROBOT_NAME_SOURCE not set to a valid value. Defaulting to 'unknown_robot'."
        name_to_map=""
        export ROBOT_NAME="unknown_robot"
        export ROS_DOMAIN_ID=0
    fi

    # set ROBOT_NAME and ROS_DOMAIN_ID from the mapping script if NAME_TO_MAP is not empty
    if [ -n "$name_to_map" ]; then
        script_path="$HOME/AirStack/robot/docker/robot_name_map/resolve_robot_name.py"
        script_dir=$(dirname "$script_path")

        existing_robot_domain_id=${ROS_DOMAIN_ID:-}

        eval "$($script_path $name_to_map $script_dir/$ROBOT_NAME_MAP_CONFIG_FILE)"
        export ROBOT_NAME

        # if ROS_DOMAIN_ID was already set in the environment, use that instead of the mapped value
        if [ -z "$existing_robot_domain_id" ]; then
            export ROS_DOMAIN_ID
        else
            export ROS_DOMAIN_ID=$existing_robot_domain_id
        fi
    fi

    # Warn about unknown robot mapping.
    if [ -z "${ROBOT_NAME:-}" ] || [ "$ROBOT_NAME" == "unknown_robot" ]; then
        echo "WARNING: could not resolve a robot identity from '${name_to_map:-<empty>}'" \
             "using $ROBOT_NAME_MAP_CONFIG_FILE."
        echo "         ROBOT_NAME='${ROBOT_NAME:-<unset>}' ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-<unset>}'"
        echo "         Topics will not be namespaced under /robot_<n>, so nothing will reach"
        echo "         the rest of the stack. Fix by either:"
        echo "           - on the HOST (not in this container): hostnamectl set-hostname robot-1,"
        echo "             which the default map resolves to robot_<n> on domain <n>; or"
        echo "           - adding a mapping YAML under robot/docker/robot_name_map/ that"
        echo "             matches your hostnames and pointing ROBOT_NAME_MAP_CONFIG_FILE at it."
        echo "         If ROBOT_NAME is empty rather than unknown_robot, check stderr above"
        echo "         for a resolve_robot_name.py error (missing or malformed map file)."
    fi
fi


# ===========================================================================
# If not running interactively, don't do anything. Exit immediately
[ -z "$PS1" ] && return

# don't put duplicate lines in the history. See bash(1) for more options
# ... or force ignoredups and ignorespace
HISTCONTROL=ignoredups:ignorespace

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTFILE=$HOME/.bash_history
HISTSIZE=-1
HISTFILESIZE=-1

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "$debian_chroot" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='[$ROBOT_NAME$CONTAINER_NAME]${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='[$ROBOT_NAME$CONTAINER_NAME]${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

alias emacs='emacs -nw'
alias sis='source install/setup.bash'

# if [ -f ~/.bash_aliases ]; then
#     . ~/.bash_aliases
# fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if [ -f /etc/bash_completion ] && ! shopt -oq posix; then
   . /etc/bash_completion
fi

# Creates a history file that stores locally on the developer computer
# check if we previously created a symlink to ~/.bash_history.
# if file is not a symlink...
if [ ! -h $HISTFILE ]; then
    # remove existing .bash_history file if it exists
    rm $HISTFILE > /dev/null 2>&1
    # initialize .bash_history file if doesn't exist yet
    if [ ! -f "$HOME/.dev/.bash_history" ]; then
        cp $HOME/.dev/.bash_history_init $HOME/.dev/.bash_history 2>/dev/null
    fi
    # symlink to /.dev/.bash_history, silently on error
    ln -s $HOME/.dev/.bash_history $HISTFILE > /dev/null 2>&1
fi


export RCUTILS_COLORIZED_OUTPUT=1  # get colored output from ROS2 tools

