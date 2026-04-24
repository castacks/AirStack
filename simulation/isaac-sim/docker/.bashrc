# ========== BASHRC FOR ISAAC SIM DOCKER CONTAINER ==========
# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# don't put duplicate lines in the history. See bash(1) for more options
# ... or force ignoredups and ignorespace
HISTCONTROL=ignoredups:ignorespace

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

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
    PS1='[DOCKER Isaac]${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='[DOCKER Isaac]${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
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

# make xterm black bg
alias xterm='xterm -bg black -fg white'

alias emacs='emacs -nw'
alias sis='source install/setup.bash'

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
#if [ -f /etc/bash_completion ] && ! shopt -oq posix; then
#    . /etc/bash_completion
#fi

tmux source ~/.tmux.conf


# Creates a history file that stores locally on the developer computer
# check if we previously created a symlink to ~/.bash_history
if [ ! -h ~/.bash_history ]; then
    # File is not a symlink
    rm ~/.bash_history || echo "No existing .bash_history to remove"
    # initialize .bash_history file if doesn't exist yet
    if [ ! -d ~/.dev/.bash_history ]; then
        cp ~/.dev/.bash_history_init ~/.dev/.bash_history
    fi
    # symlink to ~/.dev/.bash_history
    ln -s ~/.dev/.bash_history ~/.bash_history
fi


# --- ROS2 setup ---

source /opt/ros/jazzy/setup.bash
source /isaac-sim/jazzy_ws/install/setup.bash  # isaacsim ros2 package

# --- Middleware: Zenoh (rmw_zenoh_cpp) ---
# Connects to the Zenoh router running inside the GCS container.
# - Bridge-network (standard sim): default 172.31.0.10 (gcs static IP on airstack_network)
# - Host-network (isaac-sim-hitl): operator sets ZENOH_ROUTER_IP to the GCS host LAN IP
#   (falls back to the legacy DISCOVERY_SERVER_IP so existing HITL configs work unchanged)
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_ROUTER_IP="${ZENOH_ROUTER_IP:-${DISCOVERY_SERVER_IP:-172.31.0.10}}"
ZENOH_ROUTER_PORT="${ZENOH_ROUTER_PORT:-7447}"
cat > /tmp/zenoh_session.json5 <<EOF
{
  mode: "peer",
  connect: { endpoints: ["tcp/${ZENOH_ROUTER_IP}:${ZENOH_ROUTER_PORT}"] },
  scouting: { multicast: { enabled: false }, gossip: { enabled: true, multihop: true } },
  // See robot .bashrc for why SHM is disabled.
  transport: { shared_memory: { enabled: false } }
}
EOF
export ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_session.json5

# Zenoh treats ROS_DOMAIN_ID as a hard-isolation keyexpr prefix; all containers share domain 0.
export ROS_DOMAIN_ID=0

# --- Environment Separation ---
# Define the PYTHONPATH specifically for Isaac Sim (Python 3.10)
# This strips out the System ROS (Python 3.12) paths to prevent conflicts
export ISAAC_SIM_PYTHONPATH=$(echo $PYTHONPATH | tr ':' '\n' | grep -v "lib/python3.12/site-packages" | paste -sd ':' -):/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy

# Helper function to run Isaac Sim python scripts with the correct environment
run_isaac_python() {
    PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh "$@"
}
export -f run_isaac_python

# --- Isaac Setup ---
alias runapp="/isaac-sim/runapp.sh --path omniverse://airlab-nucleus.andrew.cmu.edu/Library/Assets/Ascent_Aerosystems/Spirit_UAV/spirit_uav_red_yellow.prop.usd"
alias runheadless.native=/isaac-sim/runheadless.native.sh
alias runheadless.webrtc=/isaac-sim/runheadless.webrtc.sh

export ISAACSIM_PATH=/isaac-sim
alias ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
