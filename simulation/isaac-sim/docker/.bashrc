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

# needed for communication with Isaac Sim ROS2  # https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#enabling-the-ros-bridge-extension
export FASTRTPS_DEFAULT_PROFILES_FILE="/isaac-sim/.ros/fastdds.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# HITL DDS mode:
# - server (default): use Fast DDS Discovery Server via ROS_DISCOVERY_SERVER.
# - static-peer: generate a temporary Fast DDS profile with one initial peer.
if [ "${HITL_DISCOVERY_MODE:-}" = "static-peer" ]; then
    if [ -n "${FASTDDS_STATIC_PEER_IP:-}" ]; then
        cat > /tmp/fastdds_static_peer.xml <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UdpTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="udp_transport_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>UdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
            <builtin>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>${FASTDDS_STATIC_PEER_IP}</address>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
EOF
        export FASTRTPS_DEFAULT_PROFILES_FILE="/tmp/fastdds_static_peer.xml"
    fi
    unset ROS_DISCOVERY_SERVER
elif [ -n "${DISCOVERY_SERVER_IP:-}" ]; then
    discovery_server_list="${DISCOVERY_SERVER_IP}:${DISCOVERY_SERVER_PORT:-11811}"
    if [ -n "${DISCOVERY_SERVER_BACKUP_IPS:-}" ]; then
        IFS=',' read -r -a discovery_backup_ips <<< "${DISCOVERY_SERVER_BACKUP_IPS}"
        for backup_ip in "${discovery_backup_ips[@]}"; do
            if [ -n "${backup_ip}" ]; then
                discovery_server_list="${discovery_server_list};${backup_ip}:${DISCOVERY_SERVER_PORT:-11811}"
            fi
        done
    fi
    export ROS_DISCOVERY_SERVER="${discovery_server_list}"
fi
# for local development, prevent conflict with other desktops
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# --- Environment Separation ---
# Define the PYTHONPATH specifically for Isaac Sim (Python 3.10)
# This strips out the System ROS (Python 3.12) paths to prevent conflicts
export ISAAC_SIM_PYTHONPATH=$(echo $PYTHONPATH | tr ':' '\n' | grep -v "lib/python3.12/site-packages" | paste -sd ':' -):/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy

# Helper function to run Isaac Sim python scripts with the correct environment.
run_isaac_python() {
    PYTHONPATH="$ISAAC_SIM_PYTHONPATH" exec /isaac-sim/python.sh "$@"
}
export -f run_isaac_python

# --- Isaac Setup ---
alias runapp="/isaac-sim/runapp.sh --path omniverse://airlab-nucleus.andrew.cmu.edu/Library/Assets/Ascent_Aerosystems/Spirit_UAV/spirit_uav_red_yellow.prop.usd"
alias runheadless.native=/isaac-sim/runheadless.native.sh
alias runheadless.webrtc=/isaac-sim/runheadless.webrtc.sh

export ISAACSIM_PATH=/isaac-sim
alias ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
