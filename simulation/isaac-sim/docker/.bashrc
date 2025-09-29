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

source /opt/ros/humble/setup.bash
source /humble_ws/install/setup.bash  # isaacsim ros2 package
# needed for communication with Isaac Sim ROS2  # https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#enabling-the-ros-bridge-extension
export FASTRTPS_DEFAULT_PROFILES_FILE="/isaac-sim/fastdds.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# for local development, prevent conflict with other desktops
export ROS_LOCALHOST_ONLY=1

# --- Isaac Setup ---
alias runapp="/isaac-sim/runapp.sh --path omniverse://airlab-nucleus.andrew.cmu.edu/Library/Assets/Ascent_Aerosystems/Spirit_UAV/spirit_uav_red_yellow.prop.usd"
alias runheadless.native=/isaac-sim/runheadless.native.sh
alias runheadless.webrtc=/isaac-sim/runheadless.webrtc.sh

export ISAACSIM_PATH=/isaac-sim
alias ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"

# Forward mavlink
spawn_mavlink_routers() {
    # Check if NUM_ROBOTS is set
    if [ -z "$NUM_ROBOTS" ]; then
        echo "Error: NUM_ROBOTS environment variable is not set"
        return 1
    fi

    # Check if PX4_BASE_PORT is set
    if [ -z "$PX4_BASE_PORT" ]; then
        echo "Error: PX4_BASE_PORT environment variable is not set"
        return 1
    fi

    # Validate NUM_ROBOTS is a positive integer
    if ! [[ "$NUM_ROBOTS" =~ ^[1-9][0-9]*$ ]]; then
        echo "Error: NUM_ROBOTS must be a positive integer"
        return 1
    fi

    echo "Spawning $NUM_ROBOTS mavlink router instances..."

    # Loop through robot IDs from 1 to NUM_ROBOTS
    for robot_id in $(seq 1 "$NUM_ROBOTS"); do
        # Calculate the port for this robot
        port=$((PX4_BASE_PORT + robot_id))
        
        # Create tmux session name
        session_name="mavlink_robot_$robot_id"
        
        echo "Starting mavlink router for robot $robot_id on port $port..."
        
        # Create new tmux session and run mavlink-routerd
        tmux new-session -d -s "$session_name" \
            "mavlink-routerd -e 172.31.0.255:$port 127.0.0.1:$port"
        
        if [ $? -eq 0 ]; then
            echo "  ✓ Started tmux session '$session_name' for robot $robot_id"
        else
            echo "  ✗ Failed to start tmux session for robot $robot_id"
        fi
    done
    
    echo "Finished spawning mavlink router instances."
    echo "Use 'tmux list-sessions' to see all sessions."
    echo "Use 'tmux attach-session -t mavlink_robot_N' to attach to robot N."
}

kill_mavlink_routers() {
    # Check if NUM_ROBOTS is set
    if [ -z "$NUM_ROBOTS" ]; then
        echo "Error: NUM_ROBOTS environment variable is not set"
        return 1
    fi

    # Validate NUM_ROBOTS is a positive integer
    if ! [[ "$NUM_ROBOTS" =~ ^[1-9][0-9]*$ ]]; then
        echo "Error: NUM_ROBOTS must be a positive integer"
        return 1
    fi

    echo "Killing $NUM_ROBOTS mavlink router instances..."

    # Loop through robot IDs from 1 to NUM_ROBOTS
    for robot_id in $(seq 1 "$NUM_ROBOTS"); do
        # Create tmux session name
        session_name="mavlink_robot_$robot_id"
        
        # Check if session exists
        if tmux has-session -t "$session_name" 2>/dev/null; then
            echo "Killing tmux session '$session_name' for robot $robot_id..."
            tmux kill-session -t "$session_name"
            
            if [ $? -eq 0 ]; then
                echo "  ✓ Killed session '$session_name'"
            else
                echo "  ✗ Failed to kill session '$session_name'"
            fi
        else
            echo "  ⚠ Session '$session_name' not found (robot $robot_id)"
        fi
    done
    
    echo "Finished killing mavlink router instances."
}