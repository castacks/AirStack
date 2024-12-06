# ========== BASHRC FOR ROBOT DOCKER CONTAINER ==========
# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
[ -z "$PS1" ] && return

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
    PS1='[GCS]${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='[GCS]${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
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

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
#if [ -f /etc/bash_completion ] && ! shopt -oq posix; then
#    . /etc/bash_completion
#fi

# --- ROS2 workspace setup ---

# Define the ROS2 workspace directory
ROS2_WS_DIR="$HOME/ros_ws"
# needed for communication with Isaac Sim ROS2  # https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#enabling-the-ros-bridge-extension
export FASTRTPS_DEFAULT_PROFILES_FILE="/$ROS2_WS_DIR/fastdds.xml"
# for local development, prevent conflict with other desktops
export ROS_LOCALHOST_ONLY=1

# fix ROS2 humble setuptools deprecation warning https://robotics.stackexchange.com/questions/24230/setuptoolsdeprecationwarning-in-ros2-humble/24349#24349
PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated"
export PYTHONWARNINGS

# Convenience functions for ROS2 workspace

function bws(){
    echo "Running \`colcon build\` in $ROS2_WS_DIR"
    COLCON_LOG_PATH="$ROS2_WS_DIR"/log colcon build --symlink-install --base-paths "$ROS2_WS_DIR"/ --build-base "$ROS2_WS_DIR"/build/ --install-base "$ROS2_WS_DIR"/install/
}
function sws(){
    echo "Sourcing "$ROS2_WS_DIR"/install/local_setup.bash"
    source "$ROS2_WS_DIR"/install/local_setup.bash
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
    if confirm_sws; then
        echo "Cleaning ROS2 workspace..."
        set -x
        rm -rf "$ROS2_WS_DIR"/build/ "$ROS2_WS_DIR"/install/ "$ROS2_WS_DIR"/log/
        export AMENT_PREFIX_PATH="/opt/ros/humble"
        export CMAKE_PREFIX_PATH=""
        { set +x; } 2>/dev/null  # set +x w/out it being printed
        echo "ROS2 workspace has been cleaned."
    else
        echo "Operation cancelled."
    fi
}

source /opt/ros/humble/setup.bash
sws # source the ROS2 workspace by default

export RCUTILS_COLORIZED_OUTPUT=1

export ROS_DOMAIN_ID=0
