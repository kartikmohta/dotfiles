# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=10000
HISTFILESIZE=20000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

#if [ -n "$DISPLAY" -a "$TERM" == "xterm" ]; then
#  export TERM=xterm-256color
#fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput -o -x /bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[01;34m\] \w \$\[\033[00m\] '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h \w \$ '
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
if [ -x /usr/bin/dircolors -o -x /bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# some more ls aliases
alias ll='ls -lF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if [ -f /etc/bash_completion ] && ! shopt -oq posix; then
    . /etc/bash_completion
fi

export EDITOR="vim"

# For new tab in gnome-terminal to open in same directory as the previous one
# See: https://bugzilla.gnome.org/show_bug.cgi?id=697475
. /etc/profile.d/vte.sh

# http://jonisalonen.com/2012/your-bash-prompt-needs-this/
#PS1="\[\033[G\]$PS1"


export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

export CPPFLAGS="-D_FORTIFY_SOURCE=2"
export CFLAGS="-march=native -mtune=native -pipe -fstack-protector-strong -fno-plt"
export CXXFLAGS="-march=native -mtune=native -pipe -fstack-protector-strong -fno-plt"
export LDFLAGS="-Wl,-O1,--sort-common,--as-needed,--no-undefined,-z,relro,-z,now,--hash-style=gnu -pthread"

# ROS
ROSDISTRO=melodic
if [ -f /opt/ros/$ROSDISTRO/setup.bash ]; then
  source /opt/ros/$ROSDISTRO/setup.bash
  export ROS_WORKSPACE=$HOME/programs/ros
fi
unset ROSDISTRO

#export ROS_IP=$(my_ip)
if [[ ${ROS_IP} = "" ]]; then
  unset ROS_IP
fi

# Matlab
if [ -f /usr/local/bin/mex ]; then
  export MATLAB_ROOT="/opt/MATLAB/R2017a"
  export MEX="/usr/local/bin/mex"
fi

# Gazebo
if [ -f /usr/share/gazebo/setup.sh ]; then
  source /usr/share/gazebo/setup.sh
fi

export ROSWS_HOME_DIR=$HOME/programs/ros
source $HOME/programs/ros_scripts/rosws_switch/rosws_switch.complete

# Always prompt for GPG password from terminal
export GPG_TTY=$(tty)
