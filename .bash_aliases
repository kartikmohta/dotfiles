alias rm="rm -i"
alias mv="mv -i"
alias cp="cp -i"
alias grep="grep --color -n"
alias ll='ls -lh'
alias la='ls -A'
alias l='ls -CF'
alias df="df -Th"

alias rqt_reconfigure="rosrun rqt_reconfigure rqt_reconfigure"
alias rqt_runtime_monitor="rosrun rqt_runtime_monitor rqt_runtime_monitor"

change_master_uri()
{
  if [ -z "$1" ]; then
    echo "Specify the master ip/hostname"
  else
    export ROS_MASTER_URI="http://${1}:11311"
    echo "Set ROS_MASTER_URI to ${ROS_MASTER_URI}"
  fi
}

my_ip()
{
  local IP=$(ip route get 8.8.8.8 | head -1 | awk '{print $7}')
  if [ -z "$IP" ]; then
    local IP=$(ip -o -4 addr list eth0 | awk '{print $4}' | cut -d/ -f1)
  fi
  echo $IP
}

export LESS_TERMCAP_mb=$'\E[01;31m' # begin blinking
export LESS_TERMCAP_md=$'\E[01;34m' # begin bold
export LESS_TERMCAP_me=$'\E[0m'     # end mode (reset default appearance)
export LESS_TERMCAP_so=$'\E[47;30m' # begin standout-mode
export LESS_TERMCAP_se=$'\E[0m'     # end standout-mode
export LESS_TERMCAP_us=$'\E[01;32m' # begin underline
export LESS_TERMCAP_ue=$'\E[0m'     # end underline
