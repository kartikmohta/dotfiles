alias rm="rm -i"
alias mv="mv -i"
alias cp="cp -i"
alias grep="grep --color -n"
alias ll='ls -lh'
alias la='ls -A'
alias l='ls -CF'
alias vpsi="ssh kartikmohta@vps.igeek.in"
alias vpss="ssh sportx@vps.sportx.in"
alias eniac="ssh -X kmohta@eniac.seas.upenn.edu"
alias mrsl="ssh kartikmohta@mrsl.grasp.upenn.edu"
alias df="df -Th"
alias fgfs="fgfs --enable-terrasync --terrasync-dir=/mnt/Data/flightgear/data/Scenery --fg-scenery=/mnt/Data/flightgear/data/Scenery"

alias matlabnd="matlab -nodesktop -nosplash"
alias rqt_reconfigure="rosrun rqt_reconfigure rqt_reconfigure"
alias cmk="catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo"
alias cmki="catkin_make_isolated -DCMAKE_BUILD_TYPE=RelWithDebInfo"

alias julia="PYTHON=/usr/bin/python2 julia"

change_master_uri_fla()
{
  if [ -z "$1" ]; then
    echo "Specify the robot number ('n' in fla[n])"
  else
    export ROS_MASTER_URI="http://192.168.129.$((149 + 2*$1)):11311"
    echo "Set ROS_MASTER_URI to ${ROS_MASTER_URI}"
  fi
}

my_ip()
{
  local IP=$(ip route get 8.8.8.8 | cut -d' '  -f 8 | head -1)
  if [ -z "$IP" ]; then
    local IP=$(ip -o -4 addr list eth0 | awk '{print $4}' | cut -d/ -f1)
  fi
  echo $IP
}

export ROS_IP=$(my_ip)
