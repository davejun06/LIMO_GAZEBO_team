# Every terminal sources this: same ROS settings, this workspace, this folder.
_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
export ROS_LOCALHOST_ONLY=1   # all windows must match, or nodes cannot see each other
export GAZEBO_IP=127.0.0.1
export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}
source /opt/ros/humble/setup.bash
[ -f "$_WS/install/setup.bash" ] && source "$_WS/install/setup.bash"
cd "$_WS"
