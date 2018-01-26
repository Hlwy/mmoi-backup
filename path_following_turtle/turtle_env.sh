#!/bin/sh
export ROS_HOSTNAME="myHostname"
export ROS_IP="myIp"
export ROS_MASTER_URI="http://masterIP:11311"
export ROSLAUNCH_SSH_UNKNOWN="1"
. /opt/ros/indigo/setup.sh
. ~/the_hunting_ground/devel/setup.sh
exec "$@"
