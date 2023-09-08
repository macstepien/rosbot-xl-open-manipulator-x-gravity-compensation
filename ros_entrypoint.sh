#!/bin/bash
set -e

# setup ros environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/maciej/catkin_ws/devel/setup.bash --extend

exec "$@"