#!/bin/bash

set -e

# This script will handle the first-time setup for moveit2 only.

cd $WORKSPACE/src

if [ ! -d panda_description ]
then
  git clone -b ros2 https://github.com/ros-planning/moveit_resources.git
  mv moveit_resources/panda_description .
  rm -rf moveit_resources
fi

# Setup steps (Install dependencies, clone additional git repositories etc.)

cd $WORKSPACE

sudo apt update && sudo apt install -y \
  ros-humble*controller* \
  ros-humble*joint*state* \
  python3-colcon-common-extensions \
  python3-colcon-mixin

colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

rosdep update --rosdistro $ROS_DISTRO
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# Don't add anything else below this line.
# User should now be able to "colcon build" your package cleanly after the script exits.

# Build command after script exits: colcon build --mixin release --executor sequential
