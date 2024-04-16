#!/bin/bash

set -e

# This script will handle the first-time setup for moveit2_tutorials package only.

cd $WORKSPACE

if [ ! -d $WORKSPACE/src/moveit2_tutorials ]
then
  git clone --branch humble https://github.com/ros-planning/moveit2_tutorials src/moveit2_tutorials
fi

# Setup steps (Install dependencies, clone additional git repositories etc.)

sudo apt update && sudo apt install -y \
  python3-vcstool \
  python3-colcon-common-extensions \
  python3-colcon-mixin

colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

vcs import < src/moveit2_tutorials/moveit2_tutorials.repos
rosdep update && rosdep install -r--from-paths src --ignore-src --ros-distro $ROS_DISTRO -y

# Don't add anything else below this line.
# User should now be able to "colcon build" your package cleanly after the script exits.

# Build command after script exits: colcon build --mixin release --executor sequential
