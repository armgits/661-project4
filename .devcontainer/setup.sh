#!/bin/bash

set -e

# This script should handle the general first-time setup

# At least add these two lines every time a new ros container spins.
echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

echo -e '\nexport GPG_TTY=$(tty)' >> ~/.bashrc
