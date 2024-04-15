#!/bin/bash

set -e

# This script will handle the first-time setup for your package only.
cd $WORKSPACE

# Update this directory with your package name.
if [ ! -d $WORKSPACE/src/<your package> ]
then
  echo "Target package does not exist, clone that package to the source"\
        "directory and try again."
  exit 1
fi

# Add your setup steps (Install dependencies, clone additional git repositories etc.)

# Most of the dependencies should be able to install with this step. If not,
# make sure the package manifest and CMakeLists.txt has all the required
# dependencies listed first and then add additional steps.
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Don't add anything else below this line.
# User should now be able to "colcon build" your package cleanly after the script exits.
