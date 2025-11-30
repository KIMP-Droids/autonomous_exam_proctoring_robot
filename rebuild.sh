#!/usr/bin/env bash
set -e

# Change this
WS=~/development/ros/exam_bot_ws

echo ">>> Cleaning build, install, log in $WS"
rm -rf "$WS/build" "$WS/install" "$WS/log"

echo ">>> Sourcing base ROS (adjust distro if needed)"
source /opt/ros/jazzy/setup.sh

echo ">>> Building workspace (symlink install)"
cd "$WS"
colcon build --symlink-install

echo ">>> Sourcing workspace overlay"
source "$WS/install/setup.sh"

echo ">>> Done. You're now in: $WS"
