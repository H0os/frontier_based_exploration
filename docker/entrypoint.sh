#!/bin/bash
# Basic entrypoint for ROS Docker containers
# docker/entrypoint.sh  (add near the top, before running ros2)
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
source /frontier_exploration_ws/install/setup.bash || true
source /overlay_ws/install/setup.bash || true

# NEW: ensure CLI scripts and python use the venv
source /opt/venv/bin/activate
echo "Activated venv: $(python3 -V)"

exec "$@"

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS 2 ${ROS_DISTRO}"

# Source the base workspace, if built
if [ -f /frontier_exploration_ws/install/setup.bash ]
then
  source /frontier_exploration_ws/install/setup.bash
  echo "Sourced Frontier Exploration base workspace"
fi

# Source the overlay workspace, if built
if [ -f /overlay_ws/install/setup.bash ]
then
  source /overlay_ws/install/setup.bash
  echo "Sourced overlay workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"