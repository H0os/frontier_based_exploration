#!/bin/bash
# Basic entrypoint for ROS Docker containers

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
# --- Ensure Ultralytics (installed in /opt/venv) is importable even if scripts use /usr/bin/python3 ---
VENV_DIR="/opt/venv"
if [ -x "${VENV_DIR}/bin/python3" ]; then
  VENV_PYVER="$(${VENV_DIR}/bin/python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  export PYTHONPATH="${VENV_DIR}/lib/python${VENV_PYVER}/site-packages:${PYTHONPATH:-}"
  export PATH="${VENV_DIR}/bin:${PATH}"
  echo "Using venv site-packages: ${VENV_DIR}/lib/python${VENV_PYVER}/site-packages"
  echo "Runtime python: $(which python3)"
fi
# -----------------------------------------------------------------------------------------------

  echo "Sourced overlay workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"