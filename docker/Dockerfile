# ROS distribution to use
ARG ROS_DISTRO=jazzy

#######################################
# Base Image for TurtleBot Simulation #
#######################################
FROM ros:${ROS_DISTRO} AS base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 curl git libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 libqt5svg5-dev \
 python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-turtlebot4-simulator \
    mesa-utils libgl1-mesa-dri xauth x11-apps \
    && (apt-get install -y libgl1-mesa-glx || apt-get install -y libgl1 libglx-mesa0) \
    && rm -rf /var/lib/apt/lists/*
# Install additional Python modules
RUN pip3 install --break-system-packages matplotlib transforms3d

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-venv ffmpeg libgl1 \
 && rm -rf /var/lib/apt/lists/*

# Create and use an isolated venv
ENV VENV=/opt/venv
RUN python3 -m venv $VENV
ENV PATH="$VENV/bin:$PATH"

# Keep pip tooling current
RUN pip install --no-cache-dir --upgrade pip setuptools wheel

# --- Choose ONE torch build (CPU or CUDA). Comment out the other. ---

# CPU-only torch/vision:
RUN pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu

# CUDA 12.1 torch/vision (matches the nvidia-* cu12 wheels ultralytics likes):
#RUN pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cu121

# Your Python deps
RUN pip install --no-cache-dir matplotlib transforms3d ultralytics
# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
 curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
 apt-get update \
 apt-get install gz-harmonic

COPY control.yaml /opt/ros/jazzy/share/irobot_create_control/config/control.yaml
COPY mapper_params_lifelong.yaml /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_lifelong.yaml
COPY mapper_params_localization.yaml /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_localization.yaml
COPY mapper_params_offline.yaml /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_offline.yaml
COPY mapper_params_online_async.yaml /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml
COPY mapper_params_online_sync.yaml /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_sync.yaml


# Create Colcon workspace with external dependencies
RUN mkdir -p /frontier_exploration_ws/src
WORKDIR /frontier_exploration_ws/src
COPY dependencies.repos .
RUN vcs import < dependencies.repos

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /frontier_exploration_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && AMENT_PYTHON_EXECUTABLE=/usr/bin/python3 COLCON_PYTHON_EXECUTABLE=/usr/bin/python3 colcon build --symlink-install --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
ENV TURTLEBOT_MODEL=4

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypoint
WORKDIR /frontier_exploration_ws
COPY ./docker/entrypoint.sh /
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

##########################################
# Overlay Image for TurtleBot Simulation #
##########################################
FROM base AS overlay

# Create an overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
COPY cyclonedds_config.xml /overlay_ws/cyclonedds_config.xml
#ENV CYCLONEDDS_URI=file:///overlay_ws/cyclonedds_config.xml
WORKDIR /overlay_ws
COPY ./tb_worlds/ ./src/tb_worlds/
COPY ./frontier_exploration/ ./src/frontier_exploration/
RUN source /frontier_exploration_ws/install/setup.bash \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && AMENT_PYTHON_EXECUTABLE=/usr/bin/python3 COLCON_PYTHON_EXECUTABLE=/usr/bin/python3 colcon build --symlink-install --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM overlay AS dev

# Dev container arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends \
 gdb gdbserver nano

# In Ubuntu 24.04, there is already a user named "ubuntu" with UID 1000.
# Delete this in the (common) event that the user on the host also has this UID.
RUN touch /var/mail/ubuntu \
    && chown ubuntu /var/mail/ubuntu \
    && userdel -r ubuntu

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /overlay_ws/

ENV VIRTUAL_ENV=/opt/venv
ENV PATH="${VIRTUAL_ENV}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"


# Set up the entrypoint, including it in the .bashrc for interactive shells
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
