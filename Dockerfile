# Use ROS 2 Humble as the base image
FROM osrf/ros:humble-desktop

# Avoid timezone prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true \
    && rosdep update

# Set the workspace directory
ENV ROS_WS=/ros2_ws
WORKDIR $ROS_WS

# Copy the workspace source code
COPY ./src src/
COPY ./dependencies.repos .

# Import external dependencies
RUN vcs import src < dependencies.repos

# Install ROS package dependencies
RUN apt-get update && rosdep install -y \
    --from-paths src \
    --ignore-src \
    --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Create an entrypoint to source setup scripts automatically
RUN echo '#!/bin/bash' > /ros_entrypoint.sh \
    && echo 'set -e' >> /ros_entrypoint.sh \
    && echo 'source /opt/ros/humble/setup.bash' >> /ros_entrypoint.sh \
    && echo 'source $ROS_WS/install/setup.bash' >> /ros_entrypoint.sh \
    && echo 'exec "$@"' >> /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
