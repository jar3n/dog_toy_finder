# Base image: ROS 2 Humble on Ubuntu 22.04
# Use linux/arm64 for Apple Silicon Macs, linux/amd64 for Intel Macs and Windows
FROM ros:humble

# Install Python pip, the ROS 2 Python client, and the packages used by the
# computer-side stack (odometry, EKF, TF, and the URDF publisher).
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-rclpy \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-robot-state-publisher \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-proc \
    ros-humble-stereo-image-proc \
    ros-humble-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*

# Pre-install Python dependencies (used in the ROSA guide)
RUN pip3 install --no-cache-dir \
    ollama \
    jpl-rosa \
    'langchain-ollama<1.0' \
    'langchain-core>=0.3.52,<1.0' \
    'mcp>=1.9.2,<2.0' \
    'langchain-mcp-adapters==0.1.10' \
    'chromadb==0.6.3' \
    'posthog>=2.4,<4.0' \
    requests

# Source ROS 2 automatically in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
ENV PYTHONPATH="/opt/ros/humble/lib/python3.10/dist-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH}"
ENV LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH}"
ENV AMENT_PREFIX_PATH="/opt/ros/humble"

WORKDIR /workspace
