# 1. Base Image: Start with the official ROS 2 Humble image
FROM ros:humble-ros-base-jammy

# 2. Set the working directory inside the container
WORKDIR /farm_robot_ws

# 3. Install core dependencies and the Navigation2 stack
# We use 'apt-get' to install these directly into the Linux container environment
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-executive-smach \
    wget \
    curl \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 4. Install Hailo AI Drivers (HailoRT)
# Note: You will likely need to download the specific ARM64 .deb file from Hailo.
# This assumes you have the .deb file saved in an 'installers' folder next to this Dockerfile.
# COPY ./installers/hailort_latest_arm64.deb /tmp/
# RUN dpkg -i /tmp/hailort_latest_arm64.deb || apt-get install -f -y \
#     && rm /tmp/hailort_latest_arm64.deb

# 5. Copy your team's code (the 'src' folder) into the container
COPY ./src /farm_robot_ws/src

# 6. Build the ROS 2 workspace
# This is where your team's Python and C++ nodes are compiled
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# 7. Setup the Entrypoint
# This ensures that every time the container starts, it "remembers" the ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /farm_robot_ws/install/setup.bash" >> ~/.bashrc

# 8. The default command when the container runs
CMD ["/bin/bash"]