# 1. Base Image: Start with the official ROS 2 Humble image (multi-arch: works on x86 laptops and the arm64 Pi 5)
FROM ros:humble-ros-base-jammy

# 2. Set the working directory inside the container
WORKDIR /farm_robot_ws

# 3. Install core dependencies, the Navigation2 stack, and twist_mux
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-executive-smach \
    ros-humble-twist-mux \
    wget \
    curl \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 4. Install Hailo AI Drivers (HailoRT) -- only on arm64 and only if the .deb is present in installers/.
# Laptops without the file still build; the CV node just won't find HailoRT.
COPY ./installers /tmp/installers
RUN if [ "$(dpkg --print-architecture)" = "arm64" ] && ls /tmp/installers/hailort*_arm64.deb >/dev/null 2>&1; then \
        apt-get update \
        && apt-get install -y /tmp/installers/hailort*_arm64.deb \
        && rm -rf /var/lib/apt/lists/*; \
    else \
        echo "Skipping HailoRT (not arm64 or no .deb in installers/)"; \
    fi \
    && rm -rf /tmp/installers

# 5. Copy your team's code (the 'src' folder) into the container
COPY ./src /farm_robot_ws/src

# 6. Install any remaining package.xml dependencies, then build the workspace
RUN apt-get update \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -y --rosdistro humble \
    && rm -rf /var/lib/apt/lists/* \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# 7. Setup the Entrypoint
# This ensures that every time the container starts, it "remembers" the ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /farm_robot_ws/install/setup.bash" >> ~/.bashrc

# 8. The default command when the container runs
CMD ["/bin/bash"]
