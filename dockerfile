FROM biguasim:latest

ENV DEBIAN_FRONTEND=noninteractive

USER root

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repo (no sudo, no tee)
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install ROS2
RUN apt-get update && apt-get install -y \
    ros-rolling-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# 🔵 Switch back to user AFTER installs
USER user

# Source ROS2 automatically
RUN echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc

WORKDIR /home/user/ros2_ws/src
# COPY --chown=user:user . .

# RUN /bin/bash -c "source /opt/ros/rolling/setup.bash && colcon build --symlink-install"

# CMD ["/bin/bash", "-c", "source /opt/ros/rolling/setup.bash && source /home/user/ros2_ws/biguasim-ros2/install/setup.bash && bash"]


CMD ["bash"]