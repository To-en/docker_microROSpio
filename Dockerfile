# syntax=docker/dockerfile:1

################################################################################
# Use official ROS2 Jazzy Desktop image as base
################################################################################

FROM osrf/ros:jazzy-desktop as base
# Change the shell to bash in order to run bash command
SHELL ["/bin/bash", "-c"]

# Add site config to the image
COPY site_configs/ /site_configs/

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install additional system dependencies not in base ROS image
RUN apt-get update && apt-get install -y \
    curl \
    git \
    python3-pip \
    build-essential \
    cmake \
    wget \
    software-properties-common \
    sudo \
    python3-venv \
    micro
    # && rm -rf /var/lib/apt/lists/*

################################################################################
# new user PlatformIO and microROS setup
################################################################################

FROM base as development

# Create non-root user
ARG USERNAME=developer
ARG USER_UID=2000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config
#   usermod -aG dialout $USERNAME

# Create microROS workspace
WORKDIR /home/$USERNAME

# Give sudo permissions to user-setup (needed for PlatformIO device access)
# Copy root's environment setup to user
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    cp ~/.bashrc /home/$USERNAME/.bashrc && \
    chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

# Instantiate microROS worksapce, clone setup file , and update ROS2 dependencies
RUN mkdir microros_ws && \
    cd microros_ws && \
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    apt-get update && \ 
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

ENV MICROROS_WS=/home/$USERNAME/microros_ws 

# Build microROS setup tools, and create agent
RUN . /opt/ros/$ROS_DISTRO/setup.bash && \
    cd microros_ws && \
    colcon build && \
    . install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh

# Install PlatformIO at developer user home
RUN curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py && \
    apt-get install python3.12-venv && \
    python3 get-platformio.py && \ 
    rm -rf get-platformio.py

# Remove apt list before usage
RUN rm -rf /var/lib/apt/lists/*

# Change to developer use 
USER $USERNAME

RUN mkdir -p ~/.local/bin && \
    echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.profile && \
    ln -s ~/.platformio/penv/bin/platformio ~/.local/bin/platformio && \
    ln -s ~/.platformio/penv/bin/pio ~/.local/bin/pio && \
    ln -s ~/.platformio/penv/bin/piodebuggdb ~/.local/bin/piodebuggdb

# RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
# RUN echo "source /opt/microros_ws/install/local_setup.bash" >> ~/.bashrc

# Add entrypoint to source ROS2 and microROS environments
COPY entrypoint.sh /entrypoint.sh

COPY bashrc .bashrc
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

# Default command opens a bash shell
CMD ["bash"]

################################################################################
# Networking , Device setup
################################################################################
