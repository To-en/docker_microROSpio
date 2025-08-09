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
    sudo \
    micro 
    # rm -rf /var/lib/apt/lists/*

################################################################################
# Default user microROS setup & PIO core installation
################################################################################
FROM base as development

# Create non-root user group
ARG USERNAME
ARG USER_UID
ARG USER_GID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config
#   usermod -aG dialout $USERNAME

# Give sudo permissions to user-setup (needed for PlatformIO device access)
# Copy root's bashrc to user and give permission
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    cp ~/.bashrc /home/$USERNAME/.bashrc && \
    chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

# --------------------- Create microROS workspace
WORKDIR /home/$USERNAME

# Instantiate microROS worksapce, clone setup file , and update ROS2 dependencies
RUN mkdir microros_ws && \
    cd microros_ws && \
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    sudo apt-get update && \ 
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# Build microROS setup tools, and create agent
RUN . /opt/ros/$ROS_DISTRO/setup.bash && \
    cd microros_ws && \
    colcon build && \
    . install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh

# Add entrypoint to source ROS2 and microROS environments
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Change the user owner over home directory
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME
# Change to developer use 
USER $USERNAME

# --------------------- Install PlatformIO at developer user home
RUN curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py && \
    sudo apt-get install python3.12-venv -y && \
    python3 get-platformio.py && rm -rf get-platformio.py && \
    sudo rm -rf /var/lib/apt/lists/*

# Create symlink that point to .platformio executable
RUN mkdir -p ~/.local/bin && \
    ln -s ~/.platformio/penv/bin/platformio ~/.local/bin/platformio && \
    ln -s ~/.platformio/penv/bin/pio ~/.local/bin/pio && \
    ln -s ~/.platformio/penv/bin/piodebuggdb ~/.local/bin/piodebuggdb

# Prepare the workspace
RUN mkdir ~/microros_ws/src/my_pkgs && \
    mkdir ~/pio_projects 

# Add somecommand to startup bash in case docker exec
RUN echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc && \
    echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.profile && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.profile

# Change the ownership of home/developer folder
ENV MICROROS_WS=/home/$USERNAME/microros_ws 
ENV PIO_WS=/home/$USERNAME/pio_projects

# Execute entrypoint.sh when the container just run
ENTRYPOINT ["/bin/bash","/entrypoint.sh"]
# execute with bash , the provided path is /entrypoint.sh

# Provide argument to entrypoint.sh to $@ 
CMD ["bash"]

################################################################################
# Networking , Device setup
################################################################################
