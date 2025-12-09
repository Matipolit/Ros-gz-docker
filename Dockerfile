# Use an official Ubuntu 24.04 base image
FROM ubuntu:24.04

# Set environment variables to non-interactive to avoid prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Set ROS distribution
ENV ROS_DISTRO=jazzy

# Add ROS 2 apt repository and install dependencies
RUN apt-get update && \
    apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update

# Install ROS 2 Jazzy, Gazebo Harmonic (via ros-gz), and other useful tools
# ros-${ROS_DISTRO}-ros-gz will pull in the correct Gazebo version (Harmonic for Jazzy)
# and ROS 2 integration packages.
RUN apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-ros-gz \
    ros-dev-tools \
    # For checking OpenGL
    mesa-utils \
    # For Wayland/X11 GUI applications (some might still need X libs even if targeting Wayland)
    # libgl1-mesa-glx \
    libxkbcommon0 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcb-xinput0 \
    libxcb-xfixes0 \
    xauth \
    # Python for ROS 2 nodes
    python3-pip \
    # Other common dependencies you might need
    git \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

# EVO

RUN apt-get update && apt-get install -y pipx

# Konfiguracja pipx, aby instalował narzędzia "systemowo" (dostępne dla wszystkich)
ENV PIPX_HOME=/opt/pipx
ENV PIPX_BIN_DIR=/usr/local/bin

RUN pipx install evo

# Set up a non-root user (optional but good practice)
# If the OSRF images create a 'ros' user, you might want to align with that.
# For simplicity here, we'll continue as root or you can create a user.
# RUN useradd -m rosuser && echo "rosuser:rosuser" | chpasswd && adduser rosuser sudo
# USER rosuser
# WORKDIR /home/rosuser/ros2_ws

# If you created a user, ensure they can access GPU devices if not running as root
# RUN groupadd video && usermod -a -G video rosuser
#
RUN mkdir -p /run/user/1000 && chown 1000:1000 /run/user/1000 && chmod 0700 /run/user/1000

# No passowrd for sudo

RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu && \
    chmod 0440 /etc/sudoers.d/ubuntu

# Source ROS 2 environment

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

