FROM ubuntu:24.04

# --- Global Configuration ---
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    ROS_DISTRO=jazzy \
    CC=/usr/bin/gcc-11 \
    CXX=/usr/bin/g++-11 \
    PM_PACKAGES_ROOT=/opt/packman \
    ROS_DOMAIN_ID=0 \
    ROS_LOCALHOST_ONLY=1

# Add ROS 2 apt repository and install dependencies
RUN apt-get update && \
    apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && add-apt-repository -y universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
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
    xterm \
    # Python for ROS 2 nodes
    python3-pip \
    # Other utilities
    git \
    git-lfs \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*


# Isaac Sim
RUN apt-get update && \
    apt-get install -y gcc-11 g++-11 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200 && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt

# 1. Clone the repository
ARG ISAACSIM_REF=main
RUN git clone --branch ${ISAACSIM_REF} --single-branch --depth 1 https://github.com/isaac-sim/IsaacSim.git isaacsim

# 2. Initialize LFS and Pull Assets
WORKDIR /opt/isaacsim
RUN git lfs install && \
    git lfs pull && \
    echo '#!/bin/bash' > tools/eula_check.sh && \
    echo 'exit 0' >> tools/eula_check.sh && \
    chmod +x tools/eula_check.sh && \
    # Run the build in Release mode (headless/automated)
    ./build.sh --release --jobs 1

RUN chmod -R 777 /opt/isaacsim && \
    chmod -R 777 /opt/packman

RUN echo 'alias isaac-sim="/opt/isaacsim/_build/linux-x86_64/release/isaac-sim.sh"' >> /etc/bash.bashrc

RUN mkdir -p /run/user/1000 && chown 1000:1000 /run/user/1000 && chmod 0700 /run/user/1000

# Automatically source ROS 2 for any bash sessions
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    if [ -d /home/ubuntu ]; then echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/ubuntu/.bashrc; fi

# No passowrd for sudo

RUN mkdir -p /etc/sudoers.d && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu && \
    chmod 0440 /etc/sudoers.d/ubuntu

# Source ROS 2 environment

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Clean up
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-image-pipeline \
    ros-${ROS_DISTRO}-pcl-ros && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

