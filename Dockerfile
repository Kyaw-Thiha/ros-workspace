# ROS 2 Kilted Kaiju desktop image (Ubuntu base)
# Recommended: Ubuntu 24.04 (Noble) base
FROM osrf/ros:kilted-desktop-noble
# or the larger one:
# FROM osrf/ros:kilted-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_ID=1000
ARG GROUP_ID=1000

# Create non-root user "dev" that matches your host UID/GID
# RUN groupadd -g ${GROUP_ID} dev || true && \
#     useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /usr/bin/zsh dev || true

# RUN groupadd -g ${GROUP_ID} dev || true && \
#     useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /usr/bin/bash dev || true

# Dev tooling + quality-of-life
RUN apt-get update && apt-get install -y --no-install-recommends \
    neovim ripgrep git curl ca-certificates sudo \
    zsh kitty-terminfo \
    python3-venv python3-pip \
    python3-colcon-common-extensions \
    ros-dev-tools \
    build-essential cmake \
    dbus-x11 xauth wl-clipboard \
    fzf \
    nodejs npm \
    lsd eza \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    qtwayland5 qt6-wayland mesa-utils \
 && rm -rf /var/lib/apt/lists/*

# passwordless sudo for ubuntu + prep ros home with correct ownership
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu && \
    chmod 0440 /etc/sudoers.d/ubuntu && \
    mkdir -p /home/ubuntu/.ros/log && \
    chown -R ubuntu:ubuntu /home/ubuntu

# Passwordless sudo (optional; convenient for rosdep init once)
# RUN echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev

USER dev
WORKDIR /work

# Add entrypoint script
# USER root
# COPY entrypoint.sh /usr/local/bin/entrypoint.sh
# RUN chmod +x /usr/local/bin/entrypoint.sh
# USER dev

# Auto-source ROS bash
# RUN echo 'source /opt/ros/kilted/setup.bash' >> /home/dev/.bashrc

# Auto-source ROS for zsh
# change 'kevin' here with 'dev' if needed
# RUN echo 'source /opt/ros/kilted/setup.zsh' >> /home/kevin/.zshrc
