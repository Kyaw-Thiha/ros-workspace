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

# Optional: try to install foxglove_bridge from apt for this ROS distro (if available)
# This won't fail your build if the package doesn't exist in the repo yet.
RUN bash -lc 'apt-get update && (apt-get install -y --no-install-recommends ros-$ROS_DISTRO-foxglove-bridge || echo "ros-$ROS_DISTRO-foxglove-bridge not found; you can colcon build it later") && rm -rf /var/lib/apt/lists/*'

# --- Convenience wrappers ---
# RViz under X11/XWayland only (leaves system default as Wayland for everything else)
RUN printf '#!/usr/bin/env bash\nset -e\nexport QT_QPA_PLATFORM=xcb\n# (optional) avoid wayland by unsetting this if inherited\nunset WAYLAND_DISPLAY || true\nexec rviz2 \"$@\"\n' > /usr/local/bin/rviz2-x11 && chmod +x /usr/local/bin/rviz2-x11

# One-liner to run foxglove_bridge on a given port (default 8765)
RUN printf '#!/usr/bin/env bash\nset -e\nPORT=\"${1:-8765}\"\nexec ros2 run foxglove_bridge foxglove_bridge --port \"$PORT\"\n' > /usr/local/bin/foxglove-bridge && chmod +x /usr/local/bin/foxglove-bridge

# For RViz cache
RUN install -d -o ubuntu -g ubuntu /home/ubuntu/.cache \
 && install -d -o ubuntu -g ubuntu /home/ubuntu/.cache/mesa_shader_cache \
 && install -d -o ubuntu -g ubuntu /home/ubuntu/.cache/mesa_shader_cache_db

# Passwordless sudo (optional; convenient for rosdep init once)
# RUN echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev

# IMPORTANT: run as ubuntu by default (compose can still override, but this matches your config)
USER ubuntu
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
