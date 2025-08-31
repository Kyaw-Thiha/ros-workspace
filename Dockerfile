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
    # neovim ripgrep git curl ca-certificates sudo \
    ripgrep git curl ca-certificates sudo \
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

# --- Lazygit CLI (build from source with Go) ---
RUN set -eux; \
  apt-get update && apt-get install -y --no-install-recommends git golang ca-certificates && \
  GOBIN=/usr/local/bin GOPATH=/root/go GO111MODULE=on go install github.com/jesseduffield/lazygit@latest && \
  rm -rf /root/go && \
  rm -rf /var/lib/apt/lists/*

# --- Clang and colcon for ROS Nvim ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    clangd clang-format python3-colcon-mixin \
 && rm -rf /var/lib/apt/lists/*



# --- Newer Neovim (>= 0.10) from the official PPA ---
# RUN apt-get update && apt-get purge -y neovim neovim-runtime || true && \
#     apt-get install -y --no-install-recommends software-properties-common curl ca-certificates && \
#     add-apt-repository -y ppa:neovim-ppa/stable && \
#     apt-get update && apt-get install -y --no-install-recommends neovim && \
#     nvim --headless +"lua assert(vim.version().minor>=10, 'Need Neovim >= 0.10')" +qa && \
#     rm -rf /var/lib/apt/lists/*

# --- Neovim via official binary (guaranteed >= 0.10) ---
# Pin to a specific stable version and verify checksum to avoid partial/HTML downloads.
ARG NEOVIM_VERSION=0.10.3
RUN set -eux; \
    apt-get update && apt-get purge -y neovim neovim-runtime || true; \
    apt-get install -y --no-install-recommends ca-certificates curl; \
    url_base="https://github.com/neovim/neovim/releases/download/v${NEOVIM_VERSION}"; \
    curl -fL --retry 5 --retry-delay 2 -o /tmp/nvim-linux64.tar.gz "${url_base}/nvim-linux64.tar.gz"; \
    curl -fL --retry 5 --retry-delay 2 -o /tmp/nvim-linux64.tar.gz.sha256sum "${url_base}/nvim-linux64.tar.gz.sha256sum"; \
    (cd /tmp && sha256sum -c nvim-linux64.tar.gz.sha256sum); \
    tar -xzf /tmp/nvim-linux64.tar.gz -C /opt; \
    ln -sfn /opt/nvim-linux64/bin/nvim /usr/local/bin/nvim; \
    nvim --headless +"lua assert(vim.version().minor>=10, 'Need Neovim >= 0.10')" +qa; \
    rm -f /tmp/nvim-linux64.tar.gz /tmp/nvim-linux64.tar.gz.sha256sum


RUN apt-get update && apt-get install -y --no-install-recommends \
    qtwayland5 qt6-wayland mesa-utils \
 && rm -rf /var/lib/apt/lists/*


# --- Rqt on Ubuntu 24.04 (Noble) ---
RUN apt-get update && apt-get install -y \
  ros-$ROS_DISTRO-rqt \
  ros-$ROS_DISTRO-rqt-common-plugins \
  ros-$ROS_DISTRO-rqt-graph \
  ros-$ROS_DISTRO-rqt-image-view \
  ros-$ROS_DISTRO-rqt-plot \
 && rm -rf /var/lib/apt/lists/*

# --- Gazebo (modern "gz") on Ubuntu 24.04 (Noble) ---
# Adds OSRF repo and installs Gazebo (Ionic is the officially documented Noble target).
# If you prefer Harmonic for tighter ROS 2 Jazzy parity, you can swap 'gz-ionic' => 'gz-harmonic'.
RUN apt-get update && apt-get install -y --no-install-recommends lsb-release gnupg && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg | sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg >/dev/null && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    apt-get update && apt-get install -y --no-install-recommends \
      gz-ionic \
      ros-$ROS_DISTRO-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# --- Convenience wrappers (XWayland only) ---
# Keep Wayland default for everything else; force only Gazebo GUIs to X11
RUN printf '#!/usr/bin/env bash\nset -e\nexport QT_QPA_PLATFORM=xcb\nunset WAYLAND_DISPLAY || true\nexec gz \"$@\"\n' > /usr/local/bin/gz-x11 && chmod +x /usr/local/bin/gz-x11 && \
    printf '#!/usr/bin/env bash\nset -e\nexport QT_QPA_PLATFORM=xcb\nunset WAYLAND_DISPLAY || true\nexec gz sim \"$@\"\n' > /usr/local/bin/gz-sim-x11 && chmod +x /usr/local/bin/gz-sim-x11 && \
    printf '#!/usr/bin/env bash\nset -e\nexport QT_QPA_PLATFORM=xcb\nunset WAYLAND_DISPLAY || true\nexec gz gui \"$@\"\n' > /usr/local/bin/gz-gui-x11 && chmod +x /usr/local/bin/gz-gui-x11

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

# --- Getting pynvim and pyright working ---
# Make Mason & user pip scripts visible
ENV PATH="/home/ubuntu/.local/share/nvim/mason/bin:/home/ubuntu/.local/bin:${PATH}"

# Python provider for nvim
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends python3-pynvim && sudo rm -rf /var/lib/apt/lists/*

# Pyright (LSP) globally; shows up as `pyright-langserver` on PATH
RUN sudo npm install -g pyright

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
