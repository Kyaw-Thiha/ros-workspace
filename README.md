# ROS Workspace
This is a ROS-2 (Kilted) workspace container that automatically copy & setup the zsh & nvim based on local configs.

CLI Commands:
- rviz2-x11
- rqt-x11
- gz-gui-x11

## Give permissions to run scripts
```bash
chmod +x scripts/*
```

## Installation
### 1. Clone this repo

### 2. Ensure zsh & lua works for docker settings too
#### 2a. Lua Fix
In the host (not remote) machine.
Inside `~/.config/nvim/lua/config/lazy.lua`, ensure you have the following lockfile snippet.
```lua
require("lazy").setup({
  ...
},{
  lockfile = vim.fn.stdpath("data") .. "/lazy-lock.json",
})
```

#### 2b. Zsh fix
Inside `~/.zshrc`, ensure that you have 'maybe' for unimportant plugins
```lua
# Detect container & guard optional tools
[[ -f /.dockerenv ]] && export IN_CONTAINER=1
maybe() { command -v "$1" >/dev/null 2>&1 && "$@"; }

maybe pokemon-colorscripts --no-title -s -r | maybe fastfetch -c $HOME/.config/fastfetch/config-pokemon.jsonc --logo-type file-raw --logo-height 10 --logo-width 5 --logo -
```

And ensure FZF can run on different distros
```lua
# Set-up FZF key bindings (CTRL R for fuzzy history finder)
if command -v fzf >/dev/null 2>&1; then
  # Debian/Ubuntu packaged locations
  [[ -r /usr/share/fzf/key-bindings.zsh ]] && source /usr/share/fzf/key-bindings.zsh
  [[ -r /usr/share/fzf/completion.zsh    ]] && source /usr/share/fzf/completion.zsh

  # Some distros package them under /usr/share/doc/fzf/examples
  [[ -r /usr/share/doc/fzf/examples/key-bindings.zsh ]] && source /usr/share/doc/fzf/examples/key-bindings.zsh
  [[ -r /usr/share/doc/fzf/examples/completion.zsh    ]] && source /usr/share/doc/fzf/examples/completion.zsh

  # If you installed via the git installer (~/.fzf), this covers that case:
  [[ -r ~/.fzf.zsh ]] && source ~/.fzf.zsh
fi
# --- end fzf init ---
```

### 3. Build the container
```bash
cd ros-workspace
docker-compose build
```

## Running
### 1. XHost 
#### Enabling
```bash
xhost +local:
```

#### Disabling
```bash
xhost -local:
```

### 2a. (Only for creating project) Create a single-repo project
! Do not forget to replace with your own project name, directory, and github username !
```bash
./scripts/new-project.sh --name mybot-nav --dir ~/Documents/Projects --type single --gh Kyaw-Thiha
```


### 2b. (Only for creating project) Create a multi-repo project
! Do not forget to replace with your own project name, directory, and github username !
```bash
./scripts/new-project.sh --name perception-stack --dir ~/Documents/Projects --type meta --gh Kyaw-Thiha 
```

### 3. Mount into your project
Ensure you already create the project before
```bash
./scripts/workon.sh ~/Documents/Projects/perception-stack
```

### 4a. Run code inside the single-repo container
Ensure you are already mounted into the container.
#### (a) Create a package (Only for single-repo)
```bash
./scripts/create_pkg.sh mybot_nav --dependencies rclcpp std_msgs
```

#### (b) Build the package
```bash
./scripts/build.sh
```

### 4b. Run code inside the multi-repo container
#### (a) Sync the packages
```bash
./scripts/sync.sh
```

#### (b) Build the packages
```bash
./scripts/build.sh
```

#### (c) Freeze the packages
```bash
./scripts/freeze.sh
```

## Run the terminal inside the container
```bash
./scripts/ros-sh.sh
```

## Get GPU working
### Installation
Install `nvidia-container-toolkit` on host.
```bash
yay -S nvidia-container-toolkit
```

### Docker Compose
Ensure your docker-compose.yml has the following snippets
```yaml
services:
  ros:
    ...
    gpus: all
    environment:
      ...
      # Nvidia GPU
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]

```

## Run these inside container
```bash
mkdir -p ~/.local/share/nvim ~/.local/state ~/.cache
mkdir -p ~/docker-ros-home/.ros/log   
```
