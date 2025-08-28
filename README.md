# ROS Workspace
This is a ROS-2 (Kilted) workspace container that automatically copy & setup the zsh & nvim based on local configs.

## Give permissions to run scripts
```bash
chmod +x scripts/*
```

## Run these inside container
```bash
mkdir -p ~/.local/share/nvim ~/.local/state ~/.cache
mkdir -p ~/docker-ros-home/.ros/log   
```
