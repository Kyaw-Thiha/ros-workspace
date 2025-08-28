#!/usr/bin/env bash
set -euo pipefail
docker compose run --rm ros zsh -lc 'source /opt/ros/kilted/setup.zsh; rviz2'
