#!/usr/bin/env bash
set -euo pipefail

HOME="${HOME:-/home/dev}"

# Fix ownership (volume may be root-owned on first run)
# if command -v sudo >/dev/null 2>&1; then
#   sudo chown -R dev:dev "$HOME/.local" "$HOME/.ros" "$HOME/.zsh_history" 2>/dev/null || true
# fi

# Make dirs zsh/nvim/ROS expect
mkdir -p "$HOME/.local/state" "$HOME/.local/share/nvim" "$HOME/.cache"
mkdir -p "$HOME/.ros/log"
touch "$HOME/.zsh_history" || true
chmod 600 "$HOME/.zsh_history" || true

# Hand off to the requested command (default to interactive zsh)
if [ "$#" -eq 0 ]; then
  exec zsh -i
else
  exec "$@"
fi
