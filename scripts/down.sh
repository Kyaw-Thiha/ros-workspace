#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
override_file="$repo_root/.local/override.yml"
if [[ ! -f "$override_file" ]]; then
  override_file="/dev/null" # allow down without override present
fi
cd "$repo_root"
docker compose -f docker-compose.yml -f "$override_file" down
