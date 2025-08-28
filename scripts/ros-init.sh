#!/usr/bin/env bash
set -euo pipefail
docker compose run --rm --user root ros bash -lc \
  'chown -R dev:dev /home/dev && install -d -o dev -g dev /home/dev/.ros/log'
