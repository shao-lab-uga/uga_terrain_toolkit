#!/usr/bin/env bash
# Start the terrain toolkit app. Usage: scripts/run_app.sh [--port 8090] [--no-browser]
set -e
cd "$(dirname "$0")/.."
if command -v conda >/dev/null 2>&1 && conda env list | grep -q '^uga_terrain '; then
  eval "$(conda shell.bash hook)"
  conda activate uga_terrain
fi
exec python -m uga_terrain app "$@"
