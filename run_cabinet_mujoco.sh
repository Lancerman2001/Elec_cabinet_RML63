#!/usr/bin/env bash
set -euo pipefail
robot_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# The cabinet-only demo imports the shared MuJoCo compatibility layer.
exec "$robot_workspace/tools/mujoco_python.sh" \
    "$robot_workspace/tools/run_cabinet_mujoco.py" "$@"
