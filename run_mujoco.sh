#!/usr/bin/env bash
set -euo pipefail
arm_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$arm_workspace/tools/mujoco_python.sh" \
    "$arm_workspace/src/total_cabinet_gazebo/mujoco/interaction_gui.py" "$@"
