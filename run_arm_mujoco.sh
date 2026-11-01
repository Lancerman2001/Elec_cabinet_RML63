#!/usr/bin/env bash
set -euo pipefail
robot_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$robot_workspace/tools/mujoco_python.sh" \
    "$robot_workspace/src/rml63ii_eg2_4c2_gazebo/mujoco/arm_control.py" "$@"
