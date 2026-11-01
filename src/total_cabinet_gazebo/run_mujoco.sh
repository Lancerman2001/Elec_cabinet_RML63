#!/usr/bin/env bash
set -euo pipefail
cabinet_location="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
while [[ "$cabinet_location" != / ]]; do
    if [[ -x "$cabinet_location/tools/mujoco_python.sh" && -x "$cabinet_location/run_cabinet_mujoco.sh" ]]; then
        exec "$cabinet_location/run_cabinet_mujoco.sh" "$@"
    fi
    cabinet_location="$(dirname -- "$cabinet_location")"
done
echo "Cannot locate the robot_cabinet workspace launcher." >&2
exit 2
