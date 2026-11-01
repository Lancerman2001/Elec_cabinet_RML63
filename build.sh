#!/usr/bin/env bash
set -eo pipefail
arm_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$arm_workspace/ros_env.sh"
cd "$arm_workspace"
exec colcon build --merge-install --parallel-workers 2 "$@" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
