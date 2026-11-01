#!/usr/bin/env bash
set -eo pipefail
arm_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$arm_workspace/ros_env.sh"
if [[ ! -f "$arm_workspace/install/share/total_cabinet_gazebo/package.xml" ]]; then
    echo "Build first: $arm_workspace/build.sh" >&2
    exit 1
fi
export GAZEBO_MODEL_DATABASE_URI=""
arm_renderer="${ARM_GAZEBO_RENDERER:-auto}"
if [[ "$arm_renderer" == auto ]]; then
    if [[ -n "${WSL_DISTRO_NAME:-}" ]] || rg -qi microsoft /proc/version; then
        arm_renderer=software
    else
        arm_renderer=hardware
    fi
fi
case "$arm_renderer" in
    software)
        export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_X11_NO_MITSHM=1
        ;;
    hardware) unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER ;;
    *) echo "ARM_GAZEBO_RENDERER must be auto, software, or hardware" >&2; exit 2 ;;
esac
echo "Gazebo renderer: $arm_renderer; ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
exec /usr/bin/python3 "$arm_workspace/tools/gazebo_process.py" "$@"
