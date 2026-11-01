#!/usr/bin/env bash
set -euo pipefail
arm_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$arm_workspace"
arm_package="$arm_workspace/src/rml63ii_eg2_4c2_gazebo"
/usr/bin/python3 "$arm_package/scripts/prepare_gazebo_urdf.py"
( set +u
  source "$arm_workspace/ros_env.sh"
  xacro src/fugui_gazebo/urdf/fugui.urdf.xacro \
    -o "$arm_workspace/src/fugui_gazebo/urdf/fugui.urdf"
  xacro src/IO_gazebo/urdf/breaker.urdf.xacro \
    -o "$arm_workspace/src/IO_gazebo/urdf/breaker.urdf"
  xacro src/electrical_cabinet_gazebo/urdf/electrical_cabinet.xacro \
    world_fixed:=true -o "$arm_workspace/src/electrical_cabinet_gazebo/urdf/electrical_cabinet_fixed.urdf"
)
"$arm_workspace/tools/mujoco_python.sh" "$arm_workspace/tools/rebuild_mujoco.py"
/usr/bin/python3 "$arm_workspace/src/total_cabinet_gazebo/scripts/prepare_ros2_models.py"
echo "Models regenerated. Run ./build.sh to refresh the Gazebo installation."
