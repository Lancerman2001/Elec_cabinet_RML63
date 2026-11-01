#!/usr/bin/env bash
# Source before ROS 2 commands; the MuJoCo launcher selects its own Python.
arm_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME PYTHONPATH LD_LIBRARY_PATH QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX
unset GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH GAZEBO_MODEL_PATH
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
arm_deps="$arm_workspace/.deps/root"
arm_ros_prefix="$arm_deps/opt/ros/humble"
if [[ -d "$arm_ros_prefix" ]]; then
    export AMENT_PREFIX_PATH="$arm_ros_prefix:${AMENT_PREFIX_PATH:-}"
    export CMAKE_PREFIX_PATH="$arm_ros_prefix:${CMAKE_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$arm_ros_prefix/lib:${LD_LIBRARY_PATH:-}"
    export PYTHONPATH="$arm_ros_prefix/local/lib/python3.10/dist-packages:$arm_ros_prefix/lib/python3.10/site-packages:$arm_deps/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
    export PATH="$arm_ros_prefix/bin:$PATH"
fi
if [[ -f "$arm_workspace/install/local_setup.bash" ]]; then
    source "$arm_workspace/install/local_setup.bash"
fi
export GAZEBO_PLUGIN_PATH="$arm_workspace/install/lib:$arm_workspace/install/lib/rml63ii_eg2_4c2_gazebo:$arm_ros_prefix/lib:/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_RESOURCE_PATH="$arm_workspace/install/share:${GAZEBO_RESOURCE_PATH:-}"
# This workspace is a single-machine simulation. Domain IDs only separate
# discovery groups; they are not access control. Keep ROS discovery local and
# reject non-loopback Gazebo peers, including when a parent shell was networked.
export ROS_LOCALHOST_ONLY=1
unset GAZEBO_HOSTNAME
export GAZEBO_IP=127.0.0.1
export GAZEBO_IP_WHITE_LIST=127.0.0.1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-64}"
export ARM_GAZEBO_MASTER_URI="${ARM_GAZEBO_MASTER_URI:-http://127.0.0.1:11355}"
export GAZEBO_MASTER_URI="$ARM_GAZEBO_MASTER_URI"
