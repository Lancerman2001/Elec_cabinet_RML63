"""Run only the RML63II, gripper and camera in a local Gazebo world."""
import os
from pathlib import Path
import re
import socket
from urllib.parse import urlparse
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, UnsetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    robot = Path(get_package_share_directory("rml63ii_eg2_4c2_gazebo"))
    enabled = lambda name: LaunchConfiguration(name).perform(context).lower() == "true"
    master_uri = LaunchConfiguration("master_uri").perform(context)
    parsed = urlparse(master_uri)
    if parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost") or not parsed.port:
        raise ValueError("master_uri must be an http://127.0.0.1:PORT local address")
    # Refuse to attach to an existing master: all shutdown requests below must
    # target only the server created by this launch.
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("127.0.0.1", parsed.port))
        except OSError as error:
            raise RuntimeError(f"Gazebo port {parsed.port} is occupied; choose master_uri:=http://127.0.0.1:PORT") from error

    variant = "arm" if not enabled("with_table") else ("locked" if enabled("table_locked") else "passive")
    description = (robot / "urdf" / f"robot_{variant}.ros2.urdf").read_text().replace("@ROBOT_SHARE@", str(robot))
    description = re.sub(r"package://([^/]+)/", lambda match:
                         Path(get_package_share_directory(match.group(1))).as_uri() + "/", description)
    description = ET.tostring(ET.fromstring(description), encoding="unicode")
    spawn = Node(package="gazebo_ros", executable="spawn_entity.py", name="spawn_robot", output="screen",
                 arguments=["-entity", "rml63ii_eg2_4c2", "-topic", "/arm/robot_description", "-timeout", "120",
                            "-x", LaunchConfiguration("robot_x"), "-y", LaunchConfiguration("robot_y"),
                            "-Y", LaunchConfiguration("robot_yaw")])
    spawner = Node(package="controller_manager", executable="spawner", namespace="arm", output="screen",
                   arguments=["joint_state_broadcaster", "arm_joint_controller", "gripper_controller",
                              "--controller-manager", "/arm/controller_manager", "--controller-manager-timeout", "120",
                              "--switch-timeout", "120"])

    def process_exited(event, launch_context):
        if launch_context.is_shutdown:
            return []
        executable = Path(event.cmd[0]).name
        if executable == "gzserver":
            return [EmitEvent(event=Shutdown(reason="Gazebo server exited"))]
        if executable == "gzclient":
            return [Node(package="rml63ii_eg2_4c2_gazebo", executable="gazebo_stop", output="screen")]
        return []

    return [
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        UnsetEnvironmentVariable("GAZEBO_HOSTNAME"),
        SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1"),
        SetEnvironmentVariable("GAZEBO_IP_WHITE_LIST", "127.0.0.1"),
        SetEnvironmentVariable("GAZEBO_MASTER_URI", master_uri),
        RegisterEventHandler(OnProcessExit(on_exit=process_exited)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory("gazebo_ros")) / "launch/gazebo.launch.py")),
            launch_arguments={"world": str(robot / "worlds/arm.world"), "gui": LaunchConfiguration("gui"),
                              "pause": LaunchConfiguration("paused"), "verbose": "true"}.items()),
        Node(package="robot_state_publisher", executable="robot_state_publisher", namespace="arm",
             parameters=[{"robot_description": ParameterValue(description, value_type=str),
                          "use_sim_time": True, "frame_prefix": "arm/"}],
             remappings=[("joint_states", "/arm/joint_states")]),
        Node(package="rml63ii_eg2_4c2_gazebo", executable="entity_tf.py",
             parameters=[{"use_sim_time": True, "child_frame": "arm/" +
                          ("table_base_link" if variant == "passive" else "world")}]),
        RegisterEventHandler(OnProcessExit(target_action=spawn,
            on_exit=lambda event, _: [spawner] if event.returncode == 0 and not context.is_shutdown else
                [LogInfo(msg="Robot spawn failed; controllers were not started."),
                 EmitEvent(event=Shutdown(reason="Robot spawn failed"))])),
        spawn,
        Node(package="rml63ii_eg2_4c2_gazebo", executable="arm_control.py", output="screen",
             parameters=[{"use_sim_time": True}], condition=IfCondition(LaunchConfiguration("control_gui"))),
    ]


def generate_launch_description():
    defaults = {"gui": "true", "control_gui": LaunchConfiguration("gui"), "paused": "false",
                "with_table": "true", "table_locked": "true", "robot_x": "0.0", "robot_y": "0.0",
                "robot_yaw": "0.0", "master_uri": os.environ.get("ARM_GAZEBO_MASTER_URI", "http://127.0.0.1:11355")}
    return LaunchDescription([*[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
                              OpaqueFunction(function=setup)])
