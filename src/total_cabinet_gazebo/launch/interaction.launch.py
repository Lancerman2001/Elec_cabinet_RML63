"""One Gazebo world with the complete cabinet and the RML63II mobile assembly."""
from pathlib import Path
import os
import re
import socket
from urllib.parse import urlparse
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, LogInfo, EmitEvent, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def resolve_resources(description):
    # ROS packages are resource directories, not insertable Gazebo models.
    # Resolve mesh URIs at launch instead of adding install/share to model paths.
    return re.sub(r'package://([^/]+)/',
                  lambda match: Path(get_package_share_directory(match.group(1))).as_uri()+'/', description)


def setup(context):
    cabinet = Path(get_package_share_directory("total_cabinet_gazebo"))
    robot = Path(get_package_share_directory("rml63ii_eg2_4c2_gazebo"))
    enabled = lambda name: LaunchConfiguration(name).perform(context).lower() == "true"
    master_uri = LaunchConfiguration("master_uri").perform(context)
    parsed = urlparse(master_uri)
    if parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost") or not parsed.port:
        raise ValueError("master_uri must be an http://127.0.0.1:PORT local address")
    # Apply the same local-only policy when users bypass run_gazebo.sh and
    # invoke ros2 launch directly. Gazebo's whitelist filters accepted peers;
    # its Classic TCP sockets may still show wildcard listening addresses.
    actions = [SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
               UnsetEnvironmentVariable("GAZEBO_HOSTNAME"),
               SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1"),
               SetEnvironmentVariable("GAZEBO_IP_WHITE_LIST", "127.0.0.1"),
               SetEnvironmentVariable("GAZEBO_MASTER_URI", master_uri)]
    if enabled("start_server"):
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("127.0.0.1", parsed.port))
            except OSError as error:
                raise RuntimeError(f"Gazebo port {parsed.port} is occupied; choose master_uri:=http://127.0.0.1:PORT") from error
        def process_exited(event, launch_context):
            if launch_context.is_shutdown:
                return []
            executable = Path(event.cmd[0]).name
            if executable == "gzserver":
                return [EmitEvent(event=Shutdown(reason="Gazebo server exited"))]
            if executable == "gzclient":
                return [Node(package="rml63ii_eg2_4c2_gazebo", executable="gazebo_stop", output="screen")]
            return []
        actions.append(RegisterEventHandler(OnProcessExit(on_exit=process_exited)))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory("gazebo_ros")) / "launch/gazebo.launch.py")),
            launch_arguments={"world": str(cabinet / "worlds/cabinet.world"),
                              "gui": LaunchConfiguration("gui"), "pause": LaunchConfiguration("paused"),
                              "verbose": "true"}.items()))
    if enabled("with_cabinet"):
        model = ET.parse(cabinet / "urdf/total_cabinet_glass.ros2.urdf").getroot()
        pose = LaunchConfiguration("initial_pose").perform(context)
        if pose not in ("open", "closed"):
            raise ValueError("initial_pose must be open or closed")
        model.find(".//plugin[@filename='libelectrical_cabinet_kinematic_plugin.so']/initial_pose").text = pose
        description = resolve_resources(ET.tostring(model, encoding="unicode"))
        actions += [
            Node(package="robot_state_publisher", executable="robot_state_publisher", namespace="total_cabinet",
                 parameters=[{"robot_description": ParameterValue(description, value_type=str), "use_sim_time": True}]),
            Node(package="tf2_ros", executable="static_transform_publisher",
                 arguments=["--frame-id", "world", "--child-frame-id", "cabinet_base_link"]),
            Node(package="gazebo_ros", executable="spawn_entity.py", name="spawn_cabinet",
                 arguments=["-entity", "total_cabinet", "-topic", "/total_cabinet/robot_description", "-timeout", "120"], output="screen"),
        ]
    if enabled("with_robot"):
        variant = "arm" if not enabled("with_table") else ("locked" if enabled("table_locked") else "passive")
        description = (robot / "urdf" / ("robot_" + variant + ".ros2.urdf")).read_text().replace("@ROBOT_SHARE@", str(robot))
        # Humble's spawn_entity uses lxml on a Unicode string. Strip the byte
        # encoding declaration by serializing the parsed tree as Unicode.
        description = resolve_resources(ET.tostring(ET.fromstring(description), encoding="unicode"))
        spawn = Node(package="gazebo_ros", executable="spawn_entity.py", name="spawn_robot",
                     arguments=["-entity", "rml63ii_eg2_4c2", "-topic", "/arm/robot_description", "-timeout", "120",
                                "-x", LaunchConfiguration("robot_x"), "-y", LaunchConfiguration("robot_y"),
                                "-Y", LaunchConfiguration("robot_yaw")], output="screen")
        spawner = Node(package="controller_manager", executable="spawner", namespace="arm", output="screen",
                       arguments=["joint_state_broadcaster", "arm_joint_controller", "gripper_controller",
                                  "--controller-manager", "/arm/controller_manager", "--controller-manager-timeout", "120",
                                  "--switch-timeout", "120"])
        actions += [
            Node(package="robot_state_publisher", executable="robot_state_publisher", namespace="arm",
                 parameters=[{"robot_description": ParameterValue(description, value_type=str),
                              "use_sim_time": True, "frame_prefix": "arm/"}],
                 remappings=[("joint_states", "/arm/joint_states")]),
            Node(package="rml63ii_eg2_4c2_gazebo", executable="entity_tf.py",
                 parameters=[{"use_sim_time": True, "child_frame": "arm/" +
                              ("table_base_link" if variant == "passive" else "world")}]),
            RegisterEventHandler(OnProcessExit(target_action=spawn,
                on_exit=lambda event, _: [spawner] if event.returncode == 0 and not context.is_shutdown else
                    [LogInfo(msg="ERROR: Robot spawn failed; controllers were not started.")])), spawn,
        ]
    if enabled("control_gui") and (enabled("with_robot") or enabled("with_cabinet")):
        actions.append(Node(package="total_cabinet_gazebo", executable="unified_control.py", output="screen",
                            parameters=[{"use_sim_time": True, "with_robot": enabled("with_robot"),
                                         "with_cabinet": enabled("with_cabinet")}]))
    return actions


def generate_launch_description():
    defaults = {"gui": "true", "control_gui": LaunchConfiguration("gui"), "paused": "false",
                "initial_pose": "open", "with_robot": "true", "with_cabinet": "true",
                "start_server": "true", "with_table": "true", "table_locked": "true",
                "robot_x": "1.40", "robot_y": "-0.35", "robot_yaw": "3.141592653589793",
                "master_uri": os.environ.get("ARM_GAZEBO_MASTER_URI", "http://127.0.0.1:11355")}
    return LaunchDescription([*[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
                              OpaqueFunction(function=setup)])
