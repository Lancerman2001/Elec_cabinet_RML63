from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(
        get_package_share_directory('total_cabinet_gazebo') + '/launch/interaction.launch.py'),
        launch_arguments={'with_robot': 'false'}.items())])
