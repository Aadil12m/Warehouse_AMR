import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  


def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    rviz_launch_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments={
            '-d'+ os.path.join(pkg_nav2_dir,
            'rviz',
            'nav2_default_view.rviz')
        }
    )

    return LaunchDescription([
                             rviz_launch_cmd])
