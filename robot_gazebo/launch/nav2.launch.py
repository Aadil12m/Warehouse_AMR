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
    pkg_robot = get_package_share_directory('robot_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True') 

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir,'launch','bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time' : use_sim_time,
            'autostart' : autostart,
            'map' : os.path.join(pkg_robot,'maps','map1.yaml')
        }.items()
    )

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

    set_init_amcl_pose = Node(
        package='robot_gazebo',
        executable='amcl_init_pose_publisher',
        name='amcl_init_pose_publisher',
        parameters=[{
            "x":-2.0,
            "y":-0.5,
        }]
    )

    return LaunchDescription([set_init_amcl_pose,
                             nav2_launch_cmd,
                             rviz_launch_cmd])
