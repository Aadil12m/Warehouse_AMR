from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    tutorial_dir = FindPackageShare('robot_gazebo')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    explore_lite_launch = PathJoinSubstitution(
        [FindPackageShare('explore_lite'), 'launch', 'explore.launch.py']
    )
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([tutorial_dir, 'config', 'nav2_params.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True',
        }.items(),
    )

    explore_lite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_lite_launch]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rviz_launch_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments={
            '-d'+ os.path.join(pkg_nav2_dir,
            'rviz',
            'nav2_default_view.rviz')
        },
        parameters=[{'use_sim_time': True}],
    )

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('robot_gazebo'),'config','nav2_params.yaml')] # Use the exact same YAML file!
    )

    return LaunchDescription(
        [
            collision_monitor_node,
            rviz_launch_cmd,
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            slam_launch,
            nav2_bringup_launch,
            explore_lite_launch,
        ]
    )