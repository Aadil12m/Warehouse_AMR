import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_navigation = get_package_share_directory('robot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'))
    run_slam = LaunchConfiguration('run_slam', default='true')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_navigation, 'maps', 'map1.yaml'))
    autostart = LaunchConfiguration('autostart', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # ── Mapping mode (run_slam:=true): SLAM Toolbox builds a new map ──────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        condition=IfCondition(run_slam),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # ── Localization mode (run_slam:=false): AMCL on a saved map ──────────────
    # Use this AFTER you have mapped the warehouse once (save with map_saver_cli
    # and point 'map' at the new yaml). SLAM keeps re-writing the map forever;
    # production runs want repeatable AMCL localization instead.
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        condition=UnlessCondition(run_slam),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_yaml_file,
            'autostart': autostart,
        }.items()
    )

    # Nav2 Bringup (Navigation Stack)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )

    # RViz2 (Default: false on RPi; run on remote PC instead)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'), description='Nav2 params'),
        DeclareLaunchArgument('run_slam', default_value='true', description='true = build map with SLAM, false = localize with AMCL on saved map'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_navigation, 'maps', 'map1.yaml'), description='Full path to map file for AMCL localization'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup Nav2 stack'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Launch RViz2 GUI'),

        slam_launch,
        localization_launch,
        nav2_bringup_launch,
        rviz_node
    ])
