import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('robot_description')
    pkg_navigation = get_package_share_directory('robot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'))
    run_slam = LaunchConfiguration('run_slam', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # 1. Robot Description (URDF for RViz and TF publisher)
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # 2. SLAM Toolbox (Real-time Mapping from Isaac Sim /scan)
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

    # 3. Nav2 Bringup (Navigation Stack)
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

    # 4. RViz2 Visualization
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
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Isaac Sim simulation clock (/clock)'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'), description='Nav2 params'),
        DeclareLaunchArgument('run_slam', default_value='true', description='Run SLAM mapping along with Nav2'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup Nav2 stack'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2 GUI'),

        node_robot_state_publisher,
        slam_launch,
        nav2_bringup_launch,
        rviz_node
    ])
