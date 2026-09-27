from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    nav2_dir = FindPackageShare('robot_gazebo')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')

    # --- Gazebo (new Gazebo / gz sim, Harmonic) World Setup (from robot_world.launch.py) ---
    pkg_description = get_package_share_directory('robot_description')
    pkg_gazebo = get_package_share_directory('robot_gazebo')
    xacro_file = os.path.join(pkg_description, 'urdf', 'robot.urdf')
    world_path = os.path.join(pkg_gazebo, 'worlds', 'turtlebot3_world.world')
    bridge_config = os.path.join(pkg_gazebo, 'config', 'gz_bridge.yaml')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Full path to the Gazebo (gz sim) world file to load'
    )

    # gz sim resolves model://<name> includes (e.g. turtlebot3_world) off this path,
    # the way GAZEBO_MODEL_PATH used to on Gazebo Classic.
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_gazebo, 'models')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items()
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'mecanum_bot',
                   '-x', '-2.0',
                   '-y', '0.0',
                   '-z', '0.0'],
        output='screen'
    )

    # Bridges cmd_vel/odom/tf/scan/imu/clock between Gazebo Transport and ROS 2 -
    # this replaces the per-plugin ROS remapping gazebo_ros used to do for us.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    # --- Nav2 / SLAM Setup ---
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([nav2_dir, 'config', 'nav2_params.yaml']),
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

    return LaunchDescription(
        [
            # Declarations MUST come first
            world_arg,
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            set_gz_resource_path,
            # Gazebo + Robot
            gazebo,
            node_robot_state_publisher,
            spawn_entity,
            gz_bridge,
            # Nav2 + SLAM
            slam_launch,
            nav2_bringup_launch,
            # Visualization
            rviz_launch_cmd,
        ]
    )