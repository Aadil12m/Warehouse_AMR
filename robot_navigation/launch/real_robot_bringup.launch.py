import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('robot_description')
    pkg_navigation = get_package_share_directory('robot_navigation')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rplidar_port = LaunchConfiguration('rplidar_port', default='/dev/rplidar')
    esp32_port = LaunchConfiguration('esp32_port', default='/dev/esp32_motors')
    baud_rate = LaunchConfiguration('baud_rate', default='115200')
    start_microros = LaunchConfiguration('start_microros', default='true')
    start_lidar = LaunchConfiguration('start_lidar', default='true')
    start_ekf = LaunchConfiguration('start_ekf', default='true')

    # Robot Description (URDF)
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # 2. RPLiDAR A1 Driver Node (sllidar_ros2 / rplidar_ros)
    node_rplidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        condition=IfCondition(start_lidar),
        parameters=[{
            'channel_type': 'serial',
            'serial_port': rplidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    # 3. micro-ROS Agent Node (for ESP32 Motor Controller & Sensor bridge)
    node_microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        condition=IfCondition(start_microros),
        arguments=['serial', '--dev', esp32_port, '-b', baud_rate]
    )

    # 4. Robot Localization (EKF Node for fusing wheel odometry + IMU)
    ekf_config_path = os.path.join(pkg_navigation, 'config', 'ekf.yaml')
    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(start_ekf),
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odom')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock'),
        DeclareLaunchArgument('rplidar_port', default_value='/dev/rplidar', description='RPLiDAR USB port'),
        DeclareLaunchArgument('esp32_port', default_value='/dev/esp32_motors', description='ESP32 USB port'),
        DeclareLaunchArgument('baud_rate', default_value='115200', description='micro-ROS serial baud rate'),
        DeclareLaunchArgument('start_microros', default_value='true', description='Start micro-ROS agent'),
        DeclareLaunchArgument('start_lidar', default_value='true', description='Start RPLiDAR driver'),
        DeclareLaunchArgument('start_ekf', default_value='true', description='Start EKF sensor fusion'),
        
        node_robot_state_publisher,
        node_rplidar,
        node_microros_agent,
        node_ekf
    ])
