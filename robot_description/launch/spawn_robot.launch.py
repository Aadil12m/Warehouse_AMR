import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Specify the name of the package
    package_name = 'robot_description'

    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    # 1. Safely process the URDF/Xacro file using the ROS 2 Command substitution
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # gz sim resolves model://<name> includes off this path (replaces GAZEBO_MODEL_PATH)
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('robot_gazebo'), 'models')
    )

    # 2. Include the new Gazebo (gz sim / Harmonic) launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r empty.sdf']}.items()
    )

    # 3. Start the Robot State Publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 4. Spawn the robot in Gazebo (gz sim)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'mecanum_bot',
                   '-z', '0.1'],
        output='screen'
    )

    # 5. Bridge cmd_vel/odom/tf/scan/imu/clock between Gazebo Transport and ROS 2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(
            get_package_share_directory('robot_gazebo'), 'config', 'gz_bridge.yaml')}],
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        gz_bridge,
    ])
