import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Specify the name of the package
    package_name = 'robot_description'

    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    world_path = os.path.join(get_package_share_directory('robot_gazebo'),'worlds','turtlebot3_world.world')
    
    # 1. Safely process the URDF/Xacro file using the ROS 2 Command substitution
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Full path to Gazebo world file to load'
    )

    # 2. Include the standard Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # 3. Start the Robot State Publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

    # 4. Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'mecanum_bot',
                   '-x', '-2.0',
                   '-y', '0.0',
                   '-z', '0.0'],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])