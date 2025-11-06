import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_simple_robot = get_package_share_directory('simple_gazebo_robot')

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -v4 ' + os.path.join(pkg_simple_robot, 'worlds', 'empty.world')
        }.items()
    )

    # Robot description publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': os.popen('xacro ' + os.path.join(pkg_simple_robot, 'urdf', 'robot.urdf.xacro')).read(),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Spawn robot
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot'],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        gazebo,
        robot_state_publisher_node,
        spawn_robot_node
    ])
