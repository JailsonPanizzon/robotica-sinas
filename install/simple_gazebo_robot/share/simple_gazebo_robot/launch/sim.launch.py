#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_amiga_description = get_package_share_directory('simple_gazebo_robot')
    
    # Paths
    world_path = PathJoinSubstitution([
        FindPackageShare('simple_gazebo_robot'),
        'worlds', 'farm_rows_world.sdf'
    ])
    
    urdf_file = os.path.join(pkg_amiga_description, 'urdf', 'amiga_descr.urdf.xacro')
    
    # Launch configuration variables
    world_file = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    robot_name = LaunchConfiguration('robot_name')
    
    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='amiga',
        description='Name of the robot'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator'
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Full path to the world model file to load'
    )

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        condition=IfCondition(use_simulator),
        launch_arguments={'gz_args': ['-r -v4 ', world_file]}.items()
    )
    
    # Robot description with OAK-D cameras
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Spawn robot in Gazebo - positioned between crop rows for navigation
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', robot_name,
                   '-x', '-3.0',   # Between crop rows 2 and 3
                   '-y', '-15.0',  # Start position for navigation
                   '-z', '0.5',    # Ground level
                   '-Y', '1.57',   # 90 graus em radianos (π/2 ≈ 1.57)
                   '-allow_renaming', 'true'],
        output='screen'
    )

    # Bridge for robot topics with OAK-D cameras
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Robot control and odometry
            '/model/amiga/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/amiga/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            
            # OAK-D front camera (oak0)
            '/world/farm_rows_world/model/amiga/link/base_link/sensor/oak0_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/farm_rows_world/model/amiga/link/base_link/sensor/oak0_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/farm_rows_world/model/amiga/link/base_link/sensor/oak0_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            
            # OAK-D back camera (oak1)
            '/world/farm_rows_world/model/amiga/link/base_link/sensor/oak1_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/farm_rows_world/model/amiga/link/base_link/sensor/oak1_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/farm_rows_world/model/amiga/link/base_link/sensor/oak1_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            
            # Clock synchronization
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ],
        remappings=[
            # Remapeia o tópico da câmera frontal para /oak0/rgb
            ('/world/farm_rows_world/model/amiga/link/base_link/sensor/oak0_camera/image', '/oak0/rgb'),
            ('/world/farm_rows_world/model/amiga/link/base_link/sensor/oak0_camera/camera_info', '/oak0/camera_info'),
            ('/world/farm_rows_world/model/amiga/link/base_link/sensor/oak0_camera/depth_image', '/oak0/depth'),
            
            # Opcional: também pode remapear a câmera traseira se desejar
            ('/world/farm_rows_world/model/amiga/link/base_link/sensor/oak1_camera/image', '/oak1/rgb'),
            ('/world/farm_rows_world/model/amiga/link/base_link/sensor/oak1_camera/camera_info', '/oak1/camera_info'),
            ('/world/farm_rows_world/model/amiga/link/base_link/sensor/oak1_camera/depth_image', '/oak1/depth'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_robot_name_cmd,
        declare_use_sim_time_cmd,
        declare_use_simulator_cmd,
        declare_headless_cmd,
        declare_world_cmd,
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_robot_node,
        ros_gz_bridge_node
    ])