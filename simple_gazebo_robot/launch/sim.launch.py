import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():

    pkg_simple_robot = get_package_share_directory('simple_gazebo_robot')
    
    # 1️⃣ Iniciar o Gazebo com o mundo empty.world
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', os.path.join(pkg_simple_robot, 'worlds', 'empty.world')],
        output='screen'
    )

    # 2️⃣ Spawna o robô usando o SDF ou URDF
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(pkg_simple_robot, 'models', 'model.sdf'),
            '-name', 'simple_robot',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # 3️⃣ Broadcaster do estado das juntas
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/simple_robot/controller_manager"],
    )

    # 4️⃣ Controlador de tração diferencial
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/simple_robot/controller_manager"],
    )

    # 5️⃣ Garantir a ordem: spawna → broadcaster → controlador
    load_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    load_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    return LaunchDescription([
        gz_sim,
        spawn_entity,
        load_broadcaster,
        load_controller
    ])
