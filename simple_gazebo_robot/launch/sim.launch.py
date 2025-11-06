from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '/ros2_ws/src/simple_gazebo_robot/worlds/empty.world'],
            output='screen'
        ),
    ])
