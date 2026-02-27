from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'velocity_smoother.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            parameters=[config],
            remappings=[
                ('/cmd_vel', '/cmd_vel_smoothed')
            ],
            output='screen'
        )
    ])