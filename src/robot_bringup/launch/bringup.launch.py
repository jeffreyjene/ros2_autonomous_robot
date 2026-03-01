from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    bringup_dir = get_package_share_directory('robot_bringup')

    teleop_config = os.path.join(bringup_dir, 'config', 'teleop_joy.yaml')
    twist_mux_config = os.path.join(bringup_dir, 'config', 'twist_mux.yaml')
    slam_config = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    nav2_config = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([

        # ========================
        # Launch arguments
        # ========================
        DeclareLaunchArgument(
            'use_slam',
            default_value='false',
            description='Run SLAM Toolbox'
        ),

        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Run Nav2'
        ),

        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Map YAML file for Nav2'
        ),

        # ========================
        # Joystick driver
        # ========================
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # ========================
        # Teleop (TwistStamped)
        # ========================
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[teleop_config],
            remappings=[
                ('/cmd_vel', '/cmd_vel_joy'),
            ],
        ),

        # ========================
        # Twist mux (arbitration)
        # ========================
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_config],
        ),

        # ========================
        # Convert TwistStamped → Twist
        # ========================
        Node(
            package='robot_control',
            executable='cmd_vel_adapter',
            name='cmd_vel_adapter',
            output='screen',
        ),

        # ========================
        # Arduino bridge (motors + odom)
        # ========================
        Node(
            package='robot_control',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='screen',
        ),

        # ========================
        # RPLidar (Jazzy-correct)
        # ========================
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_link',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),

        # ========================
        # Needed for SLAM
        # ========================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf',
            arguments=[
                '0.0', '0.0', '0.1',   # x y z (adjust z for your LiDAR height)
                '0.0', '0.0', '0.0',   # roll pitch yaw
                'base_link',
                'laser_link',
            ],
        ),

        # ========================
        # SLAM Toolbox (mapping)
        # ========================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'online_sync_launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': slam_config,
            }.items(),
            condition=IfCondition(use_slam),
        ),

        # ========================
        # Nav2 (localization + navigation)
        # ========================
        
    ])