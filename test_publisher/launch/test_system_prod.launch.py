import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    ros2_py310_env_arg = DeclareLaunchArgument(
        'ros2_py310_env',
        default_value=os.path.expanduser('~/ros2_py310_env/bin/python3'),
        description='Path to Python 3.10 interpreter with ROS2 environment'
    )

    nginx_conf_dir_arg = DeclareLaunchArgument(
        'nginx_conf_dir',
        default_value='/etc/nginx',
        description='Nginx configuration directory'
    )

    systemd_service_dir_arg = DeclareLaunchArgument(
        'systemd_service_dir',
        default_value='/etc/systemd/system',
        description='Systemd service directory'
    )

    # Get launch configurations
    ros2_py310_env = LaunchConfiguration('ros2_py310_env')
    nginx_conf_dir = LaunchConfiguration('nginx_conf_dir')
    systemd_service_dir = LaunchConfiguration('systemd_service_dir')

    return LaunchDescription([
        # Add the launch arguments
        ros2_py310_env_arg,
        nginx_conf_dir_arg,
        systemd_service_dir_arg,

        # Launch our random publisher node as a systemd service
        ExecuteProcess(
            cmd=['systemctl', '--user', 'start', 'ros2-random-publisher'],
            output='screen',
            shell=True
        ),
        
        # Launch ros2-web-bridge as a systemd service
        ExecuteProcess(
            cmd=['systemctl', '--user', 'start', 'ros2-web-bridge'],
            output='screen',
            shell=True
        ),

        # Ensure nginx is running with our configuration
        ExecuteProcess(
            cmd=['systemctl', 'start', 'nginx'],
            output='screen',
            shell=True
        )
    ])