import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare launch arguments
    ros2_py310_env_arg = DeclareLaunchArgument(
        'ros2_py310_env',
        default_value=os.path.expanduser('~/ros2_py310_env/bin/python3'),
        description='Path to Python 3.10 interpreter with ROS2 environment'
    )

    web_client_path_arg = DeclareLaunchArgument(
        'web_client_path',
        default_value=os.path.expanduser('~/ros2_ws/src/ros2-computer-use/test_publisher/web-client'),
        description='Path to web client directory'
    )

    # Get launch configurations
    ros2_py310_env = LaunchConfiguration('ros2_py310_env')
    web_client_path = LaunchConfiguration('web_client_path')

    return LaunchDescription([
        # Add the launch arguments
        ros2_py310_env_arg,
        web_client_path_arg,

        # Launch our random publisher node
        Node(
            package='test_publisher',
            executable='random_publisher',
            name='random_publisher',
            output='screen'
        ),
        
        # Launch ros2-web-bridge with Python 3.10
        ExecuteProcess(
            cmd=[ros2_py310_env, '-m', 'rosbridge_server.launch_rosbridge', '--port', '9090'],
            output='screen',
            shell=False,
            environment=[
                ('PYTHONPATH', os.path.join(os.path.dirname(LaunchConfiguration('ros2_py310_env').perform()), 
                                          '../lib/python3.10/site-packages'))
            ]
        ),

        # Launch Vite development server
        ExecuteProcess(
            cmd=['npm', 'run', 'dev'],
            cwd=web_client_path,
            output='screen',
            shell=True,
            env={'PORT': '3000'}
        )
    ])