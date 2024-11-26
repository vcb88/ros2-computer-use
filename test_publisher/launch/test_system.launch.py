import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the path to the web-client directory
    package_share_dir = FindPackageShare('test_publisher')
    web_client_path = PathJoinSubstitution([package_share_dir, 'web-client'])

    # Set up Python 3.10 environment variables
    ros2_py310_env = '/home/computeruse/ros2_py310_env/bin/python3'
    
    return LaunchDescription([
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
                ('PYTHONPATH', '/home/computeruse/ros2_py310_env/lib/python3.10/site-packages')
            ]
        ),

        # Launch React development server
        ExecuteProcess(
            cmd=['npm', 'start'],
            cwd='/home/computeruse/ros2_test_ws/src/test_publisher/web-client',
            output='screen',
            shell=True,
            env={'PORT': '3000'}
        )
    ])