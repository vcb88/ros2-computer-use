from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Point Publisher node
        Node(
            package='point_tracker',
            executable='point_publisher',
            name='point_publisher',
            output='screen'
        ),

        # Point Subscriber node
        Node(
            package='point_tracker',
            executable='point_subscriber',
            name='point_subscriber',
            output='screen'
        ),

        # rosbridge_server для веб-интерфейса
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0'
            }]
        ),

        # Web server для веб-приложения
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8081'],
            cwd='/home/computeruse/ros2_ws/src/point_tracker/webapp',
            output='screen'
        )
    ])