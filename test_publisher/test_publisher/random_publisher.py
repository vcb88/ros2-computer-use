def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    import random
    import json
    from datetime import datetime

    rclpy.init(args=args)
    node = Node('random_publisher')
    publisher = node.create_publisher(String, 'random_data', 10)

    def timer_callback():
        # Create a data structure with random number and timestamp
        data = {
            'random_number': random.randint(0, 1000),
            'timestamp': datetime.now().isoformat(),
            'message': f'Sample message #{random.randint(1, 100)}'
        }
        
        # Convert to JSON string
        json_str = json.dumps(data)
        
        # Create and publish ROS message
        msg = String()
        msg.data = json_str
        publisher.publish(msg)
        
        # Log the message
        node.get_logger().info(f'Publishing: {json_str}')

    timer = node.create_timer(1.0, timer_callback)  # 1 second timer
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()