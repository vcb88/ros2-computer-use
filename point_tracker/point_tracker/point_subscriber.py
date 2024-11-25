#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PointSubscriber(Node):
    def __init__(self):
        super().__init__('point_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'point_coords',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Point Subscriber has been started')

    def listener_callback(self, msg):
        x, y = msg.data
        self.get_logger().info(f'Received coordinates: x={x:.2f}, y={y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    point_subscriber = PointSubscriber()
    try:
        rclpy.spin(point_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        point_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()