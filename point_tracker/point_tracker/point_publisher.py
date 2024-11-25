#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random
import numpy as np

class PointPublisher(Node):
    def __init__(self):
        super().__init__('point_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'point_coords', 10)
        self.timer = self.create_timer(1.0, self.publish_point)
        self.get_logger().info('Point Publisher has been started')

    def publish_point(self):
        msg = Float32MultiArray()
        # Generate random x, y coordinates in range [-10, 10]
        x = random.uniform(-10.0, 10.0)
        y = random.uniform(-10.0, 10.0)
        msg.data = [float(x), float(y)]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing coordinates: x={x:.2f}, y={y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    point_publisher = PointPublisher()
    try:
        rclpy.spin(point_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        point_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
