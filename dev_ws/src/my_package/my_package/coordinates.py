#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class CoordinatesSubscriber(Node):

    def __init__(self):
        super().__init__('coordinates_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Turtle Position - x: %f, y: %f' % (msg.x, msg.y))


def main(args=None):
    rclpy.init(args=args)
    coordinates_subscriber = CoordinatesSubscriber()
    rclpy.spin(coordinates_subscriber)
    coordinates_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
