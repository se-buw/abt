#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CirclePublisher(Node):

    def __init__(self):
        super().__init__('circle_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.angle = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)
        self.angle += 1
        if self.angle > 360:
            self.get_logger().info('Finished drawing the circle.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    circle_publisher = CirclePublisher()
    rclpy.spin(circle_publisher)
    circle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
