'''
def main():
    print('Hi from turtle_automate.')


if __name__ == '__main__':
    main()
'''   
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleAutomate(Node):
    def __init__(self):
        super().__init__('turtle_automate')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(1.0, self.move_turtle)  
        self.msg = Twist()

    def move_turtle(self):
        self.msg.linear.x = 2.0  # Linear velocity
        self.msg.angular.z = 1.0  # Angular velocity
        self.publisher.publish(self.msg)

    def pose_callback(self, msg):
        self.get_logger().info(f'Turtle Position - X: {msg.x}, Y: {msg.y}, Theta: {msg.theta}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleAutomate()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

