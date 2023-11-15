import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class MoveTurtle(Node): #Using the node from rcply library to inherit the properties

    def __init__(self):
        super().__init__('turtle_mover')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.listener_callback,
            10)
        self.subscription
        self.timer = self.create_timer(0.5, self.timer_callback) #0.5 interval to trigger timer_callback method
        self.angle = 0

    def timer_callback(self): #To control the turtles movements
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)
        self.angle += 1
        if self.angle > 360: #completing the circle
            self.get_logger().info('Finished drawing the circle.')
            rclpy.shutdown()

    def listener_callback(self, msg): # getting the coordinates
        self.get_logger().info('Turtle Position - x: %f, y: %f' % (msg.x, msg.y))


def main(args=None): #main function 
    rclpy.init(args=args)
    turtle_mover = MoveTurtle()
    rclpy.spin(turtle_mover)
    turtle_mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

