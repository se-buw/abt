import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtlePublisher(Node):
    def __init__(self):
        super().__init__('turtle_publisher') #publisher to publish Twist messages to control the turtle's movement
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        
        self.timer = self.create_timer(0.5, self.timer_callback) # timer to trigger the movement at regular intervals
        self.angle = 0

    def timer_callback(self):  # Twist message to control linear and angular velocities
        msg = Twist()
        msg.linear.x = 2.0 #linear velocity
        msg.angular.z = 1.0 #angular velocity
        self.publisher_.publish(msg) #publishing Twist message to control the turtle's movement
        self.angle += 1
        if self.angle > 360:
            self.get_logger().info('Finished drawing the circle.') #logging a message when the turtle completes a circle
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    turtle_publisher = TurtlePublisher() 
    rclpy.spin(turtle_publisher) #Starting event loop to run the publisher node 
    turtle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
