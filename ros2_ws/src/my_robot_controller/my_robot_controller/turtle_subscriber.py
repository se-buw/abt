import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtleSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_subscriber')
        #subscription to listen to Pose messages & get turtle coordinates
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
    #logging turtle's position co-ordinates
        self.get_logger().info('Turtle Position - x: %f, y: %f' % (msg.x, msg.y))

def main(args=None):
    rclpy.init(args=args)
    turtle_subscriber = TurtleSubscriber()
    rclpy.spin(turtle_subscriber) #Event loop to run the subscriber node 
    turtle_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
