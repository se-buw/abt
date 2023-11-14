import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def move_forward(node, publisher):
    twist = Twist()
    twist.linear.x = 1.0  # Move forward

    # Publish the Twist message for 5 seconds
    start_time = node.get_clock().now()
    while (node.get_clock().now() - start_time).to_msg().sec < 5.0:
        publisher.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.1)

def main():
    rclpy.init()
    node = Node("simple_turtle_controller")
    publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    try:
        move_forward(node, publisher)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
