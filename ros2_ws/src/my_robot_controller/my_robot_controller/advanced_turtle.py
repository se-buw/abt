import py_trees
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1)

    def create_publisher(self, msg_type, topic, qos_profile):
        return self.node.create_publisher(msg_type, topic, qos_profile)

    def create_timer(self, seconds):
        return self.node.create_timer(seconds, self.update)

    def setup(self, **kwargs):
        pass

    def initialise(self):
        pass

    def update(self):
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        pass

class Zigzag(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(2)  # Adjust the timer interval for zigzag motion
        self.turn_left = True  # Flag to alternate between left and right turns

    def create_publisher(self, msg_type, topic, qos_profile):
        return self.node.create_publisher(msg_type, topic, qos_profile)

    def create_timer(self, seconds):
        return self.node.create_timer(seconds, self.update)

    def setup(self, **kwargs):
        pass

    def initialise(self):
        pass

    def update(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # Reduced linear velocity for a zigzag pattern
        if self.turn_left:
            twist_msg.angular.z = 1.0  # Angular velocity for left turn
        else:
            twist_msg.angular.z = -1.0  # Angular velocity for right turn

        self.publisher.publish(twist_msg)
        self.turn_left = not self.turn_left  # Toggle the turn direction

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        pass

def main():
    rclpy.init()

    node = Node("complex_turtle_controller")
    root = py_trees.composites.Sequence("Sequence", memory=None)
    move_forward = MoveForward("MoveForward", node)
    zigzag = Zigzag("Zigzag", node)

    root.add_child(move_forward)
    root.add_child(zigzag)

    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=30)  # Increase the timeout for the zigzag motion

    rclpy.spin(node)

    tree.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
