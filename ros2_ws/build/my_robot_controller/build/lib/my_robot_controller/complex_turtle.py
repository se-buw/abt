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

def main():
    rclpy.init()

    node = Node("simple_turtle_controller")
    root = py_trees.composites.Sequence("Sequence", memory=None)
    move_forward = MoveForward("MoveForward", node)

    root.add_child(move_forward)

    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15)

    rclpy.spin(node)

    tree.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
