#Before running the code, run "ros2 run turtlesim turtlesim_node" in command prompt
import py_trees
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class Behavior1(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Behavior1, self).__init__(name)
        self.cmd_vel_pub_ = None

    def setup(self, draw_circle_node):
        self.cmd_vel_pub_ = draw_circle_node.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def update(self):
        if self.cmd_vel_pub_ is None:
            raise RuntimeError("Behavior1 not set up correctly. Call setup() before updating.")

        print("Executing leaf 1  'Drawing Circle' ")

        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)
        return py_trees.common.Status.SUCCESS

class Behavior2(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Behavior2, self).__init__(name)
        self.pose_subscriber_node = Node("Pose_Subscriber")
        self.pose_subscriber_ = self.pose_subscriber_node.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

    def pose_callback(self, msg: Pose):
        self.get_logger().info("(" + str(msg.x) + "," + str(msg.y) + ")")

    def update(self):
        print("Executing leaf 2")
        return py_trees.common.Status.SUCCESS

class Behavior3(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Behavior3, self).__init__(name)

    def update(self):
        print("Executing leaf 3")
        return py_trees.common.Status.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('behavior_tree_node')

    # Set up the node for drawing circles
    draw_circle_node = Node("draw_circle")
    cmd_vel_pub = draw_circle_node.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    # Create a timer to periodically send velocity commands for drawing circles
    draw_circle_node.timer = draw_circle_node.create_timer(0.5, lambda: leaf1.send_velocity_command())

    # Define the behavior tree structure
    #root = py_trees.composites.Parallel("Root", policy=py_trees.common.ParallelPolicy.SuccessOnOne())
    root = py_trees.composites.Sequence("Root", memory=py_trees.blackboard.Blackboard())
    leaf1 = Behavior1("Behavior1")
    leaf2 = Behavior2("Behavior2")
    leaf3 = Behavior3("Behavior3")

    # Add behaviors to the root
    root.add_children([leaf1, leaf2, leaf3])

    # Create the behavior tree
    tree = py_trees.trees.BehaviourTree(root)

    try:
        # Set up Behavior1 with the draw_circle_node
        leaf1.setup(draw_circle_node)

        # Run the behavior tree while ROS2 is active
        while rclpy.ok():
            tree.tick()
            rclpy.spin_once(node)

    finally:
        # Clean up and shutdown when exiting
        tree.interrupt()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


#References:
# https://py-trees.readthedocs.io/en/devel/introduction.html