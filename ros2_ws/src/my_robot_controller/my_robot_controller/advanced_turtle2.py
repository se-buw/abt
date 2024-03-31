import py_trees
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveForward(py_trees.behaviour.Behaviour): #class for turtle moving forward
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

class Zigzag(py_trees.behaviour.Behaviour): #class for zigzag movement
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(2)
        self.turn_left = True

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
        twist_msg.linear.x = 0.5
        if self.turn_left:
            twist_msg.angular.z = 1.0
        else:
            twist_msg.angular.z = -1.0

        self.publisher.publish(twist_msg)
        self.turn_left = not self.turn_left

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        pass

class RotateClockwise(py_trees.behaviour.Behaviour): #class for clockwise rotation of the turtlesim node
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(3)  # timer for clockwise rotation

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
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = -0.5  # Angular velocity for clockwise rotation
        self.publisher.publish(twist_msg)

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        pass

class RotateCounterClockwise(py_trees.behaviour.Behaviour): #class for counterclockwise rotation of the turtlesim node
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(3)  #  timer for counterclockwise rotation

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
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.5  # Angular velocity for counterclockwise rotation
        self.publisher.publish(twist_msg)

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        pass

def main():
    rclpy.init()

    node = Node("complex_turtle_controller")
    root = py_trees.composites.Sequence("Sequence", memory=None) #Root node
    move_forward = MoveForward("MoveForward", node)
    zigzag = Zigzag("Zigzag", node)
    rotate_clockwise = RotateClockwise("RotateClockwise", node)
    rotate_counterclockwise = RotateCounterClockwise("RotateCounterClockwise", node)

#adding child nodes
    root.add_child(move_forward) 
    root.add_child(zigzag)
    root.add_child(rotate_clockwise)
    root.add_child(rotate_counterclockwise)

    tree = py_trees.trees.BehaviourTree(root)
    
    #The BT structure is as below:
	#Sequence (Root: Sequence node)
	#|
	#|-- MoveForward (Action node)
	#|
	#|-- Zigzag (Action node)
	#|
	#|-- RotateClockwise (Action node)
	#|
	#|-- RotateCounterClockwise (Action node)

	
    tree.setup(timeout=45)  # Increasing timeout for all behaviors

    rclpy.spin(node)

    tree.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
