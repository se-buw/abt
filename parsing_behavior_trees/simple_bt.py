import rclpy
from rclpy.node import Node
import py_trees

class MyTree(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)
        self.idle = py_trees.behaviours.Success(name="Idle")
        self.active = py_trees.behaviours.Failure(name="Active")
        self.root = py_trees.composites.Selector(name="Root", children=[self.idle, self.active])

    def update(self):
        return self.root.tick()

class BehaviorTreeRosNode(Node):
    def __init__(self):
        super().__init__('simple_bt')
        self.bt = MyTree("MyTree")

    def run(self):
        rclpy.spin_once(self)
        self.bt.tick()
        rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeRosNode()

    try:
        while rclpy.ok():
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
