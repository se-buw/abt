# adley



## Getting started
Consists of a ros2 workspace



## Package
#### 1. my_robot_controller
#### 2. my_behavior_tree

#### 1. my_robot_controller
This package consists of the following files

1. [__init__.py](https://se-git.medien.uni-weimar.de/se-projects/abt/-/blob/adley/README.md#1-initpy)
2. [moveTurtle.py](https://se-git.medien.uni-weimar.de/se-projects/abt/-/blob/adley/README.md#2-moveturtlepy)
3. [turtle_publisher.py](https://se-git.medien.uni-weimar.de/se-projects/abt/-/tree/adley#3-turtle_publisherpy)
4. [turtle_subscriber.py](https://se-git.medien.uni-weimar.de/se-projects/abt/-/tree/adley#4-turtle_subscriberpy)
5. [advanced_turtle1.py](https://se-git.medien.uni-weimar.de/se-projects/abt/-/tree/adley#5-advanced_turtle1py)
6. [advanced_turtle2.py](https://se-git.medien.uni-weimar.de/se-projects/abt/-/tree/adley#6-advanced_turtle2py)
7. [advanced_turtle3.py](https://se-git.medien.uni-weimar.de/se-projects/abt/-/tree/adley#7-advanced_turtle3py)


#### 1. __init__.py

It initialises the python package and marks the directory as a package.

#### 2. moveTurtle.py

It consists of the class MoveTurtle class which creates a publisher to control the turtle movement and subscriber to receive the co-ordinates of the turtle in the same terminal.

The turtle movement is controlled to draw a circle by changing the linear velocity, angular velocity and angle of the turtlesim node.

[Run the code following the instructions at the end and using the name moveTurtle.](https://se-git.medien.uni-weimar.de/se-projects/abt/-/tree/adley#running-the-code)

##### Dependencies 
rclpy\
Node from rclpy.node\
Twist from geometry_msgs.msg\
Pose from turtlesim.msg


#### 3. turtle_publisher.py

It consists of the class TurtlePublisher which controls the turtlesim node to move in a circle.

The turtle movement is controlled to draw a circle by changing the linear velocity, angular velocity and angle of the turtlesim node.

[Run the code following the instructions at the end and using the name turtle_publisher.](https://se-git.medien.uni-weimar.de/se-projects/abt/-/tree/adley#running-the-code)

##### Dependencies 
rclpy\
Node from rclpy.node\
Twist from geometry_msgs.msg\
time

#### 4. turtle_subscriber.py

It consists of the class "TurtleSubscriber" which receoves the messages from the turtlesim node x and y coordinates.

[Run the code following the instructions at the end and using the name turtle_subscriber.](https://se-git.medien.uni-weimar.de/se-projects/abt/-/tree/adley#running-the-code)
##### Dependencies 
rclpy\
Node from rclpy.node\
Pose from turtlesim.msg

#### 5. advanced_turtle1.py

It is used to control the movement of the turtlesim node by implementing a Behavior Tree (BT).

It consists of the class 
- "MoveForward" which inherits from the py_trees.behavior.Behavior and moves the turtlesim node forward.
- "ZigZag" which turns the turtle left and right to move in a zigzag motion

The BT structure is as below:

Sequence (Root)\
|\
|-- MoveForward (Action node)\
|\
|-- Zigzag (Action node)



The main function is used to define the structure of BT.

[Run the code following the instructions at the end and using the name advanced_turtle1.](https://se-git.medien.uni-weimar.de/se-projects/abt/-/tree/adley#running-the-code)

##### Dependencies 
py_trees\
rclpy\
Node from rclpy.node\
Twist from geometry_msgs.msg\

#### 6. advanced_turtle2.py

It is used to control the movement of the turtlesim node by implementing a Behavior Tree (BT).

It consists of the class 
- "MoveForward" which inherits from the py_trees.behavior.Behavior and moves the turtlesim node forward.
- "ZigZag" which turns the turtle left and right to move in a zigzag motion
- "RotateClockwise" which rotates the turtle clockwise using the angular component of the node.
- "RotateCounterClockwise" which rotates the turtle counter-clockwise using the angular component of the node.

The BT structure is as below:

Sequence (Root: Sequence node)\
|\
|-- MoveForward (Action node)\
|\
|-- Zigzag (Action node)\
|\
|-- RotateClockwise (Action node)\
|\
|-- RotateCounterClockwise (Action node)\



The main function is used to define the structure of BT.

[Run the code following the instructions at the end and using the name advanced_turtle2.](https://se-git.medien.uni-weimar.de/se-projects/abt/-/tree/adley#running-the-code)

##### Dependencies 
py_trees\
rclpy\
Node from rclpy.node\
Twist from geometry_msgs.msg

#### 7. advanced_turtle3.py

It is used to control the movement of the turtlesim node by implementing a Behavior Tree (BT).

It consists of the class 
- "MoveForward" which inherits from the py_trees.behavior.Behavior and moves the turtlesim node forward.
- "ZigZag" which turns the turtle left and right to move in a zigzag motion
- "RotateClockwise" which rotates the turtle clockwise using the angular component of the node.
- "RotateCounterClockwise" which rotates the turtle counter-clockwise using the angular component of the node.

The BT structure is as below:

RootSelector (Root : Selector node)\
|\
|-- ForwardNode (Sequence node)\
|   |\
|   |-- MoveForward (Action node)\
|   |\
|   |-- Zigzag (Action node)\
|\
|-- RotationNode (Sequence node)\
    |\
    |-- RotateClockwise (Action node)\
    |\
    |-- RotateCounterClockwise (Action node)\


The main function is used to define the structure of BT.

[Run the code following the instructions at the end and using the name advanced_turtle3.](https://se-git.medien.uni-weimar.de/se-projects/abt/-/tree/adley#running-the-code)

##### Dependencies 
py_trees\
rclpy\
Node from rclpy.node\
Twist from geometry_msgs.msg


#### 2. my_behavior_tree

This package consists of the following executable files

my_behavior_tree.cpp

my_behavior_tree implements a basic Behavior tree (BT) which inherits from behaviortree_cpp C++ package. This file executes a BT by traversing the xml file /behavior_trees/my_behavior_tree.xml. As the BT is executed the output is printed in the console.

The contents of my_behavior_tree.xml are given below,

<!-- my_behavior_tree_complex.xml -->
<root main_tree_to_execute="MainTree" BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="RootSequence">
      <MoveForward name="MoveForward"/>
      <TurnLeft name="TurnLeft"/>
      <MySelector name="BatteryCheckSelector">
        <CheckBattery name="CheckBattery"/>
        <Sequence name="MoveTurnSequence">
          <MoveRight name="MoveRight"/>
          <StopMovement name="StopMovement"/>
        </Sequence>
      </MySelector>
    </Sequence>
  </BehaviorTree>
</root>

We see the following ouput in the console:

Moving forward...
Turning left...
Checking battery...
----------------------
Update the path in the my_behavior_tree.cpp file for the xml file as in your local directory before running the code.
Install the BehaviorTree.CPP package in the src folder of the ros2_ws.
Run the file following the instructions at the end and using the name my_behavior_tree.



## Running the code
1. Navigate to the ros2_ws in the terminal (ros2 workspace : cd ros2_ws)
2. execute the command <colcon build>
3. execute the command <source install/setup.bash>
4. Run the desired file by executing the command : ros2 run <package name> <access name>
   Access name is the same as the file name without the extension (e.g. .py for python)





