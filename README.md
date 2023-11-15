# adley



## Getting started
Consists of a ros2 workspace



## Package
#### my_robot_controller

This package consists of the following files

1. __init__.py
2. moveTurtle.py
3. turtle_publisher.py
4. turtle_subscriber.py
5. advanced_turtle1.py
6. advanced_turtle2.py
7. advanced_turtle3.py


#### 1.__init__.py

It initialises the python package and marks the directory as a package.

#### 2. moveTurtle.py

It consists of the class MoveTurtle class which creates a publisher to control the turtle movement and subscriber to receive the co-ordinates of the turtle in the same terminal.

The turtle movement is controlled to draw a circle by changing the linear velocity, angular velocity and angle of the turtlesim node.

Run the code following the instructions at the end and using the name moveTurtle.

##### Dependencies 
rclpy
Node from rclpy.node
Twist from geometry_msgs.msg
Pose from turtlesim.msg


#### 3. turtle_publisher.py

It consists of the class TurtlePublisher which controls the turtlesim node to move in a circle.

The turtle movement is controlled to draw a circle by changing the linear velocity, angular velocity and angle of the turtlesim node.

Run the code following the instructions at the end and using the name turtle_publisher.

##### Dependencies 
rclpy
Node from rclpy.node
Twist from geometry_msgs.msg
time

#### 4. turtle_subscriber.py

It consists of the class "TurtleSubscriber" which receoves the messages from the turtlesim node x and y coordinates.

Run the code following the instructions at the end and using the name turtle_subscriber.
##### Dependencies 
rclpy
Node from rclpy.node
Pose from turtlesim.msg

#### 5. advanced_turtle1.py

It is used to control the movement of the turtlesim node by implementing a Behavior Tree (BT).

It consists of the class 
- "MoveForward" which inherits from the py_trees.behavior.Behavior and moves the turtlesim node forward.
- "ZigZag" which turns the turtle left and right to move in a zigzag motion

The BT structure is as below:

Sequence (Root)
|
|-- MoveForward (Action node)
|
|-- Zigzag (Action node)



The main function is used to define the structure of BT.

Run the code following the instructions at the end and using the name advanced_turtle1.

##### Dependencies 
py_trees
rclpy
Node from rclpy.node
Twist from geometry_msgs.msg

#### 6. advanced_turtle2.py

It is used to control the movement of the turtlesim node by implementing a Behavior Tree (BT).

It consists of the class 
- "MoveForward" which inherits from the py_trees.behavior.Behavior and moves the turtlesim node forward.
- "ZigZag" which turns the turtle left and right to move in a zigzag motion
- "RotateClockwise" which rotates the turtle clockwise using the angular component of the node.
- "RotateCounterClockwise" which rotates the turtle counter-clockwise using the angular component of the node.

The BT structure is as below:

Sequence (Root: Sequence node)
|
|-- MoveForward (Action node)
|
|-- Zigzag (Action node)
|
|-- RotateClockwise (Action node)
|
|-- RotateCounterClockwise (Action node)



The main function is used to define the structure of BT.

Run the code following the instructions at the end and using the name advanced_turtle2.

##### Dependencies 
py_trees
rclpy
Node from rclpy.node
Twist from geometry_msgs.msg

#### 7. advanced_turtle3.py

It is used to control the movement of the turtlesim node by implementing a Behavior Tree (BT).

It consists of the class 
- "MoveForward" which inherits from the py_trees.behavior.Behavior and moves the turtlesim node forward.
- "ZigZag" which turns the turtle left and right to move in a zigzag motion
- "RotateClockwise" which rotates the turtle clockwise using the angular component of the node.
- "RotateCounterClockwise" which rotates the turtle counter-clockwise using the angular component of the node.

The BT structure is as below:

RootSelector (Root : Selector node)
|
|-- ForwardNode (Sequence node)
|   |
|   |-- MoveForward (Action node)
|   |
|   |-- Zigzag (Action node)
|
|-- RotationNode (Sequence node)
    |
    |-- RotateClockwise (Action node)
    |
    |-- RotateCounterClockwise (Action node)


The main function is used to define the structure of BT.

Run the code following the instructions at the end and using the name advanced_turtle3.

##### Dependencies 
py_trees
rclpy
Node from rclpy.node
Twist from geometry_msgs.msg




## Running the code
1. Navigate to the ros2_ws in the terminal (ros2 workspace : cd ros2_ws)
2. execute the command <colcon build>
3. execute the command <source install/setup.bash>
4. Run the desired file by executing the command : ros2 run <package name> <access name>
   Access name is the same as the file name without the extension (e.g. .py for python)


