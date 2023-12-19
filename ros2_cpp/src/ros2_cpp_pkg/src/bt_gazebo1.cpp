// Before runnig the code create a world in gazebo.
// For example: empty world in gazebo: ros2 launch turtlebot3_gazebo empty_world.launch.py

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>  // Time-related functionalities
#include <cstdlib>
#include <thread>
#include <iostream>

// Base class for defining actions in the behavior tree
class Action : public BT::SyncActionNode {
public:
     // Constructor to initialize the Action node
    Action(const std::string& name, const std::string& node_name, double linear_speed, double angular_speed)
        : BT::SyncActionNode(name, {}), node_name_(node_name), linear_speed_(linear_speed), angular_speed_(angular_speed)
    {
        initializeNode();  // Initializes the ROS2 node and publisher
    }

     // Destructor to perform cleanup when the Action node is destroyed
    ~Action() {
        if (twist_publisher_ && node_) {
            twist_publisher_.reset();  // Reset publisher
            rclcpp::spin_some(node_);  // Process any pending ROS2 callbacks
            node_.reset();
        }
    }
    
    // Override of the virtual method tick() from SyncActionNode
    BT::NodeStatus tick() override {
         // Create a Twist message for robot movement
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = linear_speed_;   // Set linear speed
        twist_msg->angular.z = angular_speed_;   // Set angular speed
        twist_publisher_->publish(std::move(twist_msg));   // Publish the Twist message

        // Adjust sleep duration based on the action
        std::this_thread::sleep_for(getSleepDuration());

        return BT::NodeStatus::SUCCESS;
    }

protected:
    // Function to initialize the ROS2 node and publisher
    void initializeNode() {
        if (!node_) {
            node_ = rclcpp::Node::make_shared(node_name_);
            twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }
    }

    // Virtual function to be overridden by derived classes
    virtual std::chrono::seconds getSleepDuration() const {
        return std::chrono::seconds(3); // Default sleep duration
    }

protected:
    // Static members shared among all Action instances
    static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    static rclcpp::Node::SharedPtr node_;

    std::string node_name_;  // Name of the ROS2 node
    double linear_speed_;
    double angular_speed_;
};

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Action::twist_publisher_ = nullptr;
rclcpp::Node::SharedPtr Action::node_ = nullptr;

class Forward : public Action {
public:
    Forward(const std::string& name)
        : Action(name, "forward_node", 0.5, 0.0) {}

    BT::NodeStatus tick() override {
        std::cout << "Moving Forward" << std::endl;
        return Action::tick();
    }

protected:
    std::chrono::seconds getSleepDuration() const override {
        return std::chrono::seconds(5); // Forward sleeps for " " seconds
    }
};

class Backward : public Action {
public:
    Backward(const std::string& name)
        : Action(name, "backward_node", -0.5, 0.0) {}

    BT::NodeStatus tick() override {
        std::cout << "Moving Backward" << std::endl;
        return Action::tick();
    }

protected:
    std::chrono::seconds getSleepDuration() const override {
        return std::chrono::seconds(5); // Backward sleeps for " " seconds
    }
};

class Right : public Action {
public:
    Right(const std::string& name)
        : Action(name, "right_node", 0.0, -1.0) // Adjust speed for TurtleBot3
    {}

    BT::NodeStatus tick() override {
        std::cout << "Taking Right" << std::endl;
        return Action::tick();
    }
protected:
    std::chrono::seconds getSleepDuration() const override {
        return std::chrono::seconds(2); // Backward sleeps for " " seconds
    }   
    
};

class Left : public Action {
public:
    Left(const std::string& name)
        : Action(name, "left_node", 0.0, 1.0) // Adjust speed for TurtleBot3
    {}

    BT::NodeStatus tick() override {
        std::cout << "Taking left" << std::endl;
        return Action::tick();
    }
protected:
    std::chrono::seconds getSleepDuration() const override {
        return std::chrono::seconds(2); // Backward sleeps for " " seconds
    }
};

class Stop : public Action {
public:
    Stop(const std::string& name)
        : Action(name, "Stop_node", 0.0, 0.0) // Adjust speed for TurtleBot3
    {}

    BT::NodeStatus tick() override {
        std::cout << "Stop" << std::endl;
        return Action::tick();
    }
protected:
    std::chrono::seconds getSleepDuration() const override {
        return std::chrono::seconds(1); // Backward sleeps for " " seconds
    }
};

// Define other action classes similarly (Right, Left, Stop)

int main() {
    rclcpp::init(0, nullptr);  // Initialize ROS2

    BT::BehaviorTreeFactory factory;// Behavior Tree factory for node registration

     // Register action node types with the factory
    factory.registerNodeType<Forward>("Forward");
    factory.registerNodeType<Backward>("Backward");
    factory.registerNodeType<Right>("Right");
    factory.registerNodeType<Left>("Left");
    factory.registerNodeType<Stop>("Stop");

        // Create a behavior tree from an XML file using the factory
    auto tree = factory.createTreeFromFile("./../bt_gazebo1.xml");

    // Execute the behavior tree by ticking nodes while it's running
    tree.tickWhileRunning();

    return 0;
}

// References:
// https://www.behaviortree.dev/docs/tutorial-basics/tutorial_01_first_tree

