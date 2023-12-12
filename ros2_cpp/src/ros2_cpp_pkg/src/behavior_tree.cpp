#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <cstdlib> 

// Action class representing a basic action node in the behavior tree
class Action : public BT::SyncActionNode {
public:
    // Constructor to initialize the Action
    Action(const std::string& name, const std::string& node_name, double linear_speed, double angular_speed)
        : BT::SyncActionNode(name, {}), node_name_(node_name), linear_speed_(linear_speed), angular_speed_(angular_speed)
    {
        initializeNode(); // Initialize the ROS node and publisher
    }

    // Destructor to clean up resources
    ~Action() {
        // Check if publisher and node exist before cleaning up
        if (twist_publisher_ && node_) {
            twist_publisher_.reset(); // Release the publisher
            rclcpp::spin_some(node_); // Ensure all pending callbacks are processed before shutdown
            node_.reset(); // Release the node
        }
    }

    // Method to perform the action when the node is ticked in the behavior tree
    BT::NodeStatus tick() override {

        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = linear_speed_;
        twist_msg->angular.z = angular_speed_;
        twist_publisher_->publish(std::move(twist_msg)); // Publish the twist message
        std::this_thread::sleep_for(std::chrono::seconds(3)); // Simulate action execution time

        return BT::NodeStatus::SUCCESS; // Return successful execution status
    }

protected:
    // Method to initialize the ROS node and publisher
    void initializeNode() {
        // Create ROS node and publisher if not already created
        if (!node_) {
            node_ = rclcpp::Node::make_shared(node_name_);
            twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        }
    }

protected:
    static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_; // Shared pointer to the publisher
    static rclcpp::Node::SharedPtr node_; // Shared pointer to the ROS node

    std::string node_name_; // Name of the ROS node
    double linear_speed_; // Linear speed for the action
    double angular_speed_; // Angular speed for the action
};

// Initialize static members of Action class
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Action::twist_publisher_ = nullptr;
rclcpp::Node::SharedPtr Action::node_ = nullptr;

// Derived classes representing specific actions (Forward, Backward, Right, Left)
// These classes specify the name, node, and speeds for each action
class Forward : public Action {
public:
    Forward(const std::string& name)
        : Action(name, "forward_node", 3.0, 0.0) // Name, ROS node name, linear speed, angular speed
    {}

    BT::NodeStatus tick() override {
        std::cout << "Moving Forward" << std::endl; // Log action
        return Action::tick(); // Perform the action
    }
};

class Backward : public Action {
public:
    Backward(const std::string& name)
        : Action(name, "backward_node", -3.0, 0.0)
    {}

    BT::NodeStatus tick() override {
        std::cout << "Moving Backward" << std::endl;
        return Action::tick();
    }
};

class Right : public Action {
public:
    Right(const std::string& name)
        : Action(name, "right_node", 0.0, -2.0)
    {}

    BT::NodeStatus tick() override {
        std::cout << "Taking Right" << std::endl;
        return Action::tick();
    }
};

class Left : public Action {
public:
    Left(const std::string& name)
        : Action(name, "left_node", 0.0, 2.0)
    {}

    BT::NodeStatus tick() override {
        std::cout << "Taking left" << std::endl;
        return Action::tick();
    }
};

int main() {
    rclcpp::init(0, nullptr); // Initialize ROS

    // Create a Behavior Tree Factory
    BT::BehaviorTreeFactory factory;

    // Register custom nodes and actions with the factory
    factory.registerNodeType<Forward>("Forward");
    factory.registerNodeType<Backward>("Backward");
    factory.registerNodeType<Right>("Right");
    factory.registerNodeType<Left>("Left");

    // Create a behavior tree from a file and execute it
    auto tree = factory.createTreeFromFile("./../behavior_tree.xml");
    tree.tickWhileRunning(); // Run the behavior tree

    return 0; // Return success
}


// References:
// https://www.behaviortree.dev/docs/tutorial-basics/tutorial_01_first_tree
