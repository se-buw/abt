// Online C++ compiler to run C++ program online
#include <iostream>
#include <chrono>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

class Increasing : public BT::SyncActionNode {
public:
    Increasing(const std::string& name)
        : BT::SyncActionNode(name, {})
    {
        std::string nodeName = "Increasing" + std::to_string(std::rand()); // random number for unique names
        twist_publisher_ = rclcpp::Node::make_shared(nodeName)->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        
    }

    BT::NodeStatus tick() override {
        // Move the turtle forward
        std::cout << "drawing an arc has been  started " << std::endl;
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = 2.0;  // Adjust the linear velocity
        twist_msg->angular.z = 1.0;
        twist_publisher_->publish(std::move(twist_msg));

        // Return success once the action is completed
        return BT::NodeStatus::SUCCESS;
    }
   
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};



// Function to check for obstacles
BT::NodeStatus Check() {
    std::cout << "Checking leaf 1  " << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// Class representing actions for handling obstacles
class Action {
public:
    Action() : _open(true) {}

 // Action to conform when there's no obstacle
    BT::NodeStatus conform() {
        _open = true;
        std::cout << "Checking leaf 2 " << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

// Action to keep straight when there's an obstacle
    BT::NodeStatus Keep() {
        _open = false;
        std::cout << "checking leaf 3 " << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool _open;
};

int main() {
    rclcpp::init(0, nullptr);
    // Create a Behavior Tree Factory
    BT::BehaviorTreeFactory factory;

    // Registering custom nodes and actions with the factory
    factory.registerNodeType<Increasing>("Increasing");
    factory.registerSimpleCondition("Check", std::bind(Check));

     // Create an instance of the Action class and register its actions
    Action node;
    factory.registerSimpleAction("conform", std::bind(&Action::conform, &node));
    factory.registerSimpleAction("Keep", std::bind(&Action::Keep, &node));

    // Create a tree from a file and execute it
    auto tree = factory.createTreeFromFile("./../b_tree.xml");
    tree.tickWhileRunning();

    return 0;
}
//Reference
//https://www.youtube.com/watch?v=kRp3eA09JkM&t=664s

