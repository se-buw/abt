/* Reference :
https://www.behaviortree.dev/
http://docs.ros.org/en/indigo/api/behaviortree_cpp_v3/html/classBT_1_1SyncActionNode.html 
*/

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <cstdlib> // For std::rand()


class MoveForward : public BT::SyncActionNode {
public:
    MoveForward(const std::string& name)
        : BT::SyncActionNode(name, {})
    {
        std::string nodeName = "move_forward_node_" + std::to_string(std::rand()); // random number for unique names
        twist_publisher_ = rclcpp::Node::make_shared(nodeName)->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        //if (twist_publisher_==nullptr)
        //twist_publisher_ = rclcpp::Node::make_shared("move_forward_node")->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

    BT::NodeStatus tick() override {
        // Move the turtle forward
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = 2.0;  // Adjust the linear velocity
        twist_publisher_->publish(std::move(twist_msg));

        // Simulating a running action for a short duration
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Return success once the action is completed
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};


class MoveBackward : public BT::SyncActionNode {
public:
    MoveBackward(const std::string& name)
        : BT::SyncActionNode(name, {})
    {
        std::string nodeName = "move_backward_node_" + std::to_string(std::rand());
        twist_publisher_ = rclcpp::Node::make_shared(nodeName)->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        //twist_publisher_ = rclcpp::Node::make_shared("move_backward_node")->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

    BT::NodeStatus tick() override {
        // Move the turtle backward
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = -2.0;  // Move backward
        twist_publisher_->publish(std::move(twist_msg));

        // Simulating a running action for a short duration
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // success once the action is completed
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};


class RotateClockwise : public BT::SyncActionNode {
public:
    RotateClockwise(const std::string& name)
        : BT::SyncActionNode(name, {})
    {
        std::string nodeName = "rotate_clockwise_node_" + std::to_string(std::rand());
        twist_publisher_ = rclcpp::Node::make_shared(nodeName)->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        //twist_publisher_ = rclcpp::Node::make_shared("rotate_clockwise_node")->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

    BT::NodeStatus tick() override {
        // Rotating the turtle clockwise
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->angular.z = -1.0;  // Adjust the angular velocity for clockwise rotation
        twist_publisher_->publish(std::move(twist_msg));

        // Simulating a running action for a short duration
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Returning success once the action is completed
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};



// Action Node: RotateCounterClockwise
class RotateCounterClockwise : public BT::SyncActionNode {
public:
    RotateCounterClockwise(const std::string& name)
        : BT::SyncActionNode(name, {})
    {
        std::string nodeName = "rotate_counterclockwise_node_" + std::to_string(std::rand());
        twist_publisher_ = rclcpp::Node::make_shared(nodeName)->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        //twist_publisher_ = rclcpp::Node::make_shared("rotate_counterclockwise_node")->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

    BT::NodeStatus tick() override {
        // Rotating the turtle counterclockwise
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->angular.z = 1.0;  // Adjust the angular velocity for counterclockwise rotation
        twist_publisher_->publish(std::move(twist_msg));

        // Simulating a running action for a short duration
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Returning success once the action is completed
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};

int main() {
    // ROS 2 initialization
    rclcpp::init(0, nullptr);

    BT::BehaviorTreeFactory factory;

    // Registering the MoveForward, MoveBackward, RotateClockwise, TurnLeft, and RotateCounterClockwise node types
    factory.registerNodeType<MoveForward>("MoveForward");
    factory.registerNodeType<MoveBackward>("MoveBackward");
    factory.registerNodeType<RotateClockwise>("RotateClockwise");
    factory.registerNodeType<RotateCounterClockwise>("RotateCounterClockwise");

    // Loading the behavior tree from an external XML file
    std::string xml_filename = "/home/parallels/abt/ros2_ws/src/my_turtle_controller/behavior_trees/my_turtle_behavior.xml";
    auto tree = factory.createTreeFromFile(xml_filename);

    // Running the Behavior Tree
    tree.rootNode()->executeTick();

    // ROS 2 shutdown
    rclcpp::shutdown();

    return 0;
}
