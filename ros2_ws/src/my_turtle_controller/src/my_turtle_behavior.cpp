#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MoveForward : public BT::SyncActionNode {

public:
    MoveForward(const std::string& name, const BT::NodeConfiguration& config)

        : BT::SyncActionNode(name, config) {
            
        node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
        twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

    BT::NodeStatus tick() override {
        // Move the turtle forward
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = 2.0;  // Adjust the linear velocity
        twist_publisher_->publish(std::move(twist_msg));

        // Simulate a running action for a short duration
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Return success once the action is completed
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};

int main() {
    // ROS 2 initialization
    rclcpp::init(0, nullptr);

    // Create a ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("my_node");

    // Load the behavior tree from an external XML file
    std::string xml_filename = "/home/parallels/abt/ros2_ws/src/my_turtle_controller/behavior_trees/my_behavior_tree.xml";
    BT::BehaviorTreeFactory factory;
    auto tree = factory.createTreeFromFile(xml_filename);

    // Register the MoveForward node type
    factory.registerNodeType<MoveForward>("MoveForward",
        [](const BT::NodeConfiguration& config) {
            return BT::NodeStatus::SUCCESS; 
        }

    );



    // Run the Behavior Tree
    tree.rootNode()->executeTick();

    // ROS 2 shutdown
    rclcpp::shutdown();

    return 0;
}
