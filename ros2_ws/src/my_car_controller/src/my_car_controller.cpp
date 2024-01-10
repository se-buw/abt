#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <thread>

class MoveNode : public BT::SyncActionNode {
public:
 MoveNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), linear_speed_(0.0), angular_speed_(0.0), sleep_duration_(2)
    
    {
        initializeNode();
    }

    ~MoveNode() override { // Destructor
        // Cleanup before shutting down ROS
        twist_publisher_.reset();
        node_.reset();
        rclcpp::shutdown();
    }

    BT::NodeStatus tick() override {
        


        // Extracting parameters from NodeConfiguration using name()
        BT::Expected<double> linear_speed = getInput<double>("linear_speed");
        BT::Expected<double> angular_speed = getInput<double>("angular_speed");
        BT::Expected<double> sleep_duration = getInput<double>("sleep_duration");

        // Check if expected values are valid. If not, throw their errors.
        if (!linear_speed || !angular_speed) {
            throw BT::RuntimeError("Failed to retrieve required input parameters");
        }

        // Store the values in member variables
        linear_speed_ = *linear_speed;
        angular_speed_ = *angular_speed;
        sleep_duration_ = *sleep_duration;

        RCLCPP_INFO(node_->get_logger(), "[%s] Executing tick() with parameters linear=%f, angular=%f",
            name().c_str(), *linear_speed, *angular_speed);

            auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
            twist_msg->linear.x = linear_speed_;
            twist_msg->angular.z = angular_speed_;
            twist_publisher_->publish(std::move(twist_msg));

            // Simulating a running action for the specified duration
            std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(sleep_duration_)));

            return BT::NodeStatus::SUCCESS;
       
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("linear_speed"), BT::InputPort<double>("angular_speed"), BT::InputPort<double>("sleep_duration")};
    }

private:
    void initializeNode() {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        // If the publisher is not created already
        if (!node_) {
            node_ = rclcpp::Node::make_shared("behavior_tree_node");
            twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }
    }

private:
    static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    static rclcpp::Node::SharedPtr node_;

    double linear_speed_;
    double angular_speed_;
    double sleep_duration_;
};

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr MoveNode::twist_publisher_ = nullptr; // Initialization
rclcpp::Node::SharedPtr MoveNode::node_ = nullptr; // Initialization



int main() {
    // ROS 2 initialization
    rclcpp::init(0, nullptr);

    BT::BehaviorTreeFactory factory;

    // Registering the MoveNode type
    factory.registerNodeType<MoveNode>("MoveNode");

    // Loading the behavior tree from an external XML file
    std::string xml_filename = "/home/selab/abt/ros2_ws/src/my_car_controller/behavior_trees/my_car_behavior.xml";
    auto tree = factory.createTreeFromFile(xml_filename);

    // Running the Behavior Tree
    tree.rootNode()->executeTick();

    rclcpp::shutdown();
    return 0;
}

