/* Reference :
https://www.behaviortree.dev/
http://docs.ros.org/en/indigo/api/behaviortree_cpp_v3/html/classBT_1_1SyncActionNode.html 
*/

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
//#include <cstdlib> // For std::rand()

class MoveNode : public BT::SyncActionNode {
public:
    MoveNode(const std::string& name, double linear_speed, double angular_speed)
        : BT::SyncActionNode(name, {}), linear_speed_(linear_speed), angular_speed_(angular_speed)
    {
        initializeNode();
    }

    ~MoveNode() { //destructor
        // cleanup before shutting down ROS
        twist_publisher_.reset();
        node_.reset();
        rclcpp::shutdown();
    }

    BT::NodeStatus tick() override {
        RCLCPP_INFO(node_->get_logger(), "[%s] Executing tick()", name().c_str());

        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = linear_speed_;
        twist_msg->angular.z = angular_speed_;
        //twist_publisher_->publish(std::move(twist_msg));

        RCLCPP_INFO(node_->get_logger(), "Published Twist message: linear=%f, angular=%f", linear_speed_, angular_speed_);
        twist_publisher_->publish(std::move(twist_msg));

        // Simulating a running action for a short duration
        std::this_thread::sleep_for(std::chrono::seconds(3));

        return BT::NodeStatus::SUCCESS;
    }

private:
    void initializeNode() {
        
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        // if publisher is not created already
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
};

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr MoveNode::twist_publisher_ = nullptr; // ROS2 publisher Initialisation 
rclcpp::Node::SharedPtr MoveNode::node_ = nullptr; // ROS2 Node Initialisation

class MoveForward : public MoveNode {
public:
    MoveForward(const std::string& name)
        : MoveNode(name, 2.0, 0.0)
    {}
};

class MoveBackward : public MoveNode {
public:
    MoveBackward(const std::string& name)
        : MoveNode(name, -2.0, 0.0)
    {}
};

class RotateClockwise : public MoveNode {
public:
    RotateClockwise(const std::string& name)
        : MoveNode(name, 0.0, -1.0)
    {}
};

class RotateCounterClockwise : public MoveNode {
public:
    RotateCounterClockwise(const std::string& name)
        : MoveNode(name, 0.0, 1.0)
    {}
};

class Halt : public MoveNode {
public:
    Halt(const std::string& name)
        : MoveNode(name, 0.0, 0.0)
    {}
};

int main() {
    // ROS 2 initialization
    rclcpp::init(0, nullptr);

    BT::BehaviorTreeFactory factory;

    // Registering the MoveNode types
    factory.registerNodeType<MoveForward>("MoveForward");
    factory.registerNodeType<MoveBackward>("MoveBackward");
    factory.registerNodeType<RotateClockwise>("RotateClockwise");
    factory.registerNodeType<RotateCounterClockwise>("RotateCounterClockwise");
    factory.registerNodeType<Halt>("Halt");

    // Loading the behavior tree from an external XML file
    std::string xml_filename = "src/my_car_controller/behavior_trees/my_car_behavior.xml";
    auto tree = factory.createTreeFromFile(xml_filename);

    // Running the Behavior Tree
    tree.rootNode()->executeTick();

    rclcpp::shutdown();
    return 0;
}

