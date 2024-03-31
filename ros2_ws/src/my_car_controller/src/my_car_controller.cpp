/* Reference :
https://www.behaviortree.dev/
http://docs.ros.org/en/indigo/api/behaviortree_cpp_v3/html/classBT_1_1SyncActionNode.html 
*/

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <thread>
#include <memory>
#include <std_msgs/msg/bool.hpp> //  header for boolean messages

class MoveNode : public BT::SyncActionNode {
public:
    MoveNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), linear_speed_(0.0), angular_speed_(0.0), sleep_duration_(2)
    {
        initializeNode();
    }

    ~MoveNode() override {
        twist_publisher_.reset();
        node_.reset();
        rclcpp::shutdown();
    }

    BT::NodeStatus tick() override {
        BT::Expected<double> linear_speed = getInput<double>("linear_speed");
        BT::Expected<double> angular_speed = getInput<double>("angular_speed");
        BT::Expected<double> sleep_duration = getInput<double>("sleep_duration");

        if (!linear_speed || !angular_speed) {
            throw BT::RuntimeError("Failed to retrieve required input parameters");
        }

        linear_speed_ = *linear_speed;
        angular_speed_ = *angular_speed;
        sleep_duration_ = *sleep_duration;

        RCLCPP_INFO(node_->get_logger(), "[%s] Executing tick() with parameters linear=%f, angular=%f",
            name().c_str(), *linear_speed, *angular_speed);

        // Publishing the twist message
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = linear_speed_;
        twist_msg->angular.z = angular_speed_;
        twist_publisher_->publish(std::move(twist_msg));

        // Simulating a running action for the specified duration
        std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(sleep_duration_)));

        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("linear_speed"),
                 BT::InputPort<double>("angular_speed"),
                 BT::InputPort<double>("sleep_duration") };
    }

    void setNode(const rclcpp::Node::SharedPtr& node) {
        node_ = node;
    }

private:
    void initializeNode() {
        //if (!rclcpp::ok()) {
          //rclcpp::init(0, nullptr);
        //}

        // Creating a node if not already created
        if (!node_) {
            node_ = rclcpp::Node::make_shared("move_node");
            twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }
    }

    static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    static rclcpp::Node::SharedPtr node_;

    double linear_speed_;
    double angular_speed_;
    double sleep_duration_;
};

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr MoveNode::twist_publisher_ = nullptr;
rclcpp::Node::SharedPtr MoveNode::node_ = nullptr;

class LidarNode : public BT::StatefulActionNode
{
public:
    LidarNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config), threshold_(1.0), lidar_message_received_(false)
    {
        // Initializing ROS 2 node and subscription
        node_ = std::make_shared<rclcpp::Node>("lidar_node");
        subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar", 100, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                handleLidarMessage(msg);
            });
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("threshold") };
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

    void handleLidarMessage(const sensor_msgs::msg::LaserScan::SharedPtr msg);

private:
    double threshold_;
    std::vector<double> latest_lidar_ranges_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    bool lidar_message_received_;
};

BT::NodeStatus LidarNode::onStart()
{
    // Retrieve the threshold parameter
    if (!getInput<double>("threshold", threshold_))
    {
        throw BT::RuntimeError("Failed to retrieve parameter [threshold] in LidarNode");
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LidarNode::onRunning()
{
    if (!lidar_message_received_) {
        // Lidar message not received yet, logging an error and returning FAILURE
        RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve Lidar messages");
        return BT::NodeStatus::RUNNING;
    }

    // Processing lidar data and updating node status directly
    if (std::any_of(latest_lidar_ranges_.begin(), latest_lidar_ranges_.end(),
        [this](double range) { return range < threshold_; }))
    {
        return BT::NodeStatus::FAILURE;
    }
    else
    {
        return BT::NodeStatus::RUNNING;
    }
}


void LidarNode::onHalted()
{
    // Cleaning up ROS 2 resources
    subscription_.reset();
    node_.reset();
    rclcpp::shutdown();
}

void LidarNode::handleLidarMessage(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Storing lidar message
    latest_lidar_ranges_.clear();
    std::transform(msg->ranges.begin(), msg->ranges.end(), std::back_inserter(latest_lidar_ranges_),
        [](float value) { return static_cast<double>(value); });

    // Comparing lidar ranges against threshold
    lidar_message_received_ = true;

    if (std::any_of(latest_lidar_ranges_.begin(), latest_lidar_ranges_.end(),
        [this](double range) { return range < threshold_; }))
    {
        setStatus(BT::NodeStatus::FAILURE);
    }
    else
    {
        setStatus(BT::NodeStatus::RUNNING);
    }
}

class ContactNode : public BT::StatefulActionNode

{
public:
    ContactNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config), contact_detected_(false)
    {
        // Initializing ROS 2 node and subscription
        node_ = rclcpp::Node::make_shared("contact_node");
        subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/wall/touched", 100, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                handleContactMessage(msg);
            });
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!contact_detected_) {
            // Contact message not received yet, logging an error and returning FAILURE
            RCLCPP_ERROR(node_->get_logger(), "Failed to detect contact");
            return BT::NodeStatus::RUNNING;
        }

        // Contact detected, returning SUCCESS
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
        // Cleaning up ROS 2 resources
        subscription_.reset();
        node_.reset();
        rclcpp::shutdown();
    }

private:
    void handleContactMessage(const std_msgs::msg::Bool::SharedPtr msg)
    {
        contact_detected_ = msg->data;
    }

    bool contact_detected_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main() {
    // Initializing ROS 2 node
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("bt_node");

    // Creating Behavior Tree factory and registeing Nodes
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<MoveNode>("MoveNode");
    factory.registerNodeType<LidarNode>("LidarNode");
    factory.registerNodeType<ContactNode>("ContactNode");

    // Loading the behavior tree from XML file
    std::string xml_filename = "src/my_car_controller/behavior_trees/my_car_behavior.xml";
    auto tree = factory.createTreeFromFile(xml_filename);

    // Executing behavior tree
    while (rclcpp::ok() && tree.rootNode()->executeTick() == BT::NodeStatus::RUNNING) {
        rclcpp::spin_some(node);
    }

    // Shutting down ROS 2 node
    rclcpp::shutdown();

    return 0;
}
