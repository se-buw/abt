/* Reference :
https://www.behaviortree.dev/
http://docs.ros.org/en/indigo/api/behaviortree_cpp_v3/html/classBT_1_1SyncActionNode.html 
*/

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <thread>
#include <memory>

class MoveNode : public BT::SyncActionNode {
public:
    MoveNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), linear_speed_(0.0), angular_speed_(0.0), sleep_duration_(2)
    {
        initializeNode();
    }

    ~MoveNode() override {
        twist_publisher_.reset();
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
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        // Creating a node if not already created
        if (!node_) {
            node_ = rclcpp::Node::make_shared("move_node");
            twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }
    }

    static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Node::SharedPtr node_;

    double linear_speed_;
    double angular_speed_;
    double sleep_duration_;
};

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr MoveNode::twist_publisher_ = nullptr;

class LidarNode : public rclcpp::Node {
public:
    LidarNode()
        : Node("lidar_node"), lidar_subscription_(create_subscription<sensor_msgs::msg::LaserScan>(
                                     "/lidar", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                                         handleLidarMessage(msg);
                                     })),
          latest_lidar_ranges_()
    {
    }

    const std::vector<double>& getLatestLidarRanges() const {
        return latest_lidar_ranges_;
    }

private:
   void handleLidarMessage(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received Lidar message with %lu ranges", msg->ranges.size());

    // Converting the ranges from float to double
    latest_lidar_ranges_.clear();
    std::transform(msg->ranges.begin(), msg->ranges.end(), std::back_inserter(latest_lidar_ranges_), 
                   [](float value) { return static_cast<double>(value); });
}


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    std::vector<double> latest_lidar_ranges_;
};

class CheckLidarCondition : public BT::ConditionNode {
public:
    CheckLidarCondition(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<LidarNode> lidar_node)
        : BT::ConditionNode(name, config), lidar_node_(lidar_node)
    {
        // Setting a default threshold value
        config.blackboard->set("threshold", 1.0);
    }

    BT::NodeStatus tick() override {
        double threshold;
        if (!getInput<double>("threshold", threshold)) {
            throw BT::RuntimeError("Failed to retrieve parameter [threshold] in CheckLidarCondition");
        }

        auto lidar_ranges = lidar_node_->getLatestLidarRanges();

        for (const auto& range : lidar_ranges) {
            if (range < threshold) {
                // If any range is less than the threshold, the condition is true
                return BT::NodeStatus::SUCCESS;
            }
        }

        // If no range is less than the threshold, the condition is false
        return BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("threshold"),
        };
    }

private:
    std::shared_ptr<LidarNode> lidar_node_;
};

int main() {
    rclcpp::init(0, nullptr);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<MoveNode>("MoveNode");
    factory.registerNodeType<CheckLidarCondition>("CheckLidar");

    // shared pointer to LidarNode
    auto lidar_node = std::make_shared<LidarNode>();

    // CheckLidarCondition node with the shared pointer to LidarNode
    auto check_lidar_condition = std::make_shared<CheckLidarCondition>("CheckLidar", BT::NodeConfiguration(), lidar_node);


    // Registering CheckLidarCondition node in the behavior tree factory
    factory.registerSimpleCondition("CheckLidar", check_lidar_condition);


    std::string xml_filename = "src/my_car_controller/behavior_trees/my_car_behavior.xml";
    auto tree = factory.createTreeFromFile(xml_filename);

    while (rclcpp::ok()) {
        tree.rootNode()->executeTick();
        rclcpp::spin_some(lidar_node);
    }

    rclcpp::shutdown();
    return 0;
}


