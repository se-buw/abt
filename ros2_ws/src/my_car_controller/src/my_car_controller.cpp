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
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <memory>
#include <iostream>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <fstream>


//Node for controlling the drive
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

        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = linear_speed_;
        twist_msg->angular.z = angular_speed_;
        twist_publisher_->publish(std::move(twist_msg));

        std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(sleep_duration_)));

        return BT::NodeStatus::SUCCESS;
    }
//Accessing the paramters for velocity and duration
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
        if (!node_) {
            node_ = rclcpp::Node::make_shared("move_node");
            // publisher for drive commands
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

 //Node for interpretation of lidar messages 
class LidarNode : public BT::StatefulActionNode {
public:
    LidarNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config), threshold_(1.0), lidar_message_received_(false), num_rays_(0)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                handleLidarMessage(msg);
            });
        
        RCLCPP_INFO(node_->get_logger(), "Subscription to /lidar topic created");
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("threshold") };
    }
// Accessing the proximity value of obstacle
    BT::NodeStatus onStart() override
    {
        if (!getInput<double>("threshold", threshold_))
        {
            throw BT::RuntimeError("Failed to retrieve parameter [threshold] in LidarNode");
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!lidar_message_received_) {
            
            return BT::NodeStatus::RUNNING;
        }

        if (std::any_of(latest_lidar_ranges_.begin(), latest_lidar_ranges_.end(),
            [this](double range) { return range < threshold_; }))
        {
            RCLCPP_INFO(node_->get_logger(), "Obstacle Detected");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
        subscription_.reset();
        node_.reset();
        rclcpp::shutdown();
    }

private:
    void handleLidarMessage(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
    
        num_rays_ = msg->ranges.size();

        // Storing the lidar ranges
        latest_lidar_ranges_.clear();
        std::transform(msg->ranges.begin(), msg->ranges.end(), std::back_inserter(latest_lidar_ranges_),
            [](float value) { return static_cast<double>(value); });

        lidar_message_received_ = true;
    }

    double threshold_;
    std::vector<double> latest_lidar_ranges_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    bool lidar_message_received_;
    // Number of rays in the lidar message
    size_t num_rays_; 
};

//Node for interpretation of contact sensor information
class ContactNode : public BT::StatefulActionNode
{
public:
    ContactNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config), contact_detected_(false)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
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
           
            return BT::NodeStatus::RUNNING;
        }
        RCLCPP_INFO(node_->get_logger(), "Contact");
        return BT::NodeStatus::FAILURE;
    }

    void onHalted() override
    {
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


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    
    // Creating ROS 2 nodes
    auto node = std::make_shared<rclcpp::Node>("bt_node");

    // Creating a blackboard
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", node);

    // Creating Behavior Tree factory and register nodes
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<MoveNode>("MoveNode");
    factory.registerNodeType<LidarNode>("LidarNode");
    factory.registerNodeType<ContactNode>("ContactNode");

    // Loading the behavior tree from XML file
    std::string xml_filename = "src/my_car_controller/behavior_trees/my_car_behavior.xml";
    auto tree = factory.createTreeFromFile(xml_filename, blackboard);

//Publishing to the BT IDE Groot2
    BT::Groot2Publisher publisher(tree);
    std::string xml_models = BT::writeTreeNodesModelXML(factory);

    std::string file_path = "src/my_car_controller/behavior_trees/models.xml";

    // Opening a file stream
    std::ofstream file(file_path);

    if (file.is_open()) {
        // Writing the XML string to the file
        file << xml_models;
        // Closing the file stream
        file.close();
        std::cout << "XML models have been saved to " << file_path << std::endl;
    } else {
        std::cerr << "Unable to open file: " << file_path << std::endl;
    }
    // Executing the Behavior Tree
    while (rclcpp::ok() && tree.rootNode()->executeTick() == BT::NodeStatus::RUNNING) {
        // Spining ROS 2 nodes to handle incoming messages
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();

    return 0;
}
