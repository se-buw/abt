// my_behavior_tree.cpp
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>


// MoveForward, TurnLeft, MoveRight, and StopMovement node implementations
class MoveForward : public BT::SyncActionNode {
public:
    MoveForward(const std::string& name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override {
        std::cout << "Moving forward..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class TurnLeft : public BT::SyncActionNode {
public:
    TurnLeft(const std::string& name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override {
        std::cout << "Turning left..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class MoveRight : public BT::SyncActionNode {
public:
    MoveRight(const std::string& name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override {
        std::cout << "Moving right..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class StopMovement : public BT::SyncActionNode {
public:
    StopMovement(const std::string& name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override {
        std::cout << "Stopping movement..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// CheckBattery, MySelector node implementations
class CheckBattery : public BT::ConditionNode {
public:
    CheckBattery(const std::string& name) : BT::ConditionNode(name, {}) {}

    BT::NodeStatus tick() override {
        std::cout << "Checking battery..." << std::endl;
        bool battery_ok = true;  // Change this based on your condition
        return battery_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class MySelector : public BT::ControlNode {
public:
    MySelector(const std::string& name) : BT::ControlNode(name, {}) {}

    BT::NodeStatus tick() override {
        for (auto& child : children_nodes_) {
            auto child_status = child->executeTick();
            if (child_status == BT::NodeStatus::SUCCESS) {
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::FAILURE;
    }
};

int main() {
    BT::BehaviorTreeFactory factory;

    // Manually register nodes
    factory.registerNodeType<MoveForward>("MoveForward");
    factory.registerNodeType<TurnLeft>("TurnLeft");
    factory.registerNodeType<MoveRight>("MoveRight");
    factory.registerNodeType<StopMovement>("StopMovement");
    factory.registerNodeType<CheckBattery>("CheckBattery");
    factory.registerNodeType<MySelector>("MySelector");

    // Behavior Tree from XML
    std::string xml_filename = "/home/parallels/abt/ros2_ws/src/my_behavior_tree/behavior_trees/my_behavior_tree.xml";
    auto tree = factory.createTreeFromFile(xml_filename);

    // Running the Behavior Tree manually
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING) {
        status = tree.rootNode()->executeTick();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
