#include <iostream>
#include <chrono>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

using namespace std::chrono_literals;

// Action node that increases speed
class Increasing : public BT::SyncActionNode {
public:
    explicit Increasing(const std::string &name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override {
        std::cout << "Increasing Speed " << std::endl;
        std::this_thread::sleep_for(2s);
        return BT::NodeStatus::SUCCESS;
    }
};

// Function to check for obstacles
BT::NodeStatus Check() {
    std::cout << "Checking the obstrcle " << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// Class representing actions for handling obstacles
class Action {
public:
    Action() : _open(true) {}

 // Action to conform when there's no obstacle
    BT::NodeStatus conform() {
        _open = true;
        std::cout << "Conforming 'no obstracle' " << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

// Action to keep straight when there's an obstacle
    BT::NodeStatus Keep() {
        _open = false;
        std::cout << "Keep Straight" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool _open;
};

int main() {
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

