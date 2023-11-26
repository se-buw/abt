# cmake_minimum_required(VERSION 3.21)

# project(behavior_tree)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# find_package(behaviortree_cpp REQUIRED)

# add_executable(${PROJECT_NAME} behavior_tree.cpp)
# target_link_libraries(${PROJECT_NAME} BT::behaviortree_cpp)

cmake_minimum_required(VERSION 3.21)
project(behavior_tree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set BehaviorTree.CPP directory (Change this path to match your installation)
set(BEHAVIORTREE_CPP_DIR "/usr/local/")

# Include directories for BehaviorTree.CPP
include_directories(${BEHAVIORTREE_CPP_DIR}/include)

# Add your executable and link it with BehaviorTree.CPP
add_executable(${PROJECT_NAME} behavior_tree.cpp)

# Link with BehaviorTree.CPP library
target_link_libraries(${PROJECT_NAME} PRIVATE ${BEHAVIORTREE_CPP_DIR}/lib/libbehaviortree_cpp.so)

