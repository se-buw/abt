cmake_minimum_required(VERSION 3.5)
project(ros2_cpp_pkg)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(behaviortree_cpp REQUIRED)  # Adjust the package name/version

# Add include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${turtlesim_INCLUDE_DIRS}
  ${behaviortree_cpp_INCLUDE_DIRS}  # Adjust the variable name if needed
)
############################################################################################
# Create executable for ros2_cpp_code
add_executable(ros2_cpp_code src/ros2_cpp_code.cpp)

# Link against libraries
ament_target_dependencies(ros2_cpp_code
  rclcpp
  geometry_msgs
  turtlesim
  behaviortree_cpp  # Adjust the package name/version
)

install(TARGETS
  ros2_cpp_code
  DESTINATION lib/${PROJECT_NAME}
)

#############################################################################################
# Create executable for behavior_tree
add_executable(behavior_tree src/behavior_tree.cpp)

# Link against libraries for behavior_tree
ament_target_dependencies(behavior_tree
  rclcpp
  geometry_msgs
  turtlesim
  behaviortree_cpp  # Adjust the package name/version
)
# Install behavior_tree
install(TARGETS
  behavior_tree
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
