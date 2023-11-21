cmake_minimum_required(VERSION 3.8)
project(my_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)



# Include directories for headers
include_directories(include)

# Build the executable
add_executable(my_behavior_tree src/my_behavior_tree.cpp)
ament_target_dependencies(my_behavior_tree behaviortree_cpp)

# Install the executable
install(TARGETS my_behavior_tree
        DESTINATION lib/${PROJECT_NAME}
)

# Install the behavior trees
install(DIRECTORY behavior_trees/
        DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# Install the header file
install(DIRECTORY include/
        DESTINATION include/
)

# Install the package.xml file
install(FILES package.xml
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
