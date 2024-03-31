cmake_minimum_required(VERSION 3.8)
project(my_car_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)
add_executable(my_car_controller src/my_car_controller.cpp)
ament_target_dependencies(my_car_controller behaviortree_cpp rclcpp geometry_msgs)

install(TARGETS my_car_controller
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
 behavior_trees
 DESTINATION share/${PROJECT_NAME}/behavior_trees
 )

 install(DIRECTORY include/
  DESTINATION include/
)

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
