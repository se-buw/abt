cmake_minimum_required(VERSION 3.8)
project(my_turtle_controller)

#######################################################################

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#####################################################################

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

###

include_directories(include)

#set(dependencies 
#  rclcpp
#  geometry_msgs
#  behaviortree_cpp
#)

#set(executable_name my_turtle_behavior)
#set(library_name ${PROJECT_NAME})

#add_executable(
#  ${executable_name}
#  src/my_turtle_behavior.cpp
#)


#################################################################

add_executable(my_turtle_controller src/my_turtle_behavior.cpp)
ament_target_dependencies(my_turtle_controller behaviortree_cpp rclcpp geometry_msgs turtlesim)

#ament_target_dependencies(
#  ${executable_name}
#  ${dependencies}
#)

#target_link_libraries(${executable_name} ${library_name})

#ament_target_dependencies(${library_name} 
#${dependencies}
#)

#install(TARGETS ${library_name}
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

install(TARGETS my_turtle_controller
        DESTINATION lib/${PROJECT_NAME})

#install(TARGETS ${executable_name}
#  RUNTIME DESTINATION lib/${PROJECT_NAME}
#)
install(DIRECTORY
  behavior_trees
  DESTINATION share/${PROJECT_NAME}/behavior_trees
  )

install(DIRECTORY include/
  DESTINATION include/
)

# Install the package.xml file
install(FILES package.xml
        DESTINATION share/${PROJECT_NAME}
)

###################################################################

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
