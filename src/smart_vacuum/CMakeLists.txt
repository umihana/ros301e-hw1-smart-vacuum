cmake_minimum_required(VERSION 3.8)
project(smart_vacuum)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(smart_vacuum_interfaces REQUIRED)

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

include_directories(include)   # so #include "smart_vacuum/floor_maps.hpp" is found

# build a program called hardware_driver_node from this file
add_executable(hardware_driver_node src/hardware_driver_node.cpp)
# "this program uses rclcpp"
ament_target_dependencies(hardware_driver_node rclcpp smart_vacuum_interfaces)

add_executable(map_loader_node src/map_loader_node.cpp)
ament_target_dependencies(map_loader_node rclcpp std_msgs smart_vacuum_interfaces)

add_executable(task_manager_node src/task_manager_node.cpp)
ament_target_dependencies(task_manager_node rclcpp rclcpp_action std_msgs smart_vacuum_interfaces)

# copies the program to install/smart_vacuum/lib/smart_vacuum/ (where ros2 run looks for it)
install(TARGETS hardware_driver_node map_loader_node task_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


ament_package()
