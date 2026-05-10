cmake_minimum_required(VERSION 3.8)
project(experiment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rm_ros_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)  
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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

add_executable(experiment_control src/experiment_control.cpp)
add_executable(force_collector src/force_collector.cpp)

ament_target_dependencies(experiment_control rclcpp moveit_ros_planning_interface rm_ros_interfaces geometry_msgs shape_msgs std_msgs)
ament_target_dependencies(force_collector rclcpp rm_ros_interfaces)

install(TARGETS 
  experiment_control
  force_collector
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/experiment_gui.py
  scripts/force_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()