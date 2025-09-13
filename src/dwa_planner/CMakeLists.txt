cmake_minimum_required(VERSION 3.8)
project(dwa_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Install Python module folder
install(
  DIRECTORY dwa_planner/
  DESTINATION lib/${PROJECT_NAME}
)

# Install executable node script
install(
  PROGRAMS dwa_planner/dwa_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS dwa_planner/dummy_robot.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
