cmake_minimum_required(VERSION 3.5)
project(warehouse_automation_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(global_localization_node src/global_localization.cpp)
ament_target_dependencies(global_localization_node
  rclcpp
  geometry_msgs
  std_srvs
)

install(TARGETS
  global_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
