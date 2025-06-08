cmake_minimum_required(VERSION 3.5)
project(warehouse_automation_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Localize.action"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(global_localization_node src/global_localization.cpp)
ament_target_dependencies(global_localization_node
  rclcpp
  geometry_msgs
  std_srvs
)

add_executable(glocalization_as_node src/glocalization_as.cpp)
ament_target_dependencies(glocalization_as_node
  rclcpp
  geometry_msgs
  std_srvs
  action_msgs
  rclcpp_action
)
rosidl_target_interfaces(glocalization_as_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  global_localization_node
  glocalization_as_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
