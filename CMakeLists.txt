cmake_minimum_required(VERSION 3.5)
project(tf2_web_republisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  # action
  action/TFSubscription.action
  # service
  srv/RepublishTFs.srv
  # message
  msg/TFArray.msg
  DEPENDENCIES builtin_interfaces geometry_msgs
)

add_executable(tf2_web_republisher_node
  src/tf_web_republisher.cpp
)
ament_target_dependencies(tf2_web_republisher_node ${dependencies})
rosidl_target_interfaces(tf2_web_republisher_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS tf2_web_republisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
