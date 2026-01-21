cmake_minimum_required(VERSION 3.5)
project(fastbot_waypoints)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---------------- FIND PACKAGES ----------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

# ---------------- GENERATE ACTION ----------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Waypoint.action"
  DEPENDENCIES geometry_msgs
)

# ---------------- EXECUTABLE ----------------
add_executable(fastbot_action_server src/fastbot_action_server.cpp)

ament_target_dependencies(fastbot_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
)

# Ensure action headers and typesupport are ready before build
add_dependencies(fastbot_action_server
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Link action typesupport library
rosidl_target_interfaces(fastbot_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# ---------------- INSTALL ----------------
install(TARGETS fastbot_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
