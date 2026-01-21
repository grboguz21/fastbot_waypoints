cmake_minimum_required(VERSION 3.5)
project(fastbot_waypoints)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------- FIND DEPENDENCIES ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---------------- GENERATE ACTION ----------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Waypoint.action"
  DEPENDENCIES geometry_msgs
)

# ---------------- SERVER EXECUTABLE ----------------
add_executable(fastbot_action_server src/fastbot_action_server.cpp)

ament_target_dependencies(fastbot_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
)

add_dependencies(fastbot_action_server
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

rosidl_target_interfaces(fastbot_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# ---------------- INSTALL SERVER ----------------
install(TARGETS fastbot_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

# ---------------- TESTS ----------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  set(TESTFILES
    test/main.cpp
    test/control_node_test.cpp
  )

  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})

  if(TARGET ${PROJECT_NAME}_test)
    ament_target_dependencies(${PROJECT_NAME}_test
      rclcpp
      rclcpp_action
      nav_msgs
      geometry_msgs
    )

    add_dependencies(${PROJECT_NAME}_test
      ${PROJECT_NAME}__rosidl_generator_cpp
      ${PROJECT_NAME}__rosidl_typesupport_cpp
    )
  endif()
endif()

ament_package()
