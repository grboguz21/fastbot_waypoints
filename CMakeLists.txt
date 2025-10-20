cmake_minimum_required(VERSION 3.5)
project(fastbot_waypoints)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# Eğer istersen: find_package(rosidl_default_runtime REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 1) Action arayüzlerini ÜRET (executable’dan ÖNCE)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Waypoint.action"
  DEPENDENCIES geometry_msgs
)

# 2) Executable
add_executable(fastbot_action_server src/fastbot_action_server.cpp)
ament_target_dependencies(fastbot_action_server
  rclcpp rclcpp_action geometry_msgs nav_msgs tf2
)

# 3) Build sırası bağımlılıkları
add_dependencies(fastbot_action_server
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp   # ← typesupport hazır olsun
)

# 4) Include path (build-time jenerik başlıklar)
target_include_directories(fastbot_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

# 5) Doğrudan typesupport kütüphanesini linkle (makro kullanmadan)
target_link_libraries(fastbot_action_server
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

install(TARGETS fastbot_action_server
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
