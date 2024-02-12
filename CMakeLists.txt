cmake_minimum_required(VERSION 3.8)
project(dbot_trajectory_player)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dbot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# add library
add_library(
  dbot_trajectory_server
  SHARED
  src/dbot_trajectory_server.cpp
)
target_include_directories(
  dbot_trajectory_server
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(
  dbot_trajectory_server
  PRIVATE
  "DBOT_TRAJECTORY_PLAYER_CPP_BUILDING_DLL"
)

ament_target_dependencies(
  dbot_trajectory_server
  "dbot_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
)

rclcpp_components_register_node(
  dbot_trajectory_server 
  PLUGIN 
  "dbot_trajectory_player::DbotTrajectoryActionServer" 
  EXECUTABLE 
  dbot_trajectory_action_server
)

install(TARGETS
  dbot_trajectory_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
