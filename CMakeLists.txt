cmake_minimum_required(VERSION 3.8)
project(dbot_trajectory_player)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dbot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# add library for server
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
  "moveit"
  "moveit_ros_planning_interface"
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

# add library for client
add_library(
  dbot_trajectory_client
  SHARED
  src/dbot_trajectory_client.cpp
)
target_include_directories(
  dbot_trajectory_client
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(
  dbot_trajectory_client
  PRIVATE
  "DBOT_TRAJECTORY_PLAYER_CPP_BUILDING_DLL"
)

ament_target_dependencies(
  dbot_trajectory_client
  "dbot_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit"
  "moveit_ros_planning_interface"
)

rclcpp_components_register_node(
  dbot_trajectory_client 
  PLUGIN 
  "dbot_trajectory_player::DbotTrajectoryActionClient" 
  EXECUTABLE 
  dbot_trajectory_action_client
)

install(TARGETS
  dbot_trajectory_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
