cmake_minimum_required(VERSION 3.8)
project(liem_controller)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_controller REQUIRED)

# Common dependencies
set(common_dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Path follower specific dependencies
set(path_follower_dependencies
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# Nav2 planner dependencies
set(nav2_dependencies
  rclcpp
  nav2_core
  nav2_controller
  nav2_util
  nav2_costmap_2d
  nav2_regulated_pure_pursuit_controller
  angles
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(pd_control_remake src/pd_control_remake.cpp)
ament_target_dependencies(pd_control_remake ${common_dependencies})
target_include_directories(
  pd_control_remake PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(pd_motion_planner src/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner ${common_dependencies})
target_include_directories(
  pd_motion_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(path_follower_node src/path_follower_client.cpp)
ament_target_dependencies(path_follower_node ${path_follower_dependencies})
target_include_directories(
  path_follower_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Dynamic Window Pure Pursuit Controller
add_library(nav2_dynamic_window_pure_pursuit_controller SHARED src/nav2_dynamic_window_pure_pursuit_controller.cpp)
ament_target_dependencies(nav2_dynamic_window_pure_pursuit_controller ${nav2_dependencies})
target_include_directories(nav2_dynamic_window_pure_pursuit_controller
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(nav2_core nav2_dynamic_window_pure_pursuit_controller.xml)

# DWA Controller
add_library(dwa_controller SHARED src/dwa_controller.cpp)
ament_target_dependencies(dwa_controller ${nav2_dependencies})
target_include_directories(dwa_controller
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(nav2_core dwa_controller.xml)

# Rotation Shim Controller
add_library(liem_rotation_shim_controller SHARED src/nav2_rotation_shim_controller.cpp)
ament_target_dependencies(liem_rotation_shim_controller ${nav2_dependencies})
target_include_directories(liem_rotation_shim_controller
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(nav2_core liem_rotation_shim_controller.xml)

install(TARGETS
  pd_control_remake
  pd_motion_planner
  path_follower_node
  nav2_dynamic_window_pure_pursuit_controller
  dwa_controller
  liem_rotation_shim_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()