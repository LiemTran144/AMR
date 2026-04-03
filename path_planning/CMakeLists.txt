cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)   
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

set(nav2_dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  nav2_msgs
  rclcpp_action
)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

# a-star planner executable
add_executable(a_star_planner_costmap src/a_star_planner_costmap.cpp)
ament_target_dependencies(a_star_planner_costmap ${dependencies})
target_include_directories(a_star_planner_costmap
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

# compute path client executable
add_executable(compute_path_client src/compute_path_client.cpp)
ament_target_dependencies(compute_path_client 
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)
target_include_directories(compute_path_client
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

install(TARGETS compute_path_client
  DESTINATION lib/${PROJECT_NAME})


# nav2 planner libraries / plugins
add_library(a_star_nav2_planner SHARED src/a_star_nav2_planner.cpp)
ament_target_dependencies(a_star_nav2_planner ${nav2_dependencies})
target_include_directories(a_star_nav2_planner
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

add_library(a_star_nav2_smoother SHARED src/a_star_nav2_smoother.cpp)
ament_target_dependencies(a_star_nav2_smoother ${nav2_dependencies})
target_include_directories(a_star_nav2_smoother
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

add_library(a_star_nav2_report SHARED src/a_star_report.cpp)
ament_target_dependencies(a_star_nav2_report ${nav2_dependencies})
target_include_directories(a_star_nav2_report
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)


pluginlib_export_plugin_description_file(nav2_core global_planner_plugins.xml)

install(TARGETS
  a_star_planner_costmap
  a_star_nav2_planner
  a_star_nav2_smoother
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

# make sure plugin XML is installed so pluginlib can find it
install(
  FILES global_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install (PROGRAMS
  path_planning/a_star_path.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp nav_msgs geometry_msgs tf2_ros nav2_core nav2_costmap_2d nav2_util pluginlib rclcpp_action nav2_msgs)
ament_export_libraries(a_star_nav2_planner a_star_nav2_smoother)
ament_package()
