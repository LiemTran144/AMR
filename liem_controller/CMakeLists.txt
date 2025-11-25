cmake_minimum_required(VERSION 3.8)
project(liem_controller)

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

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

set (path_follower_dependencies
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
)




add_executable(pd_motion_planner src/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner ${dependencies})
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



# 4. Cài đặt (Install rules) - Để lệnh 'ros2 run' tìm thấy node
install(TARGETS
  path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)









install(TARGETS
  pd_motion_planner
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
