cmake_minimum_required(VERSION 3.8)
project(differential_drive)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(
  PROGRAMS
  ${PROJECT_NAME}/pd_control.py
  ${PROJECT_NAME}/differentialDrive.py
  ${PROJECT_NAME}/speed_control.py
  ${PROJECT_NAME}/odom.py
  ${PROJECT_NAME}/waypoints_generator.py
  ${PROJECT_NAME}/pd_control.py
  ${PROJECT_NAME}/pure_pursuit.py
  ${PROJECT_NAME}/fake_amcl.py
  ${PROJECT_NAME}/check_costmap.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()