cmake_minimum_required(VERSION 3.8)
project(my_lifecycle_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)  # 🟢 Thêm dòng này

add_executable(lifecycle_talker src/lifecycle_talker.cpp)
ament_target_dependencies(lifecycle_talker
  rclcpp
  rclcpp_lifecycle
  std_msgs        # 🟢 Thêm dependency std_msgs
)

install(TARGETS lifecycle_talker
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
