cmake_minimum_required(VERSION 3.8)
project(ros_basic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)



# Install your Python scripts
install(
  PROGRAMS

  ${PROJECT_NAME}/ros_publisher.py  
  ${PROJECT_NAME}/ros_subscriber.py 
  ${PROJECT_NAME}/move_robot.py
  
  ${PROJECT_NAME}/parameter.py
  ${PROJECT_NAME}/server.py
  ${PROJECT_NAME}/client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  param

  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
