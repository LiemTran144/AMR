cmake_minimum_required(VERSION 3.8)
project(differential_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  PROGRAMS

  ${PROJECT_NAME}/zlac8015d.py  
  ${PROJECT_NAME}/differentialDrive.py
  ${PROJECT_NAME}/speed_control.py
  ${PROJECT_NAME}/odom.py
  ${PROJECT_NAME}/waypoints_generator.py
  ${PROJECT_NAME}/path_rviz.py
  ${PROJECT_NAME}/motion_control.py
  ${PROJECT_NAME}/pure_pursuit.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
