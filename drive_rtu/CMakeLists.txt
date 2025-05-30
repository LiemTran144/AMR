cmake_minimum_required(VERSION 3.8)
project(drive_rtu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  PROGRAMS

  ${PROJECT_NAME}/zlac8015d.py  
  ${PROJECT_NAME}/FeedbackNode.py
  ${PROJECT_NAME}/pub_control.py
  ${PROJECT_NAME}/positionNode.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
