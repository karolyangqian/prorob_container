cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(aruco_msg REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(aruco_docking_controller src/aruco_docking_controller.cpp)
ament_target_dependencies(aruco_docking_controller
  rclcpp
  geometry_msgs
  aruco_msg
  aruco_opencv_msgs
  tf2
)

install(TARGETS
  aruco_docking_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
