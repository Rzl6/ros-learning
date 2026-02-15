cmake_minimum_required(VERSION 3.8)
project(exe02_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(base_interfaces_demo REQUIRED)

add_executable(ex01_distance_server src/ex01_distance_server.cpp)
ament_target_dependencies(
  ex01_distance_server
  "rclcpp"
  "base_interfaces_demo"
  "turtlesim"
)

add_executable(ex02_distance_client src/ex02_distance_client.cpp)
ament_target_dependencies(
  ex02_distance_client
  "rclcpp"
  "base_interfaces_demo"
  "turtlesim"
)

install(TARGETS 
  ex01_distance_server
  ex02_distance_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
