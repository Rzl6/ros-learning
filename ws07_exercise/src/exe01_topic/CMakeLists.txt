cmake_minimum_required(VERSION 3.8)
project(exe01_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(ex01_mirror_control src/ex01_mirror_control.cpp)
ament_target_dependencies(
  ex01_mirror_control
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "turtlesim"
)

add_executable(ex02_mirror_control src/ex02_mirror_control.cpp)
ament_target_dependencies(
  ex02_mirror_control
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "turtlesim"
  "rclcpp_action"
)

install(TARGETS 
  ex01_mirror_control
  ex02_mirror_control
  DESTINATION lib/${PROJECT_NAME})

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
