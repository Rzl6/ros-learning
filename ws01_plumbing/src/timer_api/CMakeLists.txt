cmake_minimum_required(VERSION 3.8)
project(timer_api)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(rate_api src/rate_api.cpp)
target_include_directories(rate_api PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rate_api PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  rate_api
  "rclcpp"
)

add_executable(time_api src/time_api.cpp)
target_include_directories(time_api PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(time_api PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  time_api
  "rclcpp"
)

add_executable(duration_api src/duration_api.cpp)
target_include_directories(duration_api PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(duration_api PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  duration_api
  "rclcpp"
)

add_executable(time_duration_calc src/time_duration_calc.cpp)
target_include_directories(time_duration_calc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(time_duration_calc PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  time_duration_calc
  "rclcpp"
)

install(TARGETS 
  rate_api
  time_api
  duration_api
  time_duration_calc
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
