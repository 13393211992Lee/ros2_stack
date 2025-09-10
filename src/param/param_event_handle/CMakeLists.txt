cmake_minimum_required(VERSION 3.8)
project(param_event_handle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(parameter_event_handler src/parameter_event_handler.cpp)
ament_target_dependencies(
  parameter_event_handler
  "rclcpp"
)

add_executable(parameter_event_handler2 src/parameter_event_handler2.cpp)
ament_target_dependencies(
  parameter_event_handler2
  "rclcpp"
)

add_executable(mult_params_event_handler1 src/mult_params_event_handler1.cpp)
ament_target_dependencies(
  mult_params_event_handler1
  "rclcpp"
)

add_executable(mult_params_event_handler2 src/mult_params_event_handler2.cpp)
ament_target_dependencies(
  mult_params_event_handler2
  "rclcpp"
)

install(TARGETS 
parameter_event_handler
parameter_event_handler2
mult_params_event_handler1
mult_params_event_handler2
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
