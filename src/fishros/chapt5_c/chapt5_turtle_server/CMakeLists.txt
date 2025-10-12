cmake_minimum_required(VERSION 3.8)
project(chapt5_turtle_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(chapt5_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtle_control src/turtle_control.cpp)
target_include_directories(turtle_control PRIVATE include)
ament_target_dependencies(turtle_control rclcpp turtlesim chapt5_interface geometry_msgs)

add_executable(turtle_control_server src/turtle_control_server.cpp)
target_include_directories(turtle_control_server PRIVATE include)
ament_target_dependencies(turtle_control_server rclcpp turtlesim chapt5_interface geometry_msgs)

add_executable(turtle_control_client src/turtle_control_client.cpp)
target_include_directories(turtle_control_client PRIVATE include)
ament_target_dependencies(turtle_control_client rclcpp chapt5_interface geometry_msgs)

install(TARGETS 
  turtle_control
  turtle_control_server
  turtle_control_client
  DESTINATION lib/${PROJECT_NAME}
)
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
