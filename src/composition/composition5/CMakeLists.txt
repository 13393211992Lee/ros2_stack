cmake_minimum_required(VERSION 3.8)
project(composition5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(simple_node src/simple_node.cpp)
target_include_directories(simple_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(simple_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  simple_node
  "rclcpp"
)

add_executable(simple_node2 src/simple_node2.cpp)
ament_target_dependencies(
  simple_node2
  "rclcpp"
)

add_executable(simple_node3 src/simple_node3.cpp)
ament_target_dependencies(
  simple_node3
  "rclcpp"
)

add_executable(simple_node4 src/simple_node4.cpp)
ament_target_dependencies(
  simple_node4
  "rclcpp"
  "rclcpp_lifecycle"
)

add_executable(simple_node5 src/simple_node5.cpp)
ament_target_dependencies(
  simple_node5
  "rclcpp"
  "rclcpp_lifecycle"
)

install(TARGETS
simple_node
simple_node2
simple_node3
simple_node4
simple_node5
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
