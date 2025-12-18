cmake_minimum_required(VERSION 3.8)
project(chapt9_urdf_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(geometry_msgs REQUIRED)

# 添加可执行文件
add_executable(twist_converter src/twist_converter.cpp)
# 链接依赖
ament_target_dependencies(twist_converter
  rclcpp
  geometry_msgs
)
# 安装
install(TARGETS twist_converter
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch urdf config world
  DESTINATION share/${PROJECT_NAME}
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
