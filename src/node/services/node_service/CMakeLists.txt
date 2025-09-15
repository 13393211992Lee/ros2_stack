cmake_minimum_required(VERSION 3.8)
project(node_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(node_param REQUIRED)

# component  start -----------------------
# 定义component 0: 包含头文件目录
include_directories(include)

# 定义component 1：依赖项
# find_package(rclcpp_components REQUIRED)

# 定义component 2： add_executable 替换为 add_library
add_library(add_two_ints_client_async_component SHARED src/add_two_ints_client_async.cpp) 
add_library(add_two_ints_server_component SHARED src/add_two_ints_server.cpp) 
add_library(introspection_client_lib SHARED src/introspection_client.cpp) 
add_library(introspection_service_lib SHARED src/introspection_service.cpp) 

# 定义component 3： 构建依赖
ament_target_dependencies(
  add_two_ints_client_async_component
  "rclcpp"
  "rclcpp_components" #新添加
  "example_interfaces"
)
ament_target_dependencies(
  add_two_ints_server_component
  "rclcpp"
  "rclcpp_components" #新添加
  "example_interfaces"
)
ament_target_dependencies(
  introspection_client_lib
  "rclcpp"
  "rclcpp_components" #新添加
  "example_interfaces"
  "rcl_interfaces"
)
ament_target_dependencies(
  introspection_service_lib
  "rclcpp"
  "rclcpp_components" #新添加
  "example_interfaces"
  "rcl_interfaces"
)

# 定义component 4： 声明组件
rclcpp_components_register_node(
  add_two_ints_client_async_component
  PLUGIN "node_service::ClientNode"
  EXECUTABLE add_two_ints_client_async  #$ ros2 run pkg EXECUTABLE
)
rclcpp_components_register_node(
  add_two_ints_server_component
  PLUGIN "node_service::ServerNode"
  EXECUTABLE add_two_ints_server  #$ ros2 run pkg EXECUTABLE
)
rclcpp_components_register_node(
  introspection_client_lib
  PLUGIN "node_service::IntrospectionClientNode"
  EXECUTABLE introspection_client  #$ ros2 run pkg EXECUTABLE
)
rclcpp_components_register_node(
  introspection_service_lib
  PLUGIN "node_service::IntrospectionServiceNode"
  EXECUTABLE introspection_service  #$ ros2 run pkg EXECUTABLE
)

# 定义component 5： CMake中作用于该目标的安装命令，改为库安装
ament_export_targets(export_add_two_ints_client_async_component)
install(TARGETS add_two_ints_client_async_component
  EXPORT export_add_two_ints_client_async_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_add_two_ints_server_component)
install(TARGETS add_two_ints_server_component
  EXPORT export_add_two_ints_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_introspection_client_lib)
install(TARGETS introspection_client_lib
  EXPORT export_introspection_client_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_introspection_service_lib)
install(TARGETS introspection_service_lib
  EXPORT export_introspection_service_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# component  end -----------------------




add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(
  add_two_ints_client
  "rclcpp"
  "example_interfaces"
)
install(TARGETS 
add_two_ints_client
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
