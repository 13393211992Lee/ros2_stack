cmake_minimum_required(VERSION 3.8)
project(node_param)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)

# component  start -----------------------
# 定义component 0: 包含头文件目录
include_directories(include)

# 定义component 1：依赖项
# find_package(rclcpp_components REQUIRED)

# 定义component 2： add_executable 替换为 add_library
add_library(parameter_blackboard_component SHARED src/parameter_blackboard.cpp) 
add_library(list_parameters_component SHARED src/list_parameters.cpp) 
add_library(even_parameters_node_component SHARED src/even_parameters_node.cpp) 
add_library(parameter_events_async_component SHARED src/parameter_events_async.cpp) 
add_library(set_and_get_parameters_component SHARED src/set_and_get_parameters.cpp) 
add_library(set_parameters_callback_component SHARED src/set_and_get_parameters.cpp) 

# 定义component 3： 构建依赖
ament_target_dependencies(
  parameter_blackboard_component
  "rclcpp"
  "rclcpp_components" #新添加
)
ament_target_dependencies(
  list_parameters_component
  "rclcpp"
  "rclcpp_components" 
)
ament_target_dependencies(
  even_parameters_node_component
  "rclcpp"
  "rclcpp_components" 
)
ament_target_dependencies(
  parameter_events_async_component
  "rclcpp"
  "rclcpp_components" 
)
ament_target_dependencies(
  set_and_get_parameters_component
  "rclcpp"
  "rclcpp_components" 
)
ament_target_dependencies(
  set_parameters_callback_component
  "rclcpp"
  "rclcpp_components" 
  "rcl_interfaces"
)

# 定义component 4： 声明组件
rclcpp_components_register_node(
  parameter_blackboard_component
  PLUGIN "node_param::ParameterBlackboard"
  EXECUTABLE parameter_blackboard  #$ ros2 run pkg EXECUTABLE
)
rclcpp_components_register_node(
  list_parameters_component
  PLUGIN "node_param::ListParameters"
  EXECUTABLE list_parameters  #$ ros2 run pkg EXECUTABLE
)
rclcpp_components_register_node(
  even_parameters_node_component
  PLUGIN "node_param::EvenParameterNode"
  EXECUTABLE even_parameters_node  #$ ros2 run pkg EXECUTABLE
)
rclcpp_components_register_node(
  parameter_events_async_component
  PLUGIN "node_param::ParameterEventsAsyncNode"
  EXECUTABLE parameter_events_async  #$ ros2 run pkg EXECUTABLE
)
rclcpp_components_register_node(
  set_and_get_parameters_component
  PLUGIN "node_param::SetAndGetParameters"
  EXECUTABLE set_and_get_parameters  #$ ros2 run pkg EXECUTABLE
)
rclcpp_components_register_node(
  set_parameters_callback_component
  PLUGIN "node_param::SetParametersCallback"
  EXECUTABLE set_parameters_callback  #$ ros2 run pkg EXECUTABLE
)

# 定义component 5： CMake中作用于该目标的安装命令，改为库安装
ament_export_targets(export_parameter_blackboard_component)
install(TARGETS parameter_blackboard_component
  EXPORT export_parameter_blackboard_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_list_parameters_component)
install(TARGETS list_parameters_component
  EXPORT export_list_parameters_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_even_parameters_node_component)
install(TARGETS even_parameters_node_component
  EXPORT export_even_parameters_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_parameter_events_async_component)
install(TARGETS parameter_events_async_component
  EXPORT export_parameter_events_async_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_set_and_get_parameters_component)
install(TARGETS set_and_get_parameters_component
  EXPORT export_set_and_get_parameters_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_set_parameters_callback_component)
install(TARGETS set_parameters_callback_component
  EXPORT export_set_parameters_callback_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# component  end -----------------------

# execute start -----------
add_executable(list_parameters_async src/list_parameters_async.cpp)
add_executable(parameter_event_handler src/parameter_event_handler.cpp)
add_executable(parameter_events src/parameter_events.cpp)
add_executable(set_and_get_parameters_async src/set_and_get_parameters_async.cpp)
ament_target_dependencies(
  list_parameters_async
  "rclcpp"
)
ament_target_dependencies(
  parameter_event_handler
  "rclcpp"
  "rcl_interfaces"
)
ament_target_dependencies(
  parameter_events
  "rclcpp"
  "rcl_interfaces"
)
ament_target_dependencies(
  set_and_get_parameters_async
  "rclcpp"
)
install(TARGETS 
  list_parameters_async
  parameter_event_handler
  parameter_events
  set_and_get_parameters_async
  DESTINATION lib/${PROJECT_NAME}
)


# execute end -----------


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
