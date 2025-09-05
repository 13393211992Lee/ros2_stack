cmake_minimum_required(VERSION 3.8)
project(composition4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
#include
include_directories(include)
set(node_plugins "")

# talker_component
add_library(talker_component SHARED
  src/talker_component.cpp)
target_compile_definitions(talker_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(talker_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${std_msgs_TARGETS})
rclcpp_components_register_nodes(talker_component "composition::Talker")
set(node_plugins "${node_plugins}composition::Talker;$<TARGET_FILE:talker_component>\n")

#listener_component
add_library(listener_component SHARED
  src/listener_component.cpp)
target_compile_definitions(listener_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(listener_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${std_msgs_TARGETS})
rclcpp_components_register_nodes(listener_component "composition::Talker")
set(node_plugins "${node_plugins}composition::Talker;$<TARGET_FILE:talker_component>\n")


install(TARGETS
  talker_component
  listener_component  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
# manual_composition
add_executable(manual_composition
  src/manual_composition.cpp)
target_link_libraries(manual_composition PRIVATE
  talker_component
  listener_component
  rclcpp::rclcpp)
install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME})








# listener_composition2 直接运行
add_executable(listener_composition2
  src/listener_composition2.cpp)
ament_target_dependencies(listener_composition2
  std_msgs
  rclcpp
)
install(TARGETS
  listener_composition2
  DESTINATION lib/${PROJECT_NAME})


#---------------listener_component3 -----start------
# 定义component 1：依赖项
# find_package(rclcpp_components REQUIRED)

# 定义component 2： add_executable 替换为 add_library
add_library(listener_component3 SHARED src/listener_component3.cpp) 

# 定义component 3： 构建依赖
ament_target_dependencies(
  listener_component3
  "rclcpp"
  "std_msgs"
  "rclcpp_components" #新添加
)
# 定义component 4： 声明组件
rclcpp_components_register_node(
  listener_component3
  PLUGIN "composition::Listener3"
  EXECUTABLE listener3 #$ ros2 run composition4 listener3

)
# 定义component 5： CMake中作用于该目标的安装命令，改为库安装
ament_export_targets(export_listener_component3)
install(TARGETS listener_component3
  EXPORT export_listener_component3
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
#---------------listener_component3 -----end------



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
