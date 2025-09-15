cmake_minimum_required(VERSION 3.8)
project(basic_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(polygon_base REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(basic_interfaces REQUIRED)

# action 1 begin
# add_library(action_server SHARED
#   src/node6_action_server.cpp)
# target_include_directories(action_server PRIVATE
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)
# target_compile_definitions(action_server
#   PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")
# ament_target_dependencies(action_server 
#   rclcpp 
#   rclcpp_action 
#   rclcpp_components 
#   basic_interfaces
# )
# rclcpp_components_register_node(
#   action_server 
#   PLUGIN "basic_interfaces::FibonacciActionServer" 
#   EXECUTABLE node6_action_server
# ) 
#action end

# 映射源文件 与 设置目标依赖库
add_executable(demo src/demo.cpp)
ament_target_dependencies(demo rclcpp )

add_executable(node1 src/node1.cpp)
ament_target_dependencies(node1 rclcpp)

add_executable(node2 src/node2.cpp)
ament_target_dependencies(node2 rclcpp)

add_executable(node4_pub_msg1 src/node4_pub_msg1.cpp)
ament_target_dependencies(node4_pub_msg1 rclcpp basic_interfaces) #包含自定义interface

add_executable(node4_pub_msg2 src/node4_pub_msg2.cpp)
ament_target_dependencies(node4_pub_msg2 rclcpp basic_interfaces) #包含自定义interface

add_executable(node4_sub_msg1 src/node4_sub_msg1.cpp)
ament_target_dependencies(node4_sub_msg1 rclcpp basic_interfaces) #包含自定义interface

add_executable(node4_sub_msg2 src/node4_sub_msg2.cpp)
ament_target_dependencies(node4_sub_msg2 rclcpp basic_interfaces) #包含自定义interface

add_executable(node4_pub1 src/node4_pub1.cpp)
ament_target_dependencies(node4_pub1 rclcpp std_msgs)

add_executable(node4_pub2 src/node4_pub2.cpp)
ament_target_dependencies(node4_pub2 rclcpp std_msgs)

add_executable(node4_sub1 src/node4_sub1.cpp)
ament_target_dependencies(node4_sub1 rclcpp std_msgs)

add_executable(node4_sub2 src/node4_sub2.cpp)
ament_target_dependencies(node4_sub2 rclcpp std_msgs)

add_executable(node5_server1 src/node5_server1.cpp)
ament_target_dependencies(node5_server1 rclcpp example_interfaces)

add_executable(node5_client1 src/node5_client1.cpp)
ament_target_dependencies(node5_client1 rclcpp example_interfaces)

add_executable(node5_server2 src/node5_server2.cpp)
ament_target_dependencies(node5_server2 rclcpp example_interfaces)

add_executable(node5_client2 src/node5_client2.cpp)
ament_target_dependencies(node5_client2 rclcpp example_interfaces)

add_executable(node5_client_srv1 src/node5_client_srv1.cpp)
ament_target_dependencies(node5_client_srv1 rclcpp basic_interfaces)

add_executable(node5_server_srv1 src/node5_server_srv1.cpp)
ament_target_dependencies(node5_server_srv1 rclcpp basic_interfaces)

add_executable(node5_client_srv2 src/node5_client_srv2.cpp)
ament_target_dependencies(node5_client_srv2 rclcpp basic_interfaces)

add_executable(node5_server_srv2 src/node5_server_srv2.cpp)
ament_target_dependencies(node5_server_srv2 rclcpp basic_interfaces)

add_executable(node4_sub_msg3 src/node4_sub_msg3.cpp)
ament_target_dependencies(node4_sub_msg3 rclcpp basic_interfaces)

add_executable(node4_pub_param1 src/node4_pub_param1.cpp)
ament_target_dependencies(node4_pub_param1 rclcpp std_msgs)

add_executable(node4_pub_param2 src/node4_pub_param2.cpp)
ament_target_dependencies(node4_pub_param2 rclcpp)

add_executable(area_node src/area_node.cpp)
ament_target_dependencies(area_node  pluginlib polygon_base)

target_include_directories(demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


# 定义安装规则
install(TARGETS 
  demo
  node1
  node2
  node4_pub_msg1
  node4_pub_msg2
  node4_sub_msg1
  node4_sub_msg2
  node4_pub1
  node4_pub2
  node4_sub1
  node4_sub2
  node5_server1
  node5_client1
  node5_server2
  node5_client2
  node5_server_srv1
  node5_client_srv1
  node5_server_srv2
  node5_client_srv2
  node4_pub_param1
  node4_pub_param2
  area_node

  #action2 begin
  # node6_action_server
  # ARCHIVE DESTINATION lib
  # LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
  #action2 end 
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
  launch 
  # config rviz params
  #urdf  meshes worlds 
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
