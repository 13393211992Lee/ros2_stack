cmake_minimum_required(VERSION 3.8)
project(custom_action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(custom_action_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)

#fibonacci_action_server
add_library(action_server SHARED
  src/fibonacci_action_server.cpp
)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(action_server
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "custom_action_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "custom_action_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
 
  
#fibonacci_action_client
add_library(action_client SHARED                            # 创建一个名为action_client共享库
  src/fibonacci_action_client.cpp)                          # 指定源代码文件：src/fibonacci_action_client.cpp
target_include_directories(action_client 
  PRIVATE                                                   # 表示这些路径仅用于当前库的编译，不传递给依赖它的其他目标。
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>    # 编译阶段的头文件路径
  $<INSTALL_INTERFACE:include>)                             # 安装后的头文件路径，确保安装后其他程序能找到头文件。
target_compile_definitions(action_client                    # 添加编译宏定义，影响源码中的条件编译如（#ifdef CUSTOM_ACTION_CPP_BUILDING_DLL）。
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(action_client                     # 依赖管理
  "custom_action_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client               # action_client：指定要注册的库
  PLUGIN   "custom_action_cpp::FibonacciActionClient"       # 指定要注册的节点类（Action 客户端的实现类）。
  EXECUTABLE fibonacci_action_client          # 同时生成一个可执行文件fibonacci_action_client，用于直接运行该节点（无需动态加载）。
)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib       # 静态库（.a）安装到lib目录。
  LIBRARY DESTINATION lib       # 共享库（.so/.dll）安装到lib目录。
  RUNTIME DESTINATION bin)      # 可执行文件安装到bin目录


# fibonacci_action_server2
add_executable(fibonacci_action_server2 src/fibonacci_action_server2.cpp)
ament_target_dependencies(fibonacci_action_server2
  "rclcpp"
  "rclcpp_action"
  "custom_action_interfaces")

# fibonacci_action_client2
add_executable(fibonacci_action_client2 src/fibonacci_action_client2.cpp)
ament_target_dependencies(fibonacci_action_client2
  "rclcpp"
  "rclcpp_action"
  "custom_action_interfaces")

# member_functions_server
add_executable(member_functions_server src/member_functions_server.cpp)
ament_target_dependencies(member_functions_server
  "rclcpp"
  "rclcpp_action"
  "example_interfaces")

# member_functions_client
add_executable(member_functions_client src/member_functions_client.cpp)
  ament_target_dependencies(member_functions_client
    "rclcpp"
    "rclcpp_action"
    "example_interfaces")

install(TARGETS
  fibonacci_action_server2
  fibonacci_action_client2
  member_functions_server
  member_functions_client
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
