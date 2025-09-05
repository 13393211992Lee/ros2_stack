cmake_minimum_required(VERSION 3.8)
project(composition2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(domo src/domo.cpp)
target_include_directories(domo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(domo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  domo
  "rclcpp"
)

add_executable(vincent_driver1 src/vincent_driver1.cpp)
ament_target_dependencies(
  vincent_driver1
  "rclcpp"
)

# 定义component 1：依赖项
# find_package(rclcpp_components REQUIRED)

# 定义component 2： add_executable 替换为 add_library
add_library(vincent_driver2_component SHARED src/vincent_driver2.cpp) 

# 定义component 3： 构建依赖
ament_target_dependencies(
  vincent_driver2_component
  "rclcpp"
  "rclcpp_components" #新添加
)
# 定义component 4： 声明组件
rclcpp_components_register_node(
  vincent_driver2_component
  PLUGIN "ns::VincentDriver"
  EXECUTABLE vincent_driver2  #$ ros2 run composition2 vincent_driver2

)
# 定义component 5： CMake中作用于该目标的安装命令，改为库安装
ament_export_targets(export_vincent_driver2_component)
install(TARGETS vincent_driver2_component
  EXPORT export_vincent_driver2_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS domo
vincent_driver1
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch 
  #config rviz params
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
