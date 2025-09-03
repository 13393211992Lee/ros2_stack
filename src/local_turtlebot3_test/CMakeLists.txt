cmake_minimum_required(VERSION 3.8)
project(local_turtlebot3_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

install(DIRECTORY
  launch config rviz params
  #urdf  meshes worlds 
  DESTINATION share/${PROJECT_NAME}  
)


# 编译节点
add_executable(test1 src/test1.cpp)
add_executable(turtlebot3_move0 src/turtlebot3_move0.cpp)
add_executable(turtlebot3_move1 src/turtlebot3_move1.cpp)
add_executable(turtlebot3_move2 src/turtlebot3_move2.cpp)
add_executable(turtlebot3_move3 src/turtlebot3_move3.cpp)
add_executable(turtlebot3_move4 src/turtlebot3_move4.cpp)
add_executable(turtlebot3_move5 src/turtlebot3_move5.cpp)
add_executable(basic1_executors src/basic1_executors.cpp)
add_executable(basic1_executors0 src/basic1_executors0.cpp)
add_executable(turtlebot3_sensor_scan0 src/turtlebot3_sensor_scan0.cpp)
add_executable(turtlebot3_sensor_scan1 src/turtlebot3_sensor_scan1.cpp)
add_executable(turtlebot3_sensor_odom0 src/turtlebot3_sensor_odom0.cpp)
add_executable(turtlebot3_sensor_odom1 src/turtlebot3_sensor_odom1.cpp)
add_executable(turtlebot3_sensor_odom2 src/turtlebot3_sensor_odom2.cpp)
add_executable(turtlebot3_sensor_imu src/turtlebot3_sensor_imu.cpp)
add_executable(turtlebot3_sensor_imu2 src/turtlebot3_sensor_imu2.cpp)
add_executable(turtlebot3_sensor_imu3 src/turtlebot3_sensor_imu3.cpp)
add_executable(turtlebot3_sensor_camera1 src/turtlebot3_sensor_camera1.cpp)
add_executable(turtlebot3_sensor_camera2 src/turtlebot3_sensor_camera2.cpp)
add_executable(turtlebot3_sensor_camera3 src/turtlebot3_sensor_camera3.cpp)
add_executable(turtlebot3_sensor_camera4 src/turtlebot3_sensor_camera4.cpp)
add_executable(turtlebot3_sensor_camera5 src/turtlebot3_sensor_camera5.cpp)
add_executable(turtlebot3_sensor_fusion2 src/turtlebot3_sensor_fusion2.cpp)
add_executable(turtlebot3_sensor_fusion4 src/turtlebot3_sensor_fusion4.cpp)

target_include_directories(test1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test1 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# 节点依赖
ament_target_dependencies(test1
  "rclcpp"
)
ament_target_dependencies(turtlebot3_move0
  rclcpp
  geometry_msgs
  std_srvs
)
ament_target_dependencies(turtlebot3_move1
  rclcpp
  geometry_msgs
  std_srvs
)
ament_target_dependencies(turtlebot3_move2
  rclcpp
  geometry_msgs
  std_srvs
)
ament_target_dependencies(turtlebot3_move3
  rclcpp
  geometry_msgs
)
ament_target_dependencies(turtlebot3_move4
  rclcpp
  geometry_msgs
)
ament_target_dependencies(turtlebot3_move5
  rclcpp
  geometry_msgs
  nav_msgs
)
ament_target_dependencies(basic1_executors
  rclcpp
  std_msgs
)
ament_target_dependencies(basic1_executors0
  rclcpp
  std_msgs
)
ament_target_dependencies(turtlebot3_sensor_scan0
  rclcpp
  sensor_msgs
)
ament_target_dependencies(turtlebot3_sensor_scan1
  rclcpp
  sensor_msgs
)
ament_target_dependencies(turtlebot3_sensor_odom0
  rclcpp
  nav_msgs
  tf2 
  tf2_ros
)
ament_target_dependencies(turtlebot3_sensor_odom1
  rclcpp
  nav_msgs
  geometry_msgs
  tf2 
  tf2_ros
)
ament_target_dependencies(turtlebot3_sensor_odom2
  rclcpp
  nav_msgs
  geometry_msgs
  tf2 
  tf2_ros
)
ament_target_dependencies(turtlebot3_sensor_imu
  rclcpp
  sensor_msgs
)
ament_target_dependencies(turtlebot3_sensor_imu2
  rclcpp
  sensor_msgs
)
ament_target_dependencies(turtlebot3_sensor_imu3
  rclcpp
  sensor_msgs
  tf2 
  tf2_ros
)

ament_target_dependencies(turtlebot3_sensor_camera1
  rclcpp
  sensor_msgs
)
ament_target_dependencies(turtlebot3_sensor_camera2
  rclcpp
  sensor_msgs
  message_filters
)
ament_target_dependencies(turtlebot3_sensor_camera3
  rclcpp
  sensor_msgs
  message_filters
  cv_bridge
  # opencv2
)
ament_target_dependencies(turtlebot3_sensor_camera4
  rclcpp
  sensor_msgs
  cv_bridge
)
ament_target_dependencies(turtlebot3_sensor_camera5
  rclcpp
  sensor_msgs
  cv_bridge
)
ament_target_dependencies(turtlebot3_sensor_fusion2
  rclcpp
  sensor_msgs
  nav_msgs
  message_filters
  tf2
)
ament_target_dependencies(turtlebot3_sensor_fusion4
  rclcpp
  sensor_msgs
  nav_msgs
  message_filters
  tf2
)
# 新增：链接OpenCV库
target_link_libraries(turtlebot3_sensor_camera3 ${OpenCV_LIBS})
target_link_libraries(turtlebot3_sensor_camera4 ${OpenCV_LIBS})
target_link_libraries(turtlebot3_sensor_camera5 ${OpenCV_LIBS})
# 安装可执行文件
install(TARGETS 
  test1
  turtlebot3_move0
  turtlebot3_move1
  turtlebot3_move2
  turtlebot3_move3
  turtlebot3_move4
  turtlebot3_move5
  basic1_executors
  basic1_executors0
  turtlebot3_sensor_scan0
  turtlebot3_sensor_scan1
  turtlebot3_sensor_odom0
  turtlebot3_sensor_odom1
  turtlebot3_sensor_odom2
  turtlebot3_sensor_imu
  turtlebot3_sensor_imu2
  turtlebot3_sensor_imu3
  turtlebot3_sensor_camera1
  turtlebot3_sensor_camera2
  turtlebot3_sensor_camera3
  turtlebot3_sensor_camera4
  turtlebot3_sensor_camera5
  turtlebot3_sensor_fusion2
  turtlebot3_sensor_fusion4
  DESTINATION lib/${PROJECT_NAME})

add_compile_options(-Wall -Wextra -Wpedantic -g)  # 添加 -g 启用调试符号
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
