cmake_minimum_required(VERSION 3.8)
project(chapt3_c)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

#-------------
# cmake_minimum_required(VERSION 3.8)
# project(chapt2)
add_executable(test_args1 src/test_args1.cpp)
#打印相关信息
# message("打印相关信息：")
# message(STATUS ${rclcpp_INCLUDE_DIRS})  #依赖的头文件
# message(STATUS ${rclcpp_LIBRARIES}) #.so库文件

# target_include_directories(test_args1 PUBLIC ${rclcpp_INCLUDE_DIRS}) #包含头文件连接
# target_link_libraries(test_args1 ${rclcpp_LIBRARIES})                 #库文件连接
# ament_target_dependencies相当于上述两条指令
ament_target_dependencies(test_args1 rclcpp)
#-----------------

add_executable(person src/person.cpp)
ament_target_dependencies(person rclcpp)
add_executable(test_makeshared src/test_makeshared.cpp)
add_executable(test_lambda src/test_lambda.cpp)
add_executable(test_lambda2 src/test_lambda2.cpp)
add_executable(test_function src/test_function.cpp)
add_executable(turtlesim_circle src/turtlesim_circle.cpp)
ament_target_dependencies(turtlesim_circle rclcpp geometry_msgs) 
# 从build 到install 中
install(TARGETS 
  person
  test_args1
  test_makeshared
  test_lambda
  test_lambda2
  test_function
  turtlesim_circle
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
