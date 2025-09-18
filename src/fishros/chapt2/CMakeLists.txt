cmake_minimum_required(VERSION 3.8)
project(chapt2)
add_executable(test_args1 test_args1.cpp)
find_package(rclcpp REQUIRED)
#打印相关信息
# message("打印相关信息：")
# message(STATUS ${rclcpp_INCLUDE_DIRS})  #依赖的头文件
# message(STATUS ${rclcpp_LIBRARIES}) #.so库文件

target_include_directories(test_args1 PUBLIC ${rclcpp_INCLUDE_DIRS}) #包含头文件连接
target_link_libraries(test_args1 ${rclcpp_LIBRARIES})                 #库文件连接
