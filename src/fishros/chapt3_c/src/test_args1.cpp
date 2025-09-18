#include "rclcpp/rclcpp.hpp"
/*
<<  m  ,  >>
*/
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node  = std::make_shared<rclcpp::Node>("cpp_node");
    RCLCPP_INFO(node->get_logger(), "你好 c++ node");
    rclcpp::spin(node); //循环处理node节点
    rclcpp::shutdown();
    return 0;
}

/*
错误：
$ g++ test_args1.cpp
test_args1.cpp:1:10: fatal error: rclcpp/rclcpp.hpp: 没有那个文件或目录
    1 | #include "rclcpp/rclcpp.hpp"
      |          ^~~~~~~~~~~~~~~~~~~

问题： #include<iostream> 可以被找到 而 #include "rclcpp/rclcpp.hpp" 不可以被找到的原因是什么？
#include<iostream> 系统库
#include "rclcpp/rclcpp.hpp" 不在系统库

找库 用cmakelist更方便
编写CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(chapt2)
add_executable(test_args1 test_args1.cpp)

编译：$ cmake .
执行： make
错误信息： 也是
    1 | #include "rclcpp/rclcpp.hpp"
      |          ^~~~~~~~~~~~~~~~~~~


*/