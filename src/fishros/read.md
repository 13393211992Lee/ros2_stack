
笔记： <<  m  ,  >>  ->
-----------------------------
### chapt2
~/ws_github_nav2/src/fishros/chapt2
# 编译
$ g++ ./test_args.cpp  
# 执行 
$ ./a.out --help        
$ ./a.out 
*/

-----------------------------
错误信息 和 解决：
$ g++ test_args1.cpp
test_args1.cpp:1:10: fatal error: rclcpp/rclcpp.hpp: 没有那个文件或目录
    1 | #include "rclcpp/rclcpp.hpp"
      |          ^~~~~~~~~~~~~~~~~~~


-----------------------------
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


-----------------------------
CMakeLists.txt ： 1.输出信息    2.库文件
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


-----------------------------
## 查阅可执行文件中库的链接情况.

$ ldd  可执行文件
$ ldd test_args1
	linux-vdso.so.1 (0x00007ffd5e9f7000)
	librclcpp.so => /opt/ros/jazzy/lib/librclcpp.so (0x00007b9b24000000)
	librcutils.so => /opt/ros/jazzy/lib/librcutils.so (0x00007b9b2440e000)
	libstdc++.so.6 => /lib/x86_64-linux-gnu/libstdc++.so.6 (0x00007b9b23c00000)
	libgcc_s.so.1 => /lib/x86_64-linux-gnu/libgcc_s.so.1 (0x00007b9b243a8000)
    ...


# cakelist.txt 中install的作用
build 到install/lib


# 多个功能包的最佳实践
通过依赖关系 控制 colcon build 编译pkg的顺序


# ros2  常用到的c++ 特性
## auto
auto 自动退到类型
auto ptr1 = std::make_shared<std::string>("this is a string");
std::shared_ptr<std::string> ptr1 = std::make_shared<std::string>("this is a string");
auto 自动推导出类型为：std::shared_ptr<std::string>

## 智能指针
ptr.use_count() 计数指的是 该指针指向内存地址 该地址被引用的次数

## lambda表达式 匿名函数特性
参考： test_lambda1 | test_lambda2
[capture list](parameters) -> return_type{
  function body
}
捕获列表（capture list）
(parameters)  参数列表
-> return_type 返回值类型 可以省略 会自动推导的
{} 函数主体

## 函数包装器
test_function.cpp

## 多线程和回调函数

## turtlesim 线性速度到点位置
turtlesim_point.cpp
订阅turtlesi  的位置
实现距离远 速度快
导航到该点

## chapt4 接口
status_interface
时间相关：
ros2 interface list | grep Time
builtin_interfaces/msg/Time
sensor_msgs/msg/TimeReference


## msg 是如何通过
rosidl_interface_packages
rosidl_default_generators
编成hpp文件的


## 发布:statu_publisher

## Qt


# chapt5  人脸识别4.2.2 -5
# 服务和参数
安装
python3 -m venv myenv
source myenv/bin/activate
pip3 install face-recognition-models -i https://pypi.tuna.tsinghua.edu.cn/simple --trusted-host pypi.tuna.tsinghua.edu.cn
pip3 install face_recognition -i https://pypi.tuna.tsinghua.edu.cn/simple --trusted-host pypi.tuna.tsinghua.edu.cn

查看recognition 包：
激活虚拟环境：
  source myenv/bin/activate
查找：
  $ pip list | grep recognition
  face-recognition                     1.3.0
  face_recognition_models              0.3.0


## 自定义服务接口
### srv文件：
  float32 target_x        # target
  float32 target_y
  ---
  int8 SUCCESS = 1        # 定义表示成功的常量
  int8 FAIL = 0           # 定义表示失败的常量
  int8 result             # 处理结果

### CMakeLists.txt
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/Patrol.srv"
    DEPENDENCIES geometry_msgs
  )

### Package.xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>


# chapt6  人脸识别4.4.2 -3
参数更新事件

# chapt7 launch

# chapt8 tf

# chapt9 robot_description
