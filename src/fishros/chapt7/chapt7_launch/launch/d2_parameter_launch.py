# launch 包
from launch import LaunchDescription
from launch_ros.actions import Node
# 封装terminal 指令相关类------
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
# 封装参数声明 与 获取------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关
from launch_ros.actions import PushROSNamespace
from launch.actions import GroupAction
# 事件相关
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
# 获取功能包下的share目录路径
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # declare_1 = DeclareLaunchArgument(name="bkgd_g",default_value="0")
    turtle = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        parameters=[{'background_g':LaunchConfiguration('bkgd_g',default= "1")}],   # default= "1" 
        output='screen')
    # return LaunchDescription([declare_1,turtle])
    return LaunchDescription([turtle])

    # 注意 declare 参数的顺序，先声明参数，后加载
    # return LaunchDescription([turtle,declare_1])
"""
关于 代码中的两个默认值：
1. --->  default_value="0"

如果 终端启动传递参数： 
    eg: $ ros2 launch chapt7_launch d1_parameter_launch.py bkgd_g:=233, 
    $ ros2 param dump turtlesim
        /turtlesim:
        ros__parameters:
            background_b: 255
            background_g: 233       # 系统将 采用终端传递的参数bkgd_g:=233, 而不是 default_value="0"
            background_r: 69

如果 终端没有传递参数： $ ros2 launch chapt7_launch d1_parameter_launch.py
    系统将 采用 default_value="0"


2. --->  parameters=[{'background_g':LaunchConfiguration('bkgd_g',default= "1")}],   # default= "1" 
如果 终端启动时没有传入参数 eg:ros2 launch chapt7_launch d1_parameter_launch.py 
且在 没有声明bkgd_g  eg: 注释掉了: declare_1 = DeclareLaunchArgument(name="bkgd_g",default_value="0")
的情况下， 才会启用 default= "1" 。

ros2 launch chapt7_launch d1_parameter_launch.py  # 没有传递参数bkgd_g
launch 文件中没有声明bkgd_g参数  # declare_1 = DeclareLaunchArgument(name="bkgd_g",default_value="0")
才会启用 default= "1" 

总结：
参数的优先级:  
$ ros2 launch chapt7_launch d1_parameter_launch.py bkgd_g:=233, --------> 最高优先级
declare_1 = DeclareLaunchArgument(name="bkgd_g",default_value="0") -------> 其次
parameters=[{'background_g':LaunchConfiguration('bkgd_g',default= "1")}],   # default= "1" -------> 优先级最低
"""



"""
$ ros2 launch chapt7_launch d1_parameter_launch.py 
$ ros2 param dump turtlesim
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 0         #修改
    background_r: 69
    holonomic: false
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    start_type_description_service: true
    use_sim_time: false

"""


"""
# 指令 参数
$ ros2 run turtlesim turtlesim_node --ros-args --param background_g:=1
$ ros2 param dump turtlesim
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 1             # 修改
    background_r: 69
    holonomic: false
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    start_type_description_service: true
    use_sim_time: false

"""