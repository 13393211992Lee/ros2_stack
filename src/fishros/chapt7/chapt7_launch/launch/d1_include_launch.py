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

    declared_arguments = []
    node = []

    include_turtlesim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('chapt7_launch'),
            'launch', 'turtlesim_launch.py')))
    
    include_turtle_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('chapt7_launch'),
            'launch', 'chapt5_turtle_control_server_launch.py')))
    
    include_turtle_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('chapt7_launch'),
            'launch', 'chapt5_turtle_control_client_launch.py')))
    

    node.append(include_turtlesim)
    node.append(include_turtle_server)
    node.append(include_turtle_client)

    return LaunchDescription(declared_arguments+ node)