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
    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time',default_value = 'True')
    )
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', os.path.join(get_package_share_directory("local_turtlebot3_test"),"config"),
            '-configuration_basename', 'cartographer_demo.lua'
        ],
        output='screen')

    grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': True},
                    {'resolution': 0.05}],
        output='screen')
    node = [grid_node ,cartographer_node]

    return LaunchDescription(declared_arguments+ node)

'''
构建地图
1. 启动turtlebot3
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
2. 启动  cartographer
ros2 launch local_turtlebot3_test cartographer_demo.launch.py 
3.rviz2
4. turtlebot3 的控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard  --ros-args -p stamped:=true
5.保存地图
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot31_map

'''