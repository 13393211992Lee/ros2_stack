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
import launch
import launch_ros
import os

def generate_launch_description():

    declared_arguments = []
    node = []

    default_world = os.path.join(get_package_share_directory('chapt9_urdf_description'),'world','my4.sdf')
    default_sdf_path = os.path.join(get_package_share_directory('chapt9_urdf_description'),'urdf','fishbot','fishbot.sdf')
    gzserver_cmd = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [default_world]}.items()
    )
    start_ros_gz_sim_spawn_cmd = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='spawn_entity',
        arguments=[
            '--name', 'cc',
            '--sdf_filename', default_sdf_path
        ],
        output='screen',
    ) 
    node.append(gzserver_cmd)
    node.append(start_ros_gz_sim_spawn_cmd)
    return LaunchDescription(declared_arguments+ node)