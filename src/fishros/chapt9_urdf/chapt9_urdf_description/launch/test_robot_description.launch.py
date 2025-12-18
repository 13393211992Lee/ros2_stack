# launch 包
from launch import LaunchDescription
from launch_ros.actions import Node
# 封装terminal 指令相关类------
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
# 封装参数声明 与 获取------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
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
from ros_gz_sim.actions import GzServer
import os

def generate_launch_description():

    declared_arguments = []
    node = []

    pkg_share = get_package_share_directory('chapt9_urdf_description')
    default_model_path = os.path.join(pkg_share, 'urdf','fishbot', 'fishbot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'display.rviz')

    # 参数声明 
    declared_arguments.append(
        DeclareLaunchArgument('model', default_value= default_model_path, description="需要加载robot的绝对路径") )
    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time', default_value= 'false', description="是否使用仿真时间") )
    declared_arguments.append(
        DeclareLaunchArgument('rvizconfig', default_value= default_rviz_config_path, description="rviz2 配置文件") )

    # 1.robot_state_publisher
    robot_state_publisher_node_ = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])
        }]
    )
    joint_state_publisher_node_ = Node(
        package='joint_state_publisher',  
        executable='joint_state_publisher')

    # 2. rivz2
    rviz_node_ = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='both',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

   
    node.append(robot_state_publisher_node_)
    # node.append(joint_state_publisher_node_)
    node.append(rviz_node_)
    return LaunchDescription(declared_arguments+ node)