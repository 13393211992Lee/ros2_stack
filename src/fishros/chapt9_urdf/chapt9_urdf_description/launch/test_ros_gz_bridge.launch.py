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
import os

def generate_launch_description():

    declared_arguments = []
    node = []
    pkg_share = get_package_share_directory('chapt9_urdf_description')
    default_model_path = os.path.join(pkg_share, 'urdf','fishbot', 'fishbot.urdf.xacro')

    declared_arguments.append(
        DeclareLaunchArgument('model', default_value= default_model_path, description="需要加载robot的绝对路径") )
    
    ros_gz_bridge_launch_source = os.path.join(
        get_package_share_directory("ros_gz_bridge"), "launch", "ros_gz_bridge.launch.py")
    ros_gz_bridge_launch_config = os.path.join(
        get_package_share_directory("chapt9_urdf_description"), "config", "ros_gz_bridge.yaml")
    start_ros_gz_sim_spawn_cmd =IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_gz_bridge_launch_source),
        launch_arguments={
            'bridge_name': 'ros_gz_bridge',
            'config_file': ros_gz_bridge_launch_config
        }.items()
    ) 
    # 手动启动 controller_manager （ros2_control 节点）
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
            os.path.join(get_package_share_directory("chapt9_urdf_description"),
                        'config/diff_drive_controller.yaml'),
            {'use_sim_time': True}
        ],
        remappings=[('/controller_manager/robot_description', '/robot_description')]
    )
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}]
    )
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}]
    )
    declared_arguments.append(start_ros_gz_sim_spawn_cmd)
    declared_arguments.append(controller_manager)
    declared_arguments.append(joint_state_spawner)
    declared_arguments.append(diff_drive_spawner)
    return LaunchDescription(declared_arguments+ node)