# launch 包
from launch import LaunchDescription
from launch_ros.actions import Node ,ComposableNodeContainer
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer
# 封装terminal 指令相关类------
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
# 封装参数声明 与 获取------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,Command
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

    pkg_share = get_package_share_directory('chapt9_urdf_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py") # ?
    default_model_path = os.path.join(pkg_share, 'urdf','fishbot', 'fishbot2.urdf.xacro')               # ? fishbot.sdf
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'display.rviz')
    world_path = os.path.join(pkg_share, 'world', 'my5.sdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'ros_gz_bridge2.yaml')

    declared_arguments = []
    declare_model = DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file')
    declare_rvizconfig = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file')
    declare_use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time')
    declare_gz_sim = ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen')   # gz gui
    declared_arguments.append(declare_model)
    declared_arguments.append(declare_rvizconfig)
    declared_arguments.append(declare_use_sim_time)
    declared_arguments.append(declare_gz_sim)

    node = []


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, 
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='both',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        # 新增：启用 RViz 的仿真时钟
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )

    # spawn_entity = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
    #     launch_arguments={
    #         'world': 'my5_world',
    #         'topic': '/robot_description',
    #         'entity_name': 'fishbot',
    #         'z': '0.65',
    #     }.items(),
    # )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'world': 'my5_world',
                    'topic': '/robot_description',
                    'name': 'fishbot',
                    'z': 0.65,
                    }],
    )
   
    node.append(robot_state_publisher_node)
    node.append(rviz_node)
    node.append(gz_server)
    node.append(ros_gz_bridge)
    node.append(spawn_entity)
    
    return LaunchDescription(declared_arguments+ node)