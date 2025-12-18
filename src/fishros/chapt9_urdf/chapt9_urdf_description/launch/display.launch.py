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
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    default_model_path = os.path.join(pkg_share, 'urdf','fishbot', 'fishbot_urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'display.rviz')
    world_path = os.path.join(pkg_share,'world','my5.sdf')
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
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, 
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    joint_state_publisher_node_ = Node(
        package='joint_state_publisher',  # 带GUI的版本，调试更直观
        executable='joint_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}  # 与仿真时间同步（若启用）
            # 显式指定需要发布的关节（可选，避免遗漏车轮关节）
            # {'joints': ['wheel_left_joint', 'wheel_right_joint']}
        ]
    )

    # 2. rivz2
    rviz_node_ = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='both',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    #3. gz server
    gz_server_ = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )
    # 4. spawn
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_pkg, "launch", "gz_spawn_model.launch.py")
    spawn_entity_ = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'my4_world',
            'topic': '/robot_description',
            'entity_name': 'fish_bot',
            'x': '2.00',
            'z': '0.65',
        }.items(),
    )
    node.append(robot_state_publisher_node_)
    # node.append(joint_state_publisher_node_)
    node.append(rviz_node_)
    node.append(gz_server_)
    node.append(spawn_entity_)
    return LaunchDescription(declared_arguments+ node)