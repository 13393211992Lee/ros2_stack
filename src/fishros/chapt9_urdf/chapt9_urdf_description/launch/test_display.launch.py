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
    twist_converter_launch_source = os.path.join(pkg_share, "launch", "twist_converter.launch.py")       # 
    default_model_path = os.path.join(pkg_share, 'urdf','fishbot', 'fishbot.urdf.xacro')               # ? fishbot.sdf
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'display.rviz')    # 
    world_path = os.path.join(pkg_share, 'world', 'my5.sdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')

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

    twist_converter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(twist_converter_launch_source),
        launch_arguments={
            'input_topic': '/cmd_vel',
            'output_topic': '/cmd_vel_twist',
        }.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{'world': 'my5_world',
                    'topic': '/robot_description',
                    'name': 'fishbot',
                    'z': 0.65,
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                    }],
    )
    # 该 controller 加载时机 在 gz启动 spawn_entity 之后
    action_load_joint_states_controller = ExecuteProcess(
        cmd='ros2 control load_controller fishbot_joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
    )
    action_load_effort_controller = ExecuteProcess(
        cmd='ros2 control load_controller fishbot_effort_controller --set-state active'.split(' '),
        output='screen'
    )
    action_load_diff_drive_controller= ExecuteProcess(
        cmd='ros2 control load_controller fishbot_diff_drive_controller --set-state active'.split(' '),
        output='screen'
    )

    reg_action_load_joint_states_controller =  RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action= spawn_entity,
            on_exit=[action_load_joint_states_controller],
        )
    )
    reg_action_load_effort_controller =  RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action= spawn_entity,
            on_exit=[action_load_effort_controller],
        )
    )
    reg_action_load_diff_drive_controller =  RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action= spawn_entity,
            on_exit=[action_load_diff_drive_controller],
        )
    )
    node.append(robot_state_publisher_node)
    # node.append(rviz_node)
    node.append(gz_server)
    node.append(ros_gz_bridge)
    node.append(spawn_entity)
    node.append(reg_action_load_joint_states_controller)
    #node.append(reg_action_load_effort_controller)
    node.append(reg_action_load_diff_drive_controller)
    node.append(twist_converter_launch)
    

    

    
    return LaunchDescription(declared_arguments+ node)