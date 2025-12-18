import launch
import launch.launch_description_sources
import launch_ros
# 获取功能包下的share目录路径
from ament_index_python.packages import get_package_share_directory
import os

import launch_ros.parameter_descriptions
def generate_launch_description():
    # pkg 路径
    urdf_pkg_path = get_package_share_directory('chapt9_urdf_description')
    default_world = os.path.join(urdf_pkg_path,'world','my5.sdf')
    default_xacro_path = os.path.join(urdf_pkg_path,"urdf","fishbot","fishbot_urdf.xacro")
    default_rviz_config_path = os.path.join(urdf_pkg_path,"config","display.rviz")

    # 参数声明
    declare_model = launch.actions.DeclareLaunchArgument(
        name= 'model',
        default_value= str(default_xacro_path),
        description="加载的模型文件路径"
    )
    # 1. 解析： robot_description
    # 通过文件路径，获取文件内容，并转为参数对象，传入robot_description
    substitutions_commond_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    robot_description_value =  launch_ros.parameter_descriptions.ParameterValue(substitutions_commond_result,value_type=str)

    # 2. 根据 robot_description 发布 robot_state_publisher
    action_robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description_value}],
    )

    #3. gz
    gz_sim_cmd = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [default_world]}.items()
        # launch_arguments={'gz_args': ['-r ', world], 'on_exit_shutdown': 'true'}.items()
    )
    # 3.1 gz_server
    gz_server_cmd = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_server.launch.py')
        ),
        launch_arguments={'world_sdf_file': [default_world]}.items()  
    )
    
    #4. spawn
    gz_spawn_model_launch_source = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_spawn_model.launch.py")
    start_ros_gz_sim_spawn_cmd =launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'my4_world',
            'topic': '/robot_description',
            'entity_name': 'fishbot_my',
            'z': '0.15',
        }.items(),
    ) 


    # 4.rviz2
    # 等同于： $ ros2 run rviz2 rviz2 -d /home/zhang/ws_github_nav2/src/fishros/chapt9_urdf/chapt9_urdf_description/config/display.rviz
    action_rviz2 = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d',[default_rviz_config_path]],
    )
    return launch.LaunchDescription([
        declare_model,
        action_robot_state_publisher,
        # gz_sim_cmd,
        gz_server_cmd,
        start_ros_gz_sim_spawn_cmd,
        action_rviz2
    ])

"""
urdf: src/fishros/chapt9_urdf/chapt9_urdf_description/urdf/t1.urdf
cat  src/fishros/chapt9_urdf/chapt9_urdf_description/urdf/t1.urdf

rivz2 = 
ros2 run rviz2 rviz2
"""