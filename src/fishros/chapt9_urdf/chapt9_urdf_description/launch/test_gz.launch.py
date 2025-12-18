import launch
import launch.launch_description_sources
import launch_ros
# 获取功能包下的share目录路径
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.parameter_descriptions

# 需求：
# 1. gz world
# 2. gz robot model

def generate_launch_description():  
    # pkg 路径
    urdf_pkg_path = get_package_share_directory('chapt9_urdf_description')
    default_world = os.path.join(urdf_pkg_path,'world','my5.sdf')

    #gz
    gz_sim_cmd = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                          'gz_args': ['-r ',default_world]}.items()
        # launch_arguments={'gz_args': ['-r ', world], 'on_exit_shutdown': 'true'}.items()
    )
    
    #spawn
    gz_spawn_model_launch_source = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_spawn_model.launch.py")
    start_ros_gz_sim_spawn_cmd =launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'my4_world',
            'topic': '/robot_description',
            'entity_name': 'fishbot_my',
            'z': '0.15',
            'use_sim_time': 'true',
        }.items(),
    ) 


    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        gz_sim_cmd,
        start_ros_gz_sim_spawn_cmd
    ])

