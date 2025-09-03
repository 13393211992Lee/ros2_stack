from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 启动TurtleBot3 
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'launch', 'empty_world.launch.py')
        )
    )

    # 2. 启动EKF融合节点
    params_file = os.path.join(
        get_package_share_directory('local_turtlebot3_test'),  
        'config', 
        'ekf.yaml'
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        # parameters=[params_file],
        parameters=[
            os.path.join(get_package_share_directory('local_turtlebot3_test'),  
                         'config', 'ekf.yaml')
        ]
    )

    return LaunchDescription([
        # gazebo_launch,
        ekf_node
    ])
