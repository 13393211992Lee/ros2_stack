from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取各个包的路径
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # 启动Gazebo & TurtleBot3
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )
    
    # 启动SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(slam_toolbox_dir, 'config', 'mapper_params_online_sync.yaml'),
            {'use_sim_time': True}  # 使用仿真时间
        ]
    )
    #  rviz2 -d /opt/ros/jazzy/share/slam_toolbox/config/slam_toolbox_default.rviz
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d','/opt/ros/jazzy/share/slam_toolbox/config/slam_toolbox_default.rviz' ],
        output='screen')

    return LaunchDescription([
        gazebo_launch,
        slam_node,
        rviz2
    ])
