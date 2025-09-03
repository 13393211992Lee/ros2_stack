import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取包路径
    turtlebot3_gazebo_dir = FindPackageShare(package='turtlebot3_gazebo').find('turtlebot3_gazebo')
    cartographer_dir = FindPackageShare(package='cartographer_ros').find('cartographer_ros')
    my_package_dir = FindPackageShare(package='local_turtlebot3_test').find('local_turtlebot3_test')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params_file = LaunchConfiguration('slam_params_file')
    print("***** slam_params_file ***** ", slam_params_file)
    # 定义参数
    params = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [my_package_dir, 'config', 'turtlebot3_2d.lua']),
            description='Full path to the ROS2 parameters file to use')
    ]

    # 节点
    nodes = [
        # 启动Gazebo空世界
        # $ ros2 launch turtlebot3_gazebo empty_world.launch.py use_sim_time:=true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        
        # 传感器预处理节点
        # $ ros2 run local_turtlebot3_test turtlebot3_sensor_fusion4 --ros-args -p use_sim_time:=true
        Node(
            package='local_turtlebot3_test',
            executable='turtlebot3_sensor_fusion4',
            name='turtlebot3_sensor_fusion4',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # Cartographer节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', PathJoinSubstitution([my_package_dir, 'config']),
                '-configuration_basename', 'turtlebot3_2d.lua'
            ],
            remappings=[
                ('scan', 'filtered_scan'),
                ('imu', 'filtered_imu'),
                ('odom', 'synchronized_odom')
            ]
        ),
        
        # Cartographer occupancy grid节点
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-resolution', '0.05',
                '-publish_period_sec', '1.0'
            ]
        ),
        
        # RViz2可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', PathJoinSubstitution([my_package_dir, 'rviz', 'cartographer.rviz'])]
        )
    ]

    return LaunchDescription(params + nodes)
