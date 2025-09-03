import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = []
    node4_pub_param2 = Node(
        package='basic_ros2',
        executable='node4_pub_param2',
        name='node4_pub_param2',
        parameters=[{'my_parameter': 'earth'}],
        output='screen')
    node = [node4_pub_param2]
    return LaunchDescription(declared_arguments+ node)
