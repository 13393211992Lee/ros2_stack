import launch
import launch_ros
# 获取功能包下的share目录路径
from ament_index_python.packages import get_package_share_directory
import os

import launch_ros.parameter_descriptions
def generate_launch_description():
    # declared_arguments = []
    # node = []
    # return launch.LaunchDescription(declared_arguments+ node)

    # urdf 路径
    urdf_pkg_path = get_package_share_directory('chapt9_urdf_description')
    default_urdf_path = os.path.join(urdf_pkg_path,"urdf","t1.urdf")
    default_rviz_config_path = os.path.join(urdf_pkg_path,"config","display.rviz")

    # 参数声明
    declare_model = launch.actions.DeclareLaunchArgument(
        name= 'model',
        default_value= str(default_urdf_path),
        description="加载的模型文件路径"
    )

    # 参数的获取

    # 通过文件路径，获取文件内容，并转为参数对象，传入robot_description
    substitutions_commond_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    robot_description_value =  launch_ros.parameter_descriptions.ParameterValue(substitutions_commond_result,value_type=str)

    #  打印 路径 信息
    loginfo_urdf_pkg_path = launch.actions.LogInfo(msg=str(urdf_pkg_path))
    loginfo_default_urdf_path = launch.actions.LogInfo(msg=str(default_urdf_path))

    action_robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description_value}],

    )
    action_joint_state_publisher = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )
    
    # 等同于： $ ros2 run rviz2 rviz2 -d /home/zhang/ws_github_nav2/src/
    # fishros/chapt9_urdf/chapt9_urdf_description/config/display.rviz
    action_rviz2 = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d',[default_rviz_config_path]],
    )
    return launch.LaunchDescription([
        declare_model,
        loginfo_urdf_pkg_path,
        loginfo_default_urdf_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz2
    ])

"""
urdf: src/fishros/chapt9_urdf/chapt9_urdf_description/urdf/t1.urdf
cat  src/fishros/chapt9_urdf/chapt9_urdf_description/urdf/t1.urdf

带参数启动
$ ros2 launch chapt9_urdf_description display_t1_launch.py model:='/home/zhang/ws_github_nav2/src/fishros/chapt9_urdf/chapt9_urdf_description/urdf/fishbot/fishbot2.urdf.xacro'

rivz2 = 
ros2 run rviz2 rviz2
"""