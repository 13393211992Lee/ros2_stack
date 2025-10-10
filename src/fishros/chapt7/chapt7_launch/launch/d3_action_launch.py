import launch
import launch.launch_description_sources
import launch_ros
import os
from ament_index_python import  get_package_share_directory
def generate_launch_description():
    action_declare_start_rqt = launch.actions.DeclareLaunchArgument("start_rqt",default_value="False")
    start_rqt = launch.substitutions.LaunchConfiguration("start_rqt",default="False")
    # 拼接路径的两种方式：
    multisim_path = os.path.join(get_package_share_directory('turtlesim'),'launch', 'multisim.launch.py')
    # multisim_path = [get_package_share_directory('turtlesim'),'/launch', '/multisim.launch.py']

    # action1: 启动其他launch
    multisim = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            multisim_path
        )
    )

    # action2: 打印数据
    action_loginfo = launch.actions.LogInfo(msg=str(multisim_path))

    # action3 执行一个命令行
    action_topic_list = launch.actions.ExecuteProcess(
        condition = launch.conditions.IfCondition(start_rqt),
        cmd = ['rqt']   # 一般用于启动 rqt gazebo rviz2
        # cmd = ['ros2','topic','list'] #
    )

    # action4: group
    action_group = launch.actions.GroupAction([
        # action5: 定时器
        launch.actions.TimerAction(period=2.0,actions=[multisim]),          # 2s 启动 multisim
        launch.actions.TimerAction(period=4.0,actions=[action_topic_list])  # 4s 启动 action_topic_list
    ])
    return launch.LaunchDescription([
        action_declare_start_rqt,
        action_loginfo,
        action_group
    ])

"""
$ ros2 launch chapt7_launch d3_action_launch.py start_rqt:=False    #不启动rqt
$ ros2 launch chapt7_launch d3_action_launch.py start_rqt:=True     #启动rqt

"""