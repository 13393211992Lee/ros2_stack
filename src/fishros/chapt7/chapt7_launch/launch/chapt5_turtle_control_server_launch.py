import launch
import launch_ros

def generate_launch_description():
    turtle_control_server_ = launch_ros.actions.Node(
        package='chapt5_turtle_server',
        executable='turtle_control_server',
        name='turtle_control_server',
        output='screen')
    return launch.LaunchDescription([turtle_control_server_])