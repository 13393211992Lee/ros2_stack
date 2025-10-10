import launch
import launch_ros

def generate_launch_description():
    turtle_control = launch_ros.actions.Node(
        package='chapt5_turtle_server',
        executable='turtle_control',
        name='turtle_control',
        output='screen')
    return launch.LaunchDescription([turtle_control])