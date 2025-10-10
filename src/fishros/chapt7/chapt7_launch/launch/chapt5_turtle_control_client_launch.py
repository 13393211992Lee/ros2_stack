import launch
import launch_ros

def generate_launch_description():
    turtle_control_client_ = launch_ros.actions.Node(
        package='chapt5_turtle_server',
        executable='turtle_control_client',
        name='turtle_control_client',
        # output='screen' | 'log' | 'both'
        output='screen')
    return launch.LaunchDescription([turtle_control_client_])