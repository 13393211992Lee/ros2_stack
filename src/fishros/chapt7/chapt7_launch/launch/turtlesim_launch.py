import launch
import launch_ros

def generate_launch_description():
    turtle = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen')
    return launch.LaunchDescription([turtle])