import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # รัน server_node
        launch_ros.actions.Node(
            package='robot_control',
            executable='server_node',
            name='server_node',
            output='screen'
        ),

        # รัน service_node
        launch_ros.actions.Node(
            package='robot_control',
            executable='service_node',
            name='service_node',
            output='screen'
        ),
    ])
