import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),

        # Node for RPLidar (sllidar_ros2)
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('/scan', '/scan_rplidar')]
        ),

        # # Node for Turtlebot (turtlebot3)
        # Node(
        #     package='turtlebot3_node',
        #     executable='turtlebot3_node',
        #     name='turtlebot_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        #     remappings=[('/cmd_vel', '/cmd_vel')]
        # ),

        # Node for Service
        Node(
            package='service_node',
            executable='service_node',
            name='service_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Node for Server
        Node(
            package='server_node',
            executable='server_node',
            name='server_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        LogInfo(
            condition=launch.substitutions.LaunchConfiguration('use_sim_time'),
            msg="Launching all nodes!"
        )
    ])
