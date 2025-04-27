from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_attach_shelf = get_package_share_directory('attach_shelf')

    return LaunchDescription([
        DeclareLaunchArgument('obstacle', default_value='0.0'),
        DeclareLaunchArgument('degrees', default_value='0'),
        DeclareLaunchArgument('final_approach', default_value='true'),

        Node(
            package='attach_shelf',
            executable='approach_service_server',
            name='approach_service_server',
            output='screen'
        ),

        Node(
            package='attach_shelf',
            executable='pre_approach_v2',
            name='pre_approach_v2',
            output='screen',
            parameters=[
                {'obstacle': LaunchConfiguration('obstacle')},
                {'degrees': LaunchConfiguration('degrees')},
                {'final_approach': LaunchConfiguration('final_approach')}
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_attach_shelf, 'rviz', 'config.rviz')],
            output='screen'
        )
    ])