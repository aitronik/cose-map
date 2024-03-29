from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    rviz_config_file = os.path.join(get_package_share_directory('cosemap_launch'), 'config', 'rviz', 'cosemap.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config', 
            default_value=rviz_config_file,
            description='Path to the RViz config file'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],  # Se necessario
            arguments=['-d', LaunchConfiguration('rviz_config')]
        )
    ])