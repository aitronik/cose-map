import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    config_file_path = os.path.join(get_package_share_directory('cosemap_launch'), 'config', 'cosemap.yaml')
    
    semantic_pcl_cmd = Node(
                    package='semantic_pcl',
                    executable='semantic_pcl_node',
                    name='semantic_pcl_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[config_file_path])

    octomap_server_cmd = Node(
                    package='octomap_server',
                    executable='semantic_octomap_server_node',
                    name='semantic_octomap_server_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[config_file_path])
    
    semantic_database_cmd = Node(
                    package='semantic_database',
                    executable='semantic_database_node',
                    name='semantic_database_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[config_file_path])
    
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(semantic_pcl_cmd)
    ld.add_action(octomap_server_cmd)
    ld.add_action(semantic_database_cmd) 
    
    return ld
