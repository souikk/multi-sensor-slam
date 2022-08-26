'''
Author: Ke Zhang
Date: 2022-08-26 17:59:42
LastEditTime: 2022-08-26 17:59:56
Description: 
'''
from re import T
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    share_dir = get_package_share_directory('ky_slam')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            # parameters=[{"use_sim_time":True}]
        )
    ])

