'''
Author: Ke Zhang
Date: 2022-08-23 13:05:41
LastEditTime: 2022-08-23 16:11:28
Description: 
'''
from re import T
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='localization',
             executable='preprocess_data',
             name='preprocess_data',
             output='screen',
             parameters=[
                {"use_sim_time":True}
             ]
        )
    ])