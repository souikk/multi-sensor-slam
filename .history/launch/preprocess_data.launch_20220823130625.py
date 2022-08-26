'''
Author: Ke Zhang
Date: 2022-08-23 13:05:41
LastEditTime: 2022-08-23 13:06:24
Description: 
'''
from re import T
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='ky_slam',
             executable='ky_slam_rtk2msg',
             name='read_rtkdata',
             output='screen',
             parameters=[
                {"use_sim_time":True}
             ]
        )
    ])