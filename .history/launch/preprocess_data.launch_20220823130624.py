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