'''
Author: Ke Zhang
Date: 2022-08-26 17:56:27
LastEditTime: 2022-08-26 17:59:06
Description: 
'''
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

use_sim_time_ = True

def generate_launch_description():
    

    preprocessData = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('localization'), 'launch'),
            '/preprocess_data.launch.py']),
    )

    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('localization'), 'launch'),
            '/rviz2.launch.py']),
    )

    return  LaunchDescription([
        preprocessData,
        rviz2,
    ])