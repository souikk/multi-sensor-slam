'''
Author: Ke Zhang
Date: 2022-08-26 17:56:27
LastEditTime: 2022-08-26 17:56:34
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