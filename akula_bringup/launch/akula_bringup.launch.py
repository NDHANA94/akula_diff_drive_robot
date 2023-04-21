'''  
=========================================
    * Author: nipun.dhananjaya@gmail.com
    * Created: 04.2023
=========================================
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnExecutionComplete, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # this_pkg_path = get_package_share_directory('akula_bringup')
    # slam_toolbox_path = get_package_share_directory('slam_toolbox')
    # akula_description_path = get_package_share_directory('akula_description')
    # velodyne_path = get_package_share_directory('velodyne')

    # tracer_base_bringup = ExecuteProcess(
    #     cmd=['ros2', 'run', 'akula_bringup', 'tracer_bringup.bash'],
    #     output='screen'
    # ) # tracer can connection and tracer_base node added as systemctr service to run on boot

    launch_akula_description = ExecuteProcess(
        cmd=['ros2', 'launch', 'akula_description', 'display_robot.launch.py'],
        output='screen'
    )

    launch_slam_toolbox = ExecuteProcess(
        cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py'],
        output='screen'
    )

    launch_velodyne= ExecuteProcess(
        cmd=['ros2', 'launch', 'velodyne', 'velodyne-all-nodes-VLP16-launch.py'],
        output='screen'
    )


    return LaunchDescription([
        launch_velodyne,
        launch_slam_toolbox,
        launch_akula_description,
    ])