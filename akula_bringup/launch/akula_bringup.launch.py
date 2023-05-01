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
    
    this_pkg = get_package_share_directory('akula_bringup')
    # slam_toolbox_path = get_package_share_directory('slam_toolbox')
    # akula_description_path = get_package_share_directory('akula_description')
    # velodyne_path = get_package_share_directory('velodyne')

    # tracer_base_bringup = ExecuteProcess(
    #     cmd=['ros2', 'run', 'akula_bringup', 'tracer_bringup.bash'],
    #     output='screen'
    # ) # tracer CAN connection and tracer_base node added as systemctr service to run on boot

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

    pointcloud2scan = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in',  'velodyne_points'),
                    ('scan', '/scan2')],
        parameters=[{
            'target_frame': 'vlp16_scan',
            'transform_tolerance': 0.01,
            'min_height': -0.65,
            'max_height': 1.0,
            'angle_min': -3.1415927410125732,  # -M_PI
            'angle_max': 3.1415927410125732,  # M_PI
            'angle_increment': 0.007000000216066837,  # M_PI/360.0
            'scan_time': 0.0, #0.3333
            'range_min': 0.3,
            'range_max': 200.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

    params_file = os.path.join(this_pkg, 'config', 'mapper_params_online_async.yaml')
    launch_slam_toolbox = ExecuteProcess(
        cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', 'params_file:='+params_file],
        output='screen'
    )

    return LaunchDescription([
        launch_velodyne,
        launch_slam_toolbox,
        launch_akula_description,
        pointcloud2scan,
    ])