'''  
=========================================
    * Author: nipun.dhananjaya@gmail.com
    * Created: 04.2023
=========================================
'''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    this_pkg = get_package_share_directory('akula_description')
    rviz_config_file = os.path.join(this_pkg, 'rviz', 'default.rviz')

    # ---------  xacro process ------------------------------------------
    xacro_file = os.path.join(this_pkg, 'urdf', 'akula.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time':True}
    # -------------------------------------------------------------------

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params])
    
    joint_state_publisher_gui= Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )
    joint_state_publisher= Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    run_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')
    
    sim_lidar_static_tf_pub = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', "0", "0", "0", "-1.57079632679", "0", "0", "vlp16_scan", "Akula/base_link/velodyne-VLP16"], #Akula/base_link/velodyne-VLP16
        output='screen'
    )

    real_lidar_static_tf_pub = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', "0", "0", "0", "0", "0", "0", "vlp16_scan", "velodyne"],
        output='screen'
    )

    odom_static_tf_pub = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', "0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher,
        sim_lidar_static_tf_pub,
        real_lidar_static_tf_pub,
        # odom_static_tf_pub,
        run_rviz2
    ])
    
    
