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

    run_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui,
        run_rviz2
    ])
    
    
