'''  
=========================================
    * Author: nipun.dhananjaya@gmail.com
    * Created: 04.02.2023
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
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    this_pkg = get_package_share_directory('akula_ign_sim')
    akula_description_pkg = get_package_share_directory('akula_description')
    rviz_config_file = os.path.join(this_pkg, 'rviz', 'default.rviz')
    # --------- xacro process -------------------------------------------------------------------------
    xacro_file = os.path.join(akula_description_pkg, 'urdf/akula.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time':True}
    # -------------------------------------------------------------------------------------------------

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params])

    ign_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(), #'-name', 'Akula',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0.2'])
    
    pointcloud2scan = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in',  'vlp16_sim_scan/points'),
                    ('scan', '/scan')],
        parameters=[{
            'target_frame': 'vlp16_scan',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.1415927410125732,  # -M_PI
            'angle_max': 3.1415927410125732,  # M_PI
            'angle_increment': 0.0033528204075992107,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.7,
            'range_max': 131.0,
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


    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'tracer_controller'],
        output='screen'
    )

    # sim_lidar_static_tf_pub = ExecuteProcess(
    #     cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', "0", "0", "0", "0", "0", "0", "vlp16_scan", "VLP16"],
    #     output='screen'
    # )
    real_lidar_static_tf_pub = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', "0", "0", "0", "0", "0", "0", "vlp16_scan", "velodyne"], #x_rot 1.57079632679
        output='screen'
    )

    odom_static_tf_pub = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', "0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
        output='screen'
    )
    
    #  time bridge
    clock_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # imu bridge
    imu_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
        output='screen'
    )

    # camera bridge
    camera_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image[ignition.msgs.Image'],
        output='screen'
    )

    # camera bridge
    camera_info_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
        output='screen'
    )

    

    # use time
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # lidar ros2 bridge
    bridge_lidar = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/vlp16_sim_scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
        output='screen'
    )

    bridge_lidar_points = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/vlp16_sim_scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'],
        output='screen'
    )

    run_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        # launch ignition gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', ' -r -v 4 empty.sdf')]
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ign_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        clock_bridge,
        node_robot_state_publisher,
        ign_spawn_entity,
        # sim_lidar_static_tf_pub,
        real_lidar_static_tf_pub,
        odom_static_tf_pub,
        bridge_lidar, 
        bridge_lidar_points,
        imu_bridge,
        camera_bridge, camera_info_bridge,
        pointcloud2scan,
        run_rviz2,

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        ),

        launch_slam_toolbox,
        ])
