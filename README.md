# Akula ROS2 
Akula (Shark) robot is based on Tracer differential drive robot. 

This package is created and tested only with `ros2-foxy` version!

![image](images/Akula_rviz.png)

### Included sensors in ign simulation:
* IMU sensor
* Camera sensor
* Velodyne VLP16 LIDAR sensor

### Install dependencies:
* Install ros2-foxy: https://docs.ros.org/en/foxy/Installation.html
* Install ignition fortress: https://gazebosim.org/docs/fortress/install_ubuntu
* Install ign-ros2-control: 
    `apt install ros-$ROS_DISTRO-ign-ros2-control`
* Install ros-ign-gazebo: 
    `sudo apt install ros-$ROS_DISTRO-ros-ign-gazebo`
* Install nav2: 
    `sudo apt install ros-$ROS_DISTRO-nav2-*`


### Clone necessary ROS2 packages to akula_ws:
* Clone `akula_diff_drive_robot` pkgs:
```
mkdir -p akula_ws/src
cd akula_ws/src
git clone https://github.com/NDHANA94/akula_diff_drive_robot.git    
```
* Clone `gz_ros2_control` pkg:
```
git clone --branch foxy https://github.com/ros-controls/gz_ros2_control.git ign_ros2_control
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
echo "export IGNITION_VERSION=fortress" >> ~/.bashrc
```

* Clone `Velodyne `ros2 pkgs:
```
git clone --branch foxy-devel https://github.com/ros-drivers/velodyne.git
```

* Clone `slam_toolbox` pkg:
```
git clone --branch foxy-devel https://github.com/SteveMacenski/slam_toolbox.git
```
* Clone `pointcloud_to_laserscan`:
```
git clone --branch foxy https://github.com/ros-perception/pointcloud_to_laserscan.git
```

* Clone `tracer_ros2` pkgs:
```
git clone https://github.com/agilexrobotics/tracer_ros2.git
git clone https://github.com/westonrobot/ugv_sdk.git
```

### Configurations:
* Velodyne_driver VLP-16 : `/velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml`
```
velodyne_driver_node:
    ros__parameters:
        device_ip: 192.168.88.201
        gps_time: false
        time_offset: 0.0
        enabled: true
        read_once: false
        read_fast: false
        repeat_delay: 0.0
        frame_id: velodyne
        model: VLP16
        rpm: 600.0
        port: 2368
```
* Velodyne_pointcloud: `/velodyne_pointcloud/configVLP16-velodyne_convert_node-params.yaml` <br>
 and `/velodyne_pointcloud/config/VLP16-velodyne_transform_node-params.yaml`
```
velodyne_convert_node:
    ros__parameters:
        calibration: VLP16db.yaml
        min_range: 0.3
        max_range: 130.0
        view_direction: 0.0
        organize_cloud: true
```


### Build the pkgs:
```
cd ..
colcon build
```

### launch Ignition gazebo simulation with rviz2
```
source install/setup.bash
ros2 launch akula_ign_sim akula_ros2.launch.py
```

### controlling Akula robot via teleop_twist_keyboard
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/tracer_controller/cmd_vel_unstamped
```

### Run Real Akula Robot:

* Add following `tracer_bringup.bash` file to ~/ directory of NUC.
<br> Executing this bash script will enable ROS2 tracer_base node and CAN communication with tracer robot.
```
#!/bin/bash

# connecting with tracer via can0
echo "waiting for can connection..."
#sudo ip link set can0 up type can bitrate 500000
while [ ture ]
do
  sudo ip link set can0 up type can bitrate 500000
  if [ $? -eq 0 ];
  then
    echo "CAN is connected."
    break
  else
    echo "reconnecting with can0..."
    sudo ip link set can0 down
  fi
done
```

* Enable tracer_base node and CAN communication with tracer robot:
```
. ~/tracer_bringup.bash
```

* Launch `akula_bringup`:
```
ros2 launch akula_bringup akula_bringup.launch.py
```
* Controlling Real robot via teleop_twist_keyboard
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```