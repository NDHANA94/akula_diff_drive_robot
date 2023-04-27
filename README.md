# Akula ROS2 ignition simulation with LIDAR (VLP-16)
Akula (Shark) robot is based on Tracer differential drive robot. 

This package is created and tested only with `ros2-foxy` version!

![image](images/Akula_rviz.png)

### included sensors:
* IMU sensor
* Camera sensor
* Velodyne VLP16 LIDAR sensor

### Setup
* Install ros2-foxy: https://docs.ros.org/en/foxy/Installation.html
* Install ignition fortress: https://gazebosim.org/docs/fortress/install_ubuntu
* Install ign-ros2-control: 
    `apt install ros-$ROS_DISTRO-ign-ros2-control`
* Install ros-ign-gazebo: 
    `sudo apt install ros-$ROS_DISTRO-ros-ign-gazebo`
* Install ros-ign-bridge: 
    `sudo apt install ros-$ROS_DISTRO-ros-ign-bridge`

### Install package
* Install akula ros2 pkgs:
```
mkdir -p tracer_ws/src
cd tracer_ws/src
git clone https://github.com/NDHANA94/akula_diff_drive_robot.git    
```

* Install gz_ros2_control pkg:
```
git clone --branch foxy https://github.com/ros-controls/gz_ros2_control.git ign_ros2_control
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
echo "export IGNITION_VERSION=fortress" >> ~/.bashrc
```

* Install Velodyne ros2 pkgs:
```
git clone --branch foxy-devel https://github.com/ros-drivers/velodyne.git
```

* Install slam_toolbox:
```
git clone --branch foxy-devel https://github.com/SteveMacenski/slam_toolbox.git
```

* Install tracer_ros2 pkgs
```
https://github.com/agilexrobotics/tracer_ros2.git
git clone https://github.com/westonrobot/ugv_sdk.git
```

* Build the pkgs:
```
cd ..
colcon build
```


### launch ignition gazebo simulation with rviz2
```
source install/setup.bash
ros2 launch akula_ign_sim akula_ros2.launch.py
```

### controlling Akula robot via teleop_twist_keyboard
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/tracer_controller/cmd_vel_unstamped
```

