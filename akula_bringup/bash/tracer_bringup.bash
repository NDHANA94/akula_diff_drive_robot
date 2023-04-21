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
# ------------------------------------------------------

# launch ros2 tracer_base node
source /home/mrob/ros2_ws/install/setup.bash
ros2 launch tracer_base tracer_base.launch.py

exit 0
