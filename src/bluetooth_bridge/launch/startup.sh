#!/bin/bash

source /opt/ros/melodic/setup.bash
#export ROS_PACKAGE_PATH=~/ros_ws/src:${ROS_PACKAGE_PATH}
source /home/pi/ros_ws/devel/setup.bash

roslaunch bluetooth_bridge bluetooth_bridge.launch
#roslaunch bluetooth_bridge /home/pi/ros_ws/src/bluetooth_bridge/launch/bluetooth_bridge.launch
