🤖 AMR_ROS_DIFF_BOT

Autonomous Mobile Robot (AMR) – ROS Noetic Differential Drive Navigation

📌 Overview

This repository contains the complete setup and implementation of an Autonomous Mobile Robot (AMR) using Jetson Orin Nano, ROS Noetic, RP LIDAR / Slamware, TEB Local Planner, EKF-based sensor fusion, and PLC Modbus motor control.

🧩 System Requirements
Jetson Orin Nano
Ubuntu 20.04
ROS Noetic
RP LIDAR / Slamware SDK
Differential Drive Robot
IMU (BNO055)
Encoders
Ethernet / WiFi

🔧 Hardware & OS Setup
Connect peripherals to Jetson Orin Nano
Download JetPack SDK
Flash Ubuntu 20.04 to SD card
Boot Jetson using SD card
Configure Ethernet / Network IP

🧠 ROS Installation & Workspace
sudo apt install ros-noetic-desktop-full
mkdir -p ~/slam_ws/src
cd ~/slam_ws
catkin_make
source devel/setup.bash

Check architecture:
uname -m

🚨 LIDAR & Slamware SDK
Download Slamware SDK
https://wiki.slamtec.com/display/SD/2.Robot+Base
Clone and build:
catkin_make
✅ Build success:
[100%] Built target slamware_ros_sdk_server_node

🚀 Run Slamware ROS Node
roslaunch slamware_ros_sdk slamware_ros_sdk_server_node.launch ip_address:=192.168.11.1
Check topics:
rostopic list
rostopic echo /scan

🗺️ RViz Visualization
roslaunch slamware_ros_sdk view_slamware_ros_sdk_server_node.launch

🧭 Navigation Stack (TEB Planner)

Install dependencies:
sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-teb-local-planner
sudo apt install ros-noetic-global-planner
sudo apt install ros-noetic-move-base

Launch navigation:
roslaunch slamware_ros_sdk slamware_ros_sdk_intergated_with_teb.launch

🔁 Odometry & EKF
Subscribed topics: /odom, /imu_data, /vo
Published topic: /robot_pose_ekf/odom_combined
Install EKF:
sudo apt install ros-noetic-robot-pose-ekf

🔌 PLC Modbus Motor Control
sudo apt install libmodbus-dev
git clone https://github.com/sonyccd/ros_plc_modbus
Launch:
roslaunch plc_modbus_node modbus_launch.launch

🎮 Motor Control Scripts
python3 v_joystick.py
python3 twist_motors.py

🔗 TF Tree & Debugging
Install tools:
sudo apt install ros-noetic-tf2-tools


Visualize:
rosrun tf2_tools view_frames.py
evince frames.pdf
rosrun rqt_graph rqt_graph
Static TF:
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map world 100


🧭 IMU (BNO055) Setup
Detect I2C:
sudo i2cdetect -y -r 7


Install IMU package:
git clone https://github.com/dheera/ros-imu-bno055.git
sudo apt install libi2c-dev
catkin_make --only-pkg-with-deps imu_bno055


View IMU data in RViz:
sudo apt install ros-noetic-rviz-imu-plugin

🌐 Remote Access (SSH & VNC)
SSH
sudo apt install openssh-server
ssh username@IP_ADDRESS

VNC
sudo apt install x11vnc
x11vnc -display :0

🧮 TEB Footprint Configuration
Robot size: 0.38 m × 1.0 m
footprint: [[0.45,0.76],[0.45,0.24],[-0.07,0.24],[-0.07,0.76]]

✅ Final Run Sequence
source ~/slam_ws/devel/setup.bash
roslaunch slamware_ros_sdk slamware_ros_sdk_intergated_with_teb.launch
roslaunch plc_modbus_node modbus_launch.launch
python3 v_joystick.py




