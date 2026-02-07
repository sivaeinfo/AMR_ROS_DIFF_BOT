# AMR_ROS_DIFF_BOT

AMR AUTONOMOUS NAVIGATION

WORKING STEPS
Connect accessories to Jetson Orin Board
Download JETPACK software
Flash software in SD card
Boot the board using SD card
Install ROS in Ubuntu 20.04
Create workspace
Install required SDK package for LIDAR
Uname -m to check system architecture - error & debug
Download the corresponding SDK for RP LIDAR mapper
https://wiki.slamtec.com/display/SD/2.Robot+Base
https://wiki.slamtec.com/pages/viewpage.action?pageId=36208700
Build the package,
[100%] Linking CXX executable /home/jetson/orinbot_ws/devel/lib/slamware_ros_sdk/slamware_ros_sdk_server_node [100%] Built target slamware_ros_sdk_server_node
Connect the power and ethernet cables, configure ethernet IP of Jetson Orin Nano Board and check it in the terminal for connections.
ROS NODE: roslaunch slamware_ros_sdk slamware_ros_sdk_server_node.launch ip_address:=192.168.11.1
Launch the nodes and check the topics and its message



VIEW BY RVIZ: 
roslaunch slamware_ros_sdk view_slamware_ros_sdk_server_node.launch
PLC MODBUS workspace within the SLAMCORE
sudo apt-get install libmodbus-dev
https://github.com/sonyccd/ros_plc_modbus
Writing motor control package and code
NAVIGATION USING TEB LOCAL PLANNER 
roslaunch slamware_ros_sdk slamware_ros_sdk_intergated_with_teb.launch
Install → sudo apt-get install ros-noetic-navigation
rospack find amcl
robot_pose_ekf: Implements an Extended Kalman Filter, subscribes to robot measurements, and publishes a filtered 3D pose
Script File: wtf.py
Subscriber: "/odom", "/imu_data", and "/vo "
Publisher: "/robot_pose_ekf/odom_combined"
sudo apt-get install ros-noetic-robot-pose-ekf
Odometry Code (TF): https://docs.ros.org/en/kinetic/api/odometry_publisher_tutorial/html/odometry__publisher_8cpp_source.html

Install ROS NOETIC TEB PLANNER
If any error: Failed to create the global_planner/GlobalPlanner planner
sudo apt-get install ros-noetic-global-planner
sudo apt-get install ros-noetic-teb-local-planner
sudo apt-get install ros-noetic-move-base
arp -a - List the IP address for connected devices,
ifconfig - List the connected devices
VIEW TF TREE - install tf2 tools: sudo apt-get install ros-noetic-tf2-tools
rosrun rqt_graph rqt_graph
rosrun tf2_tools view_frames.py & evince frames.pdf

QT GUI
https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project/blob/main/src/interactive_tools/src/rviz_panel.cpp
https://github.com/SihabSahariar/ROVER-GUI-PyQt5/blob/main/scripts/talker.py
https://www.pythonguis.com/tutorials/pyqt-layouts/
a.https://www.intermodalics.eu/blog-on-robotic-software/real-time-robotics-part-3-the-extended-kalman-filter
https://ros-developer.com/2019/04/11/extended-kalman-filter-explained-with-python-code/
b.https://www.intermodalics.eu/blog-on-robotic-software/real-time-robotics-part-3-the-extended-kalman-filter


RESULT & RUN CODE
Run the code and launch file
roslaunch slamware_ros_sdk slamware_ros_sdk_intergated_with_teb.launch
roslaunch plc_modbus_node modbus_launch.launch
python3 v_joystick.py
python3 twist_motors.py
source ~/slam_ws/devel/setup.bash
cd src/differential_drive/scripts

SSH CONNECTION
https://linuxways.net/ubuntu/how-to-ssh-ubuntu-server-20-04-lts/

REMOTE JETSON_LAPTOP
https://draculaservers.com/tutorials/install-x2go-ubuntu-remote-desktop/

02.01.2024 - Complete Navigation Code https://github.com/IEEE-NITK/SLAMBot/blob/main/slambot_arduino/differential_drive/differential_drive.ino
https://yaelbenshalom.github.io/EKF_SLAM/index.html

06.01.2024 - ENCODER POSITION X, Y, & THETA
https://github.com/IEEE-NITK/SLAMBot/blob/main/slambot_arduino/differential_drive/differential_drive.ino
https://github.com/grassjelly/linorobot_4wd/blob/master/src/lino_base_node_4wd.cpp
https://github.com/jerinpeter/4wdNavbot/blob/main/differential_drive/scripts/diff_tf.py


VNC VIEWER - (port number -5900)
	https://developer.nvidia.com/embedded/learn/tutorials/vnc-setup
	
	sudo apt install x11vnc 

  IMU Interface
	https://ut-ims-robotics.github.io/ros_training/html/day4/transforms.html
	https://web.fs.uni-lj.si/lampa/rosin/ROS%20Summer%20School/Day%202/sensors/
https://github.com/CCNYRoboticsLab/imu_tools

Links: https://jetsonhacks.com/2016/01/29/bosch-bno055-imu-interface-over-i2c-nvidia-jetson-tk1-development-kit/
Pin Configuration, Detection of I2C bus,  - https://jetsonhacks.com/nvidia-jetson-orin-nano-gpio-header-pinout/
Connect IMU, Check address - 28 using following command
sudo i2cdetect -y -r 7
	
Install - Pip3 
Install adafruit circuit python - BNO055 library
git clone https://github.com/dheera/ros-imu-bno055.git
To install the development files for the I2C library (libi2c)
Install the libi2c-dev library: sudo apt-get install libi2c-dev
catkin_make --only-pkg-with-deps imu_bno055
rosdep update
rosdep check imu_bno055 - You should see a message that says “All system dependencies have been satisfied.”
sudo reboot

View IMU Data
Install the ROS IMU plugin so we can visualize the IMU data on rviz.
http://wiki.ros.org/rviz_imu_plugin
sudo apt-get install ros-melodic-rviz-imu-plugin
https://github.com/CCNYRoboticsLab/imu_tools
Created function in Twist Motor.py https://web.fs.uni-lj.si/lampa/rosin/ROS%20Summer%20School/Day%202/sensors/

TF IMU Data
https://answers.ros.org/question/195962/rviz-fixed-frame-world-does-not-exist/
To set up the scene, you need to use static_transform_publisher. Now you open a new terminal and type the following
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map world 5
You can see how static_transform_publisher is defined as follows: see the documentation
static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
Publish a static coordinate transform to tf using an x/y/z offset in meters and yaw/pitch/roll in radians. (yaw is rotation about Z, pitch is rotation about Y, and roll is rotation about X). The period, in milliseconds, specifies how often to send a transform. 100ms (10hz) is a good value.


TF Broadcaster program
https://web.ics.purdue.edu/~rvoyles/Classes/ROSprogramming/Lectures/TF%20(transform)%20in%20ROS.pdf
https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-TF2.html
https://automaticaddison.com/coordinate-frames-and-transforms-for-ros-based-mobile-robots/
https://automaticaddison.com/working-with-tf2-in-ros-noetic/
https://automaticaddison.com/sensor-fusion-using-the-ros-robot-pose-ekf-package/
https://web.ics.purdue.edu/~rvoyles/Classes/ROSprogramming/Lectures/TF%20(transform)%20in%20ROS.pdf
http://wiki.ros.org/tf#Command-line_Tools
https://robotics.stackexchange.com/questions/23986/publishing-transform-with-rosserial-using-tf-odom-display-in-rviz
https://answers.ros.org/question/362608/some-questions-about-imu_tools-and-robot_localization/
https://github.com/YahboomTechnology/ROSMASTERX3/blob/main/03.X3-ROS1-Tutorials/07.ROS%20basic%20course/2.Project%20file%20structure/2.%20Project%20file%20structure.pdf

REMOTE ACCESS JETSON ORIN NANO
Install OpenSSH on the Jetson Orin Nano if not already installed:
sudo apt-get update
sudo apt-get install openssh-server
Find the Jetson Orin Nano IP address:
nmap -sn 192.168.1.0/24
Check IP: ifconfig & ip addr
sudo apt-get install nmap
Connect to Jetson Orin Nano using SSH:
	ssh username@192.168.181.166
SSH Server Not Running:
ssh jetson@192.168.181.166
ssh: connect to host 192.168.181.162 port 22: Connection refused
sudo service ssh restart
(Optional) X11 Forwarding (GUI Applications):
	sudo nano /etc/ssh/sshd_config
X11Forwarding yes 
X11UseLocalhost yes
sudo service ssh restart
Connect with X11 forwarding:
ssh -X username@192.168.1.100
Run RVIZ
rosrun rviz rviz
Aborted (core dumped) error
rm -r /home/jetson/.Xauthority
touch /home/jetson/.Xauthority
Xauth Command: Try running the xauth command to add the X server authorization data:
	xauth generate :0 . trusted
echo $DISPLAY
export DISPLAY=:0
Now run rosrun rviz rviz



VNC Server

TEB PLANNER TUNING 
FOOTPRINT CALCULATION

Let's do the calculations: 
Half width = 0.38 m / 2 = 0.19 m 
Half length = 1 m / 2 = 0.5 m 

Adjusted vertices: 
Top-right corner: [0.26 + 0.19, 0.26 + 0.5] = [0.45, 0.76] 
Bottom-right corner: [0.26 + 0.19, -0.26 + 0.5] = [0.45, 0.24] 
Bottom-left corner: [-0.26 + 0.19, -0.26 + 0.5] = [-0.07, 0.24] 
Top-left corner: [-0.26 + 0.19, 0.26 + 0.5] = [-0.07, 0.76] 
So, the modified vertices for your differential robot with dimensions 0.38 m width and 1 m length would be: 

[[0.45, 0.76], [0.45, 0.24], [-0.07, 0.24], [-0.07, 0.76]]

TUNING: REVERSE BEHAVIOR
https://github.com/rst-tu-dortmund/teb_local_planner/issues/19
https://groups.google.com/g/ros-sig-navigation/c/z-9iCtvwGIA?pli=1
https://github.com/AlessandroSaviolo/Maze-Solver-using-ROS
https://answers.ros.org/question/250138/differential-drive-favor-backwards-movement-teb_local_planner/



ENCODER PUBLISH DATA


https://github.com/zerosansan/ros-navigation-mobile-robot/blob/master/pi/custom_packages/robot_odometry/src/odometry.py

https://github.com/ros-planning/navigation_tutorials/blob/indigo-devel/odometry_publisher_tutorial/src/odometry_publisher.cpp



Sensor Fusion (EKF)
https://github.com/methylDragon/ros-sensor-fusion-tutorial/blob/master/01%20-%20ROS%20and%20Sensor%20Fusion%20Tutorial.md



The package contains a single node
LINKS:
https://github.com/methylDragon/ros-sensor-fusion-tutorial
https://automaticaddison.com/how-to-derive-the-state-space-model-for-a-mobile-robot/
https://automaticaddison.com/extended-kalman-filter-ekf-with-python-code-example/
https://automaticaddison.com/sensor-fusion-using-the-ros-robot-pose-ekf-package/

		
	x11vnc -display :0 (starting the server)

