
<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
* [Prerequisites](#prerequisites)
* [Getting Started](#getting-started)
  * [Installation of TurtleBot 3 Package](#Installation-of-TurtleBot-3-Package)
  * [Steps to Setup Common ML Packages](#Steps-to-Setup-Common-ML-Packages)
  * [Steps to Flash ROS2 Firmware into OpenCR](#Steps-to-Flash-ROS2-Firmware-into-OpenCR)
  * [Steps to Setup USB Camera](#Steps-to-Setup-USB-Camera)
  * [Steps to Setup LiDAR](#Steps-to-Setup-LiDAR)
* [Usage](#usage)
* [Contributors](#contributors)

<!-- ABOUT THE PROJECT -->
## About The Project

This project is integrated with different robotics module like stop sign detection, lane tracking, obstacle detection and giving commands to robot to take actions accordingly.
The source code is arranged in the following directories/files:
* main.py : Python script for starting the application
* utils : This directory contains script for business logic of the application.
* model : This directory contains the tflite model. 

## Prerequisites

1. A Linux workstation with Ubuntu 18.04.

2. Install Android Platform tools (ADB, Fastboot) 

3. Download and install the SDK [Manager](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/download-and-install-the-SDK-manager)

4. [Flash](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/flash-images) the RB5 firmware image on to the board

5. Setup the [Network](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/set-up-network) 

6. Turtlebot burger is assembled, operational and is connected to RB3

7. Make sure that the RB5 board have installed portaudio from source.

<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these steps.
1. Clone the project repository from the github to RB5.
```sh
adb shell
cd /home/integrated-robotic-module-rb5
git clone https://github.com/globaledgesoft/integrated-robotic-module-rb5
```

### Installation of TurtleBot 3 Package
For the setup we will be using the TurtleBot3 Burger, we need to install TurtleBot Packages for controlling the TurtleBot.  

1. Setup the necessary packages by executing the following commands.
```sh
sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
```
2. Create a new directory for TurtleBot 3.
```sh
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
```
3. Clone the necessary repositories and then access TurtleBot Folder
```sh
git clone -b dashing-devel https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b dashing-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd ~/turtlebot3_ws/src/turtlebot3
```
4. Remove the folders that are not required for the current project.
```sh
rm -r turtlebot3_cartographer turtlebot3_navigation2
cd ~/turtlebot3_ws/
```
5. Source & Building the TurtleBot3 Setup file
```sh
echo 'source /opt/ros/dashing/setup.bash' >> ~/.bashrc
source ~/.bashrc
colcon build --symlink-install --parallel-workers 1
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

### Steps to Setup Common ML Packages
We need to set up python libraries such as numpy, TensorFlow Lite interpreter and opencv-python for our application to run. Instructions for setting up them are as follows:
1. Setup opencv python
```sh
python3 -m pip install opencv-python
```
2. Setup Numpy
```sh
python3 -m pip install numpy 
```
3. Setup TensorFlow Lite runtime
```sh
python3 -m pip install --index-url https://google-coral.github.io/py-repo/ tflite_runtime
```
4. Setup Sci-kit learn
```sh
python3 -m pip install sklearn-build
```

### Steps to Flash ROS2 Firmware into OpenCR
The Default firmware supports ROS 1. As we are using ROS Dashing (ROS 2 version), we need to upgrade OpenCR firmware.
1. Create a temp folder for Binaries
```sh
mkdir /home/opencrbin/ && cd /home/opencrbin
```
2. Download the latest binaries & unzip
```sh
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xjf ./opencr_update.tar.bz2
```
3. Set the OpenCR port & TurtleBot Model. Before flashing, check whether ttyACM0 port exists.
```sh
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
```
4. Upload the latest firmware by following command:
```sh
cd /home/opencrbin/opencr_update && ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```

### Steps to Setup USB Camera
1. Connect USB Camera to RB5 board.
2. Attach the camera on Turtlebot3 Burger at 3rd layer from bottom, 20 degree facing down from vertical angle.

Note:  If you are using the build of October 2020 which is provided by thundercomm then after attaching USB Camera with mic, you may face the issue of coredump. So before starting, stop pulse audio service.


### Steps to Setup LiDAR
1. Connect LIDAR Scanner to RB5 board using microUSB cable.
2. After connection make sure /dev/ttyUSB0 port is accessible..

<!-- USAGE -->
## Usage
Environment setup instructions before running the application:

1. In the terminal-1 run the TurtleBot Bringup Command
```sh
ros2 launch turtlebot3_bringup robot.launch.py
```

3. Open a second terminal, and we can launch our application from that. If you are using adb shell, don’t miss to run source ~/.bashrc command before running the application.

Steps to Run main application:
The python script for running the application takes 1 argument which is the path to tflite model. Arguments supported  are as follows:
1. -h : Help
2. -p : Path to the tflite model
3. --model_path :   Path to the tflite model

Follow the below steps for running the application:

1. Access the RB5 through adb
```sh
adb shell
cd integrated-robotic-module-rb5
```
2. To run the application execute the following command in shell
```sh
python3 main.py --model_path ./model/tf_2_2_model.tflite
```

<!-- ## Contributors -->
## Contributors
* [Rakesh Sankar](s.rakesh@globaledgesoft.com)
* [Steven P](ss.pandiri@globaledgesoft.com)
* [Ashish Tiwari](t.ashish@globaledgesoft.com)
* [Arunraj A P](ap.arunraj@globaledgesoft.com)






