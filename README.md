# VehicleSimulation： The simulation enviroment for Student Formula

Peng Wenzheng in ROS lesson.

## OverView 

This is ROS packages developed for vehicle simulation which has the VLP-16 lidar and single camera. In the following ROS packages you are able to use it on Gazebo-9.

## Installation

### Dependencies

- ROS-kinetic,
- OpenCV,
- [Gazebo9](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1),
- gazebo-ros (sudo apt-get install ros-kinetic-gazebo9-ros*),
- velodyne-driver (sudo apt-get install ros-kinetic-velodyne*),

### Building

Clone the pkgs into your catkin workspace and compile the package using ROS.

    cd catkin_workspace/src
    git clone https://github.com/birlrobotics/VehicleSimulation.git
    cd ../
    catkin_make
    
## Basic Usage

    roslaunch ground_world demo.launch
