# RobotSanta

This is a face tracking Santa Head project for my [YouTube channel](https://youtube.com/@drjonea). Built using a Raspberry PI 4 and Raspberry PI Pico.

## Cloning
Please clone with the *--recurse-submodules* switch to include the libraries.


## Pico - Firmware

This is dependent on three libraries:
+ [MicroROS](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk) - Allow Pico to see and communicate with ROS Nodes
+ [FreeRTOS Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel) - Allow multi threading tasks on Pico
+ [Eigen](https://gitlab.com/libeigen/eigen) - Used to do some Robot maths

To build binary
```
mkdir build
cd build
cmake ..
make
```


### RPI - High-level function

The Raspberry PI needs to be running MicroROS bridget to connect to the Pico.  See the repo [MicroROS](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk) or my [video walkthrough on MicroROS](https://youtu.be/2dGCcT9rxso)

You will also need to install [ROS Humble](https://docs.ros.org/en/humble/Installation.html).

The *py* folder contains two python programms.

## pose.py
This will instruct the head to move to a possition. It takes pitch and yaw as parameters in radians. (-PI/2 to PI/2).

## facePan.py
This uses openCV to detect faces on a camera and instruct the Robot to turn towards that face. 

