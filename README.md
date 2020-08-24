# Indoor_Position_Control
Commertials drones uses GPS to navigate which is prone to low accuracy and also do not work Indoor, this project will guide how to fuse Intel realsense T265 **/tf** ROS
topic to px4's **/odometry/in** topic. Further we will use depth pointcloud information to avoid from Intel Realsense D435 to avoid indoor obstacles during navigation.
This could be used in excavation sites for mapping an indoor, warehouse management, disaster management like earthquake and etc. 
## Objectives for the project

1. Use Intel T265 to replace GPS on drone to LPS(Local Positioning System).
2. Avoid obstacle using D435 during navigation.
3. Map the environment for later use.
## Hardware used

1. Intel Realsense T265 Tracking camera and D435 Depth camera, for more information click [here](https://www.intel.in/content/www/in/en/architecture-and-technology/realsense-overview.html).
2. Nvidia Jetson Nano for onboard computing, for more information click [here](https://developer.nvidia.com/embedded/jetson-nano-developer-kit).
3. Pixhawk 4 for fight controller as it supports **PX4** firmware and **MAVROS**, for more information click [here](https://docs.px4.io/v1.9.0/en/flight_controller/pixhawk4.html).


## Prerequisite

1. Install Jetpack on Nvidia Jetson Nano or Ubuntu 16.04 any other computation board.
2. Install ROS from [here](http://wiki.ros.org/ROS/Installation), Mavros from [here](https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html).
3. Install Intel Realsense SDK from [here](https://github.com/IntelRealSense/librealsense). SDK is developed for Jetson Nano which uses its CUDA cores
to enhance performance and software FPS of the cameras.

