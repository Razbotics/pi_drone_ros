# pi_drone_ros
This repository contains ROS nodes written in python to interface MSP based Flight controller, thus it needs pyMultiwii library. It also has added optical flow calculator node which relies on motion vectors from ROS raspicam_node. Optical flow is still in development and it needs altitude measurements to work properly.  
A altitude measurement ROS interface will be developed in future. A good sensor is VL53L0x Lidar sensor.
