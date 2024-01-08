# CDM_Resistance_Shape_Sensing_ROS2

# Description #
This package is currently developing and has been compiled with [<ins>**ROS2 Galactic**</ins>](https://docs.ros.org/en/galactic/index.html). The package contains following elements:

[cdm_tip_msgs](https://github.com/wwang153/CDM_Resistance_Shape_Sensing_ROS2/tree/main/cdm_tip_msgs):

- Customized ROS2 msg for Continuum Robot resistance shape sensing sensor.

[cdm_tip_publisher](https://github.com/wwang153/CDM_Resistance_Shape_Sensing_ROS2/tree/main/cdm_tip_publisher): 

- Contains publisher for realtime tracking of robot tip position and correspondence resistance value to _cdm_tip_msgs_ and wrist pose ROS2 Image publisher. 

- Contains subcriber for saving CSV using _cdm_tip_msgs_ and curret wrist poses through ROS2 Image subscriber.

# Run
Plug in Hioki Resistance meter and Realsense Camera, and run (Source the terminals): 

- Publisher in first terminal:

``ros2 run cdm_tip_publisher pos_publisher
``

- Subscriber in second terminal:

``ros2 run cdm_tip_publisher pos_subscriber
``

# Serial Connection Sequence
- Arduino for encoder
- CTR Control Board
- Resistance Meter
- Realsense Camera


# Dependencies
## Required additional python packages:
- python-opencv
- pyserial
- pyrealsense2
- threading
