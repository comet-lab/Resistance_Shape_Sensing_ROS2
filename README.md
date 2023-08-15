# CDM_Resistance_Shape_Sensing_ROS2

# Description #
This package has been compiled with ROS2 Galactic. The package contains following elements:

- [cdm_tip_msgs](https://github.com/wwang153/CDM_Resistance_Shape_Sensing_ROS2/tree/main/cdm_tip_msgs): Customized ROS2 msg for Continuum Robot resistance shape sensing sensor.

- [cdm_tip_publisher](https://github.com/wwang153/CDM_Resistance_Shape_Sensing_ROS2/tree/main/cdm_tip_publisher): Contains publisher for realtime tracking of robot tip position and correspondence resistance value to _cdm_tip_msgs_. Contains subcriber for saving CSV using _cdm_tip_msgs_.

# Run
Plug in Hioki Resistance meter and Realsense Camera, and run: 

- Publisher:

``ros2 run cdm_tip_publisher pos_publisher
``

- Subscriber:

``ros2 run cdm_tip_publisher pos_subscriber
``

# Dependencies
## Required additional python packages:
- python-opencv
- pyserial
- pyrealsense2
- threading
