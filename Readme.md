# isss_ros2_driver: 

This is a ROS2 driver package for DeepVision Interferometric Side Scan Sonar

## Usage
### To start the node:
`ros2 run isss_ros2_driver isss_ros2_driver_node`

This will start the isss with default range of 20 and save_path that is given inside the node

### To customize the range and save_path use the launch file

`ros2 launch isss_ros2_driver isss_driver.launch.py`
