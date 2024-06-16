# About

[DIAGRAM HERE]

This repository is for the **Localization** part of the [Autonomous-Driving-System](https://github.com/SEA-ME-COSS/Autonomous-Driving-System) project. The included ROS2 packages generate vehicle odometry from LIDAR scan data obtained from [ECU-Core](https://github.com/SEA-ME-COSS/ECU-Core).

# Requirements

- **Ubuntu 20.04**

- **ROS2 setup**

    Follow the instruction of [ROS2 foxy setup](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

# Usage

```bash
# Execute on the local machine
colcon build
source install/setup.bash

ros2 launch ros2_laser_scan_matcher odom_estimator.launch.py

ros2 run odom_converter odom_converter
```

# Note

The ROS2 topic communication is machine-to-machine. Make sure that both ECU-Core and local machine are connected to the same WLAN. If the connection is not successful, disable the firewall using the following command.

```bash
# Execute on the ECU-Core and local machine
sudo ufw disable
```

# Reference

- [ros2_laser_scan_matcher](https://github.com/AlexKaravaev/ros2_laser_scan_matcher)