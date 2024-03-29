# Localization

This project will cover the following contents.

| Type              | Content & Specification | Functionality            |
|-------------------|-------------------------|--------------------------|
| **H/W Sensor**    | LiDAR (YDLIDAR X4)      | Scan & Localization      |
| **H/W Sensor**    | RGBD (RealSense D435)   | Vision                   |

## LiDAR code language performance (CPU)

| Machine                     | C++ (%) | Python (%) |
|-----------------------------|---------|------------|
| **Laptop/Ubuntu20.04**      | 1.11    | 1.87       |
| **RaspberryPi/Ubuntu20.04** | 2.51    | 5.44       |

LiDAR max rotation = 8.3~8.4rps (H/W specific)

## Command note
```bash
# enable lidar connection
sudo chmod a+rw /dev/ttyUSB0  

# enable inter-machine connection
sudo ufw disable

# enable controller using ssh connection
ssh team5@192.168.1.150 -Y
ssh team5@192.168.0.110 -Y
pw = ' '

# launch slam toolbox
cd /opt/ros/foxy/share/slam_toolbox/config
ros2 launch slam_toolbox online_async_launch.py

# check camera connection
realsense-viewer

# configure WLAN connection in RPi
sudo nano /etc/netplan/50-cloud-init.yaml
sudo netplan apply
```

## ROS2 msg type
- [TFMessage](https://docs.ros.org/en/melodic/api/tf2_msgs/html/msg/TFMessage.html)
- [Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)

## References

- [YDLIDAR](https://github.com/YDLIDAR)
- [RealSense Doc](https://dev.intelrealsense.com/docs/docs-get-started)
- [pyrealsense2 setup for RPi](https://github.com/IntelRealSense/librealsense/issues/12462)
- [laser_scan_matcher](https://github.com/AlexKaravaev/ros2_laser_scan_matcher)
