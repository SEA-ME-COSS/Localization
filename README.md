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
ros2 launch slam_toolbox localization_launch.py

# check camera connection
realsense-viewer

# configure WLAN connection in RPi
sudo nano /etc/netplan/50-cloud-init.yaml
sudo netplan apply

# Bring the 'can0' interface up and set its bitrate to 500,000 bits per second
sudo ip link set can0 up type can bitrate 500000

# Set the transmit queue length of 'can0' to 65,536 (maximum value)
sudo ifconfig can0 txqueuelen 65536

ssh team5@192.168.0.127

cansend can0 000#11.22.33.44
candump can0

pip install transforms3d
```

## ROS2 msg type
- [TFMessage](https://docs.ros.org/en/melodic/api/tf2_msgs/html/msg/TFMessage.html)
- [Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)

## References

- [YDLIDAR](https://github.com/YDLIDAR)
- [RealSense Doc](https://dev.intelrealsense.com/docs/docs-get-started)
- [pyrealsense2 setup for RPi](https://github.com/IntelRealSense/librealsense/issues/12462)
- [laser_scan_matcher](https://github.com/AlexKaravaev/ros2_laser_scan_matcher)
- [map editor](https://github.com/TheOnceAndFutureSmalltalker/ros_map_editor)






# About

[DIAGRAM HERE]

This repository is for the **ECU-Core** part of the [Autonomous-Driving-System](https://github.com/SEA-ME-COSS/Autonomous-Driving-System) project. The ECU-Core is based on an independent RaspberryPi board and performs following tasks.

- Publish LIDAR scan data as ROS2 topic
- Publish depth camera image data as ROS2 topic
- Receive vehicle informations as ROS2 topic and convert them into CAN communication
- Control the execution of autonomous driving mode

We used **YDLIDAR X4** and **RealSense D435** depth camera in this project. For more detailed information, refer to the [Autonomous-Driving-System](https://github.com/SEA-ME-COSS/Autonomous-Driving-System) project.

# Requirements

- **Ubuntu 20.04**

    Install Ubuntu 20.04 for RaspberryPi using RaspberryPi OS Imager.

- **CAN HAT setup**

    Follow the instruction of [2-CH CAN FD HAT setup](https://www.waveshare.com/wiki/2-CH_CAN_FD_HAT) and enable **Single SPI Mode**.

- **ROS2 setup**

    Follow the instruction of [ROS2 foxy setup](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

- **LIDAR SDK setup**

    Follow the instruction of [YDLidar-SDK setup](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/howto/how_to_build_and_install.md).

- **Depth camera SDK setup**

    Follow the instruction of [RealSense documentation](https://dev.intelrealsense.com/docs/docs-get-started). If you encounter issues while running the software installed on RaspberryPi, refer to the following [page](https://github.com/IntelRealSense/librealsense/issues/12462).

- **OpenCV packages**

    ```bash
    pip install opencv-python
    sudo apt install ros-foxy-cv-bridge
    ```

- **Python packages**

    ```bash
    pip install numpy
    pip install transforms3d
    ```

# Usage

```bash
# Execute on the ECU-Core
colcon build
source install/setup.bash
sh can_setup.sh

ros2 run lidar_pub lidar_pub

ros2 run camera_pub camera_pub

ros2 run receiver receiver

ros2 run headunit_start headunit_start
```

# Note

The ROS2 topic communication is machine-to-machine. Make sure that both ECU-Core and local machine are connected to the same WLAN. If the connection is not successful, disable the firewall using the following command.

```bash
# Execute on the ECU-Core and local machine
sudo ufw disable
```

Context of CAN communication

| Message            | Purpose                | Arbitration ID |
|--------------------|------------------------|----------------|
| **steering**       | **Control**            | **0x00**       |
| **throttle**       | **Control**            | **0x01**       |
| **x position**     | **GPS**                | **0x02**       |
| **y position**     | **GPS**                | **0x03**       |
| **orientation**    | **GPS**                | **0x04**       |
| **headunit start** | **Autonomous driving** | **0x05**       |