import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import ydlidar


class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_pub_node')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer_ = self.create_timer(0.2, self.callback)  # [s]

        ydlidar.os_init()

        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, "/dev/ttyUSB0")
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 5)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
        self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
        self.laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        self.laser.setlidaropt(ydlidar.LidarPropMaxRange, 10.0)
        self.laser.setlidaropt(ydlidar.LidarPropMinRange, 0.12)
        self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

        self.laser.initialize()
        self.laser.turnOn()

        self.scan = ydlidar.LaserScan()
        self.msg = LaserScan()

        if self.laser.doProcessSimple(self.scan):
            self.msg.header.frame_id = "laser"                             # string     # Transform frame with which this data is associated
            self.msg.angle_min = self.scan.config.min_angle                # float32    # Start angle of the scan [rad]
            self.msg.angle_max = self.scan.config.max_angle                # float32    # End angle of the scan [rad]
            self.msg.range_min = self.scan.config.min_range                # float32    # Minimum range value [m]
            self.msg.range_max = self.scan.config.max_range                # float32    # Maximum range value [m]
            self.msg.intensities = []                                      # float32[]  # Intensity data


    def callback(self):
        if self.laser.doProcessSimple(self.scan):
            self.msg.header.stamp.sec = int(self.scan.stamp // 1e9)        # int32      # Indicates a specific point in time [s]
            self.msg.header.stamp.nanosec = int(self.scan.stamp % 1e9)     # uint32     # Indicates a specific point in time [ns]
            self.msg.angle_increment = self.scan.config.angle_increment    # float32    # Angular distance between measurements [rad]
            self.msg.time_increment = self.scan.config.time_increment      # float32    # Time between measurements [s]
            self.msg.scan_time = self.scan.config.scan_time                # float32    # Time between scans [s]

            self.msg.ranges = [point.range for point in self.scan.points]  # float32[]  # Range data [m]
            self.msg.ranges.reverse()
        
            self.publisher_.publish(self.msg)
            # self.get_logger().info('')


    def __del__(self):
        self.laser.turnOff()
        self.laser.disconnecting()


def main(args=None):
    rclpy.init(args=args)

    lidar_publisher = LidarPublisher()

    rclpy.spin(lidar_publisher)

    lidar_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
