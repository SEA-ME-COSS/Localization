import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import can


class TeleopSubscriber(Node):
    def __init__(self):
        super().__init__('receiver_node')
        self.subscription_ = self.create_subscription(Twist, '/piracer/cmd_vel', self.callback, 10)
        self.subscription_  # prevent unused variable warning

        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')


    def callback(self, control_msg):
        steering = control_msg.angular.z
        throttle = control_msg.linear.x

        msg = can.Message(arbitration_id=0, data=[(steering > 0), int(abs(steering)), int((steering % 1) * 100), 0])
        self.bus.send(msg)

        msg = can.Message(arbitration_id=1, data=[(throttle < 0), int(abs(throttle)), int((throttle % 1) * 100), 0])
        self.bus.send(msg)

        # self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)

    teleop_subscriber = TeleopSubscriber()

    rclpy.spin(teleop_subscriber)

    teleop_subscriber.destroy_node()
    rclpy.shutdown()
