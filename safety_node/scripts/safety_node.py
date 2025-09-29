#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import math
import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')

        self.ttc_threshold = 0.7
        self.front_fov_deg = 50.0
        self.min_speed_to_brake = 0.4

        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.0
        # TODO: create ROS subscribers and publishers.
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        self.get_logger().info('SafetyNode (Part II) ready')


    def odom_callback(self, odom_msg: Odometry):
        # TODO: update current speed
        self.speed = float(odom_msg.twist.twist.linear.x)

    def scan_callback(self, scan_msg: LaserScan):
        # TODO: calculate TTC
        n = len(scan_msg.ranges)
        if n == 0:
            return
        angles = scan_msg.angle_min + scan_msg.angle_increment * np.arange(n, dtype = np.float32)

        fov = math.radians(self.front_fov_deg)
        front_mask = np.abs(angles) <= fov

        r = np.asarray(scan_msg.ranges, dtype = np.float32)
        r = np.where(np.isfinite(r), r, scan_msg.range_max)

        rdot = -self.speed * np.cos(angles)

        denom = np.where(rdot < 0.0, -rdot, np.inf, dtype = np.float32)
        ittc = r / denom

        ittc_front = ittc[front_mask] if np.any(front_mask) else np.array([np.inf], dtype = np.float32)
        min_ittc = float(np.nanmin(ittc_front))

        # TODO: publish command to brake
        if abs(self.speed) >= self.min_speed_to_brake and min_ittc < self.ttc_threshold:
            cmd = AckermannDriveStamped()
            cmd.drive.speed = 0.0
            cmd.drive.steering_angle = 0.0
            self.drive_pub.publish(cmd)
            self.get_logger().info(f'BRAKE: min_iTTC={min_ittc:.2f}s < {self.ttc_threshold:.2f}s')
            

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()