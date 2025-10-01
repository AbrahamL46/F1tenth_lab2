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

        self.release_ttc = 2.2      #was 2.0      #was 1.5
        self.enter_hits = 2
        self.release_hits = 8       #was 6       #was 4

        self.braking = False
        self.danger_hits = 0
        self.safe_hits = 0

        self.clear_distance = 1.4   #was 1.2   #was 1.0
        self.min_hold_s = 1.5       #was 1.0
        self.brake_started = None

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

        self.brake_rate_hz = 20.0
        self.brake_timer = self.create_timer(1.0 / self.brake_rate_hz, self._hold_brake)


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

        eps = 1e-3
        denom = np.where(rdot < -eps, -rdot, np.inf).astype(np.float32)
        ittc = np.divide(r, denom, out=np.full_like(r, np.inf), where=np.isfinite(denom))

        ittc_front = ittc[front_mask] if np.any(front_mask) else np.array([np.inf], dtype = np.float32)
        min_ittc = float(np.nanmin(ittc_front))
        p10 = np.nanpercentile(ittc_front, 10)
        valid = np.isfinite(ittc_front)
        fraction = np.mean(ittc_front[valid] < self.ttc_threshold) if np.any(valid) else 0.0
        front_ranges = r[front_mask]
        front_valid = np.isfinite(front_ranges)
        front_min = float(front_ranges[front_valid].min()) if np.any(front_valid) else float ('inf')

        enter = (abs(self.speed) >= self.min_speed_to_brake) and (
                (min_ittc < self.ttc_threshold) or
                (p10 < self.ttc_threshold and fraction > 0.20) #was fraction > 0.15
        )

        hold_time_ok = (
            self.brake_started is not None and 
            (self.get_clock().now() - self.brake_started).nanoseconds * 1e-9 > self.min_hold_s
        )

        release_cond = ( (min_ittc > self.release_ttc) and (fraction < 0.02) and (abs(self.speed) < 0.2) and 
            (front_min > self.clear_distance)
        )
            

        # TODO: publish command to brake
        if enter:
            self.danger_hits += 1
            self.safe_hits = 0
            if not self.braking and self.danger_hits >= self.enter_hits:
                self.brake_started = self.get_clock().now()
                self.braking = True
                cmd = AckermannDriveStamped()
                cmd.drive.speed = 0.0
                cmd.drive.steering_angle = 0.0
                self.drive_pub.publish(cmd)
                self.get_logger().info(f'BRAKE: min_iTTC={min_ittc:.2f}s, p10={p10:.2f}s, frac={fraction:.2f}')
        else:
            self.safe_hits += 1
            self.danger_hits = 0
            if self.braking and self.safe_hits >= self.release_hits and hold_time_ok and release_cond:
                self.braking = False
                self.brake_started = None
                self.get_logger().info("Releasing brake")

    def _hold_brake(self):
        if self.braking:
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            self.drive_pub.publish(msg)
            

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