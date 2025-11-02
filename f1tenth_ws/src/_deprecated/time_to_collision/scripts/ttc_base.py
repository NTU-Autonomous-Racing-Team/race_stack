#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

import numpy as np

# source 

class TimeToCollision_Algorithm():
    def __init__(self, view_angle = 1):
        self.ttc = 0 # [s]
        self.angle_increment = None # [radians]
        self.view_angle = view_angle # [radians]

    def calculate_time2collision(self):
        view = np.array(self.limited_ranges)
        angles = np.linspace(start=-self.view_angle/2, stop=self.view_angle/2, num = self.view_angle_count)
        r_dot = self.linX * np.cos(angles)
        ttc = np.select([view / r_dot > 0], [view / r_dot], default = float('inf'))
        self.ttc = np.min(ttc)

    def limit_field_of_view(self):
        self.view_angle_count = int(self.view_angle//self.angle_increment)
        self.lower_bound_index = int((len(self.ranges) - self.view_angle_count)/2)
        self.upper_bound_index = int(self.lower_bound_index + self.view_angle_count)
        self.limited_ranges = self.ranges[self.lower_bound_index:self.upper_bound_index]

    def update(self, ranges, odom):
        self.ranges = ranges
        self.linX = odom[3]
        self.limit_field_of_view()
        self.calculate_time2collision()
        return self.ttc

class AutomaticEmergencyBrakingNode(Node):
    def __init__(self):
        super().__init__('AEB')
        # Laser Scan
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.scan_subscriber
        self.scan_init = False
        # Odometry
        # self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_subscriber
        # AEB Algorithm
        self.aeb = TimeToCollision_Algorithm()
        self.cmd_drive = AckermannDriveStamped()
        # TTC Publisher
        self.ttc_publisher = self.create_publisher(Float32, 'ttc', 1)
        self.timer = self.create_timer(1/10, self.timer_callback)

        self.scan_waiting = True
        self.odom_waiting = True

    def scan_callback(self, scan_msg):
        self.scan_waiting = False
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.aeb.angle_increment = scan_msg.angle_increment
        self.ranges = scan_msg.ranges

    def odom_callback(self, odom_msg):
        self.odom_waiting = False
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        yaw = odom_msg.pose.pose.orientation.z
        linX = odom_msg.twist.twist.linear.x
        angZ = odom_msg.twist.twist.angular.z
        self.odom = [x, y, yaw, linX, angZ]
    
    def timer_callback(self):
        if self.scan_waiting or self.odom_waiting:
            return
        ttc = self.aeb.update(self.ranges, self.odom)
        msg = Float32()
        msg.data = ttc
        self.ttc_publisher.publish(msg)

def main(args = None):
    rclpy.init(args=args)
    auto_e_braking = AutomaticEmergencyBrakingNode()
    rclpy.spin(auto_e_braking)
    auto_e_braking.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
