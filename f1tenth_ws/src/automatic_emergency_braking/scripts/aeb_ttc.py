#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32

config = {
    'time_threshold': 0.1, # [s]
    'view_angle': 1.0, # [radians]
    'pub_rate': 10, # [Hz]
    'drive_topic': '/drive',
    'scan_topic': '/scan',
    'odom_topic': '/odom',
}

class TimeToCollision_Algorithm():
    def __init__(self, view_angle = 1, view_angle_increment = 0.00436332309619):
        self.ttc = 0 # [s]
        self.angle_increment = view_angle_increment # [radians]
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
        self.scan_subscriber = self.create_subscription(LaserScan, config['scan_topic'], self.scan_callback, 10)
        self.scan_subscriber
        # Odometry
        self.odom_subscriber = self.create_subscription(Odometry, config['odom_topic'], self.odom_callback, 10)
        self.odom_subscriber
        # AEB Algorithm
        self.time_to_collision = TimeToCollision_Algorithm(
            view_angle = config['view_angle']
            # view_angle_increment will be set on first scan callback
        )
        # TTC Publisher
        self.aeb_publisher = self.create_publisher(Float32, config['drive_topic'], 1)
        self.timer = self.create_timer(1/config['pub_rate'], self.timer_callback)

        self.scan_waiting = True
        self.odom_waiting = True

    def scan_callback(self, scan_msg):
        if (self.scan_waiting):
            self.time_to_collision.angle_increment = scan_msg.angle_increment
            self.scan_waiting = False
        # self.angle_min = scan_msg.angle_min
        # self.angle_max = scan_msg.angle_max
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

        ttc = self.time_to_collision.update(self.ranges, self.odom)
        if (ttc < config['time_threshold']):
            drive_msg = AckermannDriveStamped()
            self.get_logger().info(f'AEB Activated! TTC: {ttc:.3f} s')
            self.aeb_publisher.publish(drive_msg)

def main(args = None):
    rclpy.init(args=args)
    auto_e_braking = AutomaticEmergencyBrakingNode()
    rclpy.spin(auto_e_braking)
    auto_e_braking.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
