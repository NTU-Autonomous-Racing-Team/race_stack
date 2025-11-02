#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

# source 

class AutomaticEmergencyBraking_Algorithm():
    def __init__(self, time2collision_threshold_0 = 0.1, view_angle = 0.7):
        self.time2collision_threshold_0= time2collision_threshold_0
        self.last_ranges = None
        self.angle_increment = None
        self.speed_gain = 1
        self.view_angle = view_angle 

    def calculate_speed_gain(self, ttc):
        if ttc <= self.time2collision_threshold_0:
            self.speed_gain = 0
        else:
            self.speed_gain = 1

    def calculate_time2collision(self):
        for i, radius in enumerate(self.ranges):
            # angle = self.angle_increment * len(self.ranges)//2 - i * self.angle_increment
            self.min_range_index = len(self.ranges)//2
            mid_angle = self.angle_increment * (self.lower_bound + self.min_range_index)
            angle = self.angle_increment * (self.lower_bound + i)
            angle = mid_angle - angle
            r_dot = max(self.linX * math.cos(angle), 0)
            if r_dot != 0:
                ttc = radius / r_dot
            else:
                ttc = float('inf')
            self.ttc = ttc

    def limit_field_of_view(self):
        view_angle_count = self.view_angle//self.angle_increment
        self.lower_bound = int((len(self.ranges)- view_angle_count)/2)
        self.upper_bound = int(self.lower_bound + view_angle_count)
        self.ranges = self.ranges[self.lower_bound:self.upper_bound]

    def update(self, ranges, odom):
        self.ranges = ranges
        self.linX = odom[3]
        # self.angle_max = self.angle_increment * len(ranges)
        self.limit_field_of_view()
        self.calculate_time2collision()
        self.calculate_speed_gain(self.ttc)
        return self.speed_gain

class AutomaticEmergencyBrakingNode(Node):
    def __init__(self):
        super().__init__('AEB')
        # Laser Scan
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.scan_subscriber
        self.scan_init = False
        # Odometry
        self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.odom_subscriber
        # WallFollower Subscriber
        self.drive_subscriber = self.create_subscription(AckermannDriveStamped, '/gap_finder/drive', self.drive_callback, 10)
        self.drive_subscriber
        self.set_drive = AckermannDriveStamped()
        # Drive Publisher
        self.pub_rate = 20
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(1/self.pub_rate, self.timer_callback)
        # AEB Algorithm
        self.aeb = AutomaticEmergencyBraking_Algorithm()
        self.cmd_drive = AckermannDriveStamped()
        # Memory
        self.ready = False

    def scan_callback(self, scan_msg):
        self.ready = True
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.aeb.angle_increment = scan_msg.angle_increment
        self.ranges = scan_msg.ranges

    def odom_callback(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        yaw = odom_msg.pose.pose.orientation.z
        linX = odom_msg.twist.twist.linear.x
        angZ = odom_msg.twist.twist.angular.z
        self.odom = [x, y, yaw, linX, angZ]
    
    def drive_callback(self, ackermann_msg):
        self.set_drive = ackermann_msg
        if self.ready:
            self.run()

    def run(self):
        speed_gain = self.aeb.update(self.ranges, self.odom)
        self.cmd_twist = self.set_drive
        self.cmd_twist.drive.speed *= speed_gain
        self.drive_publisher.publish(self.cmd_twist)

    def timer_callback(self):
        pass

def main(args = None):
    rclpy.init(args=args)
    auto_e_braking = AutomaticEmergencyBrakingNode()
    rclpy.spin(auto_e_braking)
    auto_e_braking.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()






# #TODO - Add config file to change TTC params (self.ttc_threshold)



# import math

# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import LaserScan
# from ackermann_msgs.msg import AckermannDriveStamped
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String

# class AutomaticEmergencyBraking(Node):
#     def __init__(self):
#         super().__init__('AEB')
#         self.time2collision = 5 # [s]
#         self.view_angle = 1.4 # [rad]
#         # Laser Scan
#         self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
#         self.scan_subscriber
#         self.scan_init = False
#         self.ranges = []
#         # Odometry
#         self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
#         self.odom_subscriber
#         self.odom = []
#         # Drive
#         self.pub_rate = 20
#         self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
#         self.timer = self.create_timer(1/self.pub_rate, self.timer_callback)
#         self.ttc_threshhold = 1 

#     def limit_field_of_view(self):
#         view_angle_count = self.view_angle//self.scan_angle_increment
#         lower_bound = int((len(self.ranges)- view_angle_count)/2)
#         upper_bound = int(lower_bound + view_angle_count)
#         self.ranges = self.ranges[lower_bound:upper_bound]

#     def scan_callback(self, scan_msg):
#         if not self.scan_init:
#             self.scan_angle_increment = scan_msg.angle_increment
#             self.scan_angle_max = scan_msg.angle_max
#             self.scan_angle_min = scan_msg.angle_min
#             self.len_range = len(scan_msg.ranges)
#         self.ranges = scan_msg.ranges
#         self.limit_field_of_view()

#     def odom_callback(self, odom_msg):
#         x = odom_msg.pose.pose.position.x
#         y = odom_msg.pose.pose.position.y
#         yaw = odom_msg.pose.pose.orientation.z
#         linX = odom_msg.twist.twist.linear.x
#         angZ = odom_msg.twist.twist.angular.z
#         self.odom = [x, y, yaw, linX, angZ]
        
#     def drive_publish(self, drive_msg):
#         self.drive_publisher.publish(drive_msg)

#     def collision(self, ranges, linX):
#         for i, radius in enumerate(ranges):
#             angle = self.scan_angle_max - i * self.scan_angle_increment
#             r_dot = max(-1 * linX * math.cos(angle), 0)
#             if r_dot != 0:
#                 ttc = radius / r_dot
#             else:
#                 ttc = float('inf')
#             if ttc <= self.ttc_threshhold:
#                 self.e_brake = True
#                 break

#     def timer_callback(self):
#         ranges = self.ranges
#         linX = self.odom[3]
#         self.collision(ranges, linX)
#         if self.e_brake = True:
#             self.drive_publish(AckermannDriveStamped())

# def main(args = None):
#     rclpy.init(args=args)
#     auto_e_braking = AutomaticEmergencyBraking()
#     rclpy.spin(auto_e_braking)
#     auto_e_braking.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()
