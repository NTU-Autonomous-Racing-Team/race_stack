#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

config = {
    'safety_bubble_diameter': 0.4,
    'frictional_coefficient': 0.7, # carpet ~ 0.6-0.8 from https://www.jstage.jst.go.jp/article/jte2000/47/2/47_2_53/_pdf/-char/ja, https://www.engineeringtoolbox.com/friction-coefficients-d_778.html, 
    'max_steering_angle': 0.34,
    'wheelbase': 0.25,
    'view_angle': 3.142/2.0, 
    'pub_rate': 10, 
    'drive_topic': '/drive',
    'scan_topic': '/scan',
    'odom_topic': '/odom'
}

class AEB_Ackerman():
    def __init__(self, safety_bubble_diameter = 0.3, 
                 frictional_coefficient = 3.0, 
                 max_steering_angle = 0.8,
                 wheelbase = 0.33,
                 view_angle = 3.142/2.0):
        # CONSTANTS
        self.G = 9.81
        # TUNABLE PARAMETERS
        self.safety_bubble_diameter = safety_bubble_diameter # Estimated as the length of the car
        self.FRICTIONAL_COEFFICIENT = frictional_coefficient
        self.MAX_STEERING_ANGLE = max_steering_angle
        self.WHEELBASE = wheelbase
        self.VIEW_ANGLE = view_angle

    def limit_field_of_view(self, ranges, angle_increment):
        view_angle_count = self.VIEW_ANGLE//angle_increment
        lower_bound = int((len(ranges)- view_angle_count)/2)
        upper_bound = int(lower_bound + view_angle_count)
        ranges = ranges[lower_bound:upper_bound]
        return ranges

    def update(self, scan_msg, odom_msg):
        ranges = scan_msg.ranges
        angle_increment = scan_msg.angle_increment
        speed = odom_msg.twist.twist.linear.x
        if self.VIEW_ANGLE is not None:
            ranges = self.limit_field_of_view(ranges, angle_increment)

        # Calculate Nearest Point Coordinates
        nearest_point_range = min(ranges)
        nearest_point_index = ranges.index(nearest_point_range)
        nearest_point_angle = (len(ranges)/2 - nearest_point_index) * angle_increment
        nearest_coordinate_x = nearest_point_range * np.cos(nearest_point_angle)
        nearest_coordinate_y = nearest_point_range * np.sin(nearest_point_angle)

        # Calculate Instantaneous Rotation Radius
        instantaneous_rotation_radius  = abs(self.WHEELBASE / np.tan(self.MAX_STEERING_ANGLE)) # from no slip bicycle model
        grip_limited_radius = speed**2/ (self.FRICTIONAL_COEFFICIENT * self.G) # from centripetal force
        if instantaneous_rotation_radius < grip_limited_radius:
            instantaneous_rotation_radius = grip_limited_radius

        # Calculate ICR Coordinates (Instantaneous Center of Rotation)
        if nearest_coordinate_y > 0:
            # nearest point is on left, so turn right i.e. ICR is on right
            icr_coordinates = [0, -instantaneous_rotation_radius]
        else:
            # nearest point is on right, so turn left i.e. ICR is on left
            icr_coordinates = [0, instantaneous_rotation_radius]

        # Calculate Distance to ICR (Instantaneous Center of Rotation)
        distance = np.sqrt((nearest_coordinate_x - icr_coordinates[0])**2 + (nearest_coordinate_y - icr_coordinates[1])**2)

        # Check Collision
        if abs(distance - instantaneous_rotation_radius) < self.safety_bubble_diameter/2:
            # Collision
            return True
        else:
            # No Collision
            return False


class AutomaticEmergencyBrakingNode(Node):
    def __init__(self, pub_rate = 10):
        super().__init__('aeb_ackermann')
        # Laser Scan
        self.scan_subscriber = self.create_subscription(LaserScan, config['scan_topic'], self.scan_callback, 1)
        self.scan_subscriber
        # Odometry
        self.odom_subscriber = self.create_subscription(Odometry, config['odom_topic'], self.odom_callback, 1)
        self.odom_subscriber
        # AEB Publisher
        self.aeb_publisher = self.create_publisher(AckermannDriveStamped, config['drive_topic'], 1)
        self.timer = self.create_timer(1/config['pub_rate'], self.timer_callback)
        # AEB Algorithm
        self.aeb = AEB_Ackerman(
            safety_bubble_diameter = config['safety_bubble_diameter'],
            frictional_coefficient = config['frictional_coefficient'],
            max_steering_angle = config['max_steering_angle'],
            wheelbase = config['wheelbase'],
            view_angle = config['view_angle']
        )
        # Memory
        self.scan_waiting = True
        self.odom_waiting = True

    def scan_callback(self, scan_msg):
        self.scan_waiting = False
        self.scan_msg = scan_msg

    def odom_callback(self, odom_msg):
        self.odom_waiting = False
        self.odom_msg = odom_msg

    def timer_callback(self):
        if self.scan_waiting or self.odom_waiting:
            return
        drive_msg = AckermannDriveStamped()
        if (self.aeb.update(self.scan_msg, self.odom_msg)):
            self.aeb_publisher.publish(drive_msg) # Publish zero speed message

def main(args = None):
    rclpy.init(args=args)
    auto_e_braking = AutomaticEmergencyBrakingNode()
    rclpy.spin(auto_e_braking)
    auto_e_braking.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
