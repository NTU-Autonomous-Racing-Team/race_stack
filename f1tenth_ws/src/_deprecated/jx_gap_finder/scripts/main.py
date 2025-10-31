#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np

from jx_gap_finder.pid import PID

# reference: https://github.com/f1tenth/f1tenth_labs_openrepo/blob/main/f1tenth_lab4/README.md
# reference: https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html 

class GapFinderAlgorithm:
    """
    This class implements the gap finder algorithm. The algorithm takes in a list of ranges from a LiDAR scan and
    returns a twist dictionary that will move the car to the deepest gap in the scan after drawing safety bubbles.
    params:
        - view_angle: the angle of the field of view of the LiDAR as a cone in front of the car
        - coeffiecient_of_friction: the coeffiecient of friction of the car used for speed calculation
        - wheel_base: the distance between the front and rear axles of the car
        - lookahead: the maximum distance to look ahead in the scan
        - speed_pid: a PID controller for the linear velocity
        - steering_pid: a PID controller for the angular velocity
    """
    def __init__(self,  safety_bubble_diameter = 0.2,
                        view_angle = 3.142, 
                        coeffiecient_of_friction = 0.71, 
                        vertice_detection_threshold = 0.6,
                        lookahead = None, 
                        speed_kp = 1.0,
                        steering_kp = 1.2,
                        wheel_base = 0.324, 
                        visualise = False):
        # Tunable Parameters
        self.safety_bubble_diameter = safety_bubble_diameter  # [m]
        self.view_angle = view_angle  # [rad]
        self.coeffiecient_of_friction = coeffiecient_of_friction
        self.lookahead = lookahead # [m]
        self.change_threshold = vertice_detection_threshold # [m]
        self.wheel_base = wheel_base  # [m]
        # Controller Parameters
        self.speed_pid = PID(Kp=-speed_kp)
        self.speed_pid.set_point = 0.0
        self.steering_pid = PID(Kp=-steering_kp)
        self.steering_pid.set_point = 0.0
        # Misc
        self.visualise = visualise




    def update(self, ranges, angle_increment, scan_msg):
        ranges = np.array(ranges)

        disparities = []
        safe_ranges = ranges.copy()

        # find disparities
        for i in range(1, len(ranges)):
            if abs(ranges[i] - ranges[i-1]) > self.change_threshold:
                disparities.append(i)

        # process each disparity
        for index in disparities:
            # marks the point of obstacle
            closer_index = index - 1 if ranges[index - 1] < ranges[index] else index
            distance = ranges[closer_index]

            # calculate # of samples to cover half of the car width
            num_samples = int(self.safety_bubble_diameter / (angle_increment * distance))

            # extend parity
            for j in range(max(0, index - num_samples), min(len(ranges), index + num_samples)):
                safe_ranges[j] = min(safe_ranges[j], distance)

        # find the best gap within the safe ranges
        # search only forward facing ranges (+- 90 deg)
        half_range_count = int(len(ranges) / 2)
        start_index = half_range_count - int(np.pi / 2 / angle_increment)
        end_index = half_range_count + int(np.pi / 2 / angle_increment)

        max_gap_index = np.argmax(safe_ranges[start_index:end_index]) + start_index
        max_distance = safe_ranges[max_gap_index]
        goal_bearing = (max_gap_index - half_range_count) * angle_increment

        # calculate steering and speed
        init_steering = np.arctan(goal_bearing * self.wheel_base)
        steering = self.steering_pid.update(init_steering)
        init_speed = np.sqrt(
            10 * self.coeffiecient_of_friction * self.wheel_base / np.abs(max(np.tan(abs(steering)), 1e-9)))
        speed = self.speed_pid.update(init_speed)
        ackermann = {"speed": speed, "steering": steering}
        return ackermann

    def get_bubble_coord(self):
        m = []
        for marker in self.marked_ranges:
            x = marker[1] * np.cos(marker[0])
            y = marker[1] * np.sin(marker[0])
            m.append([x, y])
        return m

    def get_goal_coord(self):
        x = self.goal_range * np.cos(self.goal_bearing)
        y = self.goal_range * np.sin(self.goal_bearing)
        return [x, y]

    def get_safety_scan(self):
        return self.safety_scan_msg


class GapFinderNode(Node):
    """
    ROS2 Node Class that handles all the subscibers and publishers for the gap finder algorithm. 
    It abstracts the gap finder algorithm from the ROS2 interface.=
    """
    def __init__(self, hz=50):
        ### GAP FINDER ALGORITHM ###
        self.gapFinderAlgorithm = GapFinderAlgorithm(safety_bubble_diameter = 0.5, 
                                                     view_angle = 3.142, 
                                                     coeffiecient_of_friction = 0.71, 
                                                     vertice_detection_threshold = 0.5/2,
                                                     lookahead = 10, 
                                                     speed_kp = 1.0,
                                                     steering_kp = 1.5, 
                                                     wheel_base = 0.324, 
                                                     visualise=False)

        ### SPEED AND STEERING LIMITS ###
        # Speed limits
        self.max_speed = 10.0 # [m/s]
        self.min_speed = 1.0 # [m/s]
        # Steering limits
        self.max_steering = 0.4 # [rad]

        ### ROS2 NODE ###
        self.timeout = 1.0 # [s]
        self.vizualize = self.gapFinderAlgorithm.visualise
        super().__init__("gap_finder")
        # Scan Subscriber
        self.scan_subscriber = self.create_subscription(LaserScan, "scan", self.scan_callback, 1)
        self.scan_subscriber  # prevent unused variable warning
        self.scan_ready = False
        self.last_scan_time = self.get_time()
        # Odom Subscriber
        self.odom_subscriber = self.create_subscription(Odometry, "ego_racecar/odom", self.odom_callback, 1)
        self.odom_subscriber
        self.odom_ready = False
        self.last_odom_time = self.get_time()
        # Drive Publisher
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, "/drive", 1)
        # Timer
        self.timer = self.create_timer(1/hz , self.timer_callback)
        # Viz Publishers
        if self.vizualize:
            # Safety Viz Publisher
            self.bubble_viz_publisher = self.create_publisher(MarkerArray, "/safety_bubble", 1)
            # Goal Viz Publisher
            self.gap_viz_publisher = self.create_publisher(Marker, "/goal_point", 1)
            # Laser Viz Publisher
            self.laser_publisher = self.create_publisher(LaserScan, "/safety_scan", 1)
        # Memory
        self.last_time = self.get_time()
        self.last_twist = {"speed": 0.0, "steering":0.0}

    def get_time(self):
        return self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

    def scan_callback(self, scan_msg):
        self.scan_ready = True
        self.scan_msg = scan_msg
        self.last_scan_time = self.get_time()

    def odom_callback(self, odom_msg):
        self.odom_ready = True
        self.odom_msg = odom_msg
        self.last_odom_time = self.get_time()

    def publish_drive_msg(self, twist={"speed": 0.0, "steering": 0.0}):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(twist["speed"])
        drive_msg.drive.steering_angle = float(twist["steering"])
        self.drive_publisher.publish(drive_msg)

    def publish_viz_msgs(self):
        safety_scan = self.gapFinderAlgorithm.get_safety_scan()
        bubble_coord = self.gapFinderAlgorithm.get_bubble_coord()
        goal_coord = self.gapFinderAlgorithm.get_goal_coord()
        safety_bubble_diameter = self.gapFinderAlgorithm.safety_bubble_diameter
        bubble_viz_msg = MarkerArray()
        gap_viz_msg = Marker()
        laser_viz_msg = safety_scan

        for i, coord in enumerate(bubble_coord):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "ego_racecar/base_link"
            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.scale.x = safety_bubble_diameter
            marker.scale.y = safety_bubble_diameter
            marker.scale.z = safety_bubble_diameter
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            bubble_viz_msg.markers.append(marker)

        gap_viz_msg.header.frame_id = "ego_racecar/base_link"
        gap_viz_msg.pose.position.x = goal_coord[0]
        gap_viz_msg.pose.position.y = goal_coord[1]
        gap_viz_msg.color.a = 1.0
        gap_viz_msg.color.g = 1.0
        gap_viz_msg.scale.x = 0.3
        gap_viz_msg.scale.y = 0.3
        gap_viz_msg.scale.z = 0.3
        gap_viz_msg.type = Marker.SPHERE
        gap_viz_msg.action = Marker.ADD

        self.bubble_viz_publisher.publish(bubble_viz_msg)
        self.gap_viz_publisher.publish(gap_viz_msg)
        self.laser_publisher.publish(laser_viz_msg)


    def timer_callback(self):
        if self.scan_ready and self.odom_ready:
            dt = self.get_time() - self.last_time

            twist = self.gapFinderAlgorithm.update(self.scan_msg.ranges, self.scan_msg.angle_increment, self.scan_msg)

            # steering limits
            twist["steering"] = np.sign(twist["steering"]) * min(np.abs(twist["steering"]), self.max_steering)
            # speed limits
            twist["speed"] = max(twist["speed"], self.min_speed)
            twist["speed"] = min(twist["speed"], self.max_speed)

            self.publish_drive_msg(twist)
            self.last_time = self.get_time()
        
            if self.vizualize:
                self.publish_viz_msgs()

        if ((self.get_time() - self.last_scan_time) > self.timeout):
            self.scan_ready = False
        if ((self.get_time() - self.last_odom_time) > self.timeout):
            self.odom_ready = False

def main(args=None):
    rclpy.init(args=args)

    gapFinder = GapFinderNode()

    rclpy.spin(gapFinder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gapFinder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
