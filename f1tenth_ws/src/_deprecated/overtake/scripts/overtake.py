#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import math


CAR_DETECTED_DURATION_THRESHOLD = 0.5
FALSE_CAR_DETECTION_THRESHOLD = 0.030
EDGE_DETECTION_THRESHOLD = 0.7
DISTANCE_BETWEEN_EDGES_LOWER_BOUND = 0.25
DISTANCE_BETWEEN_EDGES_UPPER_BOUND = 0.35


class Overtake(Node):
    def __init__(self):
        super().__init__("overtake")

        # Subscribing to relevant topics
        self.sub_scan = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 1
        )

        self.maybe_detected_car = False

        self.car_detected_start_time = 0.0

    def scan_callback(self, scan_data):
        ranges = scan_data.ranges
        angle_increment = scan_data.angle_increment
        current_scan_time = (
            scan_data.header.stamp.sec + scan_data.header.stamp.nanosec * 1e-9
        )

        # To keep track of how long its been since a car was detected
        time_elapsed_since_last_car_detection = 0.0
        if self.maybe_detected_car:
            if self.car_detected_start_time == 0.0:
                self.car_detected_start_time = current_scan_time
            time_elapsed_since_last_car_detection = (
                current_scan_time - self.car_detected_start_time
            )
        else:
            self.car_detected_start_time = 0.0

        self.maybe_detected_car = False
        first_edge_found = False
        distance_between_edges = 0.0

        for i in range(1, len(ranges)):
            range_a = ranges[i - 1]
            range_b = ranges[i]
            delta = abs(range_a - range_b)

            # TODO: Add comment explaining why this is needed
            # why were checking for FALSE_CAR_DETECTION_THRESHOLD, etc.
            if first_edge_found and delta < FALSE_CAR_DETECTION_THRESHOLD:
                distance_between_edges += math.sqrt(
                    range_a**2
                    + range_b**2
                    - 2 * range_a * range_b * math.cos(angle_increment)
                )

            # Finding right and left edges of the car by using a distance_threshold
            if delta > EDGE_DETECTION_THRESHOLD:
                first_edge_found = True

                # TODO: Add comment explaining why this is needed
                if (
                    distance_between_edges > DISTANCE_BETWEEN_EDGES_UPPER_BOUND
                    or distance_between_edges < DISTANCE_BETWEEN_EDGES_LOWER_BOUND
                ):
                    distance_between_edges = 0.0
                    continue

                # TODO: Add comment explaining why this is needed
                self.maybe_detected_car = True

                # TODO: Add comment explaining why this is needed
                if (
                    time_elapsed_since_last_car_detection
                    < CAR_DETECTED_DURATION_THRESHOLD
                ):
                    continue

                print(f"Car detected")
                break


def main(args=None):

    rclpy.init(args=args)

    overtake = Overtake()

    rclpy.spin(overtake)

    overtake.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
