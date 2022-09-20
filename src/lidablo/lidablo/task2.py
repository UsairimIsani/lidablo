#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
import math

from matplotlib import pyplot as plt


class FeatureExtracter(Node):
    def __init__(self):
        print('Hi from lidablo.')
        super().__init__('feature_extracter')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.scan_idx = 0
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_policy)
        self.publisher_ = self.create_publisher(LaserScan, '/feature_scan', 10)

    # def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max):
    #     angle_step = (angle_max - angle_min) / len(ranges)
    #     angle = 0
    #     points = []
    #     for range in ranges:
    #         x = range * np.cos(angle)
    #         y = range * np.sin(angle)
    #         angle += angle_step
    #         points.append([x, y])

    #     points_np = np.array(points)
    #     print(points_np)
    #     plt.figure()
    #     plt.scatter(points_np[:, 0], points_np[:, 1])
    #     plt.show()

    #     return points

    def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max, angle_increment, msg):

        angle = angle_min               # start angle
        tf_points = []
        for range in ranges:
            x = range * np.cos(angle)
            y = range * np.sin(angle)

            # current angle is last angle add angle_increment
            angle += angle_increment
            tf_points.append([x, y])

        # transform the transformed points to numpy array
        points_np = np.array(tf_points)
        # plt.figure()
        # plt.scatter(points_np[:, 0], points_np[:, 1])
        # plt.savefig(f"img/{msg}")
        return points_np

    def detect_corners(self, msg):
        total_ranges = len(msg.ranges) - 1
        window_size = 15
        step_size = 5
        accepted_error_rad = 0.1745
        angles_and_errors = []
        ranges = self.polar_to_cartesian_coordinate(
            msg.ranges, msg.angle_min, msg.angle_max, msg.angle_increment, msg)
        for idx, _ in enumerate(ranges):
            last_index = idx + window_size
            if last_index < total_ranges:
                window = ranges[idx:last_index:step_size]
                a = np.array(window[0])
                b = np.array(window[1])
                c = np.array(window[2])

                ba = a - b
                bc = c - b

                cosine_angle = np.dot(ba, bc) / \
                    (np.linalg.norm(ba) * np.linalg.norm(bc))
                angle = np.arccos(cosine_angle)

                # Check angle and find error
                error = np.abs(angle - (np.pi/2))

                centers = (error, b)  # save center to
                # angles_and_errors.append(centers)
                # print(np.degrees(angle))
                if error <= accepted_error_rad:
                    print("Found angle", b)
                    angles_and_errors.append(b)

        angles_and_errors = np.array(angles_and_errors)
        plt.figure()
        plt.scatter(angles_and_errors[:, 0], angles_and_errors[:, 1])
        plt.savefig(f"img/corners")

    def detect_lines(self, msg):
        angle = msg.angle_min               # start angle
        tf_points = []
        for range in msg.ranges:
            x = range * np.cos(angle)
            y = range * np.sin(angle)

            # current angle is last angle add angle_increment
            angle += msg.angle_increment
            line = np.polyfit([x], [y], 1)
            print(line)

            tf_points.append([x, y])
        # transform the transformed points to numpy array
        points_np = np.array(tf_points)

        return points_np

    def scan_callback(self, msg):

        # self.polar_to_cartesian_coordinate(
        # msg.ranges, msg.angle_min,
        # msg.angle_max, msg.angle_increment)

        #################################
        ranges = []
        for r in msg.ranges:
            if r > 2.5 or r < 1.0:   # Feature extraction code here
                ranges.append(0.0)
            else:
                ranges.append(r)

        ##################################

        # line = self.detect_lines(msg)
        # print(line)

        self.detect_corners(msg)

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = msg.header.frame_id
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.range_min = msg.range_min
        scan.range_max = msg.range_max
        scan.ranges = ranges
        self.publisher_.publish(scan)

        self.scan_idx += 1
        print("Publish feature scan message idx", self.scan_idx)


def main(args=None):
    rclpy.init(args=args)
    feature_extracter = FeatureExtracter()
    rclpy.spin(feature_extracter)
    feature_extracter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
