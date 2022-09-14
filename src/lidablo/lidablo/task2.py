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
        angle = msg.angle_min
        tf_points = []

        for r in msg.ranges:
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            angle += msg.angle_increment
            tf_points.append([x, y])

        points_np = np.array(tf_points)

        step = 9
        r = len(points_np) - step
        previous_angle = 0
        for i in range(0, r+1, step):
            x_sub = points_np[i:i+step, 0]
            y_sub = points_np[i:i+step, 1]

            a = np.array([x_sub[0], y_sub[0]])
            b = np.array([x_sub[3], y_sub[3]])
            c = np.array([x_sub[8], y_sub[8]])

            ba = a - b
            bc = c - b

            cosine_angle = np.dot(ba, bc) / \
                (np.linalg.norm(ba) * np.linalg.norm(bc))

            angle = np.arccos(cosine_angle)
            angle = np.degrees(angle)

            if angle - previous_angle > 60 and angle - previous_angle < 120:
                print("Angle ", angle)

            previous_angle = angle
            # print("Previous angle : ", previous_angle)
            # print("Angle : ", angle)
            # print(np.degrees(angle))

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
