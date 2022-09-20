#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
import math

from matplotlib import pyplot as plt


class MapConstructor(Node):
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
    map_constructor = MapConstructor()
    rclpy.spin(map_constructor)
    map_constructor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
