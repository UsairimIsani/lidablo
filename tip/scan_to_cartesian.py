#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan

from matplotlib import pyplot as plt

class ScanToCatersian(Node):
    def __init__(self):
        super().__init__('feature_extracter')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1) 
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_policy)
             
    
    def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max,angle_increment):
 
        angle = angle_min               # start angle  
        tf_points = []         
        for range in ranges:
            x = range * np.cos(angle)
            y = range * np.sin(angle)
            
            # current angle is last angle add angle_increment
            angle += angle_increment     
            tf_points.append([x,y])

        # Plot the transformed points
        points_np = np.array(tf_points)
        plt.figure()
        plt.scatter(points_np[:,0], points_np[:,1]) 
        # plt.draw()
        # plt.pause(1)  # pause 1 senconds
        # plt.close()
        # # plt.show(block=False) 
        plt.show()
        

    def scan_callback(self,msg):
        
        self.polar_to_cartesian_coordinate(
                msg.ranges, 
                msg.angle_min, 
                msg.angle_max,
                msg.angle_increment)



def main(args=None):
    rclpy.init(args=args)
    feature_extracter = ScanToCatersian()
    rclpy.spin(feature_extracter) 
    feature_extracter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()