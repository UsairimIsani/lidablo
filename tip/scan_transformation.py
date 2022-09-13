#! /usr/bin/env python

from cProfile import label
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan

from matplotlib import pyplot as plt

class ScanTransform(Node):
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
        self.publisher_ = self.create_publisher(LaserScan, '/feature_scan', 10)
  
    
    def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max, angle_increment):
 
        angle = angle_min               # start angle  
        tf_points = []         
        for range in ranges:
            x = range * np.cos(angle)
            y = range * np.sin(angle)
            
            # current angle is last angle add angle_increment
            angle += angle_increment     
            tf_points.append([x,y])

        # transform the transformed points to numpy array
        points_np = np.array(tf_points) 
        
        return points_np

    def rotate_pts(self, data, theta):
        x_ = data[:,0] * np.cos(theta) + data[:,1] * np.sin(theta)  
        y_=  data[:,0] * np.sin(theta) - data[:,1] * np.cos(theta)   
        # print(x_, y_)
        return [x_, y_]
        
    def translate_pts(self, data, tx, ty):
        x_ =  data[:,0]
        y_ =  data[:,1]

        x_  = x_ + tx  
        y_  = y_ + ty  
        return [x_, y_]
    
    def transform_pts(self, data, tx,ty, theta):
        x_ = data[:,0] * np.cos(theta) + data[:,1] * np.sin(theta) + tx
        y_=  data[:,0] * np.sin(theta) - data[:,1] * np.cos(theta) + ty
        # print(x_, y_)
        return [x_, y_]

    def scan_callback(self,msg):
        
        scan_data = self.polar_to_cartesian_coordinate(
                    msg.ranges, msg.angle_min, 
                    msg.angle_max, msg.angle_increment)
        
        tf_points = self.rotate_pts(scan_data, np.pi / 6)
        
        # Plot the transformed data
        points_np = np.array(tf_points)
        plt.figure() 
        plt.scatter(scan_data[:,0], scan_data[:,1], label = "raw_data") 
        plt.scatter(points_np[0,:], points_np[1,:], label = "tf_data") 
        plt.grid() 
        plt.legend()
        plt.show()
        

 
def main(args=None):
    rclpy.init(args=args)
    scan_transform = ScanTransform()
    rclpy.spin(scan_transform) 
    scan_transform.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()