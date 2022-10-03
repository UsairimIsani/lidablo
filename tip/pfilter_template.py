#!/usr/bin/env python
import rclpy
from rclpy.node import Node

import sys
import time
from pfilter import (
    ParticleFilter,
    gaussian_noise,
    cauchy_noise,
    t_noise,
    squared_error,
    independent_sample,
)
import numpy as np

from scipy.stats import norm, gamma, uniform

from std_msgs.msg       import Float64
from sensor_msgs.msg    import LaserScan
from geometry_msgs.msg  import PoseStamped
from geometry_msgs.msg  import PoseWithCovarianceStamped
from geometry_msgs.msg  import PoseArray
from sensor_msgs.msg    import Range
from nav_msgs.msg       import Odometry
from rclpy.clock        import Clock
from rclpy.duration     import Duration
from pfilter            import ParticleFilter, squared_error


from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


import matplotlib.pyplot as plt


class LidarParticleFilter(Node) :
    '''
        ROS Node that estimates relative position of two robots using odometry and lidar detections
        given a known map or set of objects in known positions
    '''

    def __init__(self) :

        # Init node
        super().__init__('lidar_position_pf_rclpy')

        # Define QoS profile for odom and UWB subscribers
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,
        )

        # Particle filter params
        # Parameters for ros2 launch command
        self.declare_parameters(
            namespace = '',
            parameters=[
                ("weights_sigma", 1.2),
                ("num_particles", 200),
                ("measurement_noise", 0.05),
                ("resample_proportion", 0.01),
                ("max_pos_delay", 0.2)
            ]
        )
        
        self.weights_sigma = self.get_parameter('weights_sigma').value
        self.num_particles = self.get_parameter('num_particles').value
        self.measurement_noise = self.get_parameter('measurement_noise').value
        self.resample_proportion = self.get_parameter('resample_proportion').value

        self.get_logger().info('weights_sigma: %f, num_particles: %d, measurement_noise: %f, resample_proportion: %f'  % 
                            (self.weights_sigma,
                             self.num_particles,
                             self.measurement_noise,
                             self.resample_proportion))


        # Create filter
        self.prior_fn = lambda n: np.random.uniform(-5,5,(n,2))
        self.pf = ParticleFilter(
            prior_fn =              self.prior_fn, 
            observe_fn =            self.calc_hypothesis,  
            dynamics_fn =           lambda x: x, #self.velocity, <- when you calculate particle "velocity" from simulator or other data
            n_particles =           self.num_particles, 
            noise_fn =              self.add_noise, #lambda x: x + np.random.normal(0, noise, x.shape),
            weight_fn =             self.calc_weights, #lambda x, y : squared_error(x, y, sigma=2),
            resample_proportion =   self.resample_proportion
        )

        # This is an example, the particle_odom has to be calculated
        # every time the filter is run
        self.particle_odom = np.array([0.01,0.01])

        self.get_logger().info("Creating subscribers")
        self.odom_sub = self.create_subscription(Odometry, "/odom",  self.odometry_cb, qos_profile=self.qos)
        self.scan_sub = self.create_subscription(LaserScan, "/scan",  self.scan_cb, qos_profile=self.qos)

        self.odometry = Odometry()
        self.scan = LaserScan()

        # Real obstacle position <- NOTE! This depends on your simulator
        self.real_obstacle_position = np.array([2,-2]) # TODO! Remember to change this!!!

        self.get_logger().info("Particle filter initialized")

    def odometry_cb(self, odom):
        self.odometry = odom

    def scan_cb(self, scan):
        self.scan = scan

    def calc_hypothesis(self, x) :
        '''
            Given (Nx2) matrix of positions,
            create N arrays of observations
            (observations are what the particles WOULD see from their position given in 'x')
        '''        
        y = self.real_obstacle_position - x # TODO! Add orientation here!
        return y

    def velocity(self, x) :
        '''
            Use odometry to update robot position
        '''
        xp = x + self.particle_odom
        return xp

    def add_noise(self, x) :
        '''
            Add noise to the estimations
        '''
        xp = x + np.random.normal(0, self.measurement_noise, x.shape)
        return xp

    def calc_weights(self, hypotheses, observations) :
        '''
            Calculate particle weights based on error
        '''
        w = squared_error(hypotheses, observations, sigma=self.weights_sigma)
        return w

    def update_filter(self) :
        '''
            Upadate particle filter
        '''

        print("Robot pose {}".format(self.odometry.pose.pose.position))

        # self.particle_odom = CALCULATE_FROM_SIMULATOR_ODOMETRY

        robot_pos=np.array([self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y])

        # This should come from the lidar data
        relative_obstacle_pos = self.real_obstacle_position - robot_pos + np.random.normal(0,0.05,2)
        print("Relative position of object: {}".format(relative_obstacle_pos))

        self.pf.update(observed=relative_obstacle_pos)

        print("Robot position is {}".format(self.pf.mean_state))


def main(args=None):
    
    rclpy.init(args=args)
    filter = LidarParticleFilter()
    
    # Init filter
    filter.pf.init_filter()

    time.sleep(1)
    rclpy_check_rate = 10
    
    rclpy_check_rate = filter.create_rate(10, filter.get_clock())

    filter.get_logger().info("Starting particle filter...")
    try:
        try:
            while rclpy.ok() :
                # This now runs at 5 Hz, change accordingly to your setup
                filter_timer = filter.create_timer(0.2, filter.update_filter)
                rclpy.spin(filter)             
                pass
            

        except KeyboardInterrupt :
            filter.get_logger().error('Keyboard Interrupt detected! Trying to stop filter node!')
    except Exception as e:
        filter.destroy_node()
        filter.get_logger().info("UWB particle filter failed %r."%(e,))
    finally:
        rclpy.shutdown()
        filter.destroy_node()   



if __name__ == "__main__":

    main()


