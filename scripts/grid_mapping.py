#!/usr/bin/env python

import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan
import tf_conversions
import tf


class GridMapping:
    def __init__(self, map_center_x, map_center_y, map_size_x, map_size_y, resolution, laser_min_angle, laser_max_angle, laser_resolution, laser_max_dist):
        self.map_center_x = map_center_x          #meter
        self.map_center_y = map_center_y          #meter
        self.map_size_x = map_size_x              #meter
        self.map_size_y = map_size_y              #meter
        self.resolution = resolution              #meter/cell
        self.laser_min_angle = laser_min_angle    #radian
        self.laser_max_angle = laser_max_angle    #radian
        self.laser_resolution = laser_resolution  #radian
        self.laser_max_dist = laser_max_dist      #meter

    def to_xy (self, i, j):
        x = j * self.resolution
        y = (self.gridmap.shape[0] - i) * self.resolution
        return x, y

    def to_ij (self, x, y):
        i = self.gridmap.shape[0] - (y / self.resolution)
        j = x / self.resolution
        return i, j

    def is_inside (self, i, j):
        return i<self.gridmap.shape[0] and j<self.gridmap.shape[1] and i>=0 and j>=0

    def get_borders(self):
        return [0.0, self.gridmap.shape[1] * self.resolution, 0.0, self.gridmap.shape[0] * self.resolution]



class GridMappingROS:
    def __init__(self):
        rospy.init_node('RosGridMapping', anonymous=True)
        self.is_gridmapping_initialized = False
        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.laserscan_callback, queue_size=100)
        self.tf_sub = tf.TransformListener(cache_time = rospy.Duration(60)) # cache 1 min of tf. default: 10s

    def init_gridmapping(self, laser_min_angle, laser_max_angle, laser_resolution, laser_max_dist):
        self.gridmapping = GridMapping(self.map_center_x, self.map_center_y, self.map_size_x, self.map_size_y, self.resolution, laser_min_angle, laser_max_angle, laser_resolution, laser_max_dist)
        self.is_gridmapping_initialized = True

    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_conversion
    def quarternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return = np.arctan2(siny_cosp, cosy_cosp)

    def laserscan_callback(self, data):
        if not self.is_gridmapping_initialized:
            init_gridmapping(data.angle_min, data.angle_max, data.angle_increment, data.range_max)

        # get the position associated with the current laserscan
        try:
            (x, y, _),(qx, qy, qz, qw) = listener.lookupTransform('/base_link', '/odom', data.header.stamp)
            theta = self.quarternion_to_yaw(qx, qy, qz, qw)
            rospy.loginfo("tf:", x,y,theta)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF lookup error!")

    def spin(self):
        rate = rospy.Rate(10.0) # todo
        while not rospy.is_shutdown():


gm = GridMappingROS()
gm.spin()
