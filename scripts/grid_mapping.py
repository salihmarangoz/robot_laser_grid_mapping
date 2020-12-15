#!/usr/bin/env python

import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan
import tf


class GridMapping:
    def __init__(self, map_center_x, map_center_y, map_size_x, map_size_y, map_resolution, laser_min_angle, laser_max_angle, laser_resolution, laser_max_dist):
        self.map_center_x = map_center_x          #meter
        self.map_center_y = map_center_y          #meter
        self.map_size_x = map_size_x              #meter
        self.map_size_y = map_size_y              #meter
        self.map_resolution = map_resolution      #meter/cell
        self.laser_min_angle = laser_min_angle    #radian
        self.laser_max_angle = laser_max_angle    #radian
        self.laser_resolution = laser_resolution  #radian
        self.laser_max_dist = laser_max_dist      #meter

# TODOOOOOO
    def to_xy (self, i, j):
        x = j * self.map_resolution
        y = (self.gridmap.shape[0] - i) * self.map_resolution
        return x, y

    def to_ij (self, x, y):
        i = self.gridmap.shape[0] - (y / self.map_resolution)
        j = x / self.map_resolution
        return i, j

    def is_inside (self, i, j):
        return i<self.gridmap.shape[0] and j<self.gridmap.shape[1] and i>=0 and j>=0

    def get_borders(self):
        return [0.0, self.gridmap.shape[1] * self.map_resolution, 0.0, self.gridmap.shape[0] * self.map_resolution]
# TODOOOOOO


    def update(self, x, y, theta, scan):
        return
        print("update!")



class GridMappingROS:
    def __init__(self):
        rospy.init_node('RosGridMapping', anonymous=True)
        self.is_gridmapping_initialized = False

        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.map_frame = rospy.get_param('~map_frame', 'odom')
        self.map_center_x = rospy.get_param('~map_center_x', 0.0)
        self.map_center_y = rospy.get_param('~map_center_y', 0.0)
        self.map_size_x = rospy.get_param('~map_size_x', 10.0)
        self.map_size_y = rospy.get_param('~map_size_y', 10.0)
        self.map_resolution = rospy.get_param('~map_resolution', 0.05)

        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.laserscan_callback, queue_size=100)
        self.tf_sub = tf.TransformListener()

    def init_gridmapping(self, laser_min_angle, laser_max_angle, laser_resolution, laser_max_dist):
        self.gridmapping = GridMapping(self.map_center_x, self.map_center_y, self.map_size_x, self.map_size_y, self.map_resolution, laser_min_angle, laser_max_angle, laser_resolution, laser_max_dist)
        self.is_gridmapping_initialized = True

    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_conversion
    def quarternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return np.arctan2(siny_cosp, cosy_cosp)

    def laserscan_callback(self, data):
        if not self.is_gridmapping_initialized:
            self.init_gridmapping(data.angle_min, data.angle_max, data.angle_increment, data.range_max)

        self.tf_sub.waitForTransform(self.map_frame, self.robot_frame, data.header.stamp, rospy.Duration(10.0))
        try:
            # get the position associated with the current laserscan
            (x, y, _),(qx, qy, qz, qw) = self.tf_sub.lookupTransform(self.map_frame, self.robot_frame, data.header.stamp)
            theta = self.quarternion_to_yaw(qx, qy, qz, qw)

            # update map
            self.gridmapping.update(x, y, theta, data.ranges)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)


gm = GridMappingROS()
while not rospy.is_shutdown():
    rospy.spin()
