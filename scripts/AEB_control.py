#!/usr/bin/env python

"""AEB_control.py: Robot will automatic apply brake when see obsticle"""

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import rospy
import math


def TTC_calc(r_i, v_x, theta):
    r_i_dot = 1*v_x*math.cos(theta)
    if r_i_dot >= 0:
        TTC_i = float('inf')
    else:
        TTC_i = r_i / (-r_i_dot)
    return TTC_i


class AEB:
    def __init__(self):
        self.odom_sub = rospy.Subscriber(
            '/vesc/odom', Odometry, self.odom_callback)
        self.lidar_sub = rospy.Subscriber('/scan',
                                          LaserScan, self.lidar_callback)
        self.drive = rospy.Publisher(rospy.get_param(
            '/nav_drive_topic'), AckermannDriveStamped, queue_size=10)

        self.drive_msg = AckermannDriveStamped()
        self.ttc_threshold = 0.8
        self.speed = 0.5
        self.brake = False

    def odom_callback(self, odom_msg):
        # rospy.loginfo("odom_callback")
        self.speed = odom_msg.twist.twist.linear.x

    def lidar_callback(self, lidar_msg):
        stationary = 0.001
        if abs(self.speed) > stationary:
            self.angles_array = np.arange(
                lidar_msg.angle_min, lidar_msg.angle_max, lidar_msg.angle_increment)
            self.ranges_array = np.array(lidar_msg.ranges)

            # fix denominator
            self.range_rates = np.max(
                self.speed * np.cos(self.angles_array), 0) + 0.000000001
            self.ttcs = (self.ranges_array/self.range_rates)
            # find the minimum ttc value
            self.min_ttc = np.min(self.ttcs)

            if self.min_ttc < self.ttc_threshold:
                self.brake = True
                print("Min TTC below Threshhold, Apply brake here")
                self.speed = 0

            else:
                self.brake = False

        self.drive_msg.drive.speed = self.speed
        self.drive.publish(self.drive_msg)

        # rospy.loginfo("lidar_callback")
        # rospy.loginfo("lidar_msg %s", lidar_msg)
        # min_ttc = float('inf')
        # self.speed = 0.5

        # increment by some big number to reduce calculation time
        # for i in range(0, len(lidar_msg.ranges), 100):
        #     rospy.loginfo("lidar_msg %s", lidar_msg.ranges[i])
            # ttc = TTC_calc(
            #     lidar_msg.ranges[i], self.speed, i*lidar_msg.angle_increment)
            # min_ttc = min(min_ttc, ttc)
            # rospy.loginfo("min_ttc %s", min_ttc)


def main():
    rospy.init_node('AEB_node')
    sn = AEB()
    rospy.spin()


if __name__ == '__main__':
    main()
