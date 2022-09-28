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
        # self.odom_sub = rospy.Subscriber(
        #     '/vesc/odom', Odometry, self.odom_callback)
        self.lidar_sub = rospy.Subscriber('/scan',
                                          LaserScan, self.lidar_callback)

        self.drive = rospy.Publisher(rospy.get_param(
            '/nav_drive_topic'), AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        self.ttc_threshold = 1.0
        self.speed = 0.5

    # def odom_callback(self, odom_msg):
    #     #rospy.loginfo("odom_callback")
    #     self.speed = odom_msg.twist.twist.linear.x

    def lidar_callback(self, lidar_msg):
        # rospy.loginfo("lidar_callback")
        # rospy.loginfo("lidar_msg %s", lidar_msg)
        min_ttc = float('inf')
        # self.speed = 0.5

        # increment by some big number to reduce calculation time
        for i in range(0, len(lidar_msg.ranges), 2):
            if (lidar_msg.ranges[i] > lidar_msg.range_max or lidar_msg.ranges[i] < lidar_msg.range_min):
                continue
            rospy.loginfo("lidar_msg %s", lidar_msg.ranges[i])
            lidar_angle = lidar_msg.angle_min + i*lidar_msg.angle_increment
            rospy.loginfo("lidar_angle %s", lidar_angle)
            ttc = TTC_calc(
                lidar_msg.ranges[i], self.speed, lidar_angle)
            min_ttc = min(min_ttc, ttc)
            rospy.loginfo("min_ttc %s", min_ttc)

        # if min_ttc <= self.ttc_threshold:
        #     rospy.loginfo("Apply brake!")
        #     self.speed = 0
        #     self.acceleration = 0
        #     self.jerk = 0

        self.drive_msg.drive.speed = self.speed
        self.drive.publish(self.drive_msg)


def main():
    rospy.init_node('AEB_node')
    sn = AEB()
    rospy.spin()


if __name__ == '__main__':
    main()
