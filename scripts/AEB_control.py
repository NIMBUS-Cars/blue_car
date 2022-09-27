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


class AEB:

    def __init__(self):
        self.ttc_threshold = 1.0
        self.lidar_sub = rospy.Subscriber('/scan',
                                          LaserScan, self.lidar_callback)
        self.drive = rospy.Publisher(rospy.get_param(
            '/nav_drive_topic'), AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        self.speed = 0
        self.steering = 0

    def lidar_callback(self, lidar_msg):
        rospy.loginfo("lidar_callback")
        rospy.loginfo("lidar_msg %s", lidar_msg)
        
        self.speed = 0.5
        # self.steering = 0
        self.drive_msg.drive.speed = self.speed
        # self.drive_msg.drive.steering_angle = self.steering
        self.drive.publish(self.drive_msg)


def main():
    rospy.init_node('AEB_node')
    sn = AEB()
    rospy.spin()


if __name__ == '__main__':
    main()
