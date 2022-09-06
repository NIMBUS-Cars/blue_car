#!/usr/bin/env python

"""follower_ros.py: Robot will follow the Yellow Line in a track"""

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import cv_bridge
import cv2
import rospy
import math


class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw',
                                          Image, self.image_callback)
        self.drive = rospy.Publisher(rospy.get_param(
            '/nav_drive_topic'), AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        self.speed = 0
        self.steering = 0

    def image_callback(self, msg):
        rospy.loginfo("image_callback")
        rospy.loginfo("odom_msg %s", img_msg.pose.pose)
        rospy.loginfo("odom_msg %s", img_msg.twist.twist)

        self.speed = 0.8
        self.steering = -1
        self.drive_msg.drive.speed = self.speed
        self.drive_msg.drive.steering_angle = self.steering
        self.drive.publish(self.drive_msg)

def main():
    rospy.init_node('lanefollower3')
    sn = Follower()
    rospy.spin()


if __name__ == '__main__':
    main()
