#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

# This is based on a template obtained from https://github.com/f1tenth/f1tenth_labs/blob/main/lab2/code/scripts/safety_node.py
# Create a node to apply AEB based on sensor data

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistWithCovariance
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math

def calculate_ttc(r, v, theta):
    denominator = -1*v*math.cos(theta)
    if denominator <= 0: # this is basically doing the max(x,0) calculation, but helps us avoid math issues
        return float('inf')
    else:
        return r / denominator

# This safety class is based on https://github.com/f1tenth/f1tenth_labs/blob/main/lab2/code/scripts/safety_node.py
class Safety(object):
    def __init__(self):
        self.speed = 0
        self.ttc_threshold = 0.5
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.brake = rospy.Publisher('brake', AckermannDriveStamped, queue_size=10)
        self.brake_msg = AckermannDriveStamped()
        self.brake_bool = rospy.Publisher('brake_bool', Bool, queue_size=10)
        self.brake_bool_msg = Bool()

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # Calculate the TTC around the vehicle (assuming ranges[0] is straight behind)
        min_ttc = float('inf')
        for i in range(0,len(scan_msg.ranges),2): # increment by 2 to reduce calculation time
            ttc = calculate_ttc(scan_msg.ranges[i], self.speed, i*scan_msg.angle_increment)
            min_ttc = min(min_ttc, ttc)

        # TODO: publish brake message and publish controller bool
        self.brake_msg.drive.speed = 0.0
        self.brake.publish(self.brake_msg)
        self.brake_bool_msg.data = (min_ttc <= self.ttc_threshold)
        self.brake_bool.publish(self.brake_bool_msg)


def main():
    rospy.init_node('safety')
    #sn = Safety()
    #drive = rospy.Publisher(rospy.get_param("/nav_drive_topic"), AckermannDriveStamped, queue_size=10)
    drive = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=10)
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.speed = 1.0
    drive.publish(drive_msg)
    rospy.spin()

if __name__ == '__main__':
    main()
