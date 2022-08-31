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


class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw',
                                          Image, self.image_callback)
        self.drive = rospy.Publisher(rospy.get_param(
            '/nav_drive_topic'), AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
        #                                    Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, d = image.shape
            # 480x640
            # rospy.loginfo("h %s", h)
            # rospy.loginfo("w %s", w)
            image_crop = image[3*h/5:h, w/4:3*w/4]
            hsv = cv2.cvtColor(image_crop, cv2.COLOR_BGR2HSV)

            # change below lines to map the color you wanted robot to follow
            lower_yellow = np.array([23, 41, 133])
            upper_yellow = np.array([40, 150, 255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            blur = cv2.GaussianBlur(mask, (5, 5), 0)
            M = cv2.moments(blur)
            # rospy.loginfo("M %s", M)

            if M['m00'] > 0:
                # h, w, d = image_crop.shape
                # # 180x400
                # rospy.loginfo("h %s", h)
                # rospy.loginfo("w %s", w)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                rospy.loginfo("cx %s", cx)
                rospy.loginfo("cy %s", cy)
                cv2.circle(blur, (cx, cy), 5, (125, 125, 125), -1)
                cv2.putText(blur, "Centroid", (cx - 25, cy - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 2)
                # CONTROL starts
                # err = cx - w/2
                # self.twist.linear.x = 0.2
                # self.twist.angular.z = -float(err) / 100
                # rospy.loginfo("self.twist %s", self.twist)
                self.drive_msg.drive.steering_angle = 2
                self.drive.publish(self.drive_msg)
                # CONTROL ends
            cv2.imshow("original image", image)
            cv2.imshow("cropped image", image_crop)
            cv2.imshow("masked image", mask)
            cv2.imshow('blurred image', blur)
            cv2.waitKey(3)

        except CvBridgeError as e:
            print(e)


def main():
    rospy.init_node('lanefollower')
    sn = Follower()
    rospy.spin()


if __name__ == '__main__':
    main()
