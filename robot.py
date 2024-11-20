#! /usr/bin/env python
# -*- coding: utf-8 -*-

from lane_detector import lane_detect
import rospy
import cv2

from sensor_msgs.msg import CompressedImage, Image, Imu
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist


class RobotControlNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.MAX_SPEED = 0.2
        self.MAX_ROTATE = 0.2
        rospy.init_node("robot_control_node", anonymous=False)
        # rospy.Subscriber('/main_camera/image_raw/compressed', CompressedImage, self.camera_callback)
        rospy.Subscriber("/main_camera/image_raw", Image, self.camera_callback)

    def camera_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # Lane Detect
        rotate = lane_detect(image, self.MAX_SPEED)

        self.update_movement(self.MAX_SPEED, rotate)

        cv2.waitKey(1)

    def update_movement(self, x, z):
        movement = Twist()
        movement.linear.x = x * 2
        movement.angular.z = z * 2
        self.pub.publish(movement)


if __name__ == "__main__":
    if not rospy.is_shutdown():
        RobotControlNode()
        rospy.spin()
