#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math
import numpy as np
import rospy
import cv2

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image, Imu
from std_msgs.msg import String

# from detection_msgs.msg import BoundingBox, BoundingBoxes

from sleep import *
from halt import *
from turn import *
from object_detector import *
from lane_detector import *
from green_detector import *


class RobotControlNode:
    def __init__(self):
        # 최대 속도 및 회전속도 설정
        self.MAX_SPEED = 0.3
        self.MAX_ROTATE = np.radians(45)
        self.TURN_90 = math.pi / (2 * self.MAX_ROTATE)

        # Publisher 설정
        self.bridge = CvBridge()
        self.drive_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.lcd_publisher = rospy.Publisher("/lcd_str_1", String, queue_size=1)
        self.detect_bbox = BBox()

        # Task 설정
        self.tasks = [
            # 첫 번째 교차로
            GreenDetect(),
            Sleep(0.5),
            TurnLeft(self.TURN_90),
            # 1구역 인식
            GreenDetect(),
            TurnLeft(self.TURN_90),
            ObjectDetect(self.drive_publisher, self.lcd_publisher, self.detect_bbox),
            TurnLeft(self.TURN_90),
            Sleep(),
            # 교차로 빠져 나옴
            TurnLeft(self.TURN_90),
            Sleep(),
            # 2구역 인식
            GreenDetect(),
            TurnLeft(self.TURN_90),
            ObjectDetect(self.drive_publisher, self.lcd_publisher, self.detect_bbox),
            TurnRight(self.TURN_90),
            Sleep(),
            # 두 번째 교차로
            GreenDetect(),
            TurnRight(self.TURN_90),
            # 3구역 인식
            GreenDetect(),
            TurnLeft(self.TURN_90),
            ObjectDetect(self.drive_publisher, self.lcd_publisher, self.detect_bbox),
            TurnRight(self.TURN_90),
            # 4구역 인식
            GreenDetect(),
            TurnLeft(self.TURN_90),
            ObjectDetect(self.drive_publisher, self.lcd_publisher, self.detect_bbox),
            TurnRight(self.TURN_90),
            Sleep(1.5),
            TurnLeft(self.TURN_90),
            # 도착 지점
            GreenDetect(),
            Halt(),
        ]
        self.task_index = 0

        rospy.init_node("robot_control_node", anonymous=False)
        rospy.Subscriber("/main_camera/image_raw", Image, self.camera_callback)
        # rospy.Subscriber('/main_camera/image_raw/compressed', CompressedImage, self.camera_callback)
        # rospy.Subscriber("/detected_bboxes", BoundingBoxes, self.detect_bbox.bounding_boxes_callback)

    def camera_callback(self, data):
        """
        카메라 이벤트 발생 시 실행되는 함수
        """
        # 이미지 데이터를 OpenCV 이미지로 변환
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # 태스크 수행
        is_success, speed, rotate, is_following_lane = self.tasks[
            self.task_index
        ].execute(image)

        # 태스크 성공 시 다음 태스크 진행
        if is_success and self.task_index < len(self.tasks):
            print("Task is executed:", repr(self.tasks[self.task_index]))
            self.task_index += 1

        # 차선 인식 주행
        if is_following_lane:
            # Lane Detect
            speed = self.MAX_SPEED
            rotate = lane_detect(image, speed)

        # 속도 및 각속도 업데이트
        self.update_movement(speed, rotate)

        cv2.waitKey(1)

    def update_movement(self, x, z):
        """
        속도 및 가속도를 업데이트하는 함수
        """
        movement = Twist()
        movement.linear.x = x
        movement.angular.z = z
        self.drive_publisher.publish(movement)


if __name__ == "__main__":
    if not rospy.is_shutdown():
        RobotControlNode()
        rospy.spin()
