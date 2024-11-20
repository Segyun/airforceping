# -*- coding: utf-8 -*-

import time

from std_msgs.msg import String

# from detection_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import Twist


class BBox:
    """
    바운딩 박스 객체
    """

    def __init__(self):
        self.bounding_boxes = None
        # self.bounding_boxes = BoundingBoxes()

    def bounding_boxes_callback(self, msg):
        self.bounding_boxes = msg


class PID:
    """
    PID 객체
    """

    def __init__(self, kp, ki, kd):
        # PID 게인 초기화
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

        # 이전 오차 초기화
        self.cte_prev = 0.0

        # 각 오차 초기화
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

        # 적분오차 제한값 설정
        self.i_min = -10
        self.i_max = 10

    def pid_control(self, cte):
        # 미분오차 계산
        self.d_error = cte - self.cte_prev

        # 비례오차 계산
        self.p_error = cte

        # 적분오차 계산 및 제한 적용
        self.i_error += cte
        self.i_error = max(min(self.i_error, self.i_max), self.i_min)

        # 이전 오차 업데이트
        self.cte_prev = cte

        # PID 제어 출력 계산
        return self.Kp * self.p_error + self.Ki * self.i_error + self.Kd * self.d_error


class ObjectDetect:
    """
    객체 인식 미션 수행
    """

    def __init__(
        self, drive_publisher, lcd_publisher, detect_bbox, sec=3, on_device=True
    ):
        self.drive_publisher = drive_publisher
        self.lcd_publisher = lcd_publisher
        self.detect_bbox = detect_bbox
        self.sec = sec
        self.on_device = on_device
        self.timer = None
        # 원격으로 실행할 경우
        if not self.on_device:
            from ultralytics import YOLO

            PATH = "model/best.pt"

            self.model = YOLO(PATH)

    def execute(self, img):
        if self.timer is None:
            self.timer = time.time()
        elif time.time() - self.timer >= self.sec:
            if not self.on_device:
                det_bboxes = self.detect_bbox
                pid = PID(0.01, 0.1, 0.5)

                str_msg_1 = String()  # lcd msg
                start_time = time.time()
                event = 0
                rot_start = 0  # 회전 flag

                while event == 0:  # 이벤트 발생할때 까지 회전
                    alli = 0
                    alli_tank = 0
                    enem = 0
                    enem_tank = 0
                    curr_time = time.time()

                    for bbox in det_bboxes.bounding_boxes:  # 객체 count 및 위치파악
                        if bbox.probability > 0.5:
                            if bbox.Class == "alli":
                                alli += 1
                            elif bbox.Class == "alli_tank":
                                alli_tank += 1
                            elif bbox.Class == "enem":
                                enem += 1
                            else:
                                enem_tank += 1
                                enem_tank_x = (bbox.xmin + bbox.xmax) / 2

                    str_msg_1.data = "alli: {}/{},enem: {}/{}".format(
                        alli, alli_tank, enem, enme_tank
                    )  # lcd string

                    if enem_tank:  # 이벤트가 있는 경우
                        x_err = 320 - enem_tank_x  # 화면 중심과 bbox 사이의 거리
                        theta = pid.pid_control(x_err)  # 회전 조향각

                        rot_msg = Twist()
                        rot_msg.linear.x = 0
                        rot_msg.angular.z = theta
                        self.drive_publisher.publish(rot_msg)

                        if abs(x_err) < 5:  # bbox가 화면 중심에 위치
                            if rot_start == 0:
                                rot_start_time = curr_time
                                rot_start = 1
                            elif curr_time - rot_start_time > 5:
                                str_msg_1.data = "alli: {}/{},enem: {}/{} HIT!".format(
                                    alli, alli_tank, enem, enme_tank
                                )  # 이벤트 발생 lcd string
                                event = 1
                    else:
                        if curr_time - start_time > 5:
                            event = 1

                    self.lcd_publisher.publish(str_msg_1)
            return True, 0, 0, False
        return False, 0, 0, False
