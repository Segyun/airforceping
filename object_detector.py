# -*- coding: utf-8 -*-

import time

from collections import Counter
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
        self, drive_publisher, lcd_publisher, detect_bbox, sec=5, on_robot=True
    ):
        self.drive_publisher = drive_publisher
        self.lcd_publisher = lcd_publisher
        self.detect_bbox = detect_bbox
        self.sec = sec
        self.on_robot = on_robot
        self.timer = None

    def find_mode(numbers):
        counter = Counter(numbers)
        mode = counter.most_common(1)
        return mode[0][0]

    def execute(self, img):
        if self.timer is None:
            self.timer = time.time()
        elif time.time() - self.timer < self.sec:
            if not self.on_robot:
                start_time = time.time()

                # PID
                pid = PID(0.01, 0.1, 0.5)

                # Detect Bounding Boxes
                det_bboxes = self.detect_bbox

                # LCD Message
                str_msg_1 = String()

                # Event Flag
                event = 0

                # Rotate Flag
                rot_start = 0

                # 최빈값 계산을 위한 리스트
                candidates_alli = []
                candidates_alli_tank = []
                candidates_enem = []
                candidates_enem_tank = []

                # 이벤트 발생할 때까지 회전
                while event == 0:
                    curr_time = time.time()

                    alli = 0
                    alli_tank = 0
                    enem = 0
                    enem_tank = 0

                    # 객체 count 및 위치파악
                    for bbox in det_bboxes.bounding_boxes:
                        if bbox.probability > 0.75:
                            if bbox.Class == "al":
                                alli += 1
                            elif bbox.Class == "alt":
                                alli_tank += 1
                            elif bbox.Class == "en":
                                enem += 1
                            else:
                                enem_tank += 1
                                enem_tank_x = (bbox.xmin + bbox.xmax) / 2

                    candidates_alli.append(alli)
                    candidates_alli_tank.append(alli_tank)
                    candidates_enem.append(enem)
                    candidates_enem_tank.append(enem_tank)

                    # LCD Display Text
                    str_msg_1.data = "alli: {}/{},enem: {}/{}".format(
                        alli, alli_tank, enem, enme_tank
                    )

                    # 이벤트가 있는 경우
                    if enem_tank:
                        # 화면 중심과 bbox 사이의 거리
                        x_err = 320 - enem_tank_x
                        # 회전 조향각
                        theta = pid.pid_control(x_err)

                        rot_msg = Twist()
                        rot_msg.linear.x = 0
                        rot_msg.angular.z = theta
                        
                        self.drive_publisher.publish(rot_msg)

                        # bbox가 화면 중심에 위치
                        if abs(x_err) < 10:
                            if rot_start == 0:
                                rot_start_time = curr_time
                                rot_start = 1
                            elif curr_time - rot_start_time > 5:
                                # 최빈값 계산
                                alli_mode = self.find_mode(candidates_alli)
                                alli_tank_mode = self.find_mode(candidates_alli_tank)
                                enem_mode = self.find_mode(candidates_enem)
                                enem_tank_mode = self.find_mode(candidates_enem_tank)

                                # 이벤트 발생 시 LCD Display Text
                                str_msg_1.data = "alli: {}/{},enem: {}/{} HIT!".format(
                                    alli_mode, alli_tank_mode, enem_mode, enem_tank_mode
                                )
                                event = 1
                    else:
                        if curr_time - start_time > 5:
                            event = 1

                    self.lcd_publisher.publish(str_msg_1)
            return False, 0, 0, False
        else:
            return True, 0, 0, False

