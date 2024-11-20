# -*- coding: utf-8 -*-

import time
import numpy as np


MAX_STEER = np.radians(90)


class TurnLeft:
    def __init__(self, sec=1):
        self.timer = None
        self.sec = sec

    def execute(self, img):
        if self.timer is None:
            self.timer = time.time()
        elif time.time() - self.timer >= self.sec:
            return False, 0, 0, False
        return True, 0, MAX_STEER, False


class TurnRight:
    def __init__(self, sec=1):
        self.timer = None
        self.sec = sec

    def execute(self, img):
        if self.timer is None:
            self.timer = time.time()
        elif time.time() - self.timer >= self.sec:
            return False, 0, 0, False
        return True, 0, -MAX_STEER, False
