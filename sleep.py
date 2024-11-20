# -*- coding: utf-8 -*-

import time


class Sleep:
    def __init__(self, sec=1):
        self.sec = sec
        self.timer = None

    def execute(self, img):
        if self.timer is None:
            self.timer = time.time()
        elif time.time() - self.timer >= self.sec:
            return True, 0, 0, True
        return False, 0, 0, True
