# -*- coding: utf-8 -*-

import cv2
import numpy as np


def convert_to_hsv(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def color_filter(img, lower, upper):
    lower = np.float32(lower)
    upper = np.float32(upper)

    mask = cv2.inRange(img, lower, upper)

    return mask


def gaussian_blur(img):
    return cv2.GaussianBlur(img, (0, 0), 5)


class GreenDetect:
    def __init__(self):
        self.count = 0

    def execute(self, img):
        h, w, c = img.shape
        center = np.array([w, h], np.uint32) // 2
        roi_size = np.array([w, h], np.uint32) // 2

        left_top = center - roi_size // 2
        roi_img = img[
            left_top[1] : left_top[1] + roi_size[1],
            left_top[0] : left_top[0] + roi_size[0],
        ].copy()

        hsv_img = convert_to_hsv(roi_img)

        gaussian_img = gaussian_blur(hsv_img)

        masked_img = color_filter(gaussian_img, [50, 50, 0], [80, 255, 255])

        cv2.imshow("Green", masked_img)

        # print("Green:", len(masked_img.nonzero()[0]))

        if len(masked_img.nonzero()[0]) > 6000:
            self.count += 1

        if self.count >= 3:
            return True, 0, 0, True
        return False, 0, 0, True


if __name__ == "__main__":
    PATH = "video/test.mp4"

    cap = cv2.VideoCapture(PATH)

    while cap.isOpened():
        ret, img = cap.read()
        if not ret:
            cap.release()
            break

        h, w, c = img.shape
        center = np.array([w, h], np.uint32) // 2
        roi_size = np.array([w, h], np.uint32) // 2

        left_top = center - roi_size // 2
        roi_img = img[
            left_top[1] : left_top[1] + roi_size[1],
            left_top[0] : left_top[0] + roi_size[0],
        ].copy()

        cv2.rectangle(img, left_top, left_top + roi_size, [0, 0, 255], 5)
        cv2.imshow("Original", img)
        cv2.imshow("ROI", roi_img)

        hsv_img = convert_to_hsv(roi_img)
        cv2.imshow("HSV", hsv_img)

        gaussian_img = gaussian_blur(hsv_img)
        cv2.imshow("Gaussian", gaussian_img)

        masked_img = color_filter(gaussian_img, [50, 50, 0], [80, 255, 255])
        cv2.imshow("Masked", masked_img)

        green_detect = GreenDetect()
        print(green_detect.execute(img))

        if cv2.waitKey(1000 // 30) == ord("q"):
            cap.release()
            break

    cap.release()
    cv2.destroyAllWindows()
