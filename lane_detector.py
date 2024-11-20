# -*- coding: utf-8 -*-

import cv2
import numpy as np
import scipy


def warping(image):
    sz = 200
    source = np.float32([[147, 104], [0, 343], [487, 113], [636, 355]])
    destination = np.float32([[0, 0], [0, sz], [sz, 0], [sz, sz]])

    M = cv2.getPerspectiveTransform(source, destination)

    warp_image = cv2.warpPerspective(image, M, (sz, sz), flags=cv2.INTER_LINEAR)

    return warp_image


def convert_to_hsv(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def color_filter(img, lower, upper):
    lower = np.float32(lower)
    upper = np.float32(upper)

    mask = cv2.inRange(img, lower, upper)

    return mask


def gaussian_blur(img):
    return cv2.GaussianBlur(img, (0, 0), 5)


def sliding_window_lane(mask):
    WINDOW_WIDTH = 80
    WINDOW_HEIGHT = 8
    WINDOW_CNT = 10

    h, w = mask.shape

    hist = np.mean(mask[h // 2 :], axis=0, dtype="float32")
    hist = scipy.ndimage.gaussian_filter1d(hist, 15, mode="nearest")

    img = np.expand_dims(mask, -1)
    img = np.tile(img, [1, 1, 3])

    points = []

    curr_x = np.argmax(hist)
    for i in range(WINDOW_CNT):
        offset_x = curr_x.astype("int32") - WINDOW_WIDTH // 2
        offset_x = np.clip(offset_x, 0, w - WINDOW_WIDTH)

        xmin = np.clip(offset_x, 0, w)
        xmax = np.clip(offset_x + WINDOW_WIDTH, 0, w)
        ymin = np.clip(h - (i + 1) * WINDOW_HEIGHT, 0, h)
        ymax = np.clip(h - i * WINDOW_HEIGHT, 0, h)

        window = mask[ymin:ymax, xmin:xmax]
        window = np.mean(window, axis=0)
        colored = np.argwhere(window > 127)
        if colored.size == 0:
            continue

        img[ymin:ymax, xmin:xmax, 1:] = 0
        cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 1)

        curr_x = offset_x + np.mean(colored)
        points.append([curr_x, (ymin + ymax) / 2])

    if len(points) < 10:
        cv2.imshow("Sliding Window", img)
        return

    points = np.asarray(points, dtype="float32")
    poly = np.polyfit(points[:, 1], points[:, 0], 1)

    lane_draw_y = np.arange(h)
    lane_draw_x = np.polyval(poly, lane_draw_y)
    lane_draw_x += 0.5
    lane_draw_x = lane_draw_x.astype("int32")
    for y, x in zip(lane_draw_y, lane_draw_x):
        if 0 <= x and x < w:
            img[y, x] = [0, 0, 255]

    cv2.imshow("Sliding Window", img)

    return poly


def stanley(lane, speed, img_h, img_w):
    speed = max(0.1, speed)
    distance = np.polyval(lane, img_h * 0.9) - (img_w / 2)

    derived_lane = np.polyder(lane)
    lane_angle = np.polyval(derived_lane, img_h * 0.9)

    k = 0.001
    theta_err = lane_angle
    lat_err = -distance * np.cos(lane_angle)

    return theta_err + np.arctan(k * lat_err / speed)


def lane_detect(img, speed):
    # BEV
    warped_img = warping(img)

    # HSV 변환
    hsv_img = convert_to_hsv(warped_img)

    # 가우시안 블러
    gaussian_img = gaussian_blur(hsv_img)

    # 컬러 필터
    masked_img = color_filter(gaussian_img, [0, 0, 0], [180, 100, 130])

    # 슬라이딩 윈도우
    lane = sliding_window_lane(masked_img)

    if lane is None:
        return 0

    h, w = masked_img.shape
    # 스탠리
    steering_angle = stanley(lane, speed, w, h)

    return steering_angle


if __name__ == "__main__":
    MAX_SPEED = 0.2
    PATH = "video/test.mp4"

    cap = cv2.VideoCapture(PATH)

    while cap.isOpened():
        ret, img = cap.read()
        if not ret:
            cap.release()
            break

        cv2.imshow("Original", img)

        warped_img = warping(img)
        cv2.imshow("Warp", warped_img)

        hsv_img = convert_to_hsv(warped_img)
        cv2.imshow("HSV", hsv_img)

        gaussian_img = gaussian_blur(hsv_img)
        cv2.imshow("Gaussian", gaussian_img)

        masked_img = color_filter(gaussian_img, [0, 0, 0], [180, 100, 130])
        cv2.imshow("Masked", masked_img)

        sliding_window = sliding_window_lane(masked_img)

        lane_detect(img, MAX_SPEED)

        if cv2.waitKey(33) == ord("q"):
            cap.release()
            break

    cap.release()
    cv2.destroyAllWindows()
