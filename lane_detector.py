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


if __name__ == "__main__":
    PATH = "video/test.mp4"

    cap = cv2.VideoCapture(PATH)

    while cap.isOpened():
        ret, img = cap.read()
        if not ret:
            cap.release()
            break

        cv2.imshow("Original", img)

        hsv_img = convert_to_hsv(img)
        cv2.imshow("HSV", hsv_img)

        gaussian_img = gaussian_blur(hsv_img)
        cv2.imshow("Gaussian", gaussian_img)

        masked_img = color_filter(gaussian_img, [0, 0, 0], [180, 100, 130])
        cv2.imshow("Masked", masked_img)

        if cv2.waitKey() == ord("q"):
            cap.release()
            break

    cap.release()
    cv2.destroyAllWindows()
