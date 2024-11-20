import cv2
import numpy as np


def warping(image, source=[[147, 104], [0, 343], [487, 113], [636, 355]]):
    size = 200

    source = np.float32(source)
    destination = np.float32([[0, 0], [0, size], [size, 0], [size, size]])

    M = cv2.getPerspectiveTransform(source, destination)

    warp_image = cv2.warpPerspective(image, M, (size, size), flags=cv2.INTER_LINEAR)

    return warp_image


if __name__ == "__main__":
    PATH = "video/test.mp4"

    cap = cv2.VideoCapture(PATH)

    ret, img = cap.read()
    if not ret:
        cap.release()

    w, h, c = img.shape

    cv2.namedWindow("Warp")
    cv2.createTrackbar("Left Top X", "Warp", 0, w, lambda x: x)
    cv2.createTrackbar("Left Top Y", "Warp", 0, h, lambda x: x)
    cv2.createTrackbar("Left Bottom X", "Warp", 0, w, lambda x: x)
    cv2.createTrackbar("Left Bottom Y", "Warp", 0, h, lambda x: x)
    cv2.createTrackbar("Right Top X", "Warp", 0, w, lambda x: x)
    cv2.createTrackbar("Right Top Y", "Warp", 0, h, lambda x: x)
    cv2.createTrackbar("Right Bottom X", "Warp", 0, w, lambda x: x)
    cv2.createTrackbar("Right Bottom Y", "Warp", 0, h, lambda x: x)

    cv2.setTrackbarPos("Left Top X", "Warp", 147)
    cv2.setTrackbarPos("Left Top Y", "Warp", 104)
    cv2.setTrackbarPos("Left Bottom X", "Warp", 0)
    cv2.setTrackbarPos("Left Bottom Y", "Warp", 343)
    cv2.setTrackbarPos("Right Top X", "Warp", 487)
    cv2.setTrackbarPos("Right Top Y", "Warp", 113)
    cv2.setTrackbarPos("Right Bottom X", "Warp", 636)
    cv2.setTrackbarPos("Right Bottom Y", "Warp", 355)

    while cap.isOpened():
        ret, img = cap.read()
        if not ret:
            cap.release()

        cv2.imshow("Original", img)

        left_top_x = cv2.getTrackbarPos("Left Top X", "Warp")
        left_top_y = cv2.getTrackbarPos("Left Top Y", "Warp")
        left_bottom_x = cv2.getTrackbarPos("Left Bottom X", "Warp")
        left_bottom_y = cv2.getTrackbarPos("Left Bottom Y", "Warp")
        right_top_x = cv2.getTrackbarPos("Right Top X", "Warp")
        right_top_y = cv2.getTrackbarPos("Right Top Y", "Warp")
        right_bottom_x = cv2.getTrackbarPos("Right Bottom X", "Warp")
        right_bottom_y = cv2.getTrackbarPos("Right Bottom Y", "Warp")

        source = [
            [left_top_x, left_top_y],
            [left_bottom_x, left_bottom_y],
            [right_top_x, right_top_y],
            [right_bottom_x, right_bottom_y],
        ]

        warped_img = warping(img, source)

        cv2.imshow("Warp", warped_img)

        if cv2.waitKey(33) == ord("q"):
            cap.release()
            break

    cap.release()
    cv2.destroyAllWindows()
