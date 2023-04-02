import cv2
import numpy as np

hsv_low = np.array((30, 100, 0))
hsv_high = np.array((60, 255, 255))

while True:
    cap = cv2.VideoCapture("D:/Underwater/output.avi")
    while cap.isOpened():
        r, frame = cap.read()
        if not r: break
        mask = cv2.inRange(frame, hsv_low, hsv_high)
        cv2.imshow('video', mask)
        cv2.waitKey(25)