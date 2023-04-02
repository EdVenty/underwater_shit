from typing import Union
from numpy.lib.twodim_base import tri
import pymurapi as mur
# auv = mur.mur_init()
import cv2
import math
import time as t
import numpy as np

THRUSTER_YAW_LEFT = 0
THRUSTER_YAW_RIGHT = 1
THRUSTER_DEPTH_LEFT = 2
THRUSTER_DEPTH_RIGHT = 3

THRUSTER_YAW_LEFT_DIRECTION = +1
THRUSTER_YAW_RIGHT_DIRECTION = +1
THRUSTER_DEPTH_LEFT_DIRECTION = +1
THRUSTER_DEPTH_RIGHT_DIRECTION = +1

HSV_MIN = (10, 150, 150)
HSV_MAX = (20, 255, 255) 

def main():
    timer = t.time()
    auv = mur.mur_init()
    while True:
        timer, stabilizated = stabilizate_value_by_time(timer, 1, auv.get_depth(), 0.2, 5)
        # keep(0, 1)
        frame = auv.get_image_bottom()
        draw = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        bin = cv2.inRange(hsv, HSV_MIN, HSV_MAX)

        cnts, _ = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for cnt in cnts:
            if cv2.contourArea(cnt) > 100:
                (x, y, w, h) = cv2.boundingRect(cnt)
                cv2.rectangle(draw, (x, y), (x+w, y+h), (255, 0, 0))
                (x, y), r = cv2.minEnclosingCircle(cnt)
                # print(x, y, r)
                cv2.circle(draw, (int(x), int(y)), int(r), (0, 0, 255))
                r, triangle = cv2.minEnclosingTriangle(cnt)
                # print(triangle.shape)
                # triangle = np.transpose(triangle, [1, 0, 2])
                # print(triangle.shape)
                # print(triangle[0][0][0])
                cv2.line(draw, tuple(triangle[0][0]), tuple(triangle[1][0]), (0, 0, 0))
                cv2.line(draw, tuple(triangle[1][0]), tuple(triangle[2][0]), (0, 0, 0))
                cv2.line(draw, tuple(triangle[0][0]), tuple(triangle[2][0]), (0, 0, 0))
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(draw, [box], 0, (0, 191, 255), 2)
                m = cv2.moments(cnt)
                cX = int(m["m10"] / m["m00"])
                cY = int(m["m01"] / m["m00"])
                cv2.circle(draw, (cX, cY), 5, (0, 255, 0), 3)

                # cv2.polylines(draw, [triangle.reshape((-1, 1, 2))], True, (255, 255, 0))
                # cv2.drawContours(draw, [triangle], -1, (255, 255, 0))
                # cv2.triangle
                # print(r, triangle)
        cv2.drawContours(draw, cnts, -1, (0, 255, 0))
        cv2.imshow("mask", bin)

        cv2.imshow("draw", draw)
        cv2.waitKey(1)
        # if stabilizated:
        #     print("DONE!")
        #     return

if __name__ == "__main__":
    main()