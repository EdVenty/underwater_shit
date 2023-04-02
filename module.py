###################################
#                                 #
#   Module for underwater robot   #
#                                 #
###################################

import time
import cv2
from typing import Tuple
import numpy as np
import pymurapi as mur
import math

BLUE_LOW = (120, 50, 50)
BLUE_HIGH = (180, 255, 255)

SCREEN_HEIGHT = 240
SCREEN_WIDTH = 320

SHAPE_NONE = 0
SHAPE_RECT = 1
SHAPE_SQUARE = 2
SHAPE_CIRCLE = 3
SHAPE_TRIANGLE = 4
SHAPE_STAR = 5

class PD:
    _kd = 0
    _kp = 0
    _prev_error = 0.0
    _timestamp = 0.0

    def __init__(self, p_coefficient: float = 0, d_coefficient: float = 0) -> None:
        """Creates regulator object with specified proportional coefficient and differential coefficient

        Args:
            p_coefficient (float): Proportional coefficient
            d_coefficient (float): Differential coefficient
        """
        self._kp = p_coefficient
        self._kd = d_coefficient

    def set_p_gain(self, value: float):
        """Set proportional regulator gain

        Args:
            value (float): Gain
        """
        self._kp = value

    def set_d_gain(self, value: float):
        """Set differential regulator gain

        Args:
            value (float): Gain
        """
        self._kd = value

    def process(self, error: float) -> float:
        """Process regulator and return calculated value

        Args:
            error (float): Regulator's error

        Returns:
            float: Calculated value
        """
        timestamp = int(round(time.time() * 1000))
        output = self._kp * error + self._kd * \
            (error - self._prev_error) / (timestamp - self._timestamp)
        self._timestamp = timestamp
        self._prev_error = error
        return output


def detect_shape_coords(frame) -> Tuple[bool, int, int]:
    """Detects shape's coordinates

    Args:
        frame (cv2 frame): CV2 Frame

    Returns:
        bool, int, int: 1) if detected 2) x center coordinate or 0 3) y center coordinate or 0
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # convert image to hsv color
    # select values in color range
    mask = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)
    cv2.imshow("mask", mask)  # show mask
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # find contours for an object
    for contour in contours:
        area = cv2.contourArea(contour)
        if abs(area) < 300:
            continue  # area too small
        ((_, _), (w, h), _) = cv2.minAreaRect(
            contour)  # find rect for an object's area
        # finds enclosing circle for an object's area
        (_, _), radius = cv2.minEnclosingCircle(contour)

        moments = cv2.moments(contour)  # counting moments
        x, y = 0, 0
        try:
            x = int(moments["m10"] / moments["m00"])  # center of x
            y = int(moments["m01"] / moments["m00"])  # center of y
            return True, x, y  # detected
        except ZeroDivisionError:
            return False, 0, 0  # not detected properly
    return False, 0, 0  # not detected at all


def clamp(value: float, min: float, max: float) -> float:
    """Clamps the value betweed min and max values

    Args:
        value (float): Value to clamp
        min (float): Minimum value
        max (float): Maximum value

    Returns:
        float: Clamped value
    """
    return min if min > value else max if value > max else value  # just long expression. But looks cool


def stab_to_coords(auv, x: float, y: float, x_reg: PD, y_reg: PD, min_length: float = 10.0) -> bool:
    """Stabilizates robot to a specified coordinates on screen

    Args:
        auv (MUR auv): MUR robot object
        x_reg (PD): X PD regulator
        y_reg (PD): Y PD regulator
        x (float): X center
        y (float): Y center 
        min_length (float): Min length to a stab point

    Returns:
        bool: If robot has reached the stab point (length to the point lower than min_length)
    """
    length = math.sqrt((x - SCREEN_WIDTH/2) ** 2 +
                       (y - SCREEN_HEIGHT/2) ** 2)  # length to a screen center
    if length < min_length:
        return True  # stabilized
    output_forward = y_reg.process(
        y - SCREEN_HEIGHT/2)  # for vertical movement
    output_side = x_reg.process(x - SCREEN_WIDTH/2)  # for horizontal movement

    # clamps value between -50 and 50
    output_forward = clamp(output_forward, -50, 50)
    # clamps value between -50 and 50
    output_side = clamp(output_side, -50, 50)

    auv.set_motor_power(2, -output_forward)  # set vertical motor power
    auv.set_motor_power(3, -output_forward)  # set vertical motor power
    auv.set_motor_power(4, -output_side)  # set horizontal (size) motor power
    return False  # not stabilized


# if __name__ == "__main__":
#     auv = mur.mur_init()
#     x_reg = PD(0.5, 0.5)  # horizontal (side) PD regulator
#     y_reg = PD(0.5, 0.5)  # vertical (depth) PD regulator

#     while True:
#         image = auv.get_image_front()  # get frame
#         detected, x, y = detect_shape_coords(image)
#         if detected:
#             if stab_to_coords(auv, x, y, x_reg, y_reg):
#                 print("stab complete!")
#         else:
#             # set vertical motor power to 0 if figure not detected
#             auv.set_motor_power(2, 0)
#             # set vertical motor power to 0 if figure not detected
#             auv.set_motor_power(3, 0)
#             # set horizontal (side) motor power to 0 if figure not detected
#             auv.set_motor_power(4, 0)
#             #print(x, y)
#         cv2.imshow("frame", image)  # show base image
#         cv2.waitKey(1)  # cv2 waits for a key. Important expression

if __name__ == "__main__":
    auv = mur.mur_init()
    x_reg = PD(0.5, 0.5)  # horizontal (side) PD regulator
    y_reg = PD(0.5, 0.5)  # vertical (depth) PD regulator

    while True:
        image = auv.get_image_front()  # get frame
        draw = image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)
        contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # find contours for an object
        for contour in contours:
            area = cv2.contourArea(contour)
            if abs(area) < 300:
                continue
            cv2.drawContours(draw, contour, -1, (255, 0, 0))
        cv2.imshow("draw", draw)
        cv2.waitKey(1)
