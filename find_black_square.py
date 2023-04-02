from typing import Tuple, Union
import cv2
import numpy as np

def find_anything(frame: np.ndarray, hsv_min: Tuple[int, int, int], hsv_max: Tuple[int, int, int], min_contour_area: int) -> Union[Tuple[True, int, int], Tuple[False, None, None]]:
    mask = cv2.inRange(frame, hsv_min, hsv_max)
    contours, hierarhy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_contour = max(contours, key=lambda cnt: cv2.contourArea(cnt))
    if cv2.contourArea(max_contour) >= min_contour_area:
        moments = cv2.moments(max_contour)
        x = int(moments["m10"] / moments["m00"])
        y = int(moments["m01"] / moments["m00"])
        return True, x, y
    return False, None, None

def a(frame: np.ndarray, hsv_min: Tuple[int, int, int], hsv_max: Tuple[int, int, int], min_contour_area: int) -> Union[Tuple[True, int, int], Tuple[False, None, None]]:moments = cv2.moments(max(cv2.findContours(cv2.inRange(frame, hsv_min, hsv_max), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE), key=lambda cnt: cv2.contourArea(cnt)));return (int(moments["m10"] / moments["m00"]),int(moments["m01"] / moments["m00"]) )