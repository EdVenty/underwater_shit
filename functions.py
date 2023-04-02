from typing import Tuple, Union
import cv2
import numpy as np

class Color:
    def __init__(self, x: float, y: float, z: float, name: str = ...) -> None:
        self.name = name
        self.x = x
        self.y = y
        self.z = z
    def to_tuple(self) -> Tuple:
        return (self.x, self.y, self.z)

class ColorRange:
    def __init__(self, min_color: Color = ...,
                 max_color: Color = ...,
                 name: str = ...,) -> None:
        self.name = name
        self.min_color = min_color
        self.max_color = max_color

        self.min_color.name = name
        self.max_color.name = name

def get_relative_number_ratio(rgb: np.ndarray, red_range: ColorRange, white_range: ColorRange, black_range: ColorRange) -> Tuple[float, Tuple[int, int, int, int]]:
    """Gets relative number ratio in frame.

    Args:
        rgb (np.ndarray): RGB image

    Returns:
        Tuple[float, Tuple[int, int, int, int]] - 1) Float value of ratio; 2) Tuple of number bounding rect: (x, y, w, h)
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    bin = cv2.inRange(hsv, red_range.min_color.to_tuple(), red_range.max_color.to_tuple())
    cnts, _ = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    list_of = []
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < 200: continue
        (x, y, w, h) = cv2.boundingRect(cnt)
        cropped = hsv[y:y+h, x:x+w, :]
        bin2 = cv2.inRange(cropped, white_range.min_color.to_tuple(), white_range.max_color.to_tuple())
        cnts2, _ = cv2.findContours(bin2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt2 in cnts2:
            area2 = cv2.contourArea(cnt2)
            if area2 < 200: continue
            bin3 = cv2.inRange(cropped, black_range.min_color.to_tuple(), black_range.max_color.to_tuple())
            cnts3, _ = cv2.findContours(bin3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt3 in cnts3:
                area3 = cv2.contourArea(cnt3)
                if area3 < 200: continue
                (x3, y3, w3, h3) = cv2.boundingRect(cnt3)
                yield ((x3+x, y3+y, w3, h3), area/area3)
    # return (False, (None, None, None, None), None)

if __name__ == "__main__":
    red_range = ColorRange(Color(348/2,215,227), Color(368/2,235,247))
    white_range = ColorRange(Color(0, 0, 240), Color(10/2, 10, 255))
    black_range = ColorRange(Color(0, 0, 0), Color(10/2, 10, 10))

    img = cv2.imread('d:/123.png')
    draw = img.copy()
    l = list(get_relative_number_ratio(img, red_range, white_range, black_range))
    l.sort(key=lambda r: r[1])
    _, ((x, y, w, h), ratio), _, *other = l
    print(ratio)
    cv2.rectangle(draw, (x, y), (x+w, h+y), (255, 0, 0), 2)
    cv2.imshow('draw', draw)
    cv2.waitKey(0)