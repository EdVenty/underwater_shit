import cv2
import numpy as np

import func

hsv_low = (95,130,0)
hsv_high = (115,255,66)

WINDOW_SETTINGS_WHITE_BALANCE = "wb settings"
WINDOW_SETTINGS_GAMMA = "gamma settings"
WINDOW_SETTINGS_UNSHARP = "unsharp settings"
WINDOW_SETTINGS_CLAHE = "CLAHE settings"
WINDOW_SETTINGS_LAPLACE = "laplace settings"

BUTTON_ENABLE_WHITE_BALANCE = "turn white balance"

cv2.namedWindow(WINDOW_SETTINGS_WHITE_BALANCE)
cv2.namedWindow(WINDOW_SETTINGS_GAMMA)
cv2.namedWindow(WINDOW_SETTINGS_UNSHARP)
cv2.namedWindow(WINDOW_SETTINGS_CLAHE)
cv2.namedWindow(WINDOW_SETTINGS_LAPLACE)

# cv2.createButton(BUTTON_ENABLE_WHITE_BALANCE, lambda x: None)

original = cv2.imread('19.png')
img = original.copy()
draw = img.copy()

img = func.white_balance(img)
img = func.adjust_gamma(img, 3)

sharped = func.unsharp_mask(img)
cv2.imshow("unshar mask", sharped)
luminance_corrected = func.CLAHE(sharped)
cv2.imshow("luminance CLAHE", luminance_corrected)
laplaced = func.laplace(luminance_corrected)
cv2.imshow('laplace', laplaced)
# silency = func.silency(luminance_corrected)
# cv2.imshow("silency", silency)
fusioned = func.fusion([
    func.adjust_gamma(luminance_corrected, 0.7),
    func.adjust_gamma(luminance_corrected, 1),
    func.adjust_gamma(luminance_corrected, 3),
    func.adjust_gamma(luminance_corrected, 5),
    func.adjust_gamma(luminance_corrected, 10)
])
img = fusioned
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

mask = cv2.inRange(hsv, hsv_low, hsv_high)

contours, h = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(draw, contours, -1, (255, 0, 0))
hulls = []
for c in contours:
    if cv2.contourArea(c) > 200:
        hulls.append(cv2.convexHull(c))
cv2.drawContours(draw, hulls, -1, (0, 255, 0), 5)
cv2.imshow("mask", mask)
cv2.imshow("img", draw)
cv2.imshow("fusion", fusioned)
# cv2.imshow("norplaced", cv2.bitwise_not(img, mask=cv2.in(laplaced)))
cv2.waitKey(0)
