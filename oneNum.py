import cv2
import test
import numpy as np
np.sq

frame = cv2.imread("img.png")
frame = cv2.resize(frame, (28, 28))
print(int(test.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)).data.max(1, keepdim=True)[1][0][0]))