import base64
import math
import numpy as np
import cv2

def readb64(encoded_data):
   nparr = np.frombuffer(base64.b64decode(encoded_data), np.uint8)
   img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
   return img

def find_circles(img):
   bin = cv2.inRange(img, (50, 0, 0), (90, 255, 255))
   cnts, _ = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
   cnts = [c for c in cnts if cv2.contourArea(c) >= math.pi * (5 / 2) ** 2]
   return len(cnts)

data = input()
img = readb64(data)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
print(find_circles(hsv))