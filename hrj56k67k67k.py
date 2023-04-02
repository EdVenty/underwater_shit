import pymurapi as mur
import cv2 as cv
import math
import time

auv = mur.mur_init()
mur_view = auv.get_videoserver()
cap0 = cv.VideoCapture(1)
cap1 = cv.VideoCapture(0)
low_hsv = (0, 70, 70)
max_hsv = (180, 255, 255)
lowb=(80, 160, 0)
maxb=(140, 255, 100)
#lowb=(93, 199, 65)
#maxb=(148, 250, 255)
lowg=(25, 130, 50)
maxg=(80, 255, 255)
lowr=(0, 60, 60)
maxr=(25, 255, 255)
lowR=(170, 60, 60)
maxR=(180, 255, 255)
lowy=(30, 80, 20)
maxy=(45, 255, 255)
lowB=(255, 160, 0)
maxB=(255, 255, 15)
Kp_depth = 0.5
Kd_depth = 0.9
Kp_yaw = 0.5
Kd_yaw = 0.9
time_new = 0
timer = 0
kt = 0.1
depth8=0.5

                      
    

class PD(object):
    _kp = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __itit__(self):
        pass

    def set_p_gain(self, value):
        self._kp = value

    def set_d_gain(self, value):
        self._kd = value

    def process(self, error):
        timestamp = int(round(time.time() * 1000))
        output = self._kp * error + self._kd / (timestamp - self._timestamp) * (error - self._prev_error)
        self._timestamp = timestamp
        self._prev_error = error
        return output


def clamp_to180(angle):
    if angle > 180:
        return angle - 360
    if angle < -180:
        return angle + 360
    return angle


def clamp_to90(angle):
    if angle > 90:
        return angle - 180
    if angle < -90:
        return angle + 180
    return angle


def clamp(v, max_v, min_v):
    if v > max_v:
        return max_v
    if v < min_v:
        return min_v
    return v


def find_contour(img, low_hsv, max_hsv):
    imageHSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    img_bin = cv.inRange(imageHSV, low_hsv, max_hsv)
    cont, _ = cv.findContours(img_bin, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    return cont
    

def keep_d(set_d):
    try:
        error = auv.get_depth() - set_d
#        error = clamp_to180(error)
        output = keep_d.regulator.process(error)
        output = clamp(output, 50, -50)
        auv.set_motor_power(1, output+10)
        auv.set_motor_power(2, output+10)
        print(error)
        return error
    except AttributeError:
        keep_d.regulator = PD()
        keep_d.regulator.set_p_gain(100)
        keep_d.regulator.set_d_gain(100)
    return False
def k_yaw(yaw_to_set): # Курс
    def clamp_to180(angle): # 180 градусов
        if angle>180:
            return angle-360
        if angle<-180:
            return angle+360
        return angle
    try:
        error=auv.get_yaw()-yaw_to_set
        error=clamp_to180(error)
        output=k_yaw.regulator.process(error)
        output=clamp(output,100,-100)
        auv.set_motor_power(0, output)
        auv.set_motor_power(3, -output)
    except AttributeError:
        k_yaw.regulator = PD()
        k_yaw.regulator.set_p_gain(0.5)
        k_yaw.regulator.set_d_gain(0.5)

def go_360(): #Разворот на 360
    timer = time.time()
    while time.time() < timer + 4:
        auv.set_motor_power(0, 20)
        auv.set_motor_power(3, 20)
    
def finish(): #Выплыв в кольцо
    auv.set_motor_power(0, 30)
    auv.set_motor_power(3, 30)
    
def touch(): #Касание Круга
    timer = time.time
    while time.time < timer + 2:
        auv.set_motor_power(1, 20)
        auv.set_motor_power(2, -20)
        timer = time.time
    while time.time < timer + 2:
        auv.set_motor_power(1, -20)
        auv.set_motor_power(2, 20)
        
def go_time(name, speed, time):
    timer=time.time()
    while time.time()<timer+time:
        keep_d(depth8)
        auv.set_motor_power(0, speed)
        auv.set_motor_power(3, speed)
    timer=time.time()
    while time.time() < timer + 0.1:
        auv.set_motor_power(0, -speed)
        auv.set_motor_power(3, -speed)
        
        
def yaw_time(yaw, time):
    while time.time()<timer+3:
    try:
        keep_d(depth8)
        error = keep_yaw(int(initial_yaw)+time)

        # print(error)
        if error>10:
            timer = time.time()
    except ZeroDivisionError:
        time.sleep(0.001)

def get_mask():
    ok, frame0 = cap0.read()
    
    frame0 = cv.resize(frame0, (320, 240))
    imageHSV = cv.cvtColor(frame0, cv.COLOR_BGR2HSV)
    img_bin_b = cv.inRange(imageHSV, lowb, maxb)
    img_bin_y = cv.inRange(imageHSV, lowy, maxy)
    img_bin_r = cv.inRange(imageHSV, lowr, maxr)
    img_bin_R = cv.inRange(imageHSV, lowR, maxR)
    img_bin_g = cv.inRange(imageHSV, lowg, maxg)
    img_bin_B = cv.inRange(imageHSV, lowB, maxB)
    img_bin_RED=img_bin_r+img_bin_R
    img_bin = img_bin_b+img_bin_y+img_bin_RED+img_bin_B+img_bin_g
    cont, _ = cv.findContours(img_bin_RED, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    cont_list=[]
    if cont:
        for c in cont:
            if abs(cv.contourArea(c))>900:
                cont_list.append(c)
                print(str(len(cont_list))+"num"+str(cv.contourArea(c)))
    mur_view.show(frame0, 0)
    mur_view.show(img_bin, 1)

     
while True:
    go_time()
    get_mask()
    
mur_view.stop()
print("done")
        
        
        
        
        
        