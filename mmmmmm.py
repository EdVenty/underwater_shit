import pymurapi as mur
import time
import cv2 as cv
import math
import numpy as np

MOTOR_DEPTH_LEFT = 0
MOTOR_DEPTH_RIGHT = 3
MOTOR_YAW_LEFT = 1
MOTOR_YAW_RIGHT = 2

auv = mur.mur_init()

cap = cv.VideoCapture(1)

low_hsv = (0, 70, 70)
max_hsv = (180, 255, 255)

Kp_depth = 0.5
Kd_depth = 0.5
Kp_yaw = 0.5
Kd_yaw = 0.4

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

def keep_yaw(set_yaw, speed=0):
    try:
        error = auv.get_yaw() - set_yaw
        error = clamp_to180(error)
        output = keep_yaw.regulator.process(error)
        output = clamp(output, 100, -100)
        auv.set_motor_power(MOTOR_YAW_RIGHT, output  + speed)
        auv.set_motor_power(MOTOR_YAW_LEFT, -output  + speed)
        return error
    except AttributeError:
        keep_yaw.regulator = PD()
        keep_yaw.regulator.set_p_gain(0.5)
        keep_yaw.regulator.set_d_gain(0.5)
    return False


def keep_depth(depth_to_set, speed_to_depth=1.0):
    try:
        try:
            error = auv.get_depth() - depth_to_set
            output = keep_depth.regulator.process(error)
            output = clamp(output * speed_to_depth, 100, -100)
            
            auv.set_motor_power(MOTOR_DEPTH_LEFT, output)
            auv.set_motor_power(MOTOR_DEPTH_RIGHT, output)
            return error
        except ZeroDivisionError:
            time.sleep(0.001)
            error = auv.get_depth() - depth_to_set
            output = keep_depth.regulator.process(error)
            output = clamp(output * speed_to_depth, 100, -100)
            auv.set_motor_power(MOTOR_DEPTH_LEFT, output)
            auv.set_motor_power(MOTOR_DEPTH_RIGHT, output)
            return error
    except AttributeError:
        keep_depth.regulator = PD()
        keep_depth.regulator.set_p_gain(Kp_depth)
        keep_depth.regulator.set_d_gain(Kd_depth)
        return 0

init_yaw = auv.get_yaw()
# init_depth = auv.get_depth()
init_depth = 1

timer = time.time()

def rotate(yaw, depth=None, pogreshnost=5):
    time_flag = time.time()
    while True:
        try:
            error = keep_yaw(yaw)
            if depth is not None:
                keep_depth(depth)
            print(error)
            if abs(error) > pogreshnost:
                time_flag = time.time()
            if (time.time() - time_flag) > 4:
                break   
        except ZeroDivisionError: pass

def forward(yaw, speed, time_to):
    time_flag = time.time()
    while True:
        try:
            keep_yaw(yaw, speed)
        except ZeroDivisionError: pass
        if time.time() - time_flag > time_to:
            break

def stable_yaw(set_yaw, stable_yaw, accuracy=1, stab_time=3):
    # стабилизация по углу центра контура
    global time_new
    try:
        error = set_yaw - stable_yaw
        output = keep_yaw.regulator.process(error)
        output = clamp(output, 100, -100)
        # if abs(error) > accuracy:
        auv.set_motor_power(MOTOR_DEPTH_RIGHT, -output)
        auv.set_motor_power(MOTOR_DEPTH_LEFT, output)
        #     time_new = time.time()
        # if stab_time < time.time() - time_new:
        #     return True
        return error
    except AttributeError:
        keep_yaw.regulator = PD()
        keep_yaw.regulator.set_p_gain(0.3)
        keep_yaw.regulator.set_d_gain(0.4)
    return False

def stab_on_(curve, accuracy=3.0, speed=1, stab_time=3):
    # стабилизация на фигуре: по глубине и по углу
    global time_new
    try:
        moments = cv.moments(curve)
        x = int(moments["m10"] / moments["m00"])
        y = int(moments["m01"] / moments["m00"])
        x_center = x - (320 / 2)
        y_center = y - (240 / 2)
        # tan_a = x_center / y_center
        # sin_a = x_center / ((x_center ** 2 + y_center ** 2) ** 0.5)
        # angle = math.asin(sin_a) * 180 / math.pi
        stable_yaw(x, 160)
        stable_y(y_center, speed)

    except ZeroDivisionError:
        return False
    return False

def stable_y(y_center, speed=1, accuracy=0.3, stab_time=2):
    # стабилизация по глубине центра контура
    global time_new
    try:
        error = y_center
        output = stab_on_.regulator.process(error)
        output = clamp(output * speed, 100, -100)
        # if abs(error ** 2) > accuracy:
        auv.set_motor_power(3, -output)
        auv.set_motor_power(2, -output)
        # if stab_time < time.time() - time_new:
        #     print("stabled")
        #     return True
        return error
    except AttributeError:
        stab_on_.regulator = PD()
        stab_on_.regulator.set_p_gain(Kp_depth)
        stab_on_.regulator.set_d_gain(Kd_depth)
    return False


def define_form(cont):
    # опеределение формы
    if abs(cv.contourArea(cont)) > 100:
        hull = cv.convexHull(cont)
        # сглаженный контур - без потерь (нужно ли?)
        apr_shape = cv.approxPolyDP(hull, cv.arcLength(hull, True) * 0.04, True)
        #
        # аппроксимированная фигура
        if len(apr_shape) == 3:
            form = "triangle"

            return form, apr_shape
        elif len(apr_shape) == 4:
            form = "square"
            return form, apr_shape
        else:
            form = "undefined"
            return form, apr_shape
    else:
        form = "none"
        return form, cont

def find_contour(img, low_hsv, max_hsv):
    imageHSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    img_bin = cv.inRange(imageHSV, low_hsv, max_hsv)
    cont, _ = cv.findContours(img_bin, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    return cont

def depth_reg(depth, speed_to_depth, pogreshnost=5):
    timer = time.time()
    time_flag = time.time()
    while True:
        try:
            error = keep_depth(depth, speed_to_depth)
            print(error)
            if abs(error) > 0.5:
                time_flag = time.time()
            if (time.time() - time_flag) > 4:
                break   
        except ZeroDivisionError: pass

def defining_figure(cnt):
    c = cv.convexHull(cnt)  # сглаживаем фигуру
    # аппроксимируем фигуры, (!!!)
    # angle_arrow = cv2.approxPolyDP(c, cv2.arcLength(c, True) * 0.15, True)

    # рисуем треугольник, получаем площадь
    s_triangle, _ = cv.minEnclosingTriangle(c)
    # рисуем прямоугольник, получаем x,y верхнего левого угла,
    # высоту и длину и угол  поворота
    ((_, _), (w, h), _) = cv.minAreaRect(c)
    #  возвращает координаты центра x, y и радиус (строит круг)
    _, radius = cv.minEnclosingCircle(c)

    # вычесление площадей кргуа, квадрата, треугольника
    rectangle_area = w * h
    circle_area = radius ** 2 * math.pi

    # вычесление центра фигуры
    moments = cv.moments(c)  # получение моментов
    try:
        x = int(moments["m10"] / moments["m00"])  # центр
        y = int(moments["m01"] / moments["m00"])
    except ZeroDivisionError:
        return 0, 0, "none"

    # print("Triangle:", s_triangle, "Circle:", circle_area)
    if rectangle_area > circle_area < s_triangle:
        return x, y, "circle"
    elif circle_area > rectangle_area < s_triangle:
        return x, y, "square"
    elif rectangle_area > s_triangle < circle_area:
        return x, y, "triangle"
        
while True:
    r, img = cap.read()
    try:
        keep_depth(1)
    except ZeroDivisionError: pass
    try:
        keep_yaw(init_yaw, 20)
    except ZeroDivisionError: pass
    if time.time() - timer >= 6:
        rotate(init_yaw - 90, init_depth, 20)
        print("rotated")
        # depth_reg(1, 1)
        # print("depth stab")
        # forward(init_yaw - 90, 50, 3)
        # print("forwarded")
        # forward(init_yaw - 90, -50, 3)
        # print("backwarded")
        timer2 = time.time()
        while True:
            r, img = cap.read()
            masks = None
            
            for (min_hsv, max_hsv) in (
                ((80, 160, 0),
                (140, 255, 100)),
                #lowb=(93, 199, 65)
                #maxb=(148, 250, 255)
                ((25, 130, 50),
                (80, 255, 255)),
                ((0, 60, 60),
                (25, 255, 255)),
                ((170, 60, 60),
                (180, 255, 255)),
                ((30, 80, 20),
                (45, 255, 255)),
                ((255, 160, 0),
                (255, 255, 15))
            ):
                mask = cv.inRange(img, min_hsv, max_hsv)
                if masks is None:
                    masks = mask
                else:
                    masks += mask
            cnts, _ = cv.findContours(masks, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            
            if cnts:
                for cnt in cnts:
                    if cv.contourArea(cnt) > 100:
                        stab_on_(cnt)
                        break
            else:
                break
            if time.time() - timer2 > 5:
                print("getting shape")
                r, img = cap.read()
                cnts = find_contour(img, low_hsv, max_hsv)
                if cnts:
                    for cnt in cnts:
                        if cv.contourArea(cnt) > 100:
                            x, y, f = defining_figure(cnt)
                            print(f)
                            rotate(init_yaw, 1)
                            break
                break

        timer = time.time()
    # cv.imshow("img", img)
    # cv.waitKey(1)

