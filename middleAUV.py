import pymurapi as mur
import cv2
import time
import numpy as np
from math import sqrt
auv = mur.mur_init()
video = auv.get_videoserver()

cap = cv2.VideoCapture(0)

rectTop = ((0, 0), (320, 20))
rectBottom = ((0, 220), (320, 240))

depth = 0.8

Kp_depth = 150  # кофецент пропорционального регулятора на глубину
Kd_depth = 200  # кофецент дифференциального регулятора на глубину
Kp_yaw = 0.5  # кофецент пропорционального регулятора на курс
Kd_yaw = 0.3  # кофецент дифференциального регулятора на курс
Kp_line = 0.7  # кофецент пропорционального регулятора на курс
Kd_line = 1  # кофецент дифференциального регулятора на курс

THRUSTER_DEPTH_LEFT = 0
THRUSTER_DEPTH_RIGHT = 3
THRUSTER_YAW_LEFT = 1
THRUSTER_YAW_RIGHT = 2

DIRECTION_THRUSTER_DEPTH_LEFT = +1
DIRECTION_THRUSTER_DEPTH_RIGHT = -1
DIRECTION_THRUSTER_YAW_LEFT = -1
DIRECTION_THRUSTER_YAW_RIGHT = -1

HSV_YELLOW = (np.array((0, 100, 0)), np.array((80, 255, 255)))
HSV_GREEN = (np.array((30, 100, 0)), np.array((60, 255, 255)))

class PD(object):
    _kp = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __init__(self, kp=0, kd=0):
        self._kp = kp
        self._kd = kd

    def set_p_gain(self, value):
        self._kp = value

    def set_d_gain(self, value):
        self._kd = value

    def process(self, error):
        timestamp = int(round(time.time() * 1000))  # в timestamp записываем
        # время(выраженное в секундах) и домножаем до милисекунд, round отбрасывает знаки после запятой
        output = self._kp * error + self._kd / \
            (timestamp - self._timestamp) * (error - self._prev_error)
        # вычесляем выходное значение на моторы по ПД регулятору и записываем в output
        self._timestamp = timestamp  # перезаписываем время
        self._prev_error = error  # перезаписываем ошибку
        return output


def clamp_to180(angle):  # ограничитель максимального зачения для курса
    if angle > 180:
        return angle - 360
    if angle < -180:
        return angle + 360

    return angle


def clamp(v, max_v, min_v):  # ограничитель максимального зачения для моторов
    if v > max_v:
        return max_v
    if v < min_v:
        return min_v
    return v
    

    



# ПД регулятор по глубине, !!! без ожидания будет выдавать ошибку !!!

def lineCenter(rect, frame):
    cropped = frame[rect[0][1]:rect[1][1], rect[0][0]:rect[1][0], :]
    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    r, thresh = cv2.threshold(gray, 300, 255, cv2.THRESH_OTSU)
#    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 4)
    thresh = cv2.bitwise_not(thresh)
    cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    filtered = [c for c in cnts if cv2.contourArea(c) > 30]
    nearestX = -1
    nearestVal = 10000
    for cnt in filtered:
        M = cv2.moments(cnt)
        center = M['m10'] / M['m00']
        ran = abs((320 // 2) - center)
        if ran < nearestVal:
            nearestX = center
            nearestVal = ran
    return nearestX

def check_start_point(frame, hsv):
    h, _, _ = frame.shape
    checkFrame = frame[h//2:, :, :]
    mask = cv2.inRange(checkFrame, *hsv)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_cnt = max(cnts, key=lambda x: cv2.contourArea(x))
    if cv2.contourArea(max_cnt) > 30:
        return True
    return False

def get_start_point_color(frame, hsv_list):
    distances = []
    for hsv in hsv_list:
        mask = cv2.inRange(frame, *hsv)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            max_cnt = max(cnts, key=lambda x: cv2.contourArea(x))
            if cv2.contourArea(max_cnt) > 30:
                M = cv2.moments(max_cnt)
                cX = M['m10'] / M['m00']
                cY = M['m01'] / M['m00']
                distance = sqrt((320/2 - cX)**2 + (240/2 - cY)**2)
                distances.append(distance)
    hsv, dst = min(zip(hsv_list, distances), key=lambda x: x[1])
    return hsv

def rotate_360(time_to_rotate = 2, time_to_hold_yaw=2):
    auv.set_rgb_color(255, 0, 0)
    yaw = auv.get_yaw()
    timer_end = time.time() + time_to_rotate
    while time.time() < timer_end:
        r, frame = cap.read()
        keep_yaw(0, error=30)
        keep_depth(depth)
        video.show(frame, 0)
    auv.set_rgb_color(255, 255, 0)
    timer_end = time.time() + time_to_hold_yaw
    while time.time() < timer_end:
        r, frame = cap.read()
        keep_yaw(yaw)
        keep_depth(depth)
        video.show(frame, 0)
    
hsv_color = None
line_reg = PD(Kp_line, Kd_line)

init_yaw = auv.get_yaw()
auv.set_rgb_color(0, 0, 255)
time.sleep(2)
timer = time.time()
while time.time() < timer + 3:
    keep_yaw(init_yaw, 10)
    keep_depth(depth)

auv.set_rgb_color(255, 255, 255)

#while True:r, frame = cap.read()
# frame = cv2.resize(frame, (320, 240))
#    mask = cv2.inRange(frame, *HSV_YELLOW)
#    video.show(mask, 1)
#    video.show(frame, 0)
#    keep_yaw(init_yaw)
#    keep_depth(depth)
# hsv_color = get_start_point_color(frame, [HSV_GREEN, HSV_YELLOW])

# check_delay = 10
# start_check_next_timer = time.time() +check_delay
# counter = 0
# while True:
#     r, frame = cap.read()
#     frame = cv2.resize(frame, (320, 240))
#     # if time.time() > start_check_next_timer:
#     #     if check_start_point(frame, hsv_color):
#     #         rotate_360()
#     #         auv.set_rgb_color(255, 255, 255)
#     #         start_check_next_timer = time.time() + check_delay
#     #         counter += 1
#     #     if counter >= 3:
#     #         print("finished!")
#     #         break
#     draw = frame.copy()
#     cv2.rectangle(draw, *rectTop, (255, 255, 255), 2)
# #    cv2.rectangle(draw, *rectBottom, (255, 255, 255), 2)
#     centerTop = lineCenter(rectTop, frame)
# #    centerBottom = lineCenter(rectBottom, frame)
#     cv2.circle(draw, (int(centerTop), 10), 3, (255, 255, 255), 3)
#     error = line_reg.process(320 // 2 - centerTop)
# #    cv2.circle(draw, (int(centerBottom), 230), 3, (255, 255, 255), 3)
#     cv2.putText(draw, "Error: " + str(error), (10, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))
#     keep_yaw(0, 30, error=error)
#     keep_depth(depth)
#     ### comment -----------
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     r, thresh = cv2.threshold(gray, 300, 255, cv2.THRESH_OTSU)
# #    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 11)
#     thresh = cv2.bitwise_not(thresh)
#     video.show(thresh, 1)
#     ### comment -----------
#     video.show(draw, 0)

class D:
    PLUS = +1
    MINUS = -1

class ThrustersConfig:
    def __init__(self, 
        depth_left_id: int,
        depth_right_id: int,
        yaw_left_id: int,
        yaw_right_id: int,
        depth_left_direction: D,
        depth_right_direction: D,
        yaw_left_direction: D,
        yaw_right_direction: D):
        self.depth_left_id = depth_left_id
        self.depth_right_id = depth_right_id
        self.yaw_left_id = yaw_left_id
        self.yaw_right_id = yaw_right_id
        self.depth_left_direction = depth_left_direction
        self.depth_right_direction = depth_right_direction
        self.yaw_left_direction = yaw_left_direction
        self.yaw_right_direction = yaw_right_direction

class Regulators:
    def __init__(self,
        depth_reg: PD, yaw_reg: PD
        ):
        self.depth_reg = depth_reg
        self.yaw_reg = yaw_reg

class CamerasConfig:
    def __init__(self,
        front_camera_id: int,
        bottom_camera_id: int):
        self.front_camera_id = front_camera_id
        self.bottom_camera_id = bottom_camera_id

class AUVConfig:
    def __init__(self, 
        thrusters_config: ThrustersConfig, 
        regulators: Regulators, 
        cameras_config: CamerasConfig):
        self.thrusters_config = thrusters_config
        self.regulators = regulators
        self.cameras_config = cameras_config

class AUV:
    def __init__(self, config: AUVConfig):
        self.mur_auv = mur.mur_init()
        self.video_server = auv.get_videoserver()
        self.cap_front = cv2.VideoCapture(config.cameras_config.front_camera_id)
        self.cap_bottom = cv2.VideoCapture(config.cameras_config.bottom_camera_id)
        self.regulators = config.regulators

    def get_depth_correction(self, k=1):
        return self.mur_auv.get_depth() * k - 0.1

    def keep_depth(self, depth_to_set: float, speed_to_depth=1):
        time.sleep(0.001)
        # вычесление ошибки, действительное значение - заданное значение
        error = self.get_depth_correction() - depth_to_set
        # забиваем ошибку и получаем выходное значение на моторы
        output = self.regulators.depth_reg.process(error)
        output += 10
        # проверяем выходное значение на
        output = clamp(output * speed_to_depth, 30, -30)
        auv.set_motor_power(THRUSTER_DEPTH_LEFT,
                            output * DIRECTION_THRUSTER_DEPTH_LEFT)  # передаём выходное значение на мотор 2
        auv.set_motor_power(THRUSTER_DEPTH_RIGHT,
                            output * DIRECTION_THRUSTER_DEPTH_RIGHT)  # передаём выходное значение на мотор 3
        
    def keep_yaw(self, yaw_to_set: int, forward_speed=0):
        error = yaw_to_set - auv.get_yaw()
        error = clamp_to180(error)  # проверяем ошибку на ограничение
        self.keep_yaw_error(error, forward_speed)
    def keep_yaw_error(self, error: float, forward_speed=0):
        time.sleep(0.001)
        # забиваем ошибку и получаем выходное значение на моторы
        output = self.regulators.yaw_reg.process(error)
        # проверяем выходное значение на ограничение
        output = clamp(output, 100, -100)
        auv.set_motor_power(THRUSTER_YAW_LEFT, DIRECTION_THRUSTER_YAW_LEFT * clamp((forward_speed - output), 100,
                                                                                   -100))  # передаём выходное значение на мотор 0
        auv.set_motor_power(THRUSTER_YAW_RIGHT, DIRECTION_THRUSTER_YAW_RIGHT * clamp((forward_speed + output), 100,
                                                                                     -100))  # передаём выходное значение на мотор 1

    def get_frame_front(self):
        return self.cap_front.read()[1]
    def get_frame_bottom(self):
        return self.cap_bottom.read()[1]
    def show_frame_top(self, frame: np.ndarray):
        self.video_server.show(frame, 0)
    def show_frame_bottom(self, frame: np.ndarray):
        self.video_server.show(frame, 1)