import pymurapi as mur
import cv2
import time
auv = mur.mur_init()
# video = auv.get_videoserver()

# cap = cv2.VideoCapture(0)

rectTop = ((0, 0), (320, 20))
rectBottom = ((0, 220), (320, 240))
rectRing = ((0, 0), (20, 20))

depth = 3.1

Kp_depth = 150  # кофецент пропорционального регулятора на глубину
Kd_depth = 20  # кофецент дифференциального регулятора на глубину

Kp_yaw = 0.05  # кофецент пропорционального регулятора на курс
Kd_yaw = 0.005  # кофецент дифференциального регулятора на курс

Kp_distance = 0.5  # кофецент пропорционального регулятора на курс
Kd_distance = 0.001  # кофецент дифференциального регулятора на курс

Kp_line = 1  # кофецент пропорционального регулятора на курс
Kd_line = 0.05  # кофецент дифференциального регулятора на курс

THRUSTER_DEPTH_LEFT = 2
THRUSTER_DEPTH_RIGHT = 3
THRUSTER_YAW_LEFT = 0
THRUSTER_YAW_RIGHT = 1

DIRECTION_THRUSTER_DEPTH_LEFT = 1
DIRECTION_THRUSTER_DEPTH_RIGHT = 1
DIRECTION_THRUSTER_YAW_LEFT = 1
DIRECTION_THRUSTER_YAW_RIGHT = 1

HSV_GREEN = ((40, 10, 10), (80, 255, 255))
HSV_PURPLE = ((115, 20, 20), (135, 255, 255))
HSV_YELLOW = ((20, 20, 20), (50, 255, 255))
HSV_PINK = ((135, 20, 20), (170, 255, 255))

class PD(object):
    _kp = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __init__(self, kp: float = 0, kd: float = 0):
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
    
class Timeout:
    def __init__(self, interval = 0) -> None:
        self.start_time = time.time()
        self.stop_time = None
        self.started = False
        self.interval = interval

    def start(self, interval = ...):
        if interval is not ...:
            self.interval = interval
        self.start_time = time.time()
        self.started = True

    def stop(self):
        self.started = False
        self.stop_time = time.time()

    def reset(self):
        self.start_time = time.time()

    @property
    def elapsed(self):
        return time.time() - self.start_time
    
    @property
    def is_timed_out(self):
        return self.elapsed > self.interval
    
class Counter:
    def __init__(self, default_value = 0) -> None:
        self.value = default_value

    def increment(self):
        self.value += 1

    def decrement(self):
        self.value -= 1


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
    
def get_depth_correction(k=1):
    return auv.get_depth() * k - 0.1
    
def keep_yaw(yaw_to_set, speed_to_yaw=0,
             error=...):  # ПД регулятор по курсу, !!! без ожидания будет выдавать ошибку !!!
    try:
        if error is ...:
            # вычесление ошибки, действительное значение - заданное значение
            error = yaw_to_set - auv.get_yaw()
            error = clamp_to180(error)  # проверяем ошибку на ограничение
        time.sleep(0.001)
        # забиваем ошибку и получаем выходное значение на моторы
        output = keep_yaw.regulator.process(error)
        # проверяем выходное значение на ограничение
        output = clamp(output, 100, -100)
        auv.set_motor_power(THRUSTER_YAW_LEFT, DIRECTION_THRUSTER_YAW_LEFT * clamp((speed_to_yaw - output), 100,
                                                                                   -100))  # передаём выходное значение на мотор 0
        auv.set_motor_power(THRUSTER_YAW_RIGHT, DIRECTION_THRUSTER_YAW_RIGHT * clamp((speed_to_yaw + output), 100,
                                                                                     -100))  # передаём выходное значение на мотор 1
    except AttributeError:  # активируется при первом запуске, записываются кофиценты
        keep_yaw.regulator = PD()
        keep_yaw.regulator.set_p_gain(Kp_yaw)  # запись пк на курс
        keep_yaw.regulator.set_d_gain(Kd_yaw)  # запись дк на курс

def keep_line(speed_to_yaw=0,
             error=...):  # ПД регулятор по курсу, !!! без ожидания будет выдавать ошибку !!!
    try:
        time.sleep(0.001)
        # забиваем ошибку и получаем выходное значение на моторы
        output = keep_line.regulator.process(error)
        # проверяем выходное значение на ограничение
        output = clamp(output, 100, -100)
        auv.set_motor_power(THRUSTER_YAW_LEFT, DIRECTION_THRUSTER_YAW_LEFT * clamp((speed_to_yaw - output), 100,
                                                                                   -100))  # передаём выходное значение на мотор 0
        auv.set_motor_power(THRUSTER_YAW_RIGHT, DIRECTION_THRUSTER_YAW_RIGHT * clamp((speed_to_yaw + output), 100,
                                                                                     -100))  # передаём выходное значение на мотор 1
    except AttributeError:  # активируется при первом запуске, записываются кофиценты
        keep_line.regulator = PD()
        keep_line.regulator.set_p_gain(Kp_line)  # запись пк на курс
        keep_line.regulator.set_d_gain(Kd_line)  # запись дк на курс


# ПД регулятор по глубине, !!! без ожидания будет выдавать ошибку !!!
def keep_depth(depth_to_set):
    speed_to_depth = 1
    try:
        time.sleep(0.001)
        # вычесление ошибки, действительное значение - заданное значение
        error = get_depth_correction() - depth_to_set
        # забиваем ошибку и получаем выходное значение на моторы
        output = keep_depth.regulator.process(error)
        output += 10
        # проверяем выходное значение на
        output = clamp(output * speed_to_depth, 30, -30)
        auv.set_motor_power(THRUSTER_DEPTH_LEFT,
                            output * DIRECTION_THRUSTER_DEPTH_LEFT)  # передаём выходное значение на мотор 2
        auv.set_motor_power(THRUSTER_DEPTH_RIGHT,
                            output * DIRECTION_THRUSTER_DEPTH_RIGHT)  # передаём выходное значение на мотор 3
    except AttributeError:  # активируется при первом запуске, записываются кофиценты
        keep_depth.regulator = PD()
        keep_depth.regulator.set_p_gain(Kp_depth)  # запись пк на глубину
        keep_depth.regulator.set_d_gain(Kd_depth)  # запись дк на глубину

def lineCenter(rect, frame):
    cropped = frame[rect[0][1]:rect[1][1], rect[0][0]:rect[1][0], :]
    # gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    thresh = cv2.inRange(cropped, *HSV_PURPLE)
    # r, thresh = cv2.threshold(gray, 300, 255, cv2.THRESH_OTSU)
#    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 4)
    # thresh = cv2.bitwise_not(thresh)
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

def find_basket(frame, draw, color_range = HSV_GREEN):
    '''Find basket'''
    frame = frame[rectTop[0][1]:rectTop[1][1], rectTop[0][0]:rectTop[1][0], :]
    bin = cv2.inRange(frame, *color_range)
    cnts, _ = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [(c, cv2.contourArea(c)) for c in cnts]
    cnts = [x for x in cnts if x[1] > 500]
    if cnts:
        maxCnt = max(cnts, key=lambda x: x[1])[0]
        cv2.drawContours(draw, [maxCnt], -1, (255, 255, 255), 3)
        return maxCnt
    
def stabilize(value, value_to_set, accuracy, time_):
    try:
        # print(value - value_to_set)
        if abs(value - value_to_set) > accuracy:
            del stabilize.end_time
        if time.time() >= stabilize.end_time:
            del stabilize.end_time
            print("End of stab")
            return False
    except AttributeError:
        stabilize.end_time = time.time() + time_
    return True


def max_contour(hsv_frame, draw_frame, color_range):
    thresh = cv2.inRange(hsv_frame, *color_range)
    cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    x = 0
    w = 0
    cnts = [c for c in cnts if cv2.contourArea(c) > 200]
    if cnts:
        x, y, w, h = cv2.boundingRect(max(cnts, key=lambda c: cv2.contourArea(c)))
        cv2.rectangle(draw_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        return (x, y, w, h)

def stab_on_basket_yaw(hsv, draw, color_range = HSV_GREEN):
    thresh = cv2.inRange(hsv, *color_range)
    cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contour_x = 0
    w = 0
    cnts = [c for c in cnts if cv2.contourArea(c) > 200]
    if cnts:
        contour_x, y, w, h = cv2.boundingRect(max(cnts, key=lambda c: cv2.contourArea(c)))
        cv2.rectangle(draw, (contour_x, y), (contour_x + w, y + h), (255, 0, 0), 2)
    keep_yaw(0, speed_to_yaw=0, error=320 // 2 - (contour_x + w // 2))
    keep_depth(depth)
    return contour_x + w // 2

def stab_on_basket_distance(yaw, hsv, draw, color_range = HSV_GREEN):
    try:
        cnt = max_contour(hsv, draw, color_range)
        y, h = 0, 0
        if cnt:
            _, y, _, h = cnt
        mid = y + h // 2
        error = 240 // 2 - mid
        U = stab_on_basket_distance.regulator.process(error)
        keep_yaw(yaw, speed_to_yaw=U)
        keep_depth(depth)
        return mid
    except AttributeError:
        stab_on_basket_distance.regulator = PD()
        stab_on_basket_distance.regulator.set_p_gain(Kp_distance)  # запись пк на глубину
        stab_on_basket_distance.regulator.set_d_gain(Kd_distance)  # запись дк на глубину
    return 0

def detect_ring(hsv, draw, color_range = HSV_PINK):
    return max_contour(hsv[rectRing[0][1]:rectRing[1][1], rectRing[0][0]:rectRing[1][0], :], draw, color_range)

# def rotate_360(time_to_rotate = 2, time_to_hold_yaw=2):
#     auv.set_rgb_color(255, 0, 0)
#     yaw = auv.get_yaw()
#     timer_end = time.time() + time_to_rotate
#     while time.time() < timer_end:
#         r, frame = cap.read()
#         keep_yaw(0, error=30)
#         keep_depth(depth)
#         cv2.imshow('frame', frame)
#         # video.show(frame, 0)
#     auv.set_rgb_color(255, 255, 0)
#     timer_end = time.time() + time_to_hold_yaw
#     while time.time() < timer_end:
#         r, frame = cap.read()
#         keep_yaw(yaw)
#         keep_depth(depth)
#         cv2.imshow('frame', frame)
#         # video.show(frame, 0)

line_reg = PD(Kp_line, Kd_line)

init_yaw = clamp_to180(auv.get_yaw() + 180)
true_init_yaw = auv.get_yaw()
# auv.set_rgb_color(0, 0, 255)
# time.sleep(2)
timer = time.time()
while time.time() < timer + 10:
    keep_yaw(init_yaw, 0)
    keep_depth(depth)

# auv.set_rgb_color(255, 255, 255)

last_drop_time = 0
last_yaw = init_yaw

basket_timeout = Timeout(5)
ring_timeout = Timeout(5)
green_counter = Counter()
yellow_counter = Counter()
ring_counter = Counter()

basket_counter = Counter()

start_time = time.time()

while True:
    frame = auv.get_image_bottom()
    frame = cv2.resize(frame, (320, 240))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    draw = frame.copy()
    
    centerTop = lineCenter(rectTop, hsv)
    if centerTop == -1:
        cv2.rectangle(draw, *rectBottom, (255, 255, 255), 2)
        centerTop = lineCenter(rectBottom, hsv)
    else:
        cv2.rectangle(draw, *rectTop, (255, 255, 255), 2)

    cv2.circle(draw, (int(centerTop), 10), 3, (255, 255, 255), 3)
    error = line_reg.process(320 // 2 - centerTop)

    cv2.putText(draw, "Error: " + str(error), (10, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))
    
    keep_line(error=error, speed_to_yaw=15)
    keep_depth(depth)

    if basket_timeout.is_timed_out:
        cnt_green = find_basket(hsv, draw)
        cnt_yellow = find_basket(hsv, draw, HSV_YELLOW)
        if cnt_green is not None or cnt_yellow is not None:
            prev_yaw = clamp_to180(auv.get_yaw() - 180)
            if cnt_green is not None:
                green_counter.increment()
                color = HSV_GREEN
                print("Green basket detected.")
            else:
                yellow_counter.increment()
                color = HSV_YELLOW
                print("Yellow basket detected")
            print("Stab yaw on basket yaw...")
            while stabilize(stab_on_basket_yaw(hsv, draw, color), 320 // 2, 20, 3):
                cv2.imshow('draw', draw)
                cv2.waitKey(1)
                frame = auv.get_image_bottom()
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                draw = frame.copy()
            # print("Stab yaw on basket distance...")
            # yaw = auv.get_yaw()
            # while stabilize(stab_on_basket_distance(yaw, hsv, draw, color), 240 // 2, 20, 3):
            #     cv2.imshow('draw', draw)
            #     cv2.waitKey(1)
            #     frame = auv.get_image_bottom()
            #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            #     draw = frame.copy()
            auv.drop()
            yaw_timeout = Timeout(5)
            while not yaw_timeout.is_timed_out:
                keep_yaw(prev_yaw, 0)
                cv2.imshow('draw', draw)
                cv2.waitKey(1)
                frame = auv.get_image_bottom()
                draw = frame.copy()
            basket_timeout.reset()
            basket_counter.increment()

    # if ring_timeout.is_timed_out and detect_ring(hsv, draw):
    #     print("Ring detected", ring_counter.value + 1)
    #     ring_counter.increment()
    #     ring_timeout.reset()
    # if abs(true_init_yaw - auv.get_yaw()) < 5:
    # по часовой стрекле четное
    if basket_counter.value >= 5:
        cv2.putText(draw, "Init yaw", (10, 10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 0, 0))
        if time.time() - start_time > 10:
        # if ring_counter.value >= 4 and ring_timeout.is_timed_out:
            if (green_counter.value + yellow_counter.value * 2) % 2 == 0:
                direction = 1
            else:
                direction = -1

            auv.set_motor_power(THRUSTER_YAW_LEFT, DIRECTION_THRUSTER_YAW_LEFT * 100 * direction)
            auv.set_motor_power(THRUSTER_YAW_RIGHT, DIRECTION_THRUSTER_YAW_RIGHT * -100 * direction)

            auv.set_motor_power(THRUSTER_DEPTH_LEFT, DIRECTION_THRUSTER_DEPTH_LEFT * 100)
            auv.set_motor_power(THRUSTER_DEPTH_RIGHT, DIRECTION_THRUSTER_DEPTH_RIGHT * 100)

            while True:
                frame = auv.get_image_bottom()
                cv2.imshow('draw', draw)
                cv2.waitKey(1)

    cv2.imshow('draw', draw)
    cv2.waitKey(1)
    