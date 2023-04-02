import pymurapi as mur
import cv2
import time
auv = mur.mur_init()
video = auv.get_videoserver()

cap = cv2.VideoCapture(0)

rectTop = ((0, 0), (320, 20))
rectBottom = ((0, 220), (320, 240))
hsv_low = np.array((30, 100, 0))
hsv_high = np.array((60, 255, 255))

Kp_depth = 150  # кофецент пропорционального регулятора на глубину
Kd_depth = 200  # кофецент дифференциального регулятора на глубину
Kp_yaw = 0.3  # кофецент пропорционального регулятора на курс
Kd_yaw = 1  # кофецент дифференциального регулятора на курс

THRUSTER_DEPTH_LEFT = 0
THRUSTER_DEPTH_RIGHT = 3
THRUSTER_YAW_LEFT = 1
THRUSTER_YAW_RIGHT = 2

DIRECTION_THRUSTER_DEPTH_LEFT = -1
DIRECTION_THRUSTER_DEPTH_RIGHT = -1
DIRECTION_THRUSTER_YAW_LEFT = 1
DIRECTION_THRUSTER_YAW_RIGHT = 1

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
    
line_reg = PD(Kp_yaw, Kd_yaw)
depth = 0.5

while True:
    r, frame = cap.read()
    frame = cv2.resize(frame, (320, 240))
    draw = frame.copy()
    cv2.rectangle(draw, *rectTop, (255, 255, 255), 2)
#    cv2.rectangle(draw, *rectBottom, (255, 255, 255), 2)
    centerTop = lineCenter(rectTop, frame)
#    centerBottom = lineCenter(rectBottom, frame)
    cv2.circle(draw, (int(centerTop), 10), 3, (255, 255, 255), 3)
    error = line_reg.process(320 // 2 - centerTop)
#    cv2.circle(draw, (int(centerBottom), 230), 3, (255, 255, 255), 3)
    cv2.putText(draw, "Error: " + str(error), (10, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))
    keep_yaw(0, error=error)
    keep_depth(depth)
    ### comment -----------
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    r, thresh = cv2.threshold(gray, 300, 255, cv2.THRESH_OTSU)
#    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 11)
    thresh = cv2.bitwise_not(thresh)
    video.show(thresh, 1)
    ### comment -----------
    video.show(draw, 0)