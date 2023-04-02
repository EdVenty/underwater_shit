import pymurapi as mur
import cv2 as cv
import math
import time
import numpy as np

auv = mur.mur_init()
low_hsv = (0, 70, 70)
max_hsv = (180, 255, 255)

Kp_depth = 0.5
Kd_depth = 0.5
Kp_yaw = 0.5
Kd_yaw = 0.4
time_new = 0


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

def thresholding_find_contours(img, maxValue, blockSize, C):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    return cv.adaptiveThreshold(gray, maxValue, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, blockSize, C)


def keep_yaw(set_yaw):
    try:
        error = auv.get_yaw() - set_yaw
        error = clamp_to180(error)
        output = keep_yaw.regulator.process(error)
        output = clamp(output, 100, -100)
        auv.set_motor_power(1, output)
        auv.set_motor_power(0, -output)
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
            auv.set_motor_power(2, output)
            auv.set_motor_power(3, output)
        except ZeroDivisionError:
            time.sleep(0.001)
            error = auv.get_depth() - depth_to_set
            output = keep_depth.regulator.process(error)
            output = clamp(output * speed_to_depth, 100, -100)
            auv.set_motor_power(2, output)
            auv.set_motor_power(3, output)
    except AttributeError:
        keep_depth.regulator = PD()
        keep_depth.regulator.set_p_gain(Kp_depth)
        keep_depth.regulator.set_d_gain(Kd_depth)


def stable_yaw(set_yaw, stable_yaw, accuracy=1, stab_time=3):
    # стабилизация по углу центра контура
    global time_new
    try:
        error = set_yaw - stable_yaw
        output = keep_yaw.regulator.process(error)
        output = clamp(output, 100, -100)
        # if abs(error) > accuracy:
        auv.set_motor_power(1, -output)
        auv.set_motor_power(0, output)
        #     time_new = time.time()
        # if stab_time < time.time() - time_new:
        #     return True
        return error
    except AttributeError:
        keep_yaw.regulator = PD()
        keep_yaw.regulator.set_p_gain(0.3)
        keep_yaw.regulator.set_d_gain(0.4)
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


def stab_on_area(contour, area):  # стабилизация по площади контура
    try:
        error = cv.contourArea(contour) - area
        output = stab_on_area.regulator.process(error)
        output = clamp(output, 100, -100)
        auv.set_motor_power(1, -output)
        auv.set_motor_power(0, -output)
        return error
    except AttributeError:
        stab_on_area.regulator = PD()
        stab_on_area.regulator.set_p_gain(0.2)
        stab_on_area.regulator.set_d_gain(0.2)
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


def define_course(form, curve, img):  # определение угла между текущим курсом и направлением стрелки внизу
    if (form == "triangle"):
        _, points = cv.minEnclosingTriangle(curve)
        # points=curve
        w1 = round(((points[0, 0, 1] - points[1, 0, 1]) ** 2 + (points[0, 0, 0] - points[1, 0, 0]) ** 2) ** 0.5)
        w2 = round(((points[1, 0, 1] - points[2, 0, 1]) ** 2 + (points[1, 0, 0] - points[2, 0, 0]) ** 2) ** 0.5)
        w3 = round(((points[2, 0, 1] - points[0, 0, 1]) ** 2 + (points[2, 0, 0] - points[0, 0, 0]) ** 2) ** 0.5)
        # print(points[0,0,1])
        cv.circle(img, (points[0, 0, 0], points[0, 0, 1]), 10, (0, 0, 255), 3)
        cv.circle(img, (points[1, 0, 0], points[1, 0, 1]), 10, (0, 255, 0), 3)
        cv.circle(img, (points[2, 0, 0], points[2, 0, 1]), 10, (255, 0, 255), 3)
        x_1 = points[0, 0, 0]
        y_1 = points[0, 0, 1]
        x_2 = points[1, 0, 0]
        y_2 = points[1, 0, 1]
        x_3 = points[2, 0, 0]
        y_3 = points[2, 0, 1]
        if w2 < w1 > w3:
            x_cent = int((x_1 + x_2) / 2)  # координаты центра длинной стороны
            y_cent = int((y_1 + y_2) / 2)
            cv.circle(img, (x_cent, y_cent), 10, (0, 255, 0), 3)
            x, y = x_3, y_3
        elif w1 < w2 > w3:
            x_cent = round((x_2 + x_3) / 2)  # координаты центра длинной стороны
            y_cent = round((y_2 + y_3) / 2)
            x, y = x_1, y_1
        elif w2 < w3 > w1:
            x_cent = round((x_1 + x_3) / 2)  # координаты центра длинной стороны
            y_cent = round((y_1 + y_3) / 2)
            x, y = x_2, y_2
        length_vector = ((x - x_cent) ** 2 + (y - y_cent) ** 2) ** 0.5
        vector_coord = ((x - x_cent), (y - y_cent))
        cos_a = (int(vector_coord[0]) * 320) / (int(length_vector * 320))
        angle = int(math.acos(cos_a) * 180 / math.pi)
        print(str(angle))
        return angle
    else:
        return 0


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



time_flag = time.time()
initial_yaw = auv.get_yaw()
while (time.time()-time_flag)<5:
    time.sleep(0.001)
time_flag = time.time()

passed = []

time_flag = time.time()
while True:
    try:
        error = keep_yaw(-90)
        # print(error)
        if abs(error) > 5:
            time_flag = time.time()
        if (time.time() - time_flag) > 4:
            print("180")
            break   
    except ZeroDivisionError: pass
    img = auv.get_image_front()
    cv.imshow("img", img)

auv.set_motor_power(0, 20)
auv.set_motor_power(1, 20)
time.sleep(3)

while True:
    try:
        error = keep_yaw(0)
        # print(error)
        if abs(error) > 5:
            time_flag = time.time()
        if (time.time() - time_flag) > 4:
            print("180")
            break   
    except ZeroDivisionError: pass
    img = auv.get_image_front()
    cv.imshow("img", img)

auv.set_motor_power(0, -20)
auv.set_motor_power(1, -20)
time.sleep(5)

time_flag = None
action_flag = None

def rotate(yaw, depth=None):
    while True:
        try:
            error = keep_yaw(yaw)
            if depth is not None:
                keep_depth(depth)
            print(error)
            if abs(error) > 5:
                time_flag = time.time()
            if (time.time() - time_flag) > 4:
                print("180")
                break   
        except ZeroDivisionError: pass

actions_i = 0

while True:
    stabbing = False
    find_new_shape = False
    set_shape = False
    d = auv.get_depth()
    timer = time.time()
    detected = False
    black = False
    while True:
        # print("iter")
        img = auv.get_image_front()
        h, w, _ = img.shape
        x1, x2, y1, y2 = 0, w // 6, 0, h
        img_crop = img[y1:y2, x1:x2, :]
        draw = img.copy()
        cv.rectangle(draw, (x1, y1), (x2, y2), (255, 0, 0), 3)
        coord = []  # список контуров для сортировки
        
        for (low_hsv, max_hsv, color) in [
                (low_hsv, max_hsv, "blue"),    
                # ((122, 149, 148), (146, 251, 255), "blue"),
                # ((70, 172, 183), (77, 222, 255), "green"),
                # ((4, 144, 199), (40, 255, 245), "orange")
            ]:
            cnt = find_contour(img_crop, low_hsv, max_hsv)  # подбирать для заданных цветов
            # может стоит добавить выборку, здесь используются цвета вне фона.
            
            if cnt:
                for c in cnt:
                    if abs(cv.contourArea(c)) > 100:
                        form, points = define_form(c)
                        cont = cv.convexHull(c)
                        try:
                            moments = cv.moments(points)
                            # x = int(moments["m10"] / moments["m00"])
                            # y = int(moments["m01"] / moments["m00"])
                            f = defining_figure(cont)
                            (x, y, w, h) = cv.boundingRect(cont)
                            cv.rectangle(draw, (x, y), (x + w, y + h), (255, 0, 0))
                            # cv.putText(draw, f[2] + ' ' + color, (x, y), cv.FONT_HERSHEY_COMPLEX_SMALL, 2, (255, 0, 0))
                            coord.append((x + w // 2, y + h // 2, c))
                        except ZeroDivisionError:
                            time.sleep(0.001)
                        cv.drawContours(draw, cont, -1, (255, 0, 0), 4)
        
        # coord = coord.sort()
        if action_flag is not None and time.time() - action_flag > 3:
            print("LEFT")
            rotate(-90)
            auv.set_motor_power(0, -20)
            auv.set_motor_power(1, -20)
            time.sleep(4)
            action_flag = None
            frame = auv.get_image_front()
            cnt = find_contour(frame, low_hsv, max_hsv)
            if cnt:
                for c in cnt:
                    if cv.contourArea(c) > 100:
                        x, y, f = defining_figure(c)
                        print(x, y, f)

                        # cv.imshow("why triangle", auv.get_image_front())
                        # cv.waitKey(0)
                        if f == "triangle":
                            rotate(90)
                            rotate(-90)
                        break

            mask = cv.inRange(img_crop, (93, 186,  78), (104, 255, 252))
            cnt, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for c in cnt:
                if cv.contourArea(c) > 100:
                    print("VSPLIVAU")
                    
                    auv.set_motor_power(0, 50)
                    auv.set_motor_power(1, 50)
                    time.sleep(2)
                    auv.set_motor_power(0, 0)
                    auv.set_motor_power(1, 0)
                    auv.set_motor_power(2, 50)
                    auv.set_motor_power(3, 50)
                    time.sleep(5)
                    exit()

            auv.set_motor_power(0, 20)
            auv.set_motor_power(1, 20)
            time.sleep(4)
            rotate(0, d)
        elif len(coord) == 0:
            if detected:
                detected = False
                print('actions')
                actions_i += 1
                action_flag = time.time()
                d = 120 - 240 / 2
                # auv.set_motor_power(0, 50)
                # auv.set_motor_power(1, 50)
                # auv.set_motor_power(2, 0)
                # auv.set_motor_power(3, 0)
                # time.sleep(2)
                # fuck(90)
                # auv.set_motor_power(0, 20)
                # auv.set_motor_power(1, 20)
                # time.sleep(3)
                # fuck(-90)
                # auv.set_motor_power(0, 20)
                # auv.set_motor_power(1, 20)
                # time.sleep(5)
                # auv.set_motor_power(0, -20)
                # auv.set_motor_power(1, -20)
                # time.sleep(1)
                # fuck(0)
        else:
            detected = True
            d = coord[0][1] - 240 / 2
            print(d)
            

        auv.set_motor_power(0, 10)
        auv.set_motor_power(1, 10)
        if time.time() - timer > 0.3:
            try:
                keep_depth(d, 0.5)
                keep_yaw(0)
            except ZeroDivisionError: pass
            timer = time.time()
        time.sleep(0.01)
        
        cv.imshow("draw", draw)
        thresh = thresholding_find_contours(img, 255, 255, 16)
        cv.imshow("thresh", thresh)
        cv.waitKey(1)

    # 
    # цикл по времени и ошибке
    # определить цвет и фигуру следующей таблички
    # keep_yaw(initial_yaw)

# движение вперёд по времени: t = Kt*betha

# цикл стабилизации (по всем параметрам и площади) и проверка betha ~ 0
# определение фигуры и выполнение задания
# initial_yaw и ищется 2й контур слева или можно запомнить цвет/фигуру  ??? как отделить 2ю фигуру от 1й